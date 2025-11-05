/*

v1.2-041125 [manual wdt disable]
HTTPS POLLING SIM7080-NRF52832DK
    Works via setting up networking (APN), then HTTPS session w/ certs
    Polls server every 10s for commands; polling 7<= can cause brownouts (currently close to pin 2x 220uF, 1x 100nF, 1x 10uF & away 5x 220uF extra)
    Backend sends NOCMD if no commands pending
    1-2s (or at least 800ms) delay before AT+SHREAD (after SHREQ) is critical to avoid CME ERROR 3

    Modem PWRKEY & interrupt pin from SBC not added yet (todo) + cleanup needed to do (unndeed includes/defines, comments, code etc)
    More notes:
     *Initial setup can be sped up
     *Debug prints & delays can be removed &or reduced
     *Cert doesn't have to be sent everytime (could be in main / everytime the modem boots, perhaps implement w/ PWRKEY pin logic)

     v1.1-201025 - added SBC modem handoff to MCU x PSM&eDRX states x updated dts [psm/edrx unused, not implemented in backend yet]
     **v1.1-241025a - additional handoff logic - tested working (no button logic yet) MCU-SBC-MCU handoff cycle OK via systemctl suspend/HTTPS wake
     v1.1-241025b - handoff that "works" MCU-SBC-MCU handoff (60-90% reliable suspend/HTTPS wake) + button handoff (50-80% reliable) idek
            Note: sometimes dies, sometimes modem starts speaking chinese (nihao!)
        241025a-R - refactoring (starting w stable 241025a version not 241025b); R2 - fixed + tested
     *v1.1-271025 - Merged 241025a-R2 & 241025b - Works reliably. Tested. [Demo-ready / PoC-ready]
            Note: when SBC is active for a while, and MCU takes back control, MCU/Modem run into @cloudflare issue, mistakenly think HTTPS conn successful
     v1.1-271025a - Added PWRKEY functionality, recovery, logging states
     v1.1-291025 - Watchdog reset on nrf stuck state -- low power functionality for nrf
     v1.2-311025 - WDT+PWRKEY+R+Logging+Restfunctionality - untested
     v1.2-031125 - tested wdt+pwrkey+logging, reliable but slow
     v1.2-031125a - Optimizations + on-board debugger for enable/disable wdt added [untested] - redacted, 04 replaced w other logic
     v1.3-xx implementing PSM/eDRX & testing
    24- todo: PWRKEY, cloudfare fix?, more testing, power states, nRF sleepstates, add-on watchdog for stuck states
    **Superstable
    *Stable
    .dts added below (bottom) as git doesn't track it

*/


// ZEPHYR RTOS INCLUDES
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>


// UART CONFIGURATION
#define BAUD_RATE                   921600
#define RX_BUF_SIZE                 128
#define RESPONSE_BUF_SIZE           1024
#define RX_TIMEOUT_DELAY            500

// PWRKEY CONFIGURATION
#define PWRKEY_HIGH_MS              1075
#define PWRKEY_LOW_MS               1075
#define MODEM_RESET_HOLD_MS         13000      
#define MODEM_BOOT_HOLD_MS          1500        
#define MODEM_BOOT_WAIT_MS          10000       
#define MODEM_POST_RESET_DELAY_MS   5000 

// SERVER CONFIGURATION
#define SERVER_URL                  "seven080-mcu-backend.onrender.com"
#define SERVER_HOST                 "seven080-mcu-backend.onrender.com"
#define DEVICE_ID                   "device001"
#define POLL_INTERVAL_SEC           10

// CERT
#define CA_CERT_FILE                "server_ca.cer"
#define CA_CERT_SIZE                2048

// WATCHDOG CONFIGURATION
#define WATCHDOG_TIMEOUT_MS         30000

// DEVICE TREE REFERENCES (GPIO, UART) - pins etc
const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0)); // P0.06=TX, P0.08=RX
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios); //for pwrpin, state testing/debugging
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios); // wdt customizer
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios); // onboard LED1
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); // onboard LED2
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios); // onboard LED3, wdt
static const struct gpio_dt_spec trigger_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger0), gpios, {0}); // P0.11 to SBC
static const struct gpio_dt_spec sbc_handoff = GPIO_DT_SPEC_GET(DT_ALIAS(wakepin), gpios); // MCU P0.28 << P32 SBC

// Power key pin
static const struct gpio_dt_spec pwrkey_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger1), gpios, {0}); // P0.12 to PWRKEY (Modem)
// Watchdog
static const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
static int wdt_channel_id;

// STATE STRUCTURES
typedef struct {
    bool is_connected;
    bool is_session_active;
    int last_status_code;
    int last_data_size;
    bool is_shreq_received;
} http_state_t;

typedef struct {
    bool is_psm_enabled;
    bool is_edrx_enabled;
} power_state_t;

typedef struct {
    bool is_certificate_setup;
    bool is_expecting_body;
    bool is_response_complete;
    bool is_waiting_for_response;
    size_t response_length;
} uart_state_t;

// logging
typedef struct {
    uint32_t boot_count;
    uint32_t poll_success;
    uint32_t poll_fail;
    uint32_t handoff_count;
    uint32_t resent_count;
} device_stats_t;
static device_stats_t stats = {0};

// SBC HANDOFF ENHANCEMENTS
static volatile bool sbc_handoff_in_progress = false;
static char pending_ack_cmd[128];
static bool pending_ack = false;

// GLOBAL STATE
static http_state_t http_state = {0};
static power_state_t power_state = {0};
static uart_state_t uart_state = {0};

static bool is_start_networking = false;
static bool is_sbc_active = false;
static int packet_counter = 0;


// BUFFERS
static uint8_t rx_buffer[RX_BUF_SIZE];
static char response_buffer[RESPONSE_BUF_SIZE];
static char command_data[256];
static bool is_command_received = false;

// WORK QUEUE
static struct k_work sbc_handoff_work;
static struct k_work pwrkey_work;
static struct gpio_callback button_callback_data;
static struct gpio_callback button2_callback_data;
static struct gpio_callback sbc_handoff_cb_data;

// CERTIFICATE DATA

const char ca_certificate[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIDejCCAmKgAwIBAgIQf+UwvzMTQ77dghYQST2KGzANBgkqhkiG9w0BAQsFADBX\n"
"MQswCQYDVQQGEwJCRTEZMBcGA1UEChMQR2xvYmFsU2lnbiBudi1zYTEQMA4GA1UE\n"
"CxMHUm9vdCBDQTEbMBkGA1UEAxMSR2xvYmFsU2lnbiBSb290IENBMB4XDTIzMTEx\n"
"NTAzNDMyMVoXDTI4MDEyODAwMDA0MlowRzELMAkGA1UEBhMCVVMxIjAgBgNVBAoT\n"
"GUdvb2dsZSBUcnVzdCBTZXJ2aWNlcyBMTEMxFDASBgNVBAMTC0dUUyBSb290IFI0\n"
"MHYwEAYHKoZIzj0CAQYFK4EEACIDYgAE83Rzp2iLYK5DuDXFgTB7S0md+8Fhzube\n"
"Rr1r1WEYNa5A3XP3iZEwWus87oV8okB2O6nGuEfYKueSkWpz6bFyOZ8pn6KY019e\n"
"WIZlD6GEZQbR3IvJx3PIjGov5cSr0R2Ko4H/MIH8MA4GA1UdDwEB/wQEAwIBhjAd\n"
"BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDwYDVR0TAQH/BAUwAwEB/zAd\n"
"BgNVHQ4EFgQUgEzW63T/STaj1dj8tT7FavCUHYwwHwYDVR0jBBgwFoAUYHtmGkUN\n"
"l8qJUC99BM00qP/8/UswNgYIKwYBBQUHAQEEKjAoMCYGCCsGAQUFBzAChhpodHRw\n"
"Oi8vaS5wa2kuZ29vZy9nc3IxLmNydDAtBgNVHR8EJjAkMCKgIKAehhxodHRwOi8v\n"
"Yy5wa2kuZ29vZy9yL2dzcjEuY2JsMBMGA1UdIAQMMAowCAYGZ4EMAQIBMA0GCSqG\n"
"SIb3DQEBCwUAA4IBAQAYQrsPBtYDh5bjP2OBDwmkoWhIDDkic574y04tfzHpn+cJ\n"
"odI2D4SseesQ6bDrarZ7C30ddLibZatoKiws3UL9xnELz4ct92vID24FfVbiI1hY\n"
"+SW6FoVHkNeWIP0GCbaM4C6uVdF5dTUsMVs/ZbzNnIdCp5Gxmx5ejvEau8otR/Cs\n"
"kGN+hr/W5GvT1tMBjgWKZ1i4//emhA1JG1BbPzoLJQvyEotc03lXjTaCzv8mEbep\n"
"8RqZ7a2CPsgRbuvTPBwcOMBBmuFeU88+FSBX6+7iP0il8b4Z0QFqIwwMHfs/L6K1\n"
"vepuoxtGzi4CZ68zJpiq1UvSqTbFJjtbD4seiMHl\n"
"-----END CERTIFICATE-----\n";

const size_t ca_certificate_length = sizeof(ca_certificate) - 1;

// UART COMMS
static void uart_event_callback(const struct device *dev, struct uart_event *event, void *user_data);
static void handle_rx_character(char c);
static void process_line(void);
static void trim_trailing_cr(char *buffer, size_t *length);
static void parse_shreq_response(const char *line);
static void parse_shstate_response(const char *line);
static void capture_http_body_data(const char *line, size_t length);

// AT COMMAND HELPERS
void send_at_command(const char *command);
static bool send_at_and_wait_ok(const char *command, k_timeout_t timeout);

// NETWORK
bool setup_network(void);
bool setup_https_session(void);
void cleanup_http_session(void);
bool download_and_convert_certificate(void);

// HTTP HELPERS
static bool set_http_header(const char *key, const char *value);
static bool clear_http_headers(void);
static bool clear_http_parameters(void);
static bool setup_default_headers(void);
static bool http_get(const char *endpoint);
static bool http_read_data(int size);
static bool ssl_configure(const char *param, const char *value);

// POWER MANAGEMENT
bool psm_enable(uint32_t seconds_active, uint32_t seconds_periodic_tau);
bool psm_disable(void);
bool edrx_enable(uint32_t seconds_cycle);
bool edrx_disable(void);
static uint8_t encode_psm_timer(uint32_t seconds);
static const char* edrx_seconds_to_code(uint32_t seconds);

// MODEM PWRKEY
void pwrkey_work_handler(struct k_work *work);
void modem_hard_reset(void);
void pull_pwrkey_low(void);
void pull_pwrkey_high(void);
int modem_full_reset(void);

// PPP
bool end_ppp_session(void);

// COMMAND PROCESSING
void poll_for_commands(void);
void process_command(const char *response);
void execute_command(const char *command, int pin, const char *data);

// WAIT FUNCTIONS
bool wait_for_response_with_timeout(const char *expected, k_timeout_t timeout);
bool wait_for_modem_ready(int attempts, k_timeout_t per_try_timeout);
bool wait_for_ok_error(k_timeout_t timeout);

// CALLBACKS
static void button_pressed_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins);
static void button2_pressed_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins);
static void sbc_handoff_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins);
static void sbc_handoff_work_handler(struct k_work *work);

// THREADS
void polling_thread(void);

// WATCHDOG
static void watchdog_feed(void);
static void watchdog_init(void);
static bool is_in_recovery = false;
int handle_watchdog_recovery(void);

//debugging-tooling for wdt
bool is_skip_wdt = false;


// UART PARSING ---------------------------------------------------------------------

static void trim_trailing_cr(char *buffer, size_t *length)
{
    if (*length > 0 && buffer[*length - 1] == '\r') {
        buffer[*length - 1] = '\0';
    }
}


static void parse_shreq_response(const char *line)
{
    char *comma1 = strchr(line, ',');
    if (comma1) {
        char *comma2 = strchr(comma1 + 1, ',');
        if (comma2) {
            http_state.last_status_code = atoi(comma1 + 1);
            http_state.last_data_size = atoi(comma2 + 1);
            printk("HTTPS Status: %d, Size: %d\n", 
                http_state.last_status_code, http_state.last_data_size);
            http_state.is_shreq_received = true;
        }
    }
}


static void parse_shstate_response(const char *line)
{
    http_state.is_session_active = (strstr(line, "+SHSTATE: 1") != NULL);
    printk("HTTPS Session State: %s\n", http_state.is_session_active ? "Connected" : "Disconnected");
}


static void capture_http_body_data(const char *line, size_t length)
{
    if (strcmp(line, "OK") != 0 &&
        strcmp(line, "ERROR") != 0 &&
        strncmp(line, "+", 1) != 0) {
        
        if (length < sizeof(command_data)) {
            strncpy(command_data, line, sizeof(command_data) - 1);
            command_data[sizeof(command_data) - 1] = '\0';
            is_command_received = true;
            printk("Command data captured: %s\n", command_data);
        }
    }
}


static void process_line(void)
{
    response_buffer[uart_state.response_length] = '\0';
    trim_trailing_cr(response_buffer, &uart_state.response_length);
    
    printk("<<< %s\n", response_buffer);

    if (strcmp(response_buffer, "OK") == 0) {
        uart_state.is_response_complete = true;
        uart_state.is_waiting_for_response = false;
    } 
    else if (strcmp(response_buffer, "ERROR") == 0 ||
             strstr(response_buffer, "+CME ERROR:") != NULL) {
        uart_state.is_response_complete = true;
        uart_state.is_waiting_for_response = false;
    }
    else if (strstr(response_buffer, "+SHREQ:") != NULL) {
        parse_shreq_response(response_buffer);
        uart_state.is_response_complete = true;
    }
    else if (strstr(response_buffer, "+SHSTATE:") != NULL) {
        parse_shstate_response(response_buffer);
        uart_state.is_response_complete = true;
    }
    else if (strstr(response_buffer, "+SHREAD:") != NULL) {
        // The next line will contain the actual data
    }
    else if (uart_state.is_expecting_body && uart_state.response_length > 0) {
        capture_http_body_data(response_buffer, uart_state.response_length);
    }
    
    uart_state.response_length = 0;
}

static void handle_rx_character(char c)
{
    if (c == '\n') {
        if (uart_state.response_length > 0) {
            process_line();
        }
    } else if (c != '\r') {
        if (uart_state.response_length < sizeof(response_buffer) - 1) {
            response_buffer[uart_state.response_length++] = c;
        } else {
            uart_state.response_length = 0;
        }
    }
}


static void uart_event_callback(const struct device *dev, struct uart_event *event, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    switch (event->type) {
    case UART_RX_RDY: {
        const uint8_t *p = &event->data.rx.buf[event->data.rx.offset];
        size_t l = event->data.rx.len;

        for (size_t i = 0; i < l; i++) {
            handle_rx_character(p[i]);
        }
        break;
    }
    case UART_RX_DISABLED:
    case UART_RX_STOPPED:
        uart_rx_enable(uart0, rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_DELAY);
        break;
    default:
        break;
    }
}

// AT COMMAND HELPERS ------------------------------------------------

void send_at_command(const char *command)
{
    if (is_sbc_active && !sbc_handoff_in_progress) {
        printk("SKIP AT CMD (SBC active): %s\n", command);
        return;
    }
    printk(">>> %s\n", command);
    for (int i = 0; command[i] != '\0'; i++) {
        uart_poll_out(uart0, command[i]);
    }
    uart_poll_out(uart0, '\r');
}

bool wait_for_ok_error(k_timeout_t timeout)
{
    uart_state.is_response_complete = false;
    uart_state.is_waiting_for_response = true;

    int64_t start = k_uptime_get();
    while (!uart_state.is_response_complete && (k_uptime_get() - start) < timeout.ticks) {
        if (is_in_recovery){
            watchdog_feed();
        }
        k_msleep(is_in_recovery ? 7 : 50); //when in recovery from wdt, use 7ms for much speedier response
    }

    if (!uart_state.is_response_complete) {
        printk("Response timeout\n");
        uart_state.is_waiting_for_response = false;
        return false;
    }

    uart_state.is_waiting_for_response = false;
    return true;
}

bool wait_for_response_with_timeout(const char *expected, k_timeout_t timeout)
{
    uart_state.response_length = 0;
    uart_state.is_response_complete = false;
    uart_state.is_waiting_for_response = true;

    int64_t start = k_uptime_get();
    while (!uart_state.is_response_complete && (k_uptime_get() - start) < timeout.ticks) {
        if (uart_state.response_length > 0 && strstr(response_buffer, expected) != NULL) {
            uart_state.is_response_complete = true;
            break;
        }
        k_msleep(50);
    }

    uart_state.is_waiting_for_response = false;
    return uart_state.is_response_complete;
}


// HTTP HELPERS ------------------------------------------------

static bool clear_http_headers(void)
{
    return send_at_and_wait_ok("AT+SHCHEAD", K_SECONDS(2));
}

static bool clear_http_parameters(void)
{
    return send_at_and_wait_ok("AT+SHCPARA", K_SECONDS(2));
}

static bool set_http_header(const char *key, const char *value)
{
    char command[128];
    snprintf(command, sizeof(command), "AT+SHAHEAD=\"%s\",\"%s\"", key, value);
    return send_at_and_wait_ok(command, K_SECONDS(2));
}

static bool setup_default_headers(void)
{
    if (!clear_http_headers() || !clear_http_parameters()) {
        return false;
    }
    if (!set_http_header("User-Agent", "nRF52-IoT-Controller")) return false;
    if (!set_http_header("Accept", "*/*")) return false;
    if (!set_http_header("Cache-control", "no-cache")) return false;
    if (!set_http_header("Connection", "keep-alive")) return false;
    return true;
}

static bool http_get(const char *endpoint)
{
    char command[128];
    snprintf(command, sizeof(command), "AT+SHREQ=\"%s\",1", endpoint);
    send_at_command(command);
    return wait_for_ok_error(K_SECONDS(30));
}

static bool http_read_data(int size)
{
    char command[40];
    snprintf(command, sizeof(command), "AT+SHREAD=0,%d", size);
    if (!send_at_and_wait_ok(command, K_SECONDS(3))) {
        return false;
    }

    int64_t start = k_uptime_get();
    while (!is_command_received && (k_uptime_get() - start) < K_SECONDS(3).ticks) {
        k_msleep(50);
    }
    return is_command_received;
}

static bool ssl_configure(const char *param, const char *value)
{
    char command[128];
    if (value) {
        snprintf(command, sizeof(command), "AT+CSSLCFG=\"%s\",1,\"%s\"", param, value);
    } else {
        snprintf(command, sizeof(command), "AT+CSSLCFG=\"%s\",1", param);
    }
    return send_at_and_wait_ok(command, K_SECONDS(3));
}

// Utility: send AT and wait for OK
static bool send_at_and_wait_ok(const char *command, k_timeout_t timeout)
{
    send_at_command(command);
    return wait_for_ok_error(timeout);
}


// NETWORK SETUP ------------------------------------------------

bool setup_network(void)
{

    /*
    AT+CMEE=2 - Enable verbose errors
    AT - Init, ATE0 - echo off
    AT+IPR - Set baud rate (sbc is 921600 since CATM1 7080 upload is up to 1Mbps)
    AT+GMR - Request TA revision identification of software release
    AT+CPIN? - OK if PSWD required (NOT) else WRITE (in our case it's fine)
    AT+CGREG=1 - Enable network registration unsolicitated result code
    AT+CGREG? - Check registration status (1,1 or 1,5 = registered)
    AT+CGDCONT=1,"IP","internet.telia.ee" - Define PDP context (APN)
    AT+CNACT=0,1 - Activate PDP context (APP Network Active)
    */

    watchdog_feed();
    printk("Setting up network...\n");
    if (http_state.is_session_active){
        send_at_command("AT+CGREG?");
        if (wait_for_response_with_timeout("+CGREG:", K_SECONDS(2))) {
            if (strstr(response_buffer, "+CGREG: 1,1") != NULL || strstr(response_buffer, "+CGREG: 1,5") != NULL) {
                return true;
            }
        }
        http_state.is_session_active = false;
        http_state.is_connected = false;
    }

    if (!send_at_and_wait_ok("AT+CMEE=2", K_SECONDS(2))) return false;
    if (!send_at_and_wait_ok("AT", K_SECONDS(2))) return false;
    if (!send_at_and_wait_ok("ATE0", K_SECONDS(2))) return false;
    
    char ipr_command[32];
    snprintf(ipr_command, sizeof(ipr_command), "AT+IPR=%d", BAUD_RATE);
    if (!send_at_and_wait_ok(ipr_command, K_SECONDS(2))) return false;

    if (!send_at_and_wait_ok("AT+GMR", K_SECONDS(2))) return false;
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CPIN?");
    if (!wait_for_response_with_timeout("READY", K_SECONDS(2))) return false;
    
    if (!send_at_and_wait_ok("AT+CGREG=1", K_SECONDS(2))) return false;
    
    bool is_registered = false;
    for (int i = 0; i < 15; i++) {
        send_at_command("AT+CGREG?");
        if (wait_for_response_with_timeout("+CGREG: 1,1", K_SECONDS(2)) ||
            wait_for_response_with_timeout("+CGREG: 1,5", K_SECONDS(2))) {
            is_registered = true;
            break;
        }
        k_sleep(K_SECONDS(2));
    }
    
    if (!is_registered) {
        printk("Network registration failed\n");
        return false;
    }
    
    char pdp_command[80];
    snprintf(pdp_command, sizeof(pdp_command), "AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"");
    if (!send_at_and_wait_ok(pdp_command, K_SECONDS(3))) return false;
    
    if (!send_at_and_wait_ok("AT+CNACT=0,1", K_SECONDS(15))) {
        send_at_command("AT+CNACT?");
        if (!wait_for_response_with_timeout("+CNACT: 0,1", K_SECONDS(2))) {
            return false;
        }
        k_sleep(K_SECONDS(2));
    }
    k_sleep(K_SECONDS(1));
    
    http_state.is_connected = true;
    printk("Network setup complete\n");
    return true;
}


// HTTPS SETUP ------------------------------------------------

bool download_and_convert_certificate(void)
{
    printk("Setting up certificate for HTTPS...\n");

    if (!send_at_and_wait_ok("AT+CFSINIT", K_SECONDS(1))) return false;
    
    char command[100];
    snprintf(command, sizeof(command), "AT+CFSDFILE=3,\"%s\"", CA_CERT_FILE);
    if (!send_at_and_wait_ok(command, K_SECONDS(1))) return false;

    snprintf(command, sizeof(command), "AT+CFSWFILE=3,\"%s\",0,%d,10000", 
             CA_CERT_FILE, (int)ca_certificate_length);
    send_at_command(command);
    k_sleep(K_SECONDS(2));

    for (size_t i = 0; i < ca_certificate_length; i++) {
        uart_poll_out(uart0, ca_certificate[i]);
    }
    k_sleep(K_SECONDS(3));

    if (!send_at_and_wait_ok("AT+CFSTERM", K_SECONDS(1))) return false;

    snprintf(command, sizeof(command), "AT+CSSLCFG=\"convert\",2,\"%s\"", CA_CERT_FILE);
    if (!send_at_and_wait_ok(command, K_SECONDS(10))) return false;

    uart_state.is_certificate_setup = true;
    return true;
}

bool setup_https_session(void)
{
    /*
    CSSLCFG: Configure SSL params of a context identifier
    AT+CSSLCFG="ignorertctime",1,1 - Ignore RTC time check (1=ignore), could also set time/ignore this (time has been set to 2025 oct 1 as of v1.0-081025)
    AT+CSSLCFG="sslversion",1,3 - Set SSL version to TLS 1.2 (3)
    AT+CSSLCFG="sni",1,"seven080-mcu-backend.onrender.com" - Set SNI
    AT+SHSSL=1,"server_ca.cer" - Use the certificate for verification
    AT+SHCONF="URL","https://seven080-mcu-backend.onrender.com" - Set URL (HTTPS)
    AT+SHCONF="BODYLEN",1024 - Set body length
    AT+SHCONF="HEADERLEN",350 - Set header length
    AT+CDNSGIP="seven080-mcu-backend.onrender.com" - Test DNS resolution
    AT+SHCONN - Connect HTTPS session [main part, can take time]
    AT+SHSTATE? - Check state (1=connected)
    7s timeout for SHCONN seems to work ok, 10s+ safer
    */

    if (http_state.is_session_active) {
        printk("HTTPS session already active\n");
        return true;
    }

    printk("Setting up HTTPS session...\n");
    cleanup_http_session();

    if (!uart_state.is_certificate_setup) {
        if (!download_and_convert_certificate()) {
            printk("FATAL: Certificate setup failed.\n");
            return false;
        }
    } else {
        printk("Certificate already installed, skipping cert download\n");
    }

    // SSL Configuration
    send_at_command("AT+CSSLCFG=\"ignorertctime\",1,1");
    if (!wait_for_ok_error(K_SECONDS(3))) return false;
    if (!ssl_configure("sslversion", "3")) return false; // TLS 1.2
    if (!ssl_configure("sni", SERVER_HOST)) return false;

    char ssl_command[80];
    snprintf(ssl_command, sizeof(ssl_command), "AT+SHSSL=1,\"%s\"", CA_CERT_FILE);
    if (!send_at_and_wait_ok(ssl_command, K_SECONDS(3))) return false;

    char url_command[128];
    snprintf(url_command, sizeof(url_command), "AT+SHCONF=\"URL\",\"https://%s\"", SERVER_URL);
    if (!send_at_and_wait_ok(url_command, K_SECONDS(3))) return false;

    if (!send_at_and_wait_ok("AT+SHCONF=\"BODYLEN\",1024", K_SECONDS(3))) return false;
    if (!send_at_and_wait_ok("AT+SHCONF=\"HEADERLEN\",350", K_SECONDS(3))) return false;

    // Optional DNS test
    send_at_command("AT+CDNSGIP=\"" SERVER_HOST "\"");
    wait_for_ok_error(K_SECONDS(10));
    k_sleep(K_SECONDS(2));

    if (!send_at_and_wait_ok("AT+SHCONN", K_SECONDS(30))) {
        printk("HTTPS connection failed\n");
        return false;
    }

    send_at_command("AT+SHSTATE?");
    if (!wait_for_response_with_timeout("+SHSTATE: 1", K_SECONDS(5))) {
        printk("HTTPS session not active\n");
        return false;
    }

    http_state.is_session_active = true;
    printk("HTTPS session setup complete\n");
    return true;
}

void cleanup_http_session(void)
{
    if (is_sbc_active && !sbc_handoff_in_progress) {
        printk("cleanup_http_session: SBC active - skipping AT, marking inactive\n");
        http_state.is_session_active = false;
        return;
    }

    // Full HTTP/SSL teardown
    if (http_state.is_session_active) {
        printk("Cleaning up HTTPS session...\n");
        send_at_command("AT+SHDISC");   // Disconnect
        wait_for_ok_error(K_SECONDS(5));
        http_state.is_session_active = false;
    }

    send_at_command("AT+SHSSL=0");
    wait_for_ok_error(K_SECONDS(2));
    send_at_command("AT+SHTRDT");
    wait_for_ok_error(K_SECONDS(2));
    send_at_command("AT+SHCHEAD");
    wait_for_ok_error(K_SECONDS(2));
    send_at_command("AT+SHCPARA"); 
    wait_for_ok_error(K_SECONDS(2));
}


// COMMAND POLLING ------------------------------------------------

void poll_for_commands(void)
{
    watchdog_feed();
    if (is_sbc_active) {
        printk("ERR: SBC ACTIVE - SKIP POLLING\n");
        return;
    }

    if (!http_state.is_session_active && !setup_https_session()) {
        printk("Failed to setup HTTPS session\n");
        return;
    }
    
    printk("Polling for commands...\n");
    
    if (!setup_default_headers()) {
        printk("ERROR: Failed to setup HTTP headers\n");
        cleanup_http_session();
        return;
    }
    
    http_state.last_status_code = 0;
    http_state.last_data_size = 0;
    http_state.is_shreq_received = false;
    
    char endpoint[100];
    snprintf(endpoint, sizeof(endpoint), "/api/poll/%s", DEVICE_ID);
    if (!http_get(endpoint)) {
        printk("HTTPS request failed or timeout\n");
        cleanup_http_session();
        return;
    }
    
    printk("Waiting for +SHREQ response...\n");
    int64_t start = k_uptime_get();
    while (!http_state.is_shreq_received && (k_uptime_get() - start) < K_SECONDS(10).ticks) {
        k_msleep(100);
    }
    
    if (!http_state.is_shreq_received) {
        printk("ERROR: +SHREQ response timeout!\n");
        return;
    }
    
    if (http_state.last_status_code == 200 && http_state.last_data_size > 0) {
        memset(command_data, 0, sizeof(command_data));
        is_command_received = false;
        uart_state.is_expecting_body = true;

        printk("Boutta READ packet n=%d\n", packet_counter++);
        if (packet_counter > 100000) packet_counter = 0;

        if (http_read_data(http_state.last_data_size)) {
            printk("Received command: %s\n", command_data);
            process_command(command_data);
            
            // Send acknowledgment
            char acknowledgement_url[128];
            snprintf(acknowledgement_url, sizeof(acknowledgement_url), "AT+SHREQ=\"/api/ack/%s/OK\",1", DEVICE_ID);

            if (is_sbc_active && !sbc_handoff_in_progress) {
                strncpy(pending_ack_cmd, acknowledgement_url, sizeof(pending_ack_cmd) - 1);
                pending_ack_cmd[sizeof(pending_ack_cmd) - 1] = '\0';
                pending_ack = true;
                printk("ACK queued until MCU regains modem control\n");
            } else {
                send_at_command(acknowledgement_url);
                wait_for_ok_error(K_SECONDS(7));
            }
        } else {
            printk("No command data received within timeout\n");
        }
        uart_state.is_expecting_body = false;
    } else {
        printk("No commands available (Status: %d, Size: %d)\n", 
               http_state.last_status_code, http_state.last_data_size);
    }
    
    k_msleep(500);
}


// COMMAND EXECUTION ------------------------------------------------

void process_command(const char *response)
{
    printk("Processing command: %s\n", response);

    if (strstr(response, "NOCMD") != NULL) {
        printk("No commands waiting\n");
        return;
    }

    char command[32] = {0};
    int pin = 0;
    char data[64] = {0};
    if (sscanf(response, "CMD:%31[^,],PIN:%d,DATA:%63s", command, &pin, data) == 3) {
        printk("Parsed - CMD: %s, PIN: %d, DATA: %s\n", command, pin, data);
        execute_command(command, pin, data);
    } else {
        printk("Failed to parse command string\n");
    }
}

void execute_command(const char *command, int pin, const char *data)
{
    printk("Executing: %s on pin %d\n", command, pin);
    
    if (strcmp(command, "LED_ON") == 0 && gpio_is_ready_dt(&led0)) {
        gpio_pin_set_dt(&led0, 1);
        printk("LED ON\n");
    }
    else if (strcmp(command, "LED_OFF") == 0 && gpio_is_ready_dt(&led0)) {
        gpio_pin_set_dt(&led0, 0);
        printk("LED OFF\n");
    }
    else if (strcmp(command, "TOGGLE") == 0 && gpio_is_ready_dt(&led0)) {
        gpio_pin_toggle_dt(&led0);
        printk("LED TOGGLED\n");
    }
    else if (strcmp(command, "BOOT") == 0 && gpio_is_ready_dt(&trigger_pin)) {
        printk("Triggering SBC boot sequence...\n");
        gpio_pin_set_dt(&trigger_pin, 0);
        k_sleep(K_MSEC(250));
        gpio_pin_set_dt(&trigger_pin, 1);
        is_sbc_active = true;
        printk("Boot trigger complete + SBC-active state entered\n");
    }
    else if (strcmp(command, "PULSE") == 0 && pin == 11 && gpio_is_ready_dt(&trigger_pin)) {
        gpio_pin_set_dt(&trigger_pin, 0);
        k_sleep(K_MSEC(500));
        gpio_pin_set_dt(&trigger_pin, 1);
        is_sbc_active = true;
        printk("Pulsed pin %d [SBC PIN]. SBC-active state entered\n", pin);
    }
    else if (strcmp(command, "BLINK") == 0 && gpio_is_ready_dt(&led0)) {
        int count = data[0] ? atoi(data) : 3;
        if (count <= 0) count = 3;
        for (int i = 0; i < count; i++) {
            gpio_pin_set_dt(&led0, 1);
            k_sleep(K_MSEC(200));
            gpio_pin_set_dt(&led0, 0);
            k_sleep(K_MSEC(200));
        }
        printk("Blink sequence complete (%d times)\n", count);
    }
    else if (strcmp(command, "STATUS") == 0) {
        printk("Status requested - would send telemetry here\n");
    }
    else {
        printk("Unknown command: %s\n", command);
    }
}


// POWER MANAGEMENT ------------------------------------------------

/* 
    CURRENTLY UNUSED - EXAMPLE IMPLEMENTATION ONLY
    usage:
    psm_enable(30, 3600); - enable PSM w 30s active 1h TAU
    edrx_enable(60); 60s cycle 
*/

static uint8_t encode_psm_timer(uint32_t seconds)
{
    if (seconds <= 2) return seconds;
    if (seconds <= 120 && seconds % 6 == 0) return (seconds / 6) + 2;
    if (seconds <= 1800 && seconds % 60 == 0) return (seconds / 60) + 22;
    if (seconds <= 10800 && seconds % 360 == 0) return (seconds / 360) + 52;
    if (seconds <= 31536000 && seconds % 86400 == 0) return (seconds / 86400) + 82;
    return 0;
}

static const char* edrx_seconds_to_code(uint32_t seconds)
{
    if (seconds <= 5) return "0000";
    if (seconds <= 10) return "0001";
    if (seconds <= 20) return "0010";
    if (seconds <= 41) return "0011";
    if (seconds <= 61) return "0100";
    if (seconds <= 82) return "0101";
    if (seconds <= 102) return "0110";
    if (seconds <= 123) return "0111";
    if (seconds <= 143) return "1000";
    if (seconds <= 164) return "1001";
    if (seconds <= 328) return "1010";
    if (seconds <= 655) return "1011";
    if (seconds <= 1311) return "1100";
    if (seconds <= 2621) return "1101";
    if (seconds <= 5243) return "1110";
    return "1111";
}

bool psm_enable(uint32_t seconds_active, uint32_t seconds_periodic_tau)
{
    uint8_t t3324 = encode_psm_timer(seconds_active);
    uint8_t t3412 = encode_psm_timer(seconds_periodic_tau);
    if (t3324 == 0 || t3412 == 0) {
        printk("PSM: Invalid timer values\n");
        return false;
    }
    char command[64];
    snprintf(command, sizeof(command), "AT+CPSMS=1,,,\"%08b\",\"%08b\"", t3412, t3324);
    if (!send_at_and_wait_ok(command, K_SECONDS(5))) {
        printk("Failed to enable PSM\n");
        return false;
    }
    power_state.is_psm_enabled = true;
    return true;
}

bool psm_disable(void)
{
    if (!send_at_and_wait_ok("AT+CPSMS=0", K_SECONDS(5))) {
        printk("Failed to disable PSM\n");
        return false;
    }
    power_state.is_psm_enabled = false;
    return true;
}

bool edrx_enable(uint32_t seconds_cycle)
{
    const char* edrx_code = edrx_seconds_to_code(seconds_cycle);
    char command[40];
    snprintf(command, sizeof(command), "AT+CEDRXS=1,4,\"%s\"", edrx_code);
    if (!send_at_and_wait_ok(command, K_SECONDS(5))) {
        printk("Failed to enable eDRX\n");
        return false;
    }
    power_state.is_edrx_enabled = true;
    return true;
}

bool edrx_disable(void)
{
    if (!send_at_and_wait_ok("AT+CEDRXS=0,4", K_SECONDS(5))) {
        printk("Failed to disable eDRX\n");
        return false;
    }
    power_state.is_edrx_enabled = false;
    return true;
}

void modem_hard_reset(void){
    if (!gpio_is_ready_dt(&pwrkey_pin)) {
        printk("Modem reset pin not ready\n");
        return;
    }

    printk("Performing modem hard reset...\n");
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_HIGH_MS)); //minimum 1000ms, +75ms to be safe (1-12s)
    gpio_pin_set_dt(&pwrkey_pin, 0);
    k_sleep(K_SECONDS(3)); // wait for modem to shutdown
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_HIGH_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    k_sleep(K_SECONDS(3)); // wait for modem to startup
    stats.boot_count++;
    printk("Modem reboot complete\n");
}

void pull_pwrkey_low(void){
    if (!gpio_is_ready_dt(&pwrkey_pin)) {
        printk("PWRKEY pin not ready\n");
        return;
    }
    gpio_pin_set_dt(&pwrkey_pin, 0);
    k_sleep(K_MSEC(PWRKEY_LOW_MS));
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(5000));
    printk("PWRKEY PULSED LOW\n");
}

void pull_pwrkey_high(void){ //likely for boot
    if (!gpio_is_ready_dt(&pwrkey_pin)) {
        printk("PWRKEY pin not ready\n");
        return;
    }
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_HIGH_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    k_sleep(K_MSEC(5000));
    printk("PWRKEY PULSED HIGH\n");
}


void pwrkey_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    pull_pwrkey_high();
}


int modem_full_reset(void) {
    if (!gpio_is_ready_dt(&pwrkey_pin)) {
        printk("Modem reset pin not ready\n");
        return 1;
    }

    watchdog_feed();
    stats.boot_count++;
    printk("Boot attempt #%u\n", stats.boot_count);

    printk("Performing full modem reset...\n");
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(13000));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    k_sleep(K_SECONDS(5)); // allow modem to settle
    send_at_command("AT");
    if (wait_for_ok_error(K_SECONDS(5))) {
        printk("Modem full reset complete and responsive\n");
        return 0;
    }

    printk("ERROR: Modem NOT RESPONSIVE after FULL RESET, attempting boot sequence...\n");

    watchdog_feed();
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(1500));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    k_sleep(K_SECONDS(5));

    send_at_command("AT");
    if (wait_for_ok_error(K_SECONDS(5))) {
        printk("Modem BOOT SUCCESS after fallback!\n");
        return 0;
    }

    watchdog_feed();
    printk("Trying hard reset [REBOOT]\n");
    modem_hard_reset();
    send_at_command("AT");
    if (wait_for_ok_error(K_SECONDS(5))) {
        printk("Modem RECOVERED\n");
        return 0;
    } else {
        printk("CRITICAL FAILURE: Modem FAILED after all recovery attempts\n");
    }

    printk("LOG: boot_count=%u, poll_success=%u, poll_fail=%u, handoff_count=%u, resent_count=%u\n",
           stats.boot_count, stats.poll_success, stats.poll_fail,
           stats.handoff_count, stats.resent_count);
    return 1;
}



// PPP & SBC HANDOFF ------------------------------------------------

bool wait_for_modem_ready(int attempts, k_timeout_t per_try_timeout)
{
    for (int i = 0; i < attempts; i++) {
        if (is_sbc_active && !sbc_handoff_in_progress) {
            printk("wait_for_modem_ready abort: SBC active\n");
            return false;
        }
        send_at_command("AT");
        if (wait_for_ok_error(per_try_timeout)) {
            printk("Modem ready (AT -> OK)\n");
            return true;
        }
        printk("Modem not ready yet (attempt %d/%d). Retrying...\n", i+1, attempts);
        k_msleep(200 + (i * 100));
    }
    return false;
}

bool end_ppp_session(void)
{
    watchdog_feed();
    printk("Ending PPP session...\n");
    k_sleep(K_MSEC(1100));
    uart_poll_out(uart0, '+');
    uart_poll_out(uart0, '+');
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(3000)); // <-- increased from 1100 to 3000 ms

    if (!wait_for_ok_error(K_SECONDS(5))) {
        printk("FAILED TO STOP PPP\n");
        return false;
    }
    printk("PPP escape successful\n");

    send_at_command("ATH");
    if (!wait_for_ok_error(K_SECONDS(2))) {
        printk("Failed to hang up data call\n");
        return false;
    }

    http_state.is_connected = false;
    http_state.is_session_active = false;
    return true;
}

static void sbc_handoff_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    k_msleep(100); // debounce

    int val = gpio_pin_get_dt(&sbc_handoff);
    printk("SBC handoff work handler (pin=%d)\n", val);

    if (val == 1) {
        // SBC became active → relinquish
        is_sbc_active = true;
        sbc_handoff_in_progress = false;
        printk("SBC ACTIVE → MCU cleans up\n");
        cleanup_http_session();
        
        if (device_is_ready(uart0) && !pm_device_is_busy(uart0)){
            uart_tx_abort(uart0);
            pm_device_action_run(uart0, PM_DEVICE_ACTION_SUSPEND);
            printk("UART SUSPENDED\n");
        }
        watchdog_feed();
        k_sleep(K_FOREVER);
        return;
    }
    else{
        printk("SBC handed off → MCU resuming control\n");
        stats.handoff_count++;
        sbc_handoff_in_progress = true;
        is_sbc_active = false;
        if (device_is_ready(uart0)){
            pm_device_action_run(uart0, PM_DEVICE_ACTION_RESUME);
            uart_rx_enable(uart0, rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_DELAY);
            printk("UART RESUME\n");
        }
    }
    sbc_handoff_in_progress = false;
    is_start_networking = true;


    // Retry PPP termination up to 4 times
    bool ended = false;
    for (int attempt = 0; attempt < 4; attempt++) {
        if (end_ppp_session()) {
            ended = true;
            break;
        }
        printk("PPP end attempt %d failed, retrying...\n", attempt + 1);
        k_msleep(300 + (attempt * 200));
    }

    if (!ended) {
        printk("Warning: PPP end failed; proceeding anyway\n");
    }

    k_sleep(K_MSEC(500));

    // Ensure modem is responsive
    if (!wait_for_modem_ready(5, K_SECONDS(2))) {
        printk("Modem unresponsive – resetting via PWRKEY\n");
        modem_hard_reset();
        if (!wait_for_modem_ready(10, K_SECONDS(3))) {
            printk("Modem still dead after reset\n");
            return;
        }
    }

    k_sleep(K_MSEC(500));

    // Re-establish HTTPS
    if (!setup_https_session()) {
        printk("HTTPS setup failed after handoff\n");
    }

    // Send pending ACK if any
    if (pending_ack && !is_sbc_active) {
        printk("Sending pending ACK: %s\n", pending_ack_cmd);
        send_at_command(pending_ack_cmd);
        wait_for_ok_error(K_SECONDS(7));
        pending_ack = false;
    }

    sbc_handoff_in_progress = false;
}


// INTERRUPT CALLBACKS ------------------------------------------------

static void button_pressed_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(callback); ARG_UNUSED(pins);
    printk("Button pressed! Starting communication test...\n");
    is_start_networking = true;
}

static void sbc_handoff_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(callback); ARG_UNUSED(pins);
    printk("SBC handoff IRQ -> scheduling sbc_handoff_work\n");
    k_work_submit(&sbc_handoff_work);
}

static void button2_pressed_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(callback); ARG_UNUSED(pins);
    printk("Button2 pressed! PWRKEY-TEST\n");
    k_work_submit(&pwrkey_work);
}


// THREADS ------------------------------------------------

void polling_thread(void)
{
    printk("Starting polling thread...\n");
    while (!is_start_networking) {
        k_sleep(K_MSEC(100));
    }
    printk("Button triggered networking start!\n");
    
    if (!setup_network()) {
        printk("Network setup failed!\n");
        return;
    }
    
    while (1) {
        if (!is_sbc_active) {
            poll_for_commands();
            k_sleep(K_SECONDS(POLL_INTERVAL_SEC));
            (http_state.last_status_code == 200) ? stats.poll_success++ : stats.poll_fail++;
        } else {
            watchdog_feed();
            printk("SBC active - MCU waiting/idle\n");
            k_sleep(K_SECONDS(5));
        }
    }
}


// WATCHDOG ------------------------------------------------

static void watchdog_feed(void)
{   
    if (wdt && device_is_ready(wdt)) {
        wdt_feed(wdt, wdt_channel_id);
    }
}

static void watchdog_init(void)
{
    if (!device_is_ready(wdt)) {
        printk("Watchdog device not ready\n");
        return;
    }

    struct wdt_timeout_cfg wdt_config = {
        .window.min = 0U,
        .window.max = WATCHDOG_TIMEOUT_MS,
        .flags = WDT_FLAG_RESET_SOC,
        .callback = NULL
    };

    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if (wdt_channel_id < 0) {
        printk("Failed to install watchdog timeout: %d\n", wdt_channel_id);
        return;
    }
    
    int error = wdt_setup(wdt, 0);
    if (error < 0) {
        printk("Failed to setup watchdog: %d\n", error);
        return;
    }

    printk("Watchdog started: [T: %dms]\n", WATCHDOG_TIMEOUT_MS);
}


int handle_watchdog_recovery(void)
{
    printk("INITIATING WDT RECOVERY\n");
    is_in_recovery = true;

    if (wait_for_modem_ready(2, K_SECONDS(1))) {    // wait for modem to respond
        http_state.is_session_active = false;
        memset(response_buffer, 0, sizeof(response_buffer)); //CAN REMOVE ! experimental 
        uart_state.response_length = 0;                      //CAN REMOVE ! experimental 
        send_at_command("AT+SHSTATE?");     // check if HTTPS is plausible
        
        int64_t start = k_uptime_get();
        while ((k_uptime_get() - start) < K_SECONDS(3).ticks) {
            if (strstr(response_buffer, "+SHSTATE:") != NULL) {
                break; 
            }
            k_msleep(20);
            watchdog_feed();
        }

        if (http_state.is_session_active) {
            printk("HTTPS session intact — resuming\n");
        }
        return 0;
    }

    watchdog_feed();
    printk("Modem unresponsive — trying PPP escape (+++)\n");
    //sending +++ as modem might be in PPP mode (mode SBC uses), +++ cancels it and enables AT commands
    k_sleep(K_MSEC(1100));
    uart_poll_out(uart0, '+');
    uart_poll_out(uart0, '+');
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(1400));

    if (wait_for_ok_error(K_MSEC(1000))) { //if +++ responds with OK it's most certainly back from PPP and will have certs
        printk("PPP escape succeeded\n");
        uart_state.is_certificate_setup = true;
        return 0;
    }

    // None work, modem likely powered off or hung

    printk("PPP escape failed — toggling PWRKEY\n"); 
    watchdog_feed();
    pull_pwrkey_high();
    if (send_at_and_wait_ok("AT", K_SECONDS(1))) {
        printk("Modem RESPONSIVE after PWRKEY TOGGLE\n");
        return 0;
    }
    
    watchdog_feed();
    modem_hard_reset();
    if (send_at_and_wait_ok("AT", K_SECONDS(3))) {
        printk("Modem RESPONSIVE after HARD REBOOT\n");
        return 0;
    }

    printk("Modem STILL UNRESPONSIVE — performing FULL FACTORY RESET - WAITING 15 SECONDS - REBOOT NRF TO CANCEL\n");
    watchdog_feed();
    k_sleep(K_SECONDS(15));
    return modem_full_reset();
}


K_THREAD_DEFINE(polling_tid, 4096, polling_thread, NULL, NULL, NULL, 7, 0, 0);


int main(void)
{
    if (gpio_is_ready_dt(&led2)) gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    if (gpio_is_ready_dt(&button3)) {
        gpio_pin_set_dt(&led2, 1);
        gpio_pin_configure_dt(&button3, GPIO_INPUT | GPIO_PULL_UP);
        printk("Press B3 to disable watchdog\n");

        for (int i = 0; i < 30; i++) {
            if (gpio_pin_get_dt(&button3)) { // active low
                is_skip_wdt = true;
                break;
            }
            k_sleep(K_MSEC(100));
        }
    }
    gpio_pin_set_dt(&led2, 0);
    if (!is_skip_wdt) watchdog_init();
    else printk("DEBUG MODE [WDT DISABLED MANUALLY]\n");
    uint32_t rr = NRF_POWER->RESETREAS;
    if (rr & POWER_RESETREAS_DOG_Msk) printk("WDT RESET DETECTED");

    printk("IoT Controller Starting...\n");
    k_work_init(&sbc_handoff_work, sbc_handoff_work_handler);
    k_work_init(&pwrkey_work, pwrkey_work_handler);

    if (!device_is_ready(uart0)) {
        printk("UART not ready!\n");
        return 0;
    }

    uart_callback_set(uart0, uart_event_callback, NULL);
    if (uart_rx_enable(uart0, rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_DELAY)) {
        printk("Failed to enable UART RX\n");
        return 0;
    }

    // Configure LEDs
    if (gpio_is_ready_dt(&led0)) gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    if (gpio_is_ready_dt(&led1)) gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

    // Configure trigger pin
    if (gpio_is_ready_dt(&trigger_pin)) {
        gpio_pin_configure_dt(&trigger_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&trigger_pin, 1); // default high
    }

    // Button
    if (gpio_is_ready_dt(&button)) {
        gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&button_callback_data, button_pressed_callback, BIT(button.pin));
        gpio_add_callback(button.port, &button_callback_data);
        printk("Button configured with interrupt\n");
    }

    if (gpio_is_ready_dt(&button2)) {
        gpio_pin_configure_dt(&button2, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&button2_callback_data, button2_pressed_callback, BIT(button2.pin));
        gpio_add_callback(button2.port, &button2_callback_data);
        printk("Button2 configured with interrupt\n");
    }

    // SBC handoff
    if (gpio_is_ready_dt(&sbc_handoff)) {
        gpio_pin_configure_dt(&sbc_handoff, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&sbc_handoff, GPIO_INT_EDGE_BOTH | GPIO_INT_WAKEUP);
        gpio_init_callback(&sbc_handoff_cb_data, sbc_handoff_callback, BIT(sbc_handoff.pin));
        gpio_add_callback(sbc_handoff.port, &sbc_handoff_cb_data);
        printk("SBC handoff pin configured with falling-edge interrupt\n");
    }

    // Modem PWRKEY
    if (gpio_is_ready_dt(&pwrkey_pin)){
        gpio_pin_configure_dt(&pwrkey_pin, GPIO_OUTPUT_INACTIVE);
        printk("PWRKEY pin configured\n");
    }

    watchdog_feed();

    uint32_t reason_for_boot = NRF_POWER->RESETREAS;
    if (reason_for_boot & POWER_RESETREAS_DOG_Msk) {
        printk("Watchdog reset detected\n");
        NRF_POWER->RESETREAS = 0xFFFFFFFF;
        if (handle_watchdog_recovery() == 0) is_start_networking = true;
        (is_start_networking == true) ? printk("[WDTR SUCCESS] - ") : printk("[WDTR ERROR] - ");
        is_in_recovery = false;
    }

    printk("Boot #%u complete. Waiting for button press to start networking... unless WDTR\n", stats.boot_count++);

    return 0;

}


/*

chosen {
		zephyr,console = &uart0;
		zephyr,shell-uart = &uart0;
		zephyr,uart-mcumgr = &uart0;
		zephyr,bt-mon-uart = &uart0;
		zephyr,bt-c2h-uart = &uart0;
		zephyr,sram = &sram0;
		zephyr,flash = &flash0;
		zephyr,code-partition = &slot0_partition;
	};

	leds {
		compatible = "gpio-leds";

		led0: led_0 {
			gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;
			label = "Green LED 0";
		};

		led1: led_1 {
			gpios = <&gpio0 18 GPIO_ACTIVE_LOW>;
			label = "Green LED 1";
		};

		led2: led_2 {
			gpios = <&gpio0 19 GPIO_ACTIVE_LOW>;
			label = "Green LED 2";
		};

		led3: led_3 {
			gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;
			label = "Green LED 3";
		};
	};

	pwmleds {
		compatible = "pwm-leds";

		pwm_led0: pwm_led_0 {
			pwms = <&pwm0 0 PWM_MSEC(20) PWM_POLARITY_INVERTED>;
		};
	};

	buttons {
		compatible = "gpio-keys";
		wakeup-source;

		button0: button_0 {
			gpios = <&gpio0 13 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
			label = "Push button switch 0";
			zephyr,code = <INPUT_KEY_0>;
		};

		button1: button_1 {
			gpios = <&gpio0 14 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
			label = "Push button switch 1";
			zephyr,code = <INPUT_KEY_1>;
		};

		button2: button_2 {
			gpios = <&gpio0 15 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
			label = "Push button switch 2";
			zephyr,code = <INPUT_KEY_2>;
		};

		button3: button_3 {
			gpios = <&gpio0 16 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
			label = "Push button switch 3";
			zephyr,code = <INPUT_KEY_3>;
		};
	};

	gpio_trigger {
		compatible = "gpio-leds";
		trigger_pin: trigger_0 {
			gpios = <&gpio0 11 GPIO_ACTIVE_HIGH>;
			label = "Wakeup Trigger Pin"; //sbc wakeup
		};
		trigger_pin2: trigger_1 {
			gpios = <&gpio0 12 GPIO_ACTIVE_HIGH>;
			label = "Wakeup Trigger Pin 2"; //modem wakeup
		};

	};

	gpio_wake {
		compatible = "gpio-keys";
		wake_pin: wake_pin {
			gpios = <&gpio0 28 (GPIO_PULL_DOWN)>;
			label = "SBC Handoff signal"; //handoff from sbc
			zephyr,code = <INPUT_KEY_4>;
		};
	};


////////ARDUINO HEADER, ADC////////////


aliases {
		led0 = &led0;
		led1 = &led1;
		led2 = &led2;
		led3 = &led3;
		pwm-led0 = &pwm_led0;
		sw0 = &button0;
		sw1 = &button1;
		sw2 = &button2;
		sw3 = &button3;
		bootloader-led0 = &led0;
		mcuboot-button0 = &button0;
		mcuboot-led0 = &led0;
		watchdog0 = &wdt0;
		trigger0 = &trigger_pin;
		trigger1 = &trigger_pin2;
		wakepin = &wake_pin;
		modem = &sim7080;
	};

&reg {
	regulator-initial-mode = <NRF5X_REG_MODE_DCDC>;
};

&adc {
	status = "okay";
};

&uicr {
	gpio-as-nreset;
};

&nfct {
	status = "okay";
};

&gpiote {
	status = "okay";
};

&gpio0 {
	status = "okay";
};

//SIM7080 PART IS UNNECESSARY NOW, HANDLING IT RAW NOW

arduino_serial: &uart0 {
	status = "okay";
	compatible = "nordic,nrf-uarte";
	current-speed = <115200>;
	pinctrl-0 = <&uart0_default>;
	pinctrl-1 = <&uart0_sleep>;
	pinctrl-names = "default", "sleep";
	
	#address-cells = <1>;
	#size-cells = <0>;
	
	sim7080: modem@0 {
			compatible = "simcom,sim7080";
			reg = <0>;
			mdm-power-gpios = <&gpio0 2 GPIO_ACTIVE_LOW>; 
			status = "okay";
		};
};

arduino_i2c: &i2c0 {
	compatible = "nordic,nrf-twi";
	status = "okay";
	pinctrl-0 = <&i2c0_default>;
	pinctrl-1 = <&i2c0_sleep>;
	pinctrl-names = "default", "sleep";
};

&i2c1 {
	compatible = "nordic,nrf-twi";
	pinctrl-0 = <&i2c1_default>;
	pinctrl-1 = <&i2c1_sleep>;
	pinctrl-names = "default", "sleep";
};

&pwm0 {
	status = "okay";
	pinctrl-0 = <&pwm0_default>;
	pinctrl-1 = <&pwm0_sleep>;
	pinctrl-names = "default", "sleep";
};

&spi0 {
	compatible = "nordic,nrf-spi";
	pinctrl-0 = <&spi0_default>;
	pinctrl-1 = <&spi0_sleep>;
	pinctrl-names = "default", "sleep";
};

&spi1 {
	compatible = "nordic,nrf-spi";
	status = "okay";
	pinctrl-0 = <&spi1_default>;
	pinctrl-1 = <&spi1_sleep>;
	pinctrl-names = "default", "sleep";
};

arduino_spi: &spi2 {
	compatible = "nordic,nrf-spi";
	status = "okay";
	cs-gpios = <&arduino_header 16 GPIO_ACTIVE_LOW>; 
	pinctrl-0 = <&spi2_default>;
	pinctrl-1 = <&spi2_sleep>;
	pinctrl-names = "default", "sleep";
};

&flash0 {
	partitions {
		compatible = "fixed-partitions";
		#address-cells = <1>;
		#size-cells = <1>;

		boot_partition: partition@0 {
			label = "mcuboot";
			reg = <0x00000000 0xc000>;
		};

		slot0_partition: partition@c000 {
			label = "image-0";
			reg = <0x0000C000 0x37000>;
		};

		slot1_partition: partition@43000 {
			label = "image-1";
			reg = <0x00043000 0x37000>;
		};

		storage_partition: partition@7a000 {
			label = "storage";
			reg = <0x0007a000 0x00006000>;
		};
	};
};

&ccm {
	status = "disabled";
};

&ecb {
	status = "disabled";
};

&rng {
	status = "okay";
};

*/

/* 1.2-031125 RTT logs of general operation (reliable but slow):
05> Command data captured: NOCMD
05> Received command: NOCMD
05> Processing command: NOCMD
05> No commands waiting
05> >>> AT+SHREQ="/api/ack/device001/OK",1
05> <<< OK
05> <<< +SHREQ: "GET",200,2
05> HTTPS Status: 200, Size: 2
05> <<< NORMAL POWER DOWN
05> <<< +APP PDP: 0,DEACTIVE
05> Polling for commands...
05> >>> AT+SHCHEAD
05> *** Booting nRF Connect SDK v3.1.0-6c6e5b32496e ***
05> *** Using Zephyr OS v4.1.99-1612683d4010 ***
05> Watchdog started: [T: 30000ms]
05> WDT RESET DETECTEDIoT Controller Starting...
05> Button configured with interrupt
05> Button2 configured with interrupt
05> SBC handoff pin configured with falling-edge interrupt
05> PWRKEY pin configured
05> Watchdog reset detected
05> INITIATING WDT RECOVERY
05> >>> AT
05> Starting polling thread...
05> Response timeout
05> Modem not ready yet (attempt 1/2). Retrying...
05> >>> AT
05> Response timeout
05> Modem not ready yet (attempt 2/2). Retrying...
05> Modem unresponsive â trying PPP escape (+++)
05> Response timeout
05> PPP escape failed â toggling PWRKEY
05> <<< RDY
05> <<< +CFUN: 1
05> <<< +CPIN: READY
05> <<< SMS Ready
05> PWRKEY PULSED HIGH
05> >>> AT
05> <<< AT
05> <<< OK
05> Modem RESPONSIVE after PWRKEY TOGGLE
05> [WDTR SUCCESS] - Boot #0 complete. Waiting for button press to start networking... unless WDTR
05> Button triggered networking start!
05> Setting up network...
05> >>> AT+CMEE=2
05> <<< AT+CMEE=2
05> <<< OK
05> >>> AT
05> <<< AT
05> <<< OK
05> >>> ATE0
05> <<< ATE0
05> <<< OK
05> >>> AT+IPR=921600
05> <<< OK
05> >>> AT+GMR
05> <<< Revision:1951B16SIM7080
05> <<< OK
05> >>> AT+CPIN?
05> <<< +CPIN: READY
05> <<< OK
05> >>> AT+CGREG=1
05> <<< OK
05> >>> AT+CGREG?
05> <<< +CGREG: 1,1
05> <<< OK
05> >>> AT+CGDCONT=1,"IP","internet.telia.ee"
05> <<< OK
05> >>> AT+CNACT=0,1
05> <<< OK
05> <<< +APP PDP: 0,ACTIVE
05> Network setup complete
05> Setting up HTTPS session...
05> >>> AT+SHSSL=0
05> <<< OK
05> >>> AT+SHTRDT
05> <<< ERROR
05> >>> AT+SHCHEAD
05> <<< +CME ERROR: operation not allowed
05> >>> AT+SHCPARA
05> <<< +CME ERROR: operation not allowed
05> Setting up certificate for HTTPS...
05> >>> AT+CFSINIT
05> <<< OK
05> >>> AT+CFSDFILE=3,"server_ca.cer"
05> <<< OK
05> >>> AT+CFSWFILE=3,"server_ca.cer",0,1265,10000
05> <<< DOWNLOAD
05> <<< OK
05> >>> AT+CFSTERM
05> <<< OK
05> >>> AT+CSSLCFG="convert",2,"server_ca.cer"
05> <<< OK
05> >>> AT+CSSLCFG="ignorertctime",1,1
05> <<< OK
05> >>> AT+CSSLCFG="sslversion",1,"3"
05> <<< OK
05> >>> AT+CSSLCFG="sni",1,"seven080-mcu-backend.onrender.com"
05> <<< OK
05> >>> AT+SHSSL=1,"server_ca.cer"
05> <<< OK
05> >>> AT+SHCONF="URL","https://seven080-mcu-backend.onrender.com"
05> <<< OK
05> >>> AT+SHCONF="BODYLEN",1024
05> <<< OK
05> >>> AT+SHCONF="HEADERLEN",350
05> <<< OK
05> >>> AT+CDNSGIP="seven080-mcu-backend.onrender.com"
05> <<< OK
05> <<< +CDNSGIP: 1,"seven080-mcu-backend.onrender.com","216.24.57.7"
05> >>> AT+SHCONN
05> <<< OK
05> >>> AT+SHSTATE?
05> <<< +SHSTATE: 1
05> HTTPS Session State: Connected
05> <<< OK
05> HTTPS session setup complete
05> Polling for commands...
05> >>> AT+SHCHEAD
05> <<< OK
05> >>> AT+SHCPARA
05> <<< OK
05> >>> AT+SHAHEAD="User-Agent","nRF52-IoT-Controller"
05> <<< OK
05> >>> AT+SHAHEAD="Accept","/"
05> <<< OK
05> >>> AT+SHAHEAD="Cache-control","no-cache"
05> <<< OK
05> >>> AT+SHAHEAD="Connection","keep-alive"
05> <<< OK
05> >>> AT+SHREQ="/api/poll/device001",1
05> <<< OK
05> Waiting for +SHREQ response...
05> <<< +SHREQ: "GET",200,5
05> HTTPS Status: 200, Size: 5
05> Boutta READ packet n=0
05> >>> AT+SHREAD=0,5
05> <<< OK
05> <<< +SHREAD: 5
05> <<< NOCMD
05> Command data captured: NOCMD
05> Received command: NOCMD
05> Processing command: NOCMD
05> No commands waiting
*/