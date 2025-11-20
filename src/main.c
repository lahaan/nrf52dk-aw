/* 

    v1.0-FSM INITIAL VERSION (experimental)
    handoff works, recovery kinda kinda works, https works, wdt works but is kinda useless, sleep is kinda useless
        needs: superior recovery, more testing, stat tracking, mem retention, proper sleep when sbc active, proper pwrkey usage, psm/edrx funcs
    compared to 271025a (similar version): -400 lines of code, easier to understand

 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_power.h>
#include <hal/nrf_gpio.h>

// ---------------- CONFIGURATION ---------------- //

// UART
#define BAUD_RATE                   921600
#define RX_BUF_SIZE                 128     
#define RESPONSE_BUF_SIZE           1024
#define RX_TIMEOUT_DELAY            500

// TIMING & PINS
#define PWRKEY_HIGH_MS              1100
#define POLL_INTERVAL_SEC           10
#define WATCHDOG_TIMEOUT_MS         30000
#define MODEM_BOOT_DELAY_SEC        12      
#define MAX_POLL_FAILURES           3       // Reconnect after this many failed polls

// SERVER
#define SERVER_URL                  "seven080-mcu-backend.onrender.com"
#define SERVER_HOST                 "seven080-mcu-backend.onrender.com"
#define DEVICE_ID                   "device001"
#define CA_CERT_FILE                "server_ca.cer"

// ---------------- HARDWARE REFERENCES ---------------- //

const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec trigger_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger0), gpios, {0});
static const struct gpio_dt_spec sbc_handoff = GPIO_DT_SPEC_GET(DT_ALIAS(wakepin), gpios);
static const struct gpio_dt_spec pwrkey_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger1), gpios, {0});
static const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));

// ---------------- DATA STRUCTURES ---------------- //

typedef enum {
    STATE_INIT,         
    STATE_RECOVERY,     
    STATE_HARD_RESET,   
    STATE_NET_SETUP,    
    STATE_HTTPS_SETUP,  
    STATE_POLLING_ACTIVE,
    STATE_POLLING_IDLE,
    STATE_SBC_OWNED,
    STATE_SBC_RECLAIM,
    STATE_SYSTEM_OFF
} machine_state_t;

typedef struct {
    bool is_connected;
    bool is_session_active;
    int last_status_code;
    int last_data_size;
    bool is_shreq_received; 
} http_state_t;

typedef struct {
    bool is_certificate_setup;
    bool is_response_complete;
    size_t response_length;
} uart_state_t;

typedef struct {
    uint32_t boot_count;
    uint32_t reset_reason;
    uint32_t handoff_count;
    uint32_t https_fail_count; 
    uint32_t poll_fail_count; // Track consecutive poll failures
} device_stats_t;

// ---------------- GLOBAL STATE ---------------- //

static __attribute__((section(".noinit"))) http_state_t http_state;
static __attribute__((section(".noinit"))) uart_state_t uart_state;
static __attribute__((section(".noinit"))) device_stats_t stats;

static int wdt_channel_id;
static bool is_skip_wdt = false;

// BUFFERS
static uint8_t rx_ping[RX_BUF_SIZE];
static uint8_t rx_pong[RX_BUF_SIZE];
static char response_buffer[RESPONSE_BUF_SIZE];
static char command_data[256];
static bool is_command_received = false;

// FSM Globals
static machine_state_t current_state = STATE_INIT;
static volatile bool flag_sbc_request_active = false;
static struct gpio_callback sbc_handoff_cb_data;
static struct gpio_callback button_callback_data;
static struct gpio_callback button3_callback_data;

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

// ---------------- FORWARD DECLARATIONS ---------------- //

static void watchdog_feed(void);
static void watchdog_init(void);
static bool send_at_power_safe(const char *cmd, k_timeout_t timeout, int recharge_ms);
void execute_command(const char *command, int pin, const char *data);
void cleanup_http_session(void);
void send_raw(const char *command);

// ---------------- UART PROCESSING ---------------- //

static void parse_accumulated_response(void)
{
    if (strstr(response_buffer, "+SHREQ:")) {
        char *ptr = strstr(response_buffer, "+SHREQ:");
        char *comma1 = strchr(ptr, ',');
        if (comma1) {
            char *comma2 = strchr(comma1 + 1, ',');
            if (comma2) {
                http_state.last_status_code = atoi(comma1 + 1);
                http_state.last_data_size = atoi(comma2 + 1);
            }
        }
    }
    
    if (strstr(response_buffer, "+SHSTATE:")) {
        http_state.is_session_active = (strstr(response_buffer, "+SHSTATE: 1") != NULL);
    }

    if (strstr(response_buffer, "CMD:") && !is_command_received) {
        char *cmd_start = strstr(response_buffer, "CMD:");
        if (strlen(cmd_start) < sizeof(command_data)) {
            strncpy(command_data, cmd_start, sizeof(command_data) - 1);
            char *end = strstr(command_data, "\r");
            if (end) *end = '\0';
            is_command_received = true;
            printk("Command Data Captured: %s\n", command_data);
        }
    }
}

static void uart_event_callback(const struct device *dev, struct uart_event *event, void *user_data)
{
    switch (event->type) {
    case UART_RX_RDY: {
        const uint8_t *p = event->data.rx.buf + event->data.rx.offset;
        size_t len = event->data.rx.len;

        for (size_t i = 0; i < len; i++) {
            if (uart_state.response_length < RESPONSE_BUF_SIZE - 1) {
                response_buffer[uart_state.response_length++] = (char)p[i];
                response_buffer[uart_state.response_length] = '\0';
            }
        }

        if (strstr(response_buffer, "OK") || 
            strstr(response_buffer, "ERROR") || 
            strstr(response_buffer, "+CME ERROR")) {
            parse_accumulated_response();
            uart_state.is_response_complete = true;
        }
        else if (strstr(response_buffer, "+SHREQ:")) {
            parse_accumulated_response();
        }
        break;
    }

    case UART_RX_BUF_REQUEST:
        {
            uint8_t *next_buf = (event->data.rx_buf.buf == rx_ping) ? rx_pong : rx_ping;
            uart_rx_buf_rsp(uart0, next_buf, RX_BUF_SIZE);
        }
        break;

    case UART_RX_BUF_RELEASED:
    case UART_RX_STOPPED:
        break;

    case UART_RX_DISABLED:
        uart_rx_enable(uart0, rx_ping, RX_BUF_SIZE, RX_TIMEOUT_DELAY);
        break;

    default:
        break;
    }
}

// ---------------- AT COMMAND PRIMITIVES ---------------- //

void send_raw(const char *command) {
    printk(">>> %s\n", command);
    uart_state.response_length = 0;
    response_buffer[0] = '\0';
    uart_state.is_response_complete = false;
    
    for (int i = 0; command[i] != '\0'; i++) uart_poll_out(uart0, command[i]);
    uart_poll_out(uart0, '\r');
}

static bool send_at_power_safe(const char *cmd, k_timeout_t timeout, int recharge_ms) {
    if (flag_sbc_request_active) return false;
    if (recharge_ms > 0) k_msleep(recharge_ms);

    send_raw(cmd);

    int64_t start = k_uptime_get();
    int64_t last_print = start;

    while ((k_uptime_get() - start) < timeout.ticks) {
        if (flag_sbc_request_active) return false;

        if (uart_state.is_response_complete) {
             printk("RX: %s\n", response_buffer);
             if (strstr(response_buffer, "OK")) return true;
             if (strstr(response_buffer, "ERROR")) return false;
             return true; 
        }
        
        if (k_uptime_get() - last_print > 2000) {
            printk("."); 
            last_print = k_uptime_get();
        }
        
        watchdog_feed();
        k_msleep(10);
    }
    printk("\nTIMEOUT on %s\n", cmd);
    return false;
}

// ---------------- COMMAND LOGIC ---------------- //

void execute_command(const char *command, int pin, const char *data)
{
    printk("EXECUTING: %s PIN:%d DATA:%s\n", command, pin, data);
    
    if (strcmp(command, "LED_ON") == 0 && gpio_is_ready_dt(&led0)) {
        gpio_pin_set_dt(&led0, 1);
    }
    else if (strcmp(command, "LED_OFF") == 0 && gpio_is_ready_dt(&led0)) {
        gpio_pin_set_dt(&led0, 0);
    }
    else if (strcmp(command, "TOGGLE") == 0 && gpio_is_ready_dt(&led0)) {
        gpio_pin_toggle_dt(&led0);
    }
    else if (strcmp(command, "BOOT") == 0 && gpio_is_ready_dt(&trigger_pin)) {
        gpio_pin_set_dt(&trigger_pin, 0);
        k_sleep(K_MSEC(250));
        gpio_pin_set_dt(&trigger_pin, 1);
        flag_sbc_request_active = true;
    }
    else if (strcmp(command, "PULSE") == 0 && pin == 11) {
        gpio_pin_set_dt(&trigger_pin, 0);
        k_sleep(K_MSEC(500));
        gpio_pin_set_dt(&trigger_pin, 1);
    }
}

void process_command_string(const char *response)
{
    if (strstr(response, "NOCMD")) return;
    char command[32] = {0};
    int pin = 0;
    char data[64] = {0};
    if (sscanf(response, "CMD:%31[^,],PIN:%d,DATA:%63s", command, &pin, data) == 3) {
        execute_command(command, pin, data);
    }
}

// ---------------- FSM STATE FUNCTIONS ---------------- //

void cleanup_http_session(void) {
    send_at_power_safe("AT+SHDISC", K_SECONDS(2), 100);
    send_at_power_safe("AT+SHSSL=0", K_SECONDS(1), 100);
    http_state.is_session_active = false;
    http_state.is_connected = false;
}

machine_state_t run_init(void) {
    printk("--- STATE: INIT ---\n");
    printk("Reset Reason: 0x%08X\n", stats.reset_reason);
    stats.https_fail_count = 0; 

    if ((stats.reset_reason & NRF_POWER_RESETREAS_DOG_MASK) || 
        (stats.reset_reason & NRF_POWER_RESETREAS_SREQ_MASK)) {
        printk("Detected Crash/Soft Reset. Attempting Recovery...\n");
        return STATE_RECOVERY;
    }

    printk("Cold Boot. Assuming Modem needs Start.\n");
    return STATE_HARD_RESET; 
}

machine_state_t run_recovery(void) {
    printk("--- STATE: RECOVERY ---\n");
    
    printk("Sending +++...\n");
    k_sleep(K_MSEC(1100));
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(20));
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(20));
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(2000)); 

    if (send_at_power_safe("AT", K_SECONDS(1), 0)) {
        printk("Modem Recovered via Escape/AT.\n");
        return STATE_NET_SETUP;
    }

    return STATE_HARD_RESET;
}

machine_state_t run_hard_reset(void) {
    printk("--- STATE: HARD RESET ---\n");
    if (!gpio_is_ready_dt(&pwrkey_pin)) return STATE_HARD_RESET;

    stats.https_fail_count = 0; 
    stats.poll_fail_count = 0;

    for (int i=0; i<3; i++) {
        if (send_at_power_safe("AT", K_MSEC(500), 100)) {
            printk("Modem is ALIVE, skipping PWRKEY toggle.\n");
            return STATE_NET_SETUP;
        }
    }

    printk("Modem Unresponsive - Toggling PWRKEY (Attempt 1)...\n");

    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_HIGH_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    
    printk("Waiting %ds for boot...\n", MODEM_BOOT_DELAY_SEC);
    for(int i=0; i<MODEM_BOOT_DELAY_SEC; i++) {
        k_sleep(K_SECONDS(1));
        watchdog_feed();
        if (i > 4) send_raw("AT"); 
    }

    if (send_at_power_safe("AT", K_SECONDS(2), 0)) {
        return STATE_NET_SETUP;
    }

    printk("Still dead. Toggling PWRKEY (Attempt 2)...\n");
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_HIGH_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    
    printk("Waiting %ds for boot...\n", MODEM_BOOT_DELAY_SEC);
    for(int i=0; i<MODEM_BOOT_DELAY_SEC; i++) {
        k_sleep(K_SECONDS(1));
        watchdog_feed();
        if (i > 4) send_raw("AT");
    }

    if (send_at_power_safe("AT", K_SECONDS(2), 0)) {
        return STATE_NET_SETUP;
    }

    printk("CRITICAL: Modem dead after 2 toggles. Triggering System Reset via WDT.\n");
    while(1) {
        k_sleep(K_SECONDS(1));
    }
    return STATE_HARD_RESET;
}

machine_state_t run_net_setup(void) {
    printk("--- STATE: NET SETUP ---\n");

    send_at_power_safe("ATE0", K_SECONDS(1), 0);
    send_at_power_safe("AT+CMEE=2", K_SECONDS(1), 0);
    send_at_power_safe("AT+CGREG=1", K_SECONDS(1), 0);
    
    send_at_power_safe("AT+CGREG?", K_SECONDS(1), 0);
    if (strstr(response_buffer, "1,1") || strstr(response_buffer, "1,5")) {
        return STATE_HTTPS_SETUP;
    }

    if (!send_at_power_safe("AT+CFUN=1", K_SECONDS(10), 1000)) return STATE_HARD_RESET;
    if (!send_at_power_safe("AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"", K_SECONDS(2), 500)) return STATE_HARD_RESET;
    
    send_at_power_safe("AT+CNACT?", K_SECONDS(1), 0);
    if (!strstr(response_buffer, "0,1")) {
        send_at_power_safe("AT+CNACT=0,1", K_SECONDS(15), 1000);
    }

    for(int i=0; i<40; i++) {
        if (flag_sbc_request_active) return STATE_SBC_OWNED;
        
        send_at_power_safe("AT+CGREG?", K_SECONDS(1), 0);
        if (strstr(response_buffer, "1,1") || strstr(response_buffer, "1,5")) {
            http_state.is_connected = true;
            return STATE_HTTPS_SETUP;
        }
        k_sleep(K_SECONDS(2));
    }

    return STATE_HARD_RESET;
}

machine_state_t run_https_setup(void) {
    printk("--- STATE: HTTPS SETUP ---\n");
    
    send_at_power_safe("AT+SHDISC", K_SECONDS(2), 0);
    
    send_at_power_safe("AT+CNACT?", K_SECONDS(1), 0);
    if (!strstr(response_buffer, "0,1")) {
         send_at_power_safe("AT+CNACT=0,1", K_SECONDS(5), 500);
    }

    send_at_power_safe("AT+CSSLCFG=\"ignorertctime\",1,1", K_SECONDS(1), 0);
    send_at_power_safe("AT+CSSLCFG=\"sslversion\",1,3", K_SECONDS(1), 0);
    char sni[128]; snprintf(sni, sizeof(sni), "AT+CSSLCFG=\"sni\",1,\"%s\"", SERVER_HOST);
    send_at_power_safe(sni, K_SECONDS(1), 0);

    if (!uart_state.is_certificate_setup) {
        send_at_power_safe("AT+CFSINIT", K_SECONDS(1), 0);
        char cmd[64]; snprintf(cmd, sizeof(cmd), "AT+CFSWFILE=3,\"%s\",0,%d,10000", CA_CERT_FILE, (int)ca_certificate_length);
        send_raw(cmd);
        k_sleep(K_MSEC(500));
        
        for(size_t i=0; i<ca_certificate_length; i++) {
            uart_poll_out(uart0, ca_certificate[i]);
            if (i % 50 == 0) watchdog_feed();
        }
        
        k_sleep(K_SECONDS(1));
        send_at_power_safe("AT+CFSTERM", K_SECONDS(1), 0);
        snprintf(cmd, sizeof(cmd), "AT+CSSLCFG=\"convert\",2,\"%s\"", CA_CERT_FILE);
        send_at_power_safe(cmd, K_SECONDS(5), 1000);
        uart_state.is_certificate_setup = true;
    }

    char ssl[64]; snprintf(ssl, sizeof(ssl), "AT+SHSSL=1,\"%s\"", CA_CERT_FILE);
    send_at_power_safe(ssl, K_SECONDS(1), 0);
    char url[128]; snprintf(url, sizeof(url), "AT+SHCONF=\"URL\",\"https://%s\"", SERVER_URL);
    send_at_power_safe(url, K_SECONDS(1), 0);
    send_at_power_safe("AT+SHCONF=\"BODYLEN\",1024", K_SECONDS(1), 0);
    send_at_power_safe("AT+SHCONF=\"HEADERLEN\",350", K_SECONDS(1), 0);

    if (send_at_power_safe("AT+SHCONN", K_SECONDS(60), 2000)) {
        http_state.is_session_active = true;
        stats.https_fail_count = 0;
        stats.poll_fail_count = 0;
        return STATE_POLLING_ACTIVE;
    }

    stats.https_fail_count++;
    printk("HTTPS Setup Failed (%d/3)\n", stats.https_fail_count);
    
    if (stats.https_fail_count >= 3) {
        printk("Too many HTTPS failures. Forcing Hard Reset.\n");
        return STATE_HARD_RESET;
    }

    return STATE_NET_SETUP;
}

machine_state_t run_polling_active(void) {
    printk("--- STATE: POLLING ---\n");

    if (!http_state.is_session_active) return STATE_HTTPS_SETUP;

    http_state.last_status_code = 0;
    http_state.last_data_size = 0;

    send_at_power_safe("AT+SHCHEAD", K_SECONDS(1), 0);
    send_at_power_safe("AT+SHAHEAD=\"User-Agent\",\"nRF52-IoT\"", K_SECONDS(1), 0);
    send_at_power_safe("AT+SHAHEAD=\"Connection\",\"keep-alive\"", K_SECONDS(1), 0);

    char get_cmd[128]; snprintf(get_cmd, sizeof(get_cmd), "AT+SHREQ=\"/api/poll/%s\",1", DEVICE_ID);
    
    if (!send_at_power_safe(get_cmd, K_SECONDS(30), 500)) {
        stats.poll_fail_count++;
        printk("Poll CMD Timeout (%d/%d)\n", stats.poll_fail_count, MAX_POLL_FAILURES);
        
        if (stats.poll_fail_count >= MAX_POLL_FAILURES) {
            printk("Polling died. Rebuilding HTTPS.\n");
            return STATE_HTTPS_SETUP;
        }
        return STATE_POLLING_IDLE;
    }

    int64_t start = k_uptime_get();
    while (!http_state.last_status_code && (k_uptime_get() - start) < K_SECONDS(15).ticks) {
        if (flag_sbc_request_active) return STATE_SBC_OWNED;
        watchdog_feed();
        k_msleep(50);
    }

    if (http_state.last_status_code == 200 && http_state.last_data_size > 0) {
        // Successful Poll - Reset Fail Counter
        stats.poll_fail_count = 0;
        
        is_command_received = false;
        char read_cmd[64]; snprintf(read_cmd, sizeof(read_cmd), "AT+SHREAD=0,%d", http_state.last_data_size);
        send_at_power_safe(read_cmd, K_SECONDS(5), 0);

        if (is_command_received) {
            process_command_string(command_data);
            http_state.last_status_code = 0;
            
            snprintf(get_cmd, sizeof(get_cmd), "AT+SHREQ=\"/api/ack/%s/OK\",1", DEVICE_ID);
            send_at_power_safe(get_cmd, K_SECONDS(10), 1000);
            
            // Synchronize ACK URC
            int64_t ack_start = k_uptime_get();
            while (!http_state.last_status_code && (k_uptime_get() - ack_start) < K_SECONDS(5).ticks) {
                 watchdog_feed();
                 k_msleep(50);
            }
            http_state.last_status_code = 0;
        }
    }
    // Handle non-200 responses
    else if (http_state.last_status_code != 0 && http_state.last_status_code != 200) {
         stats.poll_fail_count++;
         printk("HTTP Error %d. Fail Count: %d\n", http_state.last_status_code, stats.poll_fail_count);
         if (stats.poll_fail_count >= MAX_POLL_FAILURES) return STATE_HTTPS_SETUP;
    }

    http_state.last_status_code = 0;
    return STATE_POLLING_IDLE;
}

machine_state_t run_sbc_reclaim(void) {
    printk("--- STATE: SBC RECLAIM ---\n");
    stats.handoff_count++;

    k_sleep(K_SECONDS(2));
    
    printk("Escaping PPP...\n");
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(20));
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(20));
    uart_poll_out(uart0, '+');
    k_sleep(K_SECONDS(2));

    send_at_power_safe("ATH", K_SECONDS(2), 0);

    if (send_at_power_safe("AT", K_SECONDS(2), 0)) {
        return STATE_NET_SETUP;
    }

    return STATE_HARD_RESET;
}

// ---------------- INTERRUPTS ---------------- //

static void sbc_handoff_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    // Simply flag that the pin is High. 
    // We do not clear it here; State logic handles clearing.
    if (gpio_pin_get_dt(&sbc_handoff) == 1) {
        flag_sbc_request_active = true;
    }
}

static void button_pressed_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    printk("Button pressed\n");
}

static void button3_pressed_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    is_skip_wdt = true;
    printk("WDT Disable Requested\n");
}

// ---------------- WATCHDOG ---------------- //

static void watchdog_init(void) {
    if (is_skip_wdt) return;
    struct wdt_timeout_cfg wdt_config = {
        .window.min = 0U,
        .window.max = WATCHDOG_TIMEOUT_MS,
        .flags = WDT_FLAG_RESET_SOC,
        .callback = NULL
    };
    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    wdt_setup(wdt, 0);
}

static void watchdog_feed(void) {
    if (!is_skip_wdt && wdt && device_is_ready(wdt)) wdt_feed(wdt, wdt_channel_id);
}

// ---------------- MAIN ---------------- //

int main(void)
{
    printk("System Booting... FSM v3.1 (Polled SBC Detect)\n");

    // Init Hardware
    if (gpio_is_ready_dt(&led0)) gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    if (gpio_is_ready_dt(&trigger_pin)) gpio_pin_configure_dt(&trigger_pin, GPIO_OUTPUT_INACTIVE);
    if (gpio_is_ready_dt(&pwrkey_pin)) gpio_pin_configure_dt(&pwrkey_pin, GPIO_OUTPUT_INACTIVE);
    
    if (gpio_is_ready_dt(&button)) {
        gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&button_callback_data, button_pressed_callback, BIT(button.pin));
        gpio_add_callback(button.port, &button_callback_data);
    }
    if (gpio_is_ready_dt(&button3)) {
        gpio_pin_configure_dt(&button3, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&button3_callback_data, button3_pressed_callback, BIT(button3.pin));
        gpio_add_callback(button3.port, &button3_callback_data);
    }

    if (gpio_is_ready_dt(&sbc_handoff)) {
        gpio_pin_configure_dt(&sbc_handoff, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&sbc_handoff, GPIO_INT_EDGE_BOTH);
        gpio_init_callback(&sbc_handoff_cb_data, sbc_handoff_callback, BIT(sbc_handoff.pin));
        gpio_add_callback(sbc_handoff.port, &sbc_handoff_cb_data);
        
        // Initial check
        if (gpio_pin_get_dt(&sbc_handoff) == 1) flag_sbc_request_active = true;
    }

    // Init UART
    if (!device_is_ready(uart0)) return 0;
    uart_callback_set(uart0, uart_event_callback, NULL);
    uart_rx_enable(uart0, rx_ping, RX_BUF_SIZE, RX_TIMEOUT_DELAY);

    // Read and Clear Reset Reason
    stats.reset_reason = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFF;
    
    if ((stats.reset_reason & NRF_POWER_RESETREAS_OFF_MASK) == 0) {
        memset(&http_state, 0, sizeof(http_state));
        memset(&uart_state, 0, sizeof(uart_state));
    }

    watchdog_init();

    // Start in INIT state to decide logic
    current_state = STATE_INIT;

    while(1) {
        
        // GLOBAL SBC CHECK
        if (flag_sbc_request_active && current_state != STATE_SBC_OWNED) {
            printk("!!! SBC INTERRUPT !!! Switching to SBC_OWNED\n");
            cleanup_http_session(); 
            current_state = STATE_SBC_OWNED;
        }

        watchdog_feed();

        switch (current_state) {
            case STATE_INIT:
                current_state = run_init();
                break;
            case STATE_RECOVERY:
                current_state = run_recovery();
                break;
            case STATE_HARD_RESET:
                current_state = run_hard_reset();
                break;
            case STATE_NET_SETUP:
                current_state = run_net_setup();
                break;
            case STATE_HTTPS_SETUP:
                current_state = run_https_setup();
                break;
            case STATE_POLLING_ACTIVE:
                current_state = run_polling_active();
                break;
            case STATE_POLLING_IDLE:
                for (int i = 0; i < POLL_INTERVAL_SEC * 10; i++) {
                    if (flag_sbc_request_active) break;
                    
                    // FALLBACK: Polled check in case Interrupt missed
                    if (gpio_pin_get_dt(&sbc_handoff) == 1) {
                        flag_sbc_request_active = true;
                        break;
                    }

                    k_msleep(100);
                    watchdog_feed();
                }
                if (!flag_sbc_request_active) current_state = STATE_POLLING_ACTIVE;
                break;
            case STATE_SBC_OWNED:
                // Exit only if pin goes LOW
                if (gpio_pin_get_dt(&sbc_handoff) == 0) {
                    // Debounce exit slightly
                    k_msleep(100);
                    if (gpio_pin_get_dt(&sbc_handoff) == 0) {
                        flag_sbc_request_active = false;
                        current_state = STATE_SBC_RECLAIM;
                    }
                }
                k_msleep(100);
                break;
            case STATE_SBC_RECLAIM:
                current_state = run_sbc_reclaim();
                break;
            default:
                current_state = STATE_HARD_RESET;
                break;
        }
    }
    return 0;
}
