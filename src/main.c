/*

v1.1-241025
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
     v1.1-241025a - additional handoff logic - tested working (no button logic yet) MCU-SBC-MCU handoff cycle OK via systemctl suspend/HTTPS wake

    .dts added below (bottom) as git doesn't track it
*/



#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>

// UART & GPIO
const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0)); //P0.06=TX, P0.08=RX
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

// Control pins
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios); //oboard LED1
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); //oboard LED2
static const struct gpio_dt_spec trigger_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger0), gpios, {0}); //P0.11 to SBC (defined in dts)

static const struct gpio_dt_spec sbc_handoff = GPIO_DT_SPEC_GET(DT_ALIAS(wakepin), gpios); //MCU P0.28 << P32 SBC [handoff signal]

static struct k_work sbc_handoff_work;
static struct gpio_callback button_cb_data;
static bool start_networking = false;
static bool expecting_http_body = false;
static bool shreq_response_received = false;

// Server config
#define SERVER_URL "seven080-mcu-backend.onrender.com"
#define SERVER_HOST "seven080-mcu-backend.onrender.com"
#define DEVICE_ID "device001"
#define POLL_INTERVAL_SEC 10

// certs
#define CA_CERT_FILE "server_ca.cer"
#define CA_CERT_SIZE 2048

// UART buffers
#define RX_BUF_SIZE 128 // MAX buffer that the 52832 can handle over UART without crashing tf out
#define RESPONSE_BUF_SIZE 1024  // Increased for HTTPS responses
static int32_t RX_TIMEOUT_DELAY = 500;
static uint8_t rx_buf[RX_BUF_SIZE];
static char response_buf[RESPONSE_BUF_SIZE];

static int BAUD_RATE = 921600;

static size_t resp_len = 0;
static bool response_complete = false;
static bool waiting_for_response = false;
static bool is_cert_setup = false;
static bool sbc_is_active = false;
static int last_http_status_code = 0;
static int last_http_data_size = 0;
static int x = 0;

// Command buffer
static char command_data[256];
static bool command_received = false;

// Network state
static bool network_connected = false;
static bool http_session_active = false;

// Forward declarations
void send_at_command(const char *cmd);
bool wait_for_response_with_timeout(const char *expected, k_timeout_t timeout);
bool wait_for_ok_error(k_timeout_t timeout);
bool setup_network(void);
bool download_and_convert_certificate(void);
bool setup_https_session(void);
void cleanup_http_session(void);
void poll_for_commands(void);
void process_command(const char *response);
void execute_command(const char *cmd, int pin, const char *data);

// Google GTS4 CA certificate from openssl [PEM]
const char ca_cert[] =
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

const size_t ca_cert_len = sizeof(ca_cert) - 1;

//efficiency states
static bool psm_state = false;
static bool edrx_state = false;

static const char* edrx_seconds_to_code(uint32_t seconds) {
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

bool end_ppp_session(void) {
    printk("Ending PPP session...\n");

    k_sleep(K_MSEC(1100)); // +100ms buffer to be safe (1 second minimum)
    uart_poll_out(uart0, '+');
    uart_poll_out(uart0, '+');
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(1100));

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

    network_connected = true;
    http_session_active = false;
    return true;
}

static void sbc_handoff_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    printk("sbc_handoff_work_handler: handling SBC handoff in thread context\n");

    sbc_is_active = false;
    if (end_ppp_session()) {
        printk("Resuming MCU control (from work handler)...\n");
        is_cert_setup = true;   

        if (!setup_https_session()) {
            printk("HTTPS setup failed after SBC handoff!\n");
        }
    } else {
        printk("end_ppp_session() FAILED in work handler\n");
        sbc_is_active = false;
    }
}

bool download_and_convert_certificate(void) {
    /*
    AT+CFSINIT - Get flash data buffer (initialize)
    AT+CFSDFILE=3,"server_ca.cer" - Define file in flash
    AT+CFSWFILE=3,"server_ca.cer",0,2048,10000 - Write file (2048 bytes max here)
    AT+CFSTERM - Free the flash buffer allocated by CFSINIT
    AT+CSSLCFG="convert",2,"server_ca.cer" - Convert the PEM to internal format (Configure SSL params of a context identifier)
    */

    printk("Setting up certificate for HTTPS...\n");

    send_at_command("AT+CFSINIT"); // Get flash data buffer (initialize)
    k_sleep(K_SECONDS(1));
    
    char cmd_buf[100];
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CFSDFILE=3,\"%s\"", CA_CERT_FILE); 
    send_at_command(cmd_buf);
    k_sleep(K_SECONDS(1));

    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CFSWFILE=3,\"%s\",0,%d,10000", 
             CA_CERT_FILE, ca_cert_len);
    printk(">>> %s\n", cmd_buf);
    for (int i = 0; cmd_buf[i] != '\0'; i++) {
        uart_poll_out(uart0, cmd_buf[i]);
    }
    uart_poll_out(uart0, '\r');
    
    k_sleep(K_SECONDS(2));
    
    printk("Sending certificate data...\n");
    for (size_t i = 0; i < ca_cert_len; i++) {
        uart_poll_out(uart0, ca_cert[i]);
    }
    
    k_sleep(K_SECONDS(3)); 

    send_at_command("AT+CFSTERM");
    k_sleep(K_SECONDS(1));

    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CSSLCFG=\"convert\",2,\"%s\"", CA_CERT_FILE);
    send_at_command(cmd_buf);
    if (!wait_for_ok_error(K_SECONDS(10))) {
        return false;
    }

    is_cert_setup = true;

    return true;
}


void button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed! Starting communication test...\n");
    start_networking = true;
}

static uint8_t encode_psm_timer(uint32_t seconds){
    //PSM timer encoding according to 3GPP TS (T3324/T3412)
    if (seconds <= 2) return seconds;
    if (seconds <= 120) {
        if (seconds % 6 == 0) return (seconds / 6) + 2;
    }
    if (seconds <= 1800) {
        if (seconds % 60 == 0) return (seconds / 60) + 22;
    }
    if (seconds <= 10800) {
        if (seconds % 360 == 0) return (seconds / 360) + 52;
    }
    if (seconds <= 31536000) {
        if (seconds % 86400 == 0) return (seconds / 86400) + 82;
    }
    return 0;
}

bool psm_enable(uint32_t seconds_active, uint32_t seconds_periodic_tau) {
    uint8_t t3324 = encode_psm_timer(seconds_active);
    uint8_t t3412 = encode_psm_timer(seconds_periodic_tau);

    if (t3324 == 0 || t3412 == 0) {
        printk("PSM: Invalid timer values\n");
        return false;
    }

    char cmd[64];
    // Format: "00000101"..whatever (8-bit as binary string)
    snprintf(cmd, sizeof(cmd), "AT+CPSMS=1,,,\"%08b\",\"%08b\"", t3412, t3324);

    send_at_command(cmd);

    if (!wait_for_ok_error(K_SECONDS(5))) {
        printk("Failed to enable PSM\n");
        return false;
    }

    psm_state = true;
    return true;
}

bool edrx_enable(uint32_t seconds_cycle) {
    const char* edrx_code = edrx_seconds_to_code(seconds_cycle);
    char cmd[40];

    //4 for CAT-M
    snprintf(cmd, sizeof(cmd), "AT+CEDRXS=1,4,\"%s\"", edrx_code);

    send_at_command(cmd);
    if (!wait_for_ok_error(K_SECONDS(5))) {
        printk("Failed to enable eDRX\n");
        return false;
    }

    edrx_state = true;
    return true;
}

bool psm_disable(void) {
    send_at_command("AT+CPSMS=0");
    if (!wait_for_ok_error(K_SECONDS(5))) {
        printk("Failed to disable PSM\n");
        return false;
    }
    psm_state = false;
    return true;
}

bool edrx_disable(void) {
    send_at_command("AT+CEDRXS=0,4");
    if (!wait_for_ok_error(K_SECONDS(5))) {
        printk("Failed to disable eDRX\n");
        return false;
    }
    edrx_state = false;
    return true;
}

/*
    usage:
    psm_enable(30, 3600); - enable PSM w 30s active 1h TAU
    edrx_enable(60); 60s cycle 

*/


static void sbc_handoff_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* quick log and schedule work; do NOT call blocking apis here */
    printk("SBC handoff IRQ -> scheduling sbc_handoff_work\n");
    k_work_submit(&sbc_handoff_work);
}

static void uart_evt_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    switch (evt->type) {
    case UART_RX_RDY: {
        const uint8_t *p = &evt->data.rx.buf[evt->data.rx.offset];
        size_t l = evt->data.rx.len;

        for (size_t i = 0; i < l; i++) {
            char c = p[i];

            if (c == '\n') {
                if (resp_len > 0) {
                    response_buf[resp_len] = '\0';
                    
                    // Trim trailing CR if present
                    if (resp_len > 0 && response_buf[resp_len-1] == '\r') {
                        response_buf[resp_len-1] = '\0';
                    }
                    
                    printk("<<< %s\n", response_buf);

                    // Check for standard responses
                    if (strcmp(response_buf, "OK") == 0) {
                        response_complete = true;
                        waiting_for_response = false;
                    } else if (strcmp(response_buf, "ERROR") == 0 ||
                               strstr(response_buf, "+CME ERROR:") != NULL) {
                        response_complete = true;
                        waiting_for_response = false;
                    }
                    // Parse HTTP response
                    else if (strstr(response_buf, "+SHREQ:") != NULL) {
                        char *comma1 = strchr(response_buf, ',');
                        if (comma1) {
                            char *comma2 = strchr(comma1 + 1, ',');
                            if (comma2) {
                                last_http_status_code = atoi(comma1 + 1);
                                last_http_data_size = atoi(comma2 + 1);
                                printk("HTTPS Status: %d, Size: %d\n", 
                                    last_http_status_code, last_http_data_size);
                                shreq_response_received = true;  // ADD THIS LINE
                            }
                        }
                        response_complete = true;
                    }
                    // Parse HTTP state
                    else if (strstr(response_buf, "+SHSTATE:") != NULL) {
                        http_session_active = (strstr(response_buf, "+SHSTATE: 1") != NULL);
                        printk("HTTPS Session State: %s\n", http_session_active ? "Connected" : "Disconnected");
                        response_complete = true;
                    }
                    // Capture HTTP data (starts with +SHREAD:)
                    else if (strstr(response_buf, "+SHREAD:") != NULL) {
                        // The next line will contain the actual data
                    }
                    // Capture actual data after +SHREAD
                    else if (expecting_http_body && resp_len > 0) {
                    if (strcmp(response_buf, "OK") != 0 &&
                        strcmp(response_buf, "ERROR") != 0 &&
                        strncmp(response_buf, "+", 1) != 0) {

                        if (resp_len < sizeof(command_data)) {
                            strncpy(command_data, response_buf, sizeof(command_data) - 1);
                            command_data[sizeof(command_data) - 1] = '\0';
                            command_received = true;
                            printk("Command data captured: %s\n", command_data);
                        }
                    }
                }
                    resp_len = 0;
                }
            } else if (c != '\r') {  // Ignore carriage returns
                if (resp_len < sizeof(response_buf) - 1) {
                    response_buf[resp_len++] = c;
                } else {
                    // Buffer overflow, reset
                    resp_len = 0;
                }
            }
        }
        break;
    }
    case UART_RX_DISABLED:
    case UART_RX_STOPPED:
        uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), RX_TIMEOUT_DELAY);
        break;
    default:
        break;
    }
}

void send_at_command(const char *cmd) {
    printk(">>> %s\n", cmd);
    for (int i = 0; cmd[i] != '\0'; i++) {
        uart_poll_out(uart0, cmd[i]);
    }
    uart_poll_out(uart0, '\r');
}

bool wait_for_ok_error(k_timeout_t timeout) {
    response_complete = false;
    waiting_for_response = true;

    int64_t start = k_uptime_get();
    while (!response_complete && (k_uptime_get() - start) < timeout.ticks) {
        k_msleep(50);
    }

    if (!response_complete) {
        printk("Response timeout\n");
        waiting_for_response = false;
        return false;
    }

    waiting_for_response = false;
    return true;
}

bool wait_for_response_with_timeout(const char *expected, k_timeout_t timeout) {
    resp_len = 0;
    response_complete = false;
    waiting_for_response = true;

    int64_t start = k_uptime_get();
    while (!response_complete && (k_uptime_get() - start) < timeout.ticks) {
        if (resp_len > 0 && strstr(response_buf, expected) != NULL) {
            response_complete = true;
            break;
        }
        k_msleep(50);
    }

    waiting_for_response = false;
    return response_complete;
}

bool setup_network(void) {
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

    printk("Setting up network...\n");
    
    send_at_command("AT+CMEE=2");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    send_at_command("AT");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    send_at_command("ATE0");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    char ipr_cmd[32];
    snprintf(ipr_cmd, sizeof(ipr_cmd), "AT+IPR=%d", BAUD_RATE);
    send_at_command(ipr_cmd);
    if (!wait_for_ok_error(K_SECONDS(2))) return false;

    send_at_command("AT+GMR");
    k_sleep(K_SECONDS(2));

    send_at_command("AT+CPIN?");
    if (!wait_for_response_with_timeout("READY", K_SECONDS(2))) return false;
    
    send_at_command("AT+CGREG=1");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    bool registered = false;
    for (int i = 0; i < 15; i++) {
        send_at_command("AT+CGREG?");
        if (wait_for_response_with_timeout("+CGREG: 1,1", K_SECONDS(2)) ||
            wait_for_response_with_timeout("+CGREG: 1,5", K_SECONDS(2))) {
            printk("Network registered!\n");
            registered = true;
            break;
        }
        k_sleep(K_SECONDS(2));
    }
    
    if (!registered) {
        printk("Network registration failed\n");
        return false;
    }
    
    send_at_command("AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"");
    if (!wait_for_ok_error(K_SECONDS(3))) return false;
    
    send_at_command("AT+CNACT=0,1");
    if (!wait_for_ok_error(K_SECONDS(15))) {
        send_at_command("AT+CNACT?");
        if (!wait_for_response_with_timeout("+CNACT: 0,1", K_SECONDS(2))) {
            return false;
        }
    }

    send_at_command("AT+CNACT?");
    if (!wait_for_response_with_timeout("+CNACT: 0,1", K_SECONDS(2))) {
        return false;
    }
    
    network_connected = true;
    printk("Network setup complete\n");
    return true;
}

bool setup_https_session(void) {
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

    if (http_session_active) {
        printk("HTTPS session already active\n");
        return true;
    }

    printk("Setting up HTTPS session...\n");
    // Clean session
    cleanup_http_session();

    // Setup certificate
    if (!is_cert_setup) {
        send_at_command("AT+CSSLCFG=\"ignorertctime\",1,1");
        k_sleep(K_MSEC(500));
        if (!download_and_convert_certificate()) {
            printk("FATAL: Certificate setup failed.\n");
            return false;
        }
        is_cert_setup = true;
    } else {
        printk("Certificate already installed, skipping cert download\n");
    }

    // Configure SSL - Using TLS 1.2
    send_at_command("AT+CSSLCFG=\"sslversion\",1,3");
    if (!wait_for_ok_error(K_SECONDS(3))) {
        printk("SSL version configuration failed\n");
        return false;
    }

    send_at_command("AT+CSSLCFG=\"sni\",1,\"seven080-mcu-backend.onrender.com\"");
    k_sleep(K_MSEC(500));

    char ssl_cmd[100];
    snprintf(ssl_cmd, sizeof(ssl_cmd), "AT+SHSSL=1,\"%s\"", CA_CERT_FILE);
    send_at_command(ssl_cmd);
    if (!wait_for_ok_error(K_SECONDS(3))) {
        printk("SSL configuration failed\n");
        return false;
    }
    
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+SHCONF=\"URL\",\"https://%s\"", SERVER_URL);
    send_at_command(cmd);
    if (!wait_for_ok_error(K_SECONDS(3))) {
        printk("URL configuration failed\n");
        return false;
    }
    
    send_at_command("AT+SHCONF=\"BODYLEN\",1024");
    if (!wait_for_ok_error(K_SECONDS(3))) {
        printk("BODYLEN configuration failed\n");
        return false;
    }
    
    send_at_command("AT+SHCONF=\"HEADERLEN\",350");
    if (!wait_for_ok_error(K_SECONDS(3))) {
        printk("HEADERLEN configuration failed\n");
        return false;
    }

    printk("Testing DNS resolution...\n");
    send_at_command("AT+CDNSGIP=\"seven080-mcu-backend.onrender.com\"");
    if (wait_for_ok_error(K_SECONDS(10))) {
        printk("DNS resolution successful\n");
    } else {
        printk("DNS resolution failed\n");
    }
    k_sleep(K_SECONDS(2));

    // CONNECTION
    send_at_command("AT+SHCONN");
    if (!wait_for_ok_error(K_SECONDS(30))) {
        printk("HTTPS connection failed\n");
        return false;
    }

    k_sleep(K_SECONDS(2));

    send_at_command("AT+SHSTATE?");
    if (!wait_for_response_with_timeout("+SHSTATE: 1", K_SECONDS(5))) {
        printk("HTTPS session not active\n");
        return false;
    }
    
    http_session_active = true;
    printk("HTTPS session setup complete\n");
    return true;
}

void cleanup_http_session(void) {
    if (http_session_active) {
        printk("Cleaning up HTTPS session...\n");
        send_at_command("AT+SHDISC");
        wait_for_ok_error(K_SECONDS(5));
        http_session_active = false;
    }
}

void poll_for_commands(void) {
    if (sbc_is_active) {
        printk("ERR: SBC ACTIVE - SKIP POLLING\n");
        return;
    }

    if (!http_session_active) {
        if (!setup_https_session()) {
            printk("Failed to setup HTTPS session\n");
            return;
        }
    }
    
    printk("Polling for commands...\n");
    
    send_at_command("AT+SHCHEAD");
    if (!wait_for_ok_error(K_SECONDS(2))) {
        printk("Clear headers failed\n");
        cleanup_http_session();
        return;
    }
    
    send_at_command("AT+SHCPARA");
    if (!wait_for_ok_error(K_SECONDS(2))) {
        printk("Clear parameters failed\n");
        cleanup_http_session();
        return;
    }
    
    send_at_command("AT+SHAHEAD=\"User-Agent\",\"nRF52-IoT-Controller\"");
    if (!wait_for_ok_error(K_SECONDS(2))) {
        printk("User-Agent header failed\n");
        cleanup_http_session();
        return;
    }
    
    send_at_command("AT+SHAHEAD=\"Accept\",\"*/*\"");
    if (!wait_for_ok_error(K_SECONDS(2))) {
        printk("Accept header failed\n");
        cleanup_http_session();
        return;
    }
    
    send_at_command("AT+SHAHEAD=\"Cache-control\",\"no-cache\"");
    if (!wait_for_ok_error(K_SECONDS(2))) {
        printk("Cache-control header failed\n");
        cleanup_http_session();
        return;
    }
    
    send_at_command("AT+SHAHEAD=\"Connection\",\"keep-alive\"");
    if (!wait_for_ok_error(K_SECONDS(2))) {
        printk("Connection header failed\n");
        cleanup_http_session();
        return;
    }
    
    last_http_status_code = 0;
    last_http_data_size = 0;
    shreq_response_received = false;
    
    char poll_url[128];
    snprintf(poll_url, sizeof(poll_url), "AT+SHREQ=\"/api/poll/%s\",1", DEVICE_ID);
    send_at_command(poll_url);
    
    if (!wait_for_ok_error(K_SECONDS(30))) {
        printk("HTTPS request failed or timeout\n");
        cleanup_http_session();
        return;
    }
    
    printk("Waiting for +SHREQ response...\n");
    int64_t start = k_uptime_get();
    while (!shreq_response_received && (k_uptime_get() - start) < K_SECONDS(10).ticks) {
        k_msleep(100);
    }
    
    if (!shreq_response_received) {
        printk("ERROR: +SHREQ response timeout!\n");
        return;
    }
    
    int actual_data_size = last_http_data_size;
    printk("Confirmed data size: %d\n", actual_data_size);
    
    // success?
    if (last_http_status_code == 200 && actual_data_size > 0) {
        printk("Reading response data, size: %d\n", actual_data_size);
        
        // Clear previous command data and flags
        memset(command_data, 0, sizeof(command_data));
        command_received = false;
        resp_len = 0;
        
        expecting_http_body = true;

        printk("Boutta READ packet n=%d\n",x++);
        k_sleep(K_SECONDS(1)); // Give modem time to prepare data
        x > 100000 ? x = 0 : (void)0;
        x == 99999 ? printk("MEM: PACKET COUNT RESET\n") : (void)0;
        
        char read_cmd[32];
        snprintf(read_cmd, sizeof(read_cmd), "AT+SHREAD=0,%d", actual_data_size);
        send_at_command(read_cmd);
        
        // Wait for the OK response to AT+SHREAD command
        if (!wait_for_ok_error(K_SECONDS(3))) {
            printk("AT+SHREAD command failed\n");
            expecting_http_body = false;
            return;
        }
        
        // Now wait for the actual data to arrive in the UART callback
        start = k_uptime_get();
        while (!command_received && (k_uptime_get() - start) < K_SECONDS(3).ticks) {
            k_msleep(50);
        }

        expecting_http_body = false;

        if (command_received) {
            printk("Received command: %s\n", command_data);
            process_command(command_data);
            
            // Send acknowledgment
            char ack_url[128];
            snprintf(ack_url, sizeof(ack_url), "AT+SHREQ=\"/api/ack/%s/OK\",1", DEVICE_ID);
            send_at_command(ack_url);
            wait_for_ok_error(K_SECONDS(7));
        } else {
            printk("No command data received within timeout\n");
        }
    } else {
        printk("No commands available (Status: %d, Size: %d)\n", 
               last_http_status_code, actual_data_size);
    }
    
    k_msleep(500);
}

void process_command(const char *response) {
    printk("Processing command: %s\n", response);

    if (strstr(response, "NOCMD") != NULL) {
        printk("No commands waiting\n");
        return;
    }

    //format: CMD:TOGGLE,PIN:17,DATA:none
    char cmd[32] = {0};
    int pin = 0;
    char data[64] = {0};
    if (sscanf(response, "CMD:%31[^,],PIN:%d,DATA:%63s", cmd, &pin, data) == 3) {
        printk("Parsed - CMD: %s, PIN: %d, DATA: %s\n", cmd, pin, data);
        execute_command(cmd, pin, data);
    } else {
        printk("Failed to parse command string\n");
    }
}

void execute_command(const char *cmd, int pin, const char *data) {
    printk("Executing: %s on pin %d\n", cmd, pin);
    
    if (strcmp(cmd, "LED_ON") == 0) {
        if (gpio_is_ready_dt(&led0)) {
            gpio_pin_set_dt(&led0, 1);
            printk("LED ON\n");
        }
    }
    else if (strcmp(cmd, "LED_OFF") == 0) {
        if (gpio_is_ready_dt(&led0)) {
            gpio_pin_set_dt(&led0, 0);
            printk("LED OFF\n");
        }
    }
    else if (strcmp(cmd, "TOGGLE") == 0) {
        if (gpio_is_ready_dt(&led0)) {
            gpio_pin_toggle_dt(&led0);
            printk("LED TOGGLED\n");
        }
    }

    else if (strcmp(cmd, "BOOT") == 0) { //changed from LO-HI-LO to HI-LO-HI as per SBC req (sbc default 3.3v pull none, active none)
        if (gpio_is_ready_dt(&trigger_pin)) {
            printk("Triggering SBC boot sequence...\n");
            gpio_pin_set_dt(&trigger_pin, 0);
            k_sleep(K_MSEC(250));
            gpio_pin_set_dt(&trigger_pin, 1);
            printk("Boot trigger complete + SBC-active state entered\n");

            sbc_is_active = true;
        }
    }
    else if (strcmp(cmd, "PULSE") == 0) { //500ms
        if (pin == 11 && gpio_is_ready_dt(&trigger_pin)) {
            gpio_pin_set_dt(&trigger_pin, 0);
            k_sleep(K_MSEC(500));
            gpio_pin_set_dt(&trigger_pin, 1);
            printk("Pulsed pin %d [SBC PIN]. SBC-active state entered \n", pin);

            sbc_is_active = true;
        }
    }
    else if (strcmp(cmd, "BLINK") == 0) {
        if (gpio_is_ready_dt(&led0)) {
            int count = 3;
            if (data[0] != '\0') count = atoi(data);
            if (count <= 0) count = 3;
            
            for (int i = 0; i < count; i++) {
                gpio_pin_set_dt(&led0, 1);
                k_sleep(K_MSEC(200));
                gpio_pin_set_dt(&led0, 0);
                k_sleep(K_MSEC(200));
            }
            printk("Blink sequence complete (%d times)\n", count);
        }
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        printk("Status requested - would send telemetry here\n");
    }
    else {
        printk("Unknown command: %s\n", cmd);
    }
}

void polling_thread(void) {
    printk("Starting polling thread...\n");
    
    while (!start_networking) {
        k_sleep(K_MSEC(100));
    }
    
    printk("Button triggered networking start!\n");
    
    if (!setup_network()) {
        printk("Network setup failed!\n");
        return;
    }
    
    while (1) {
        if (!sbc_is_active){
            poll_for_commands();
            k_sleep(K_SECONDS(POLL_INTERVAL_SEC));
        }
        else {
            printk("SBC active - MCU waiting/idle\n");
            k_sleep(K_SECONDS(5));
        }
    }
}

K_THREAD_DEFINE(polling_tid, 4096, polling_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
    printk("IoT Controller Starting...\n");
    static bool is_cert_state = false; //safeguard
    k_work_init(&sbc_handoff_work, sbc_handoff_work_handler);

    if (!device_is_ready(uart0)) {
        printk("UART not ready!\n");
        return 0;
    }

    uart_callback_set(uart0, uart_evt_cb, NULL);
    int err = uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), RX_TIMEOUT_DELAY);
    if (err) {
        printk("Failed to enable UART RX: %d\n", err);
        return 0;
    }

    if (gpio_is_ready_dt(&led0)) {
        err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
        if (err) {
            printk("Failed to configure led0: %d\n", err);
        }
    }

    if (gpio_is_ready_dt(&led1)) {
        err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
        if (err) {
            printk("Failed to configure led1: %d\n", err);
        }
    }

    if (gpio_is_ready_dt(&trigger_pin)) {
        err = gpio_pin_configure_dt(&trigger_pin, GPIO_OUTPUT_INACTIVE);
        if (err) {
            printk("Failed to configure trigger_pin: %d\n", err);
        }
    }

    if (gpio_is_ready_dt(&button)) {
        err = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
        if (err) {
            printk("Failed to configure button: %d\n", err);
        }

        err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
        if (err) {
            printk("Failed to configure button interrupt: %d\n", err);
        }

        gpio_init_callback(&button_cb_data, button_pressed_cb, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb_data);
        printk("Button configured with interrupt\n");
        gpio_pin_set_dt(&trigger_pin, 1);
    }

    if (gpio_is_ready_dt(&sbc_handoff)) {
        err = gpio_pin_configure_dt(&sbc_handoff, GPIO_INPUT);
        if (err) {
            printk("Failed to configure sbc_handoff: %d\n", err);
        }

        // Trigger on falling edge: SBC turning OFF
        err = gpio_pin_interrupt_configure_dt(&sbc_handoff, GPIO_INT_EDGE_TO_INACTIVE);
        if (err) {
            printk("Failed to configure sbc_handoff interrupt: %d\n", err);
        }

        static struct gpio_callback sbc_handoff_cb_data;
        gpio_init_callback(&sbc_handoff_cb_data, sbc_handoff_cb, BIT(sbc_handoff.pin));
        gpio_add_callback(sbc_handoff.port, &sbc_handoff_cb_data);

        printk("SBC handoff pin configured with falling-edge interrupt\n");
    }


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

/*
v1.1-221025 log rtt:
00> *** Booting nRF Connect SDK v3.1.0-6c6e5b32496e ***
00> *** Using Zephyr OS v4.1.99-1612683d4010 ***
00> IoT Controller Starting...
00> Button configured with interrupt
00> SBC handoff pin configured with falling-edge interrupt
00> Starting polling thread...
00> <<< NORMAL POWER DOWN
00> <<< ~}#À!}%}(} }$#[~
00> <<< RDY
00> <<< +CFUN: 1
00> <<< +CPIN: READY
00> <<< SMS Ready
00> Button pressed! Starting communication test...
00> Button triggered networking start!
00> Setting up network...
00> >>> AT+CMEE=2
00> <<< AT+CMEE=2
00> <<< OK
00> >>> AT
00> <<< AT
00> <<< OK
00> >>> ATE0
00> <<< ATE0
00> <<< OK
00> >>> AT+IPR=921600
00> <<< OK
00> >>> AT+GMR
00> <<< Revision:1951B16SIM7080
00> <<< OK
00> >>> AT+CPIN?
00> <<< +CPIN: READY
00> <<< OK
00> >>> AT+CGREG=1
00> <<< OK
00> >>> AT+CGREG?
00> <<< +CGREG: 1,1
00> <<< OK
00> Network registered!
00> >>> AT+CGDCONT=1,"IP","internet.telia.ee"
00> <<< OK
00> >>> AT+CNACT=0,1
00> <<< OK
00> <<< +APP PDP: 0,ACTIVE
00> >>> AT+CNACT?
00> <<< +CNACT: 0,1,"10.52.204.103"
00> <<< +CNACT: 1,0,"0.0.0.0"
00> <<< +CNACT: 2,0,"0.0.0.0"
00> <<< +CNACT: 3,0,"0.0.0.0"
00> <<< OK
00> Network setup complete
00> Setting up HTTPS session...
00> >>> AT+CSSLCFG="ignorertctime",1,1
00> <<< OK
00> Setting up certificate for HTTPS...
00> >>> AT+CFSINIT
00> <<< OK
00> >>> AT+CFSDFILE=3,"server_ca.cer"
00> <<< OK
00> >>> AT+CFSWFILE=3,"server_ca.cer",0,1265,10000
00> <<< DOWNLOAD
00> Sending certificate data...
00> <<< OK
00> >>> AT+CFSTERM
00> <<< OK
00> >>> AT+CSSLCFG="convert",2,"server_ca.cer"
00> <<< OK
00> >>> AT+CSSLCFG="sslversion",1,3
00> <<< OK
00> >>> AT+CSSLCFG="sni",1,"seven080-mcu-backend.onrender.com"
00> <<< OK
00> >>> AT+SHSSL=1,"server_ca.cer"
00> <<< OK
00> >>> AT+SHCONF="URL","https://seven080-mcu-backend.onrender.com"
00> <<< OK
00> >>> AT+SHCONF="BODYLEN",1024
00> <<< OK
00> >>> AT+SHCONF="HEADERLEN",350
00> <<< OK
00> Testing DNS resolution...
00> >>> AT+CDNSGIP="seven080-mcu-backend.onrender.com"
00> <<< OK
00> DNS resolution successful
00> <<< +CDNSGIP: 1,"seven080-mcu-backend.onrender.com","216.24.57.7"
00> >>> AT+SHCONN
00> <<< OK
00> >>> AT+SHSTATE?
00> <<< +SHSTATE: 1
00> HTTPS Session State: Connected
00> <<< OK
00> HTTPS session setup complete
00> Polling for commands...
00> >>> AT+SHCHEAD
00> <<< OK
00> >>> AT+SHCPARA
00> <<< OK
00> >>> AT+SHAHEAD="User-Agent","nRF52-IoT-Controller"
00> <<< OK
00> >>> AT+SHAHEAD="Accept","/"
00> <<< OK
00> >>> AT+SHAHEAD="Cache-control","no-cache"
00> <<< OK
00> >>> AT+SHAHEAD="Connection","keep-alive"
00> <<< OK
00> >>> AT+SHREQ="/api/poll/device001",1
00> <<< OK
00> Waiting for +SHREQ response...
00> <<< +SHREQ: "GET",200,27
00> HTTPS Status: 200, Size: 27
00> Confirmed data size: 27
00> Reading response data, size: 27
00> Boutta READ packet n=0
00> >>> AT+SHREAD=0,27
00> <<< OK
00> <<< +SHREAD: 27
00> <<< CMD:LED_ON,PIN:17,DATA:none
00> Command data captured: CMD:LED_ON,PIN:17,DATA:none
00> Received command: CMD:LED_ON,PIN:17,DATA:none
00> Processing command: CMD:LED_ON,PIN:17,DATA:none
00> Parsed - CMD: LED_ON, PIN: 17, DATA: none
00> Executing: LED_ON on pin 17
00> LED ON
00> >>> AT+SHREQ="/api/ack/device001/OK",1
00> <<< OK
00> <<< +SHREQ: "GET",200,2
00> HTTPS Status: 200, Size: 2
00> Polling for commands...
00> >>> AT+SHCHEAD
00> <<< OK
00> >>> AT+SHCPARA
00> <<< OK
00> >>> AT+SHAHEAD="User-Agent","nRF52-IoT-Controller"
00> <<< OK
00> >>> AT+SHAHEAD="Accept","/"
00> <<< OK
00> >>> AT+SHAHEAD="Cache-control","no-cache"
00> <<< OK
00> >>> AT+SHAHEAD="Connection","keep-alive"
00> <<< OK
00> >>> AT+SHREQ="/api/poll/device001",1
00> <<< OK
00> Waiting for +SHREQ response...
00> <<< +SHREQ: "GET",200,5
00> HTTPS Status: 200, Size: 5
00> Confirmed data size: 5
00> Reading response data, size: 5
00> Boutta READ packet n=1
00> >>> AT+SHREAD=0,5
00> <<< OK
00> <<< +SHREAD: 5
00> <<< NOCMD
00> Command data captured: NOCMD
00> Received command: NOCMD
00> Processing command: NOCMD
00> No commands waiting
00> >>> AT+SHREQ="/api/ack/device001/OK",1
00> <<< OK
00> <<< +SHREQ: "GET",200,2
00> HTTPS Status: 200, Size: 2
00> Polling for commands...
00> >>> AT+SHCHEAD
00> <<< OK
00> >>> AT+SHCPARA
00> <<< OK
00> >>> AT+SHAHEAD="User-Agent","nRF52-IoT-Controller"
00> <<< OK
00> >>> AT+SHAHEAD="Accept","/
00> <<< OK
00> >>> AT+SHAHEAD="Cache-control","no-cache"
00> <<< OK
00> >>> AT+SHAHEAD="Connection","keep-alive"
00> <<< OK
00> >>> AT+SHREQ="/api/poll/device001",1
00> <<< OK
00> Waiting for +SHREQ response...
00> <<< +SHREQ: "GET",200,25
00> HTTPS Status: 200, Size: 25
00> Confirmed data size: 25
00> Reading response data, size: 25
00> Boutta READ packet n=2
00> >>> AT+SHREAD=0,25
00> <<< OK
00> <<< +SHREAD: 25
00> <<< CMD:BOOT,PIN:11,DATA:none
00> Command data captured: CMD:BOOT,PIN:11,DATA:none
00> Received command: CMD:BOOT,PIN:11,DATA:none
00> Processing command: CMD:BOOT,PIN:11,DATA:none
00> Parsed - CMD: BOOT, PIN: 11, DATA: none
00> Executing: BOOT on pin 11
00> Triggering SBC boot sequence...
00> Boot trigger complete + SBC-active state entered
00> >>> AT+SHREQ="/api/ack/device001/OK",1
00> <<< OK
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
00> SBC active - MCU waiting/idle
also, sometimes:
00> *** Booting nRF Connect SDK v3.1.0-6c6e5b32496e 
00> *** Using Zephyr OS v4.1.99-1612683d4010 
00> IoT Controller Starting...
00> Button configured with interrupt
00> SBC handoff pin configured with falling-edge interrupt
00> Starting polling thread...
00> SBC powered down >>> handoff signal low
00> Ending PPP session...
00> [00:00:32.186,889] <err> os: ***** MPU FAULT *****
00> [00:00:32.186,920] <err> os:   Data Access Violation
00> 
00> [00:00:32.186,920] <err> os:   MMFAR Address: 0x0
00> [00:00:32.186,950] <err> os: r0/a1:  0x20000ba0  r1/a2:  0x00000000  r2/a3:  0x00000000
00> [00:00:32.186,950] <err> os: r3/a4:  0x20000d08 r12/ip:  0x00000000 r14/lr:  0x00006211
00> [00:00:32.186,950] <err> os:  xpsr:  0x01000221
00> [00:00:32.186,981] <err> os: Faulting instruction address (r15/pc): 0x0000604e
00> [00:00:32.187,011] <err> os: >>> ZEPHYR FATAL ERROR 19: Unknown error on CPU 0
00> [00:00:32.187,011] <err> os: Fault during interrupt handling
00> 
00> [00:00:32.187,072] <err> os: Current thread: 0x20000b88 (unknown)
00> [00:00:32.480,834] <err> os: Halting system
*/