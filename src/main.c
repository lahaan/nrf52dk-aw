/*

v1.0-081025
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

    .dts added below (bottom) as git doesn't track it
*/



#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>
#include "cert.h";

// UART & GPIO
const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0)); //P0.06=TX, P0.08=RX
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

// Control pins
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios); //oboard LED1
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); //oboard LED2
static const struct gpio_dt_spec wake_up_scb_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger0), gpios, {0}); //P0.11 to SBC (defined in dts)


static struct gpio_callback button_callback_data;
static bool is_networking_started = false;
static bool is_expecting_http_body = false;
static bool has_shreq_received_response = false; // AT command SHREQ

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
static uint8_t rx_buffer[RX_BUF_SIZE];
static char response_buffer[RESPONSE_BUF_SIZE];

static int BAUD_RATE = 921600;

static size_t response_length = 0;
static bool is_response_complete = false;
static bool is_waiting_for_response = false;
static int last_http_status_code = 0;
static int last_http_data_size = 0;

static char command_data_buffer[256];
static bool is_command_received = false;

// Network state
static bool is_network_connected = false;
static bool is_http_session_active = false;

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


const size_t ca_cert_lenght = sizeof(ca_cert) - 1;

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
    
    char command_buffer[100];
    snprintf(command_buffer, sizeof(command_buffer), "AT+CFSDFILE=3,\"%s\"", CA_CERT_FILE); 
    send_at_command(command_buffer);
    k_sleep(K_SECONDS(1));

    snprintf(command_buffer, sizeof(command_buffer), "AT+CFSWFILE=3,\"%s\",0,%d,10000", CA_CERT_FILE, ca_cert_lenght);
    printk(">>> %s\n", command_buffer);
    for (int i = 0; command_buffer[i] != '\0'; i++) {
        uart_poll_out(uart0, command_buffer[i]);
    }
    uart_poll_out(uart0, '\r');
    
    k_sleep(K_SECONDS(2));
    
    printk("Sending certificate data...\n");
    for (size_t i = 0; i < ca_cert_lenght; i++) {
        uart_poll_out(uart0, ca_cert[i]);
    }
    
    k_sleep(K_SECONDS(3)); 

    send_at_command("AT+CFSTERM");
    k_sleep(K_SECONDS(1));

    snprintf(command_buffer, sizeof(command_buffer), "AT+CSSLCFG=\"convert\",2,\"%s\"", CA_CERT_FILE);
    send_at_command(command_buffer);
    if (!wait_for_ok_error(K_SECONDS(10))) {
        return false;
    }

    return true;
}


void button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed! Starting communication test...\n");
    is_networking_started = true;
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
                if (response_length > 0) {
                    response_buffer[response_length] = '\0';
                    
                    // Trim trailing CR if present
                    if (response_length > 0 && response_buffer[response_length-1] == '\r') {
                        response_buffer[response_length-1] = '\0';
                    }
                    
                    printk("<<< %s\n", response_buffer);

                    // Check for standard responses
                    if (strcmp(response_buffer, "OK") == 0 || strcmp(response_buffer, "ERROR") == 0 || strstr(response_buffer, "+CME ERROR:") != NULL) {
                        is_response_complete = true;
                        is_waiting_for_response = false;
                    }
                    // Parse HTTP response
                    else if (strstr(response_buffer, "+SHREQ:") != NULL) {
                        char *comma1 = strchr(response_buffer, ',');
                        if (comma1) {
                            char *comma2 = strchr(comma1 + 1, ',');
                            if (comma2) {
                                last_http_status_code = atoi(comma1 + 1);
                                last_http_data_size = atoi(comma2 + 1);
                                printk("HTTPS Status: %d, Size: %d\n", last_http_status_code, last_http_data_size);
                                has_shreq_received_response = true;  // ADD THIS LINE
                            }
                        }
                        is_response_complete = true;
                    }
                    // Parse HTTP state
                    else if (strstr(response_buffer, "+SHSTATE:") != NULL) {
                        is_http_session_active = (strstr(response_buffer, "+SHSTATE: 1") != NULL);
                        printk("HTTPS Session State: %s\n", is_http_session_active ? "Connected" : "Disconnected");
                        is_response_complete = true;
                    }
                    // Capture HTTP data (starts with +SHREAD:)
                    else if (strstr(response_buffer, "+SHREAD:") != NULL) {
                        // The next line will contain the actual data
                    }
                    // Capture actual data after +SHREAD
                    else if (is_expecting_http_body && response_length > 0) {
                    if (strcmp(response_buffer, "OK") != 0 &&
                        strcmp(response_buffer, "ERROR") != 0 &&
                        strncmp(response_buffer, "+", 1) != 0) {

                        if (response_length < sizeof(command_data_buffer)) {
                            strncpy(command_data_buffer, response_buffer, sizeof(command_data_buffer) - 1);
                            command_data_buffer[sizeof(command_data_buffer) - 1] = '\0';
                            is_command_received = true;
                            printk("Command data captured: %s\n", command_data_buffer);
                        }
                    }
                }
                    response_length = 0;
                }
            } else if (c != '\r') {  // Ignore carriage returns
                if (response_length < sizeof(response_buffer) - 1) {
                    response_buffer[response_length++] = c;
                } else {
                    // Buffer overflow, reset
                    response_length = 0;
                }
            }
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

void send_at_command(const char *cmd) {
    printk(">>> %s\n", cmd);
    for (int i = 0; cmd[i] != '\0'; i++) {
        uart_poll_out(uart0, cmd[i]);
    }
    uart_poll_out(uart0, '\r');
}

bool wait_for_ok_error(k_timeout_t timeout) {
    is_response_complete = false;
    is_waiting_for_response = true;

    int64_t start = k_uptime_get();
    while (!is_response_complete && (k_uptime_get() - start) < timeout.ticks) {
        k_msleep(50);
    }

    if (!is_response_complete) {
        printk("Response timeout\n");
        is_waiting_for_response = false;
        return false;
    }

    is_waiting_for_response = false;
    return true;
}

bool wait_for_response_with_timeout(const char *expected, k_timeout_t timeout) {
    response_length = 0;
    is_response_complete = false;
    is_waiting_for_response = true;

    int64_t start = k_uptime_get();
    while (!is_response_complete && (k_uptime_get() - start) < timeout.ticks) {
        if (response_length > 0 && strstr(response_buffer, expected) != NULL) {
            is_response_complete = true;
            break;
        }
        k_msleep(50);
    }

    is_waiting_for_response = false;
    return is_response_complete;
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
    
    is_network_connected = true;
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

    printk("Setting up HTTPS session...\n");
    // Clean session
    cleanup_http_session();

    send_at_command("AT+CSSLCFG=\"ignorertctime\",1,1");
    k_sleep(K_MSEC(500));

    // Setup certificate
    if (!download_and_convert_certificate()) {
        printk("FATAL: Certificate setup failed. Cannot proceed securely.\n");
        return false; 
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
    
    is_http_session_active = true;
    printk("HTTPS session setup complete\n");
    return true;
}

void cleanup_http_session(void) {
    if (is_http_session_active) {
        printk("Cleaning up HTTPS session...\n");
        send_at_command("AT+SHDISC");
        wait_for_ok_error(K_SECONDS(5));
        is_http_session_active = false;
    }
}

void error_handler(void (*a)(), int timeout) {

}

void poll_for_commands(void) {
    if (!is_http_session_active) {
        if (!setup_https_session()) {
            printk("Failed to setup HTTPS session\n");
            return;
        }
    }
    
    printk("Polling for commands...\n");
    
    // Clear headers and parameters
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
    has_shreq_received_response = false;
    
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
    while (!has_shreq_received_response && (k_uptime_get() - start) < K_SECONDS(10).ticks) {
        k_msleep(100);
    }
    
    if (!has_shreq_received_response) {
        printk("ERROR: +SHREQ response timeout!\n");
        return;
    }
    
    int actual_data_size = last_http_data_size;
    printk("Confirmed data size: %d\n", actual_data_size);
    
    // success?
    if (last_http_status_code == 200 && actual_data_size > 0) {
        printk("Reading response data, size: %d\n", actual_data_size);
        
        // Clear previous command data and flags
        memset(command_data_buffer, 0, sizeof(command_data_buffer));
        is_command_received = false;
        response_length = 0;
        
        is_expecting_http_body = true;

        //printk("Boutta READ packet n=%d\n",x++);
        k_sleep(K_SECONDS(1)); // Give modem time to prepare data
        
        char read_cmd[32];
        snprintf(read_cmd, sizeof(read_cmd), "AT+SHREAD=0,%d", actual_data_size);
        send_at_command(read_cmd);
        
        // Wait for the OK response to AT+SHREAD command
        if (!wait_for_ok_error(K_SECONDS(3))) {
            printk("AT+SHREAD command failed\n");
            is_expecting_http_body = false;
            return;
        }
        
        // Now wait for the actual data to arrive in the UART callback
        start = k_uptime_get();
        while (!is_command_received && (k_uptime_get() - start) < K_SECONDS(3).ticks) {
            k_msleep(50);
        }

        is_expecting_http_body = false;

        if (is_command_received) {
            printk("Received command: %s\n", command_data_buffer);
            process_command(command_data_buffer);
            
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
        if (gpio_is_ready_dt(&wake_up_scb_pin)) {
            printk("Triggering SBC boot sequence...\n");
            gpio_pin_set_dt(&wake_up_scb_pin, 0);
            k_sleep(K_MSEC(250));
            gpio_pin_set_dt(&wake_up_scb_pin, 1);
            printk("Boot trigger complete\n");
        }
    }
    else if (strcmp(cmd, "PULSE") == 0) { //500ms
        if (pin == 11 && gpio_is_ready_dt(&wake_up_scb_pin)) {
            gpio_pin_set_dt(&wake_up_scb_pin, 0);
            k_sleep(K_MSEC(500));
            gpio_pin_set_dt(&wake_up_scb_pin, 1);
            printk("Pulsed pin %d\n", pin);
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
    
    while (!is_networking_started) {
        k_sleep(K_MSEC(100));
    }
    
    printk("Button triggered networking start!\n");
    
    if (!setup_network()) {
        printk("Network setup failed!\n");
        return;
    }
    
    while (1) {
        poll_for_commands();
        k_sleep(K_SECONDS(POLL_INTERVAL_SEC));
    }
}

K_THREAD_DEFINE(polling_tid, 4096, polling_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
    printk("IoT Controller Starting...\n");

    if (!device_is_ready(uart0)) {
        printk("UART not ready!\n");
        return 0;
    }

    uart_callback_set(uart0, uart_evt_cb, NULL);
    int err = uart_rx_enable(uart0, rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_DELAY);
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

    if (gpio_is_ready_dt(&wake_up_scb_pin)) {
        err = gpio_pin_configure_dt(&wake_up_scb_pin, GPIO_OUTPUT_INACTIVE);
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

        gpio_init_callback(&button_callback_data, button_pressed_cb, BIT(button.pin));
        gpio_add_callback(button.port, &button_callback_data);
        printk("Button configured with interrupt\n");
        gpio_pin_set_dt(&wake_up_scb_pin, 1);
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
		wakeup-source;
		wake_pin: wake_pin {
			gpios = <&gpio0 28 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
			label = "Wakeup Pin"; //sbc interrupting sbc for wake
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