#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>

// UART & GPIO
const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

// Control pins
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec trigger_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger0), gpios, {0});

static struct gpio_callback button_cb_data;
static bool start_networking = false;

// Server configuration - USING HTTPS
#define SERVER_URL "seven080-mcu-backend.onrender.com"
#define SERVER_HOST "seven080-mcu-backend.onrender.com" // For Host header
#define DEVICE_ID "device001"
#define POLL_INTERVAL_SEC 10

// Certificate configuration - you'll need to get the actual certificate
#define CA_CERT_FILE "server_ca.cer"
#define CA_CERT_SIZE 2048 // Adjust based on actual certificate size

// UART buffers - Increased sizes for HTTPS
#define RX_BUF_SIZE 128
#define RESPONSE_BUF_SIZE 1024  // Increased for HTTPS responses
static int32_t RX_TIMEOUT_DELAY = 500;
static uint8_t rx_buf[RX_BUF_SIZE];
static char response_buf[RESPONSE_BUF_SIZE];
static size_t resp_len = 0;
static bool response_complete = false;
static bool waiting_for_response = false;
static int last_http_status_code = 0;
static int last_http_data_size = 0;

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

const char ca_cert[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n"
;

const size_t ca_cert_len = sizeof(ca_cert) - 1; // Exclude null terminator

void send_raw_data(const char *data, size_t len) {
    printk("--- Sending raw data (%d bytes) ---\n", len);
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart0, data[i]);
    }
    printk("--- Raw data sent ---\n");
}

bool download_and_convert_certificate(void) {
    printk("Setting up certificate for HTTPS...\n");

    send_at_command("AT+CFSINIT");
    k_sleep(K_SECONDS(1));
    
    char cmd_buf[100];
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CFSDFILE=3,\"%s\"", CA_CERT_FILE);
    send_at_command(cmd_buf);
    k_sleep(K_SECONDS(1));

    // Send the write command
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CFSWFILE=3,\"%s\",0,%d,10000", 
             CA_CERT_FILE, ca_cert_len);
    printk(">>> %s\n", cmd_buf);
    for (int i = 0; cmd_buf[i] != '\0'; i++) {
        uart_poll_out(uart0, cmd_buf[i]);
    }
    uart_poll_out(uart0, '\r');
    
    // Wait exactly 2 seconds for DOWNLOAD, then send regardless
    k_sleep(K_SECONDS(2));
    
    printk("Sending certificate data...\n");
    for (size_t i = 0; i < ca_cert_len; i++) {
        uart_poll_out(uart0, ca_cert[i]);
    }
    
    // Wait for OK
    k_sleep(K_SECONDS(3));
    
    send_at_command("AT+CFSTERM");
    k_sleep(K_SECONDS(1));

    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CSSLCFG=\"convert\",2,\"%s\"", CA_CERT_FILE);
    send_at_command(cmd_buf);
    if (!wait_for_ok_error(K_SECONDS(10))) {
        return false;
    }

    return true;
}


void button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed! Starting communication test...\n");
    start_networking = true;
}

// Improved UART callback
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
                    else if (waiting_for_response && last_http_data_size > 0 && 
                             resp_len > 10 && response_buf[0] == '{') {
                        // This looks like JSON data - store it
                        if (resp_len < sizeof(command_data) - 1) {
                            strncpy(command_data, response_buf, sizeof(command_data) - 1);
                            command_data[sizeof(command_data) - 1] = '\0';
                            command_received = true;
                            printk("Command data captured: %s\n", command_data);
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
    printk("Setting up network...\n");
    
    // Enable verbose errors
    send_at_command("AT+CMEE=1");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    // Basic modem init
    send_at_command("AT");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    send_at_command("ATE0");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    // Check SIM
    send_at_command("AT+CPIN?");
    if (!wait_for_response_with_timeout("READY", K_SECONDS(2))) return false;
    
    // Network registration
    send_at_command("AT+CGREG=1");
    if (!wait_for_ok_error(K_SECONDS(2))) return false;
    
    // Wait for registration
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
    
    // Set APN
    send_at_command("AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"");
    if (!wait_for_ok_error(K_SECONDS(3))) return false;
    
    // Activate PDP context
    send_at_command("AT+CNACT=0,1");
    if (!wait_for_ok_error(K_SECONDS(15))) {
        // Check if already active
        send_at_command("AT+CNACT?");
        if (!wait_for_response_with_timeout("+CNACT: 0,1", K_SECONDS(2))) {
            return false;
        }
    }
    
    // Check if active
    send_at_command("AT+CNACT?");
    if (!wait_for_response_with_timeout("+CNACT: 0,1", K_SECONDS(2))) {
        return false;
    }
    
    network_connected = true;
    printk("Network setup complete\n");
    return true;
}

bool setup_https_session(void) {
    printk("Setting up HTTPS session...\n");
    
    // Clean up any existing session first
    cleanup_http_session();

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

    // IMPORTANT: Use the certificate for verification
    char ssl_cmd[100];
    snprintf(ssl_cmd, sizeof(ssl_cmd), "AT+SHSSL=1,\"%s\"", CA_CERT_FILE);
    send_at_command(ssl_cmd);
    if (!wait_for_ok_error(K_SECONDS(3))) {
        printk("SSL configuration failed\n");
        return false;
    }
    
    // Configure HTTPS parameters with larger buffers
    char cmd[128];
    
    // Set URL - USING HTTPS
    snprintf(cmd, sizeof(cmd), "AT+SHCONF=\"URL\",\"https://%s\"", SERVER_URL);
    send_at_command(cmd);
    if (!wait_for_ok_error(K_SECONDS(3))) {
        printk("URL configuration failed\n");
        return false;
    }
    
    // Use larger buffer sizes as shown in app note
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
    
    // Connect HTTPS session
    send_at_command("AT+SHCONN");
    if (!wait_for_ok_error(K_SECONDS(30))) {  // Increased timeout for HTTPS
        printk("HTTPS connection failed\n");
        return false;
    }
    
    // Check state
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
    if (!http_session_active) {
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
    
    // Add headers as shown in app note
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
    
    // Make GET request to poll endpoint - USING HTTPS
    char poll_url[128];
    snprintf(poll_url, sizeof(poll_url), "AT+SHREQ=\"/api/poll/%s\",1", DEVICE_ID);
    send_at_command(poll_url);
    
    if (!wait_for_ok_error(K_SECONDS(30))) {
        printk("HTTPS request failed or timeout\n");
        cleanup_http_session();
        return;
    }
    
    // Check if we got a successful response
    if (last_http_status_code == 200 && last_http_data_size > 0) {
        printk("Reading response data, size: %d\n", last_http_data_size);
        
        // Read the response data
        char read_cmd[32];
        int read_size = last_http_data_size;
        if (read_size > 500) read_size = 500; // Conservative limit
        
        snprintf(read_cmd, sizeof(read_cmd), "AT+SHREAD=0,%d", read_size);
        send_at_command(read_cmd);
        
        // Wait for data with longer timeout
        command_received = false;
        if (wait_for_ok_error(K_SECONDS(10)) && command_received) {
            printk("Received command: %s\n", command_data);
            process_command(command_data);
            
            // Send acknowledgment - USING HTTPS
            char ack_url[128];
            snprintf(ack_url, sizeof(ack_url), "AT+SHREQ=\"/api/ack/%s/OK\",1", DEVICE_ID);
            send_at_command(ack_url);
            wait_for_ok_error(K_SECONDS(10));
        } else {
            printk("No command data received\n");
        }
    } else {
        printk("No commands available (Status: %d)\n", last_http_status_code);
    }
}

void process_command(const char *response) {
    printk("Processing command: %s\n", response);
    
    // Simple JSON parsing - look for command structure
    if (strstr(response, "NOCMD") != NULL || strstr(response, "no command") != NULL) {
        printk("No commands waiting\n");
        return;
    }
    
    // Extract command, pin, and data from JSON
    // Expected format: {"command":"LED_ON","pin":17,"data":"some_data"}
    char cmd[32] = {0};
    int pin = 0;
    char data[64] = {0};
    
    // Parse command
    const char *cmd_start = strstr(response, "\"command\":\"");
    if (cmd_start) {
        cmd_start += 11; // Skip "\"command\":\""
        const char *cmd_end = strchr(cmd_start, '\"');
        if (cmd_end) {
            int len = cmd_end - cmd_start;
            if (len < sizeof(cmd)) {
                strncpy(cmd, cmd_start, len);
                cmd[len] = '\0';
            }
        }
    }
    
    // Parse pin
    const char *pin_start = strstr(response, "\"pin\":");
    if (pin_start) {
        pin_start += 6; // Skip "\"pin\":"
        pin = atoi(pin_start);
    }
    
    // Parse data
    const char *data_start = strstr(response, "\"data\":\"");
    if (data_start) {
        data_start += 8; // Skip "\"data\":\""
        const char *data_end = strchr(data_start, '\"');
        if (data_end) {
            int len = data_end - data_start;
            if (len < sizeof(data)) {
                strncpy(data, data_start, len);
                data[len] = '\0';
            }
        }
    }
    
    if (strlen(cmd) > 0) {
        printk("Parsed - CMD: %s, PIN: %d, DATA: %s\n", cmd, pin, data);
        execute_command(cmd, pin, data);
    } else {
        printk("No valid command found in response\n");
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
    else if (strcmp(cmd, "BOOT") == 0) {
        if (gpio_is_ready_dt(&trigger_pin)) {
            printk("Triggering SBC boot sequence...\n");
            gpio_pin_set_dt(&trigger_pin, 1);
            k_sleep(K_SECONDS(1));
            gpio_pin_set_dt(&trigger_pin, 0);
            printk("Boot trigger complete\n");
        }
    }
    else if (strcmp(cmd, "PULSE") == 0) {
        if (pin == 11 && gpio_is_ready_dt(&trigger_pin)) {
            gpio_pin_set_dt(&trigger_pin, 1);
            k_sleep(K_MSEC(500));
            gpio_pin_set_dt(&trigger_pin, 0);
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

// Main polling thread
void polling_thread(void) {
    printk("Starting polling thread...\n");
    
    // Wait for button press to start networking
    while (!start_networking) {
        k_sleep(K_MSEC(100));
    }
    
    printk("Button triggered networking start!\n");
    
    // Setup network
    if (!setup_network()) {
        printk("Network setup failed!\n");
        return;
    }
    
    // Main polling loop
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

    /* Register UART callback and enable RX */
    uart_callback_set(uart0, uart_evt_cb, NULL);
    int err = uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), RX_TIMEOUT_DELAY);
    if (err) {
        printk("Failed to enable UART RX: %d\n", err);
        return 0;
    }

    /* Configure GPIOs */
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
    }

    return 0;
}




/*

PEM CERTIFICATE ()

-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
*/