/* 
 * CORE DEFINITIONS 
 */
// ZEPHYR RTOS INCLUDES
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
#define PWRKEY_HIGH_MS              1075
#define MODEM_BOOT_WAIT_MS          10000 
#define POLL_INTERVAL_SEC           10
#define WATCHDOG_TIMEOUT_MS         30000

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
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec trigger_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger0), gpios, {0});
static const struct gpio_dt_spec sbc_handoff = GPIO_DT_SPEC_GET(DT_ALIAS(wakepin), gpios);
static const struct gpio_dt_spec pwrkey_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger1), gpios, {0});
static const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));

// ---------------- DATA STRUCTURES ---------------- //

typedef enum {
    STATE_HARD_RESET,
    STATE_NET_SETUP,
    STATE_HTTPS_SETUP,
    STATE_POLLING_ACTIVE,
    STATE_POLLING_IDLE,
    STATE_SBC_OWNED,    // MCU waits, SBC owns modem
    STATE_SBC_RECLAIM,  // Transitioning back from SBC
    STATE_SYSTEM_OFF    // Optional deep sleep
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
    bool is_expecting_body;
    bool is_response_complete;
    size_t response_length;
} uart_state_t;

typedef struct {
    uint32_t boot_count;
    uint32_t handoff_count;
} device_stats_t;

// ---------------- GLOBAL STATE ---------------- //

static __attribute__((section(".noinit"))) http_state_t http_state;
static __attribute__((section(".noinit"))) uart_state_t uart_state;
static __attribute__((section(".noinit"))) device_stats_t stats;
static __attribute__((section(".noinit"))) bool was_sleeping;

static int wdt_channel_id;
static bool is_skip_wdt = false;

// Global Buffers
static uint8_t rx_buffer[RX_BUF_SIZE];
static char response_buffer[RESPONSE_BUF_SIZE];
static char command_data[256];
static bool is_command_received = false;

// FSM Globals
static machine_state_t current_state = STATE_HARD_RESET;
static volatile bool flag_sbc_request_active = false;
static struct gpio_callback sbc_handoff_cb_data;
static struct gpio_callback button_callback_data;
static struct gpio_callback button3_callback_data;

// CERTIFICATE
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

// ---------------- UART PROCESSING ---------------- //

static void trim_trailing_cr(char *buffer, size_t *length)
{
    if (*length > 0 && buffer[*length - 1] == '\r') {
        buffer[*length - 1] = '\0';
        (*length)--;
    }
}

static void process_line(void)
{
    response_buffer[uart_state.response_length] = '\0';
    trim_trailing_cr(response_buffer, &uart_state.response_length);
    
    // Debug print can be noisy, enable if needed
    // printk("<<< %s\n", response_buffer);

    if (strcmp(response_buffer, "OK") == 0) {
        uart_state.is_response_complete = true;
    } 
    else if (strstr(response_buffer, "ERROR") != NULL) {
        uart_state.is_response_complete = true;
    }
    else if (strstr(response_buffer, "+SHREQ:") != NULL) {
        char *comma1 = strchr(response_buffer, ',');
        if (comma1) {
            char *comma2 = strchr(comma1 + 1, ',');
            if (comma2) {
                http_state.last_status_code = atoi(comma1 + 1);
                http_state.last_data_size = atoi(comma2 + 1);
                uart_state.is_response_complete = true;
            }
        }
    }
    else if (strstr(response_buffer, "+SHSTATE:") != NULL) {
        http_state.is_session_active = (strstr(response_buffer, "+SHSTATE: 1") != NULL);
        uart_state.is_response_complete = true;
    }
    else if (uart_state.is_expecting_body && uart_state.response_length > 0) {
        if (strcmp(response_buffer, "OK") != 0 && 
            strcmp(response_buffer, "ERROR") != 0 && 
            strncmp(response_buffer, "+", 1) != 0) {
            
            if (uart_state.response_length < sizeof(command_data)) {
                strncpy(command_data, response_buffer, sizeof(command_data) - 1);
                command_data[sizeof(command_data) - 1] = '\0';
                is_command_received = true;
                printk("Command Data Captured: %s\n", command_data);
            }
        }
    }
    uart_state.response_length = 0;
}

static void uart_event_callback(const struct device *dev, struct uart_event *event, void *user_data)
{
    switch (event->type) {
    case UART_RX_RDY: {
        const uint8_t *p = &event->data.rx.buf[event->data.rx.offset];
        size_t l = event->data.rx.len;

        for (size_t i = 0; i < l; i++) {
            char c = (char)p[i];
            if (c == '\n') {
                if (uart_state.response_length > 0) process_line();
            } else if (c != '\r') {
                if (uart_state.response_length < sizeof(response_buffer) - 1) {
                    response_buffer[uart_state.response_length++] = c;
                } else {
                    uart_state.response_length = 0; // Overflow protection
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

// ---------------- AT COMMAND PRIMITIVES ---------------- //

void send_raw(const char *command) {
    printk(">>> %s\n", command);
    for (int i = 0; command[i] != '\0'; i++) uart_poll_out(uart0, command[i]);
    uart_poll_out(uart0, '\r');
}

/* 
 * Core FSM Helper: Sends command, waits for OK/ERROR or specific logic.
 * Returns: true (OK), false (ERROR or Timeout or Interrupt)
 * Checks 'flag_sbc_request_active' continuously to abort early.
 */
static bool send_at_power_safe(const char *cmd, k_timeout_t timeout, int recharge_ms) {
    // 1. Check Interrupt immediately
    if (flag_sbc_request_active) return false;

    // 2. Capacitor Recharge Delay
    if (recharge_ms > 0) k_msleep(recharge_ms);

    // 3. Prepare State
    uart_state.is_response_complete = false;
    uart_state.response_length = 0;
    memset(response_buffer, 0, sizeof(response_buffer));

    // 4. Transmit
    send_raw(cmd);

    // 5. Wait Loop
    int64_t start = k_uptime_get();
    while ((k_uptime_get() - start) < timeout.ticks) {
        // INTERRUPT CHECK
        if (flag_sbc_request_active) return false;

        // COMPLETION CHECK
        if (uart_state.is_response_complete) {
             if (strstr(response_buffer, "OK")) return true;
             // Some commands return stats via SHREQ callback logic, handled in process_line
             if (strstr(response_buffer, "+SHREQ:")) return true; 
             if (strstr(response_buffer, "ERROR")) return false;
             // Fallback: if we got a completion flag but no OK/ERROR, assume valid if not explicit error
             return true; 
        }
        
        // Feed while waiting
        watchdog_feed();
        k_msleep(10);
    }
    return false; // Timeout
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
        // Software trigger to handoff to SBC
        gpio_pin_set_dt(&trigger_pin, 0);
        k_sleep(K_MSEC(250));
        gpio_pin_set_dt(&trigger_pin, 1);
        // The physical pin wiring should trigger the interrupt, but we can force the flag too
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
    if (!http_state.is_session_active) return;
    send_at_power_safe("AT+SHDISC", K_SECONDS(2), 100);
    send_at_power_safe("AT+SHSSL=0", K_SECONDS(1), 100);
    http_state.is_session_active = false;
    http_state.is_connected = false;
}

machine_state_t run_hard_reset(void) {
    printk("--- STATE: HARD RESET ---\n");
    if (!gpio_is_ready_dt(&pwrkey_pin)) return STATE_HARD_RESET; // Stuck

    // Physical toggle sequence
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_HIGH_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    
    k_sleep(K_SECONDS(3)); // Shutdown wait
    
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_HIGH_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    
    // Allow boot
    k_sleep(K_SECONDS(5));
    
    // Verify life
    if (send_at_power_safe("AT", K_SECONDS(2), 0)) {
        return STATE_NET_SETUP;
    }
    // Retry if dead
    stats.boot_count++;
    return STATE_HARD_RESET;
}

machine_state_t run_net_setup(void) {
    printk("--- STATE: NET SETUP ---\n");

    send_at_power_safe("ATE0", K_SECONDS(1), 0);
    send_at_power_safe("AT+CMEE=2", K_SECONDS(1), 0);
    
    // Check if already registered (Retained state optimization)
    send_at_power_safe("AT+CGREG?", K_SECONDS(1), 0);
    if (strstr(response_buffer, "1,1") || strstr(response_buffer, "1,5")) {
        return STATE_HTTPS_SETUP;
    }

    // Force Setup
    if (!send_at_power_safe("AT+CFUN=1", K_SECONDS(10), 1000)) return STATE_HARD_RESET;
    
    // Set APN
    if (!send_at_power_safe("AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"", K_SECONDS(2), 500)) return STATE_HARD_RESET;

    // Activate
    if (!send_at_power_safe("AT+CNACT=0,1", K_SECONDS(15), 1000)) {
        // Check if already active
        send_at_power_safe("AT+CNACT?", K_SECONDS(1), 0);
        if (!strstr(response_buffer, "0,1")) return STATE_HARD_RESET;
    }

    // Poll Registration
    for(int i=0; i<15; i++) {
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
    
    // Check existing
    send_at_power_safe("AT+SHSTATE?", K_SECONDS(1), 0);
    if (http_state.is_session_active) return STATE_POLLING_ACTIVE;

    // SSL Config
    send_at_power_safe("AT+CSSLCFG=\"ignorertctime\",1,1", K_SECONDS(1), 0);
    send_at_power_safe("AT+CSSLCFG=\"sslversion\",1,3", K_SECONDS(1), 0);
    char sni[128]; snprintf(sni, sizeof(sni), "AT+CSSLCFG=\"sni\",1,\"%s\"", SERVER_HOST);
    send_at_power_safe(sni, K_SECONDS(1), 0);

    // Cert Check (Simplified: assume if file exists on modem it's good, or just set it)
    // In production, check AT+CFSFILE logic. Here we just enforce it.
    if (!uart_state.is_certificate_setup) {
        send_at_power_safe("AT+CFSINIT", K_SECONDS(1), 0);
        char cmd[64]; snprintf(cmd, sizeof(cmd), "AT+CFSWFILE=3,\"%s\",0,%d,10000", CA_CERT_FILE, (int)ca_certificate_length);
        send_raw(cmd);
        k_sleep(K_MSEC(500));
        for(size_t i=0; i<ca_certificate_length; i++) uart_poll_out(uart0, ca_certificate[i]);
        k_sleep(K_SECONDS(1));
        send_at_power_safe("AT+CFSTERM", K_SECONDS(1), 0);
        snprintf(cmd, sizeof(cmd), "AT+CSSLCFG=\"convert\",2,\"%s\"", CA_CERT_FILE);
        send_at_power_safe(cmd, K_SECONDS(5), 1000);
        uart_state.is_certificate_setup = true;
    }

    // HTTP Config
    char ssl[64]; snprintf(ssl, sizeof(ssl), "AT+SHSSL=1,\"%s\"", CA_CERT_FILE);
    send_at_power_safe(ssl, K_SECONDS(1), 0);
    char url[128]; snprintf(url, sizeof(url), "AT+SHCONF=\"URL\",\"https://%s\"", SERVER_URL);
    send_at_power_safe(url, K_SECONDS(1), 0);
    send_at_power_safe("AT+SHCONF=\"BODYLEN\",1024", K_SECONDS(1), 0);
    send_at_power_safe("AT+SHCONF=\"HEADERLEN\",350", K_SECONDS(1), 0);

    // Connect (High Current)
    if (send_at_power_safe("AT+SHCONN", K_SECONDS(20), 2000)) {
        http_state.is_session_active = true;
        return STATE_POLLING_ACTIVE;
    }

    return STATE_NET_SETUP; // Failed connection, maybe net drop
}

machine_state_t run_polling_active(void) {
    printk("--- STATE: POLLING ---\n");

    // Setup Headers
    send_at_power_safe("AT+SHCHEAD", K_SECONDS(1), 0);
    send_at_power_safe("AT+SHAHEAD=\"User-Agent\",\"nRF52-IoT\"", K_SECONDS(1), 0);
    send_at_power_safe("AT+SHAHEAD=\"Connection\",\"keep-alive\"", K_SECONDS(1), 0);

    // Send GET
    http_state.is_shreq_received = false;
    http_state.last_data_size = 0;
    char get_cmd[128]; snprintf(get_cmd, sizeof(get_cmd), "AT+SHREQ=\"/api/poll/%s\",1", DEVICE_ID);
    
    if (!send_at_power_safe(get_cmd, K_SECONDS(15), 500)) {
        // Request failed
        return STATE_POLLING_IDLE; // Try again later
    }

    // Wait for +SHREQ via loop (send_at handles response buffering, but we need specific parsing)
    // The uart_event_callback parses +SHREQ and sets http_state globals
    int64_t start = k_uptime_get();
    while (!http_state.last_status_code && (k_uptime_get() - start) < K_SECONDS(10).ticks) {
        if (flag_sbc_request_active) return STATE_SBC_OWNED;
        watchdog_feed();
        k_msleep(50);
    }

    if (http_state.last_status_code == 200 && http_state.last_data_size > 0) {
        uart_state.is_expecting_body = true;
        is_command_received = false;
        char read_cmd[64]; snprintf(read_cmd, sizeof(read_cmd), "AT+SHREAD=0,%d", http_state.last_data_size);
        send_at_power_safe(read_cmd, K_SECONDS(3), 0);

        // Wait for data capture
        start = k_uptime_get();
        while (!is_command_received && (k_uptime_get() - start) < K_SECONDS(5).ticks) {
            k_msleep(50);
        }
        uart_state.is_expecting_body = false;

        if (is_command_received) {
            process_command_string(command_data);
            // Ack
            snprintf(get_cmd, sizeof(get_cmd), "AT+SHREQ=\"/api/ack/%s/OK\",1", DEVICE_ID);
            send_at_power_safe(get_cmd, K_SECONDS(5), 1000);
        }
    }

    // Reset for next loop
    http_state.last_status_code = 0;
    return STATE_POLLING_IDLE;
}

machine_state_t run_sbc_reclaim(void) {
    printk("--- STATE: SBC RECLAIM ---\n");
    stats.handoff_count++;

    // 1. Wait for SBC to stop talking
    k_sleep(K_SECONDS(2));
    
    // 2. Escape PPP (Hayes Standard: Silence -> +++ -> Silence)
    printk("Escaping PPP...\n");
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(20));
    uart_poll_out(uart0, '+');
    k_sleep(K_MSEC(20));
    uart_poll_out(uart0, '+');
    k_sleep(K_SECONDS(2)); // Required post-guard time

    // 3. Cleanup
    send_at_power_safe("ATH", K_SECONDS(2), 0);

    // 4. Check Responsiveness
    if (send_at_power_safe("AT", K_SECONDS(2), 0)) {
        // Re-enable our session state logic
        return STATE_NET_SETUP; // Go verify net before polling
    }

    return STATE_HARD_RESET; // Modem hung
}

// ---------------- INTERRUPTS ---------------- //

static void sbc_handoff_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    // ISR context: Fast, no logic.
    // Active High logic assumption based on "Pin went LOW, we need to send +++"
    // So if Pin is 1, SBC owns it.
    flag_sbc_request_active = (gpio_pin_get_dt(&sbc_handoff) == 1);
}

static void button_pressed_callback(const struct device *dev, struct gpio_callback *callback, uint32_t pins)
{
    // Manual forcing of state if needed
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
    printk("System Booting... FSM v2.0\n");

    // 1. Hardware Init
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

    // 2. SBC Interrupt Setup
    if (gpio_is_ready_dt(&sbc_handoff)) {
        gpio_pin_configure_dt(&sbc_handoff, GPIO_INPUT);
        // Monitor both edges to detect Claim and Release
        gpio_pin_interrupt_configure_dt(&sbc_handoff, GPIO_INT_EDGE_BOTH);
        gpio_init_callback(&sbc_handoff_cb_data, sbc_handoff_callback, BIT(sbc_handoff.pin));
        gpio_add_callback(sbc_handoff.port, &sbc_handoff_cb_data);
        
        // Initial check
        flag_sbc_request_active = (gpio_pin_get_dt(&sbc_handoff) == 1);
    }

    // 3. UART Init
    if (!device_is_ready(uart0)) return 0;
    uart_callback_set(uart0, uart_event_callback, NULL);
    uart_rx_enable(uart0, rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_DELAY);

    // 4. Init Retained State handling (Cold boot vs Sleep wake)
    if ((NRF_POWER->RESETREAS & NRF_POWER_RESETREAS_OFF_MASK) == 0) {
        memset(&http_state, 0, sizeof(http_state));
        memset(&uart_state, 0, sizeof(uart_state));
    }
    NRF_POWER->RESETREAS = 0xFFFFFFFF;

    watchdog_init();

    // ---------------- SUPER LOOP ---------------- //
    while(1) {
        
        // GLOBAL PRIORITY CHECK: SBC HANDOFF
        if (flag_sbc_request_active && current_state != STATE_SBC_OWNED) {
            printk("!!! SBC INTERRUPT !!! Switching to SBC_OWNED\n");
            cleanup_http_session(); 
            current_state = STATE_SBC_OWNED;
        }

        watchdog_feed();

        switch (current_state) {
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
                // Wait loop with continuous interrupt checking
                for (int i = 0; i < POLL_INTERVAL_SEC * 10; i++) {
                    if (flag_sbc_request_active) break;
                    k_msleep(100);
                    watchdog_feed();
                }
                if (!flag_sbc_request_active) current_state = STATE_POLLING_ACTIVE;
                break;

            case STATE_SBC_OWNED:
                // Passive state. We just feed the dog and check the pin.
                // The interrupt handler updates the flag, but we poll pin in case we missed an edge
                if (gpio_pin_get_dt(&sbc_handoff) == 0) {
                    flag_sbc_request_active = false;
                    current_state = STATE_SBC_RECLAIM;
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
