/*
    FSM-v2.3c
     PSM disabled before ALL handoffs (CMD + Manual) x Early detection of manual handoff before MUX switch x Adjusted polling interval
     date: 05/12/25
*/

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/reboot.h> 
#include <string.h>
#include <stdlib.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_power.h>
#include <hal/nrf_gpio.h>

#define BAUD_RATE                   921600
#define RX_BUF_SIZE                 128     
#define RESPONSE_BUF_SIZE           1024    
#define RX_TIMEOUT_DELAY            500     

// TIMING
#define PWRKEY_PRESS_MS             1200    
#define PWRKEY_WAKE_MS              1200    
#define MODEM_BOOT_WAIT_SEC         6       
#define POLL_INTERVAL_SEC           8       
#define MAX_CONSECUTIVE_FAILS       3       
#define MAX_HARD_RESETS             3       
#define MAGIC_CODE                  0xDEADBEEF

// POWER SAVING
#define PSM_SETTINGS                "1,,,\"00100001\",\"00010101\"" 
#define EDRX_SETTINGS               "1,4,\"0010\""

// SERVER
#define SERVER_HOST                 "seven080-mcu-backend.onrender.com"
#define SERVER_URL                  "https://" SERVER_HOST
#define DEVICE_ID                   "device001"
#define CA_CERT_FILE                "server_ca.cer"

const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec trigger_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger0), gpios, {0});
static const struct gpio_dt_spec sbc_handoff = GPIO_DT_SPEC_GET(DT_ALIAS(wakepin), gpios);
static const struct gpio_dt_spec pwrkey_pin = GPIO_DT_SPEC_GET_OR(DT_ALIAS(trigger1), gpios, {0});
static const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));

typedef enum {
    STATE_BOOT_WAIT,
    STATE_IDLE,
    STATE_INIT,
    STATE_CHECK_MODEM,
    STATE_NET_SETUP,
    STATE_HTTPS_SETUP,
    STATE_POLLING,
    STATE_SBC_PREP,
    STATE_SBC_OWNED,
    STATE_RECOVERY,
    STATE_HARD_RESET
} machine_state_t;

typedef struct {
    bool is_session_active;
    int last_status_code;
    int last_data_size;
} http_state_t;

static http_state_t http_state;

// MEMORY RETENTION
typedef struct {
    uint32_t magic;
    bool psm_active; 
} retained_data_t;

static __attribute__((section(".noinit"))) retained_data_t retained;

// GLOBALS
static machine_state_t current_state = STATE_BOOT_WAIT;
static int wdt_channel_id = -1;
static bool is_skip_wdt = false;
static bool flag_start_network = false;
static bool flag_sbc_active = false;
static bool flag_sbc_handoff_pending = false; // NEW: Early detection flag
static volatile bool flag_modem_brownout = false; 

// COUNTERS
static int consecutive_fails = 0;
static int hard_reset_count = 0;

// UART
static uint8_t rx_buf[RX_BUF_SIZE];
static char response_buffer[RESPONSE_BUF_SIZE];
static size_t response_len = 0;
static bool response_complete = false;
static char command_data[256];
static bool is_cmd_received = false;

// CERT
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

static void watchdog_feed(void);
static void watchdog_init(void);
static bool disable_psm_and_edrx(void);


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    switch (evt->type) {
    case UART_RX_RDY: {
        for (int i = 0; i < evt->data.rx.len; i++) {
            char c = evt->data.rx.buf[evt->data.rx.offset + i];
            if (response_len < RESPONSE_BUF_SIZE - 1) {
                response_buffer[response_len++] = c;
                response_buffer[response_len] = '\0';
            }
        }
        if (strstr(response_buffer, "RDY") || strstr(response_buffer, "POWER DOWN")) {
            flag_modem_brownout = true;
            response_complete = true; 
        }
        if (strstr(response_buffer, "OK\r\n") || strstr(response_buffer, "ERROR") || strstr(response_buffer, "+CME ERROR")) {
            response_complete = true;
        }
        if (strstr(response_buffer, "+SHREQ:")) {
            char *p = strstr(response_buffer, "+SHREQ:");
            sscanf(p, "+SHREQ: \"%*[^\"]\",%d,%d", &http_state.last_status_code, &http_state.last_data_size);
            if (http_state.last_status_code == 0) {
                char *c1 = strchr(p, ',');
                if (c1) {
                    http_state.last_status_code = atoi(c1+1);
                    char *c2 = strchr(c1+1, ',');
                    if (c2) http_state.last_data_size = atoi(c2+1);
                }
            }
        }
        if (strstr(response_buffer, "CMD:")) {
            char *start = strstr(response_buffer, "CMD:");
            strncpy(command_data, start, sizeof(command_data)-1);
            is_cmd_received = true;
        }
        break;
    }
    case UART_RX_DISABLED:
        uart_rx_enable(uart0, rx_buf, RX_BUF_SIZE, RX_TIMEOUT_DELAY);
        break;
    default: break;
    }
}

// ---------------- PRIMITIVES ---------------- //

bool send_at(const char *cmd, k_timeout_t timeout) {
    printk("> %s", cmd);
    response_len = 0;
    response_buffer[0] = '\0';
    response_complete = false;
    flag_modem_brownout = false; 

    for(int i=0; cmd[i]; i++) uart_poll_out(uart0, cmd[i]);
    uart_poll_out(uart0, '\r');

    int64_t start = k_uptime_get();
    while ((k_uptime_get() - start) < timeout.ticks) {
        if (flag_modem_brownout) {
            printk(" -> MODEM RESET DETECTED!\n");
            return false; 
        }
        if (response_complete) {
            bool ok = (strstr(response_buffer, "OK") != NULL);
            printk(" -> %s\n", ok ? "OK" : "ERR");
            return ok;
        }
        watchdog_feed();
        
        // Emergency handoff check
        if (flag_sbc_handoff_pending && current_state != STATE_SBC_PREP) {
            printk(" -> HANDOFF PENDING, ABORTING\n");
            return false;
        }
        
        k_msleep(10);
    }
    printk(" -> TIMEOUT\n");
    return false;
}

static bool disable_psm_and_edrx(void) {
    printk(">>> Disabling PSM/eDRX before handoff...\n");
    bool success = true;
    
    // Disable PSM
    if (!send_at("AT+CPSMS=0", K_MSEC(1000))) {
        printk("WARNING: Failed to disable PSM\n");
        success = false;
    }
    
    // Disable eDRX
    if (!send_at("AT+CEDRXS=0", K_MSEC(1000))) {
        printk("WARNING: Failed to disable eDRX\n");
        success = false;
    }
    
    // Verify PSM disable
    if (send_at("AT+CPSMS?", K_MSEC(500))) {
        if (strstr(response_buffer, "+CPSMS: 0")) {
            printk(">>> PSM successfully disabled\n");
            retained.psm_active = false;
        } else {
            printk("WARNING: PSM still active after disable attempt\n");
            success = false;
        }
    }
    
    return success;
}

void execute_command(const char *raw) {
    char cmd[16] = {0}; int pin = 0; char arg[32] = {0};
    char *p_cmd = strstr(raw, "CMD:");
    char *p_pin = strstr(raw, "PIN:");
    char *p_dat = strstr(raw, "DATA:");
    
    if (p_cmd) sscanf(p_cmd, "CMD:%15[^,]", cmd);
    if (p_pin) sscanf(p_pin, "PIN:%d", &pin);
    if (p_dat) sscanf(p_dat, "DATA:%31s", arg);

    printk("EXEC: %s\n", cmd);

    if (strcmp(cmd, "LED_ON") == 0) gpio_pin_set_dt(&led0, 1);
    else if (strcmp(cmd, "LED_OFF") == 0) gpio_pin_set_dt(&led0, 0);
    else if (strcmp(cmd, "TOGGLE") == 0) gpio_pin_toggle_dt(&led0);
    else if (strcmp(cmd, "BOOT") == 0) {
        printk("CMD BOOT: Pre-configuring modem for SBC...\n");
        
        if (!disable_psm_and_edrx()) {
            printk("ERROR: Failed to clean modem config!\n");
            printk("Proceeding anyway, but SBC may inherit PSM...\n");
        }
        
        // Boot SBC (This will switch MUX)
        printk("Booting SBC...\n");
        gpio_pin_set_dt(&trigger_pin, 0); 
        k_msleep(250);
        gpio_pin_set_dt(&trigger_pin, 1); 
        
        flag_sbc_handoff_pending = true;
    }
}

machine_state_t run_boot_wait(void) {
    printk("--- BOOT WAIT (3s) ---\n");
    gpio_pin_set_dt(&led0, 1);
    for(int i=0; i<30; i++) {
        if (gpio_pin_get_dt(&button3) == 1) {
            is_skip_wdt = true;
            printk("WDT DISABLED\n");
            for(int k=0; k<5; k++) { gpio_pin_toggle_dt(&led0); k_msleep(50); }
        }
        k_msleep(100);
    }
    gpio_pin_set_dt(&led0, 0);
    return STATE_INIT;
}

machine_state_t run_init(void) {
    uint32_t rr = nrf_power_resetreas_get(NRF_POWER);
    nrf_power_resetreas_clear(NRF_POWER, 0xFFFFFFFF);
    
    if (rr & NRF_POWER_RESETREAS_OFF_MASK) {
        printk("Wake from Sleep -> RECOVERY\n");
        return STATE_RECOVERY;
    }
    
    if (retained.magic != MAGIC_CODE) {
        printk("Cold Boot -> Clearing Retention\n");
        retained.magic = MAGIC_CODE;
        retained.psm_active = false; 
    } else {
        printk("Retained State: PSM=%d\n", retained.psm_active);
    }

    if (rr & NRF_POWER_RESETREAS_DOG_MASK) {
        printk("WDT Reset -> CHECK MODEM\n");
        return STATE_CHECK_MODEM; 
    }
    return STATE_IDLE;
}

machine_state_t run_idle(void) {
    if (flag_start_network) {
        flag_start_network = false;
        return STATE_CHECK_MODEM;
    }
    k_msleep(100);
    watchdog_feed();
    return STATE_IDLE;
}

machine_state_t run_check_modem(void) {
    printk("--- CHECK MODEM (Strikes: %d) ---\n", consecutive_fails);
    
    if (consecutive_fails >= MAX_CONSECUTIVE_FAILS) {
        return STATE_HARD_RESET;
    }

    send_at("ATE0", K_MSEC(500)); 
    if (send_at("AT", K_MSEC(500))) return STATE_NET_SETUP;
    
    if (retained.psm_active) {
        printk("Modem Sleeping (PSM). Waking...\n");
        gpio_pin_set_dt(&pwrkey_pin, 1);
        k_sleep(K_MSEC(PWRKEY_WAKE_MS)); 
        gpio_pin_set_dt(&pwrkey_pin, 0);
        
        printk("Waiting 3s for wake...\n");
        k_sleep(K_SECONDS(3));
        
        if (send_at("AT", K_MSEC(500))) {
            printk("Modem Woke Up!\n");
            return STATE_NET_SETUP; 
        }
    }

    printk("Modem Dead -> HARD RESET\n");
    return STATE_HARD_RESET;
}

machine_state_t run_hard_reset(void) {
    printk("--- HARD RESET (%d/%d) ---\n", hard_reset_count + 1, MAX_HARD_RESETS);
    
    if (hard_reset_count >= MAX_HARD_RESETS) {
        printk("CRITICAL FAILURE -> SYSTEM REBOOT\n");
        k_sleep(K_SECONDS(2));
        sys_reboot(SYS_REBOOT_COLD);
    }

    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_PRESS_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    
    printk("Boot Wait (%ds)...\n", MODEM_BOOT_WAIT_SEC);
    for(int i=0; i<MODEM_BOOT_WAIT_SEC; i++) {
        k_sleep(K_SECONDS(1));
        watchdog_feed();
    }
    
    hard_reset_count++; 
    consecutive_fails = 0; 
    retained.psm_active = false;
    
    if (send_at("AT", K_MSEC(500))) return STATE_NET_SETUP;
    return STATE_HARD_RESET;
}

machine_state_t run_net_setup(void) {
    printk("--- NET SETUP ---\n");
    
    send_at("AT+CMEE=2", K_MSEC(500));
    
    send_at("AT+CFUN=0", K_SECONDS(5));
    k_msleep(1000);
    
    send_at("AT+CFUN=1", K_SECONDS(5));
    
    bool sim_ready = false;
    for(int i=0; i<10; i++) {
        if(send_at("AT+CPIN?", K_MSEC(500))) {
            if(strstr(response_buffer, "READY")) {
                sim_ready = true;
                break;
            }
        }
        k_sleep(K_MSEC(500));
    }
    if(!sim_ready) printk("Warning: SIM not ready yet...\n");

    // Configure PSM/eDRX
    printk("Configuring PSM/eDRX...\n");
    k_sleep(K_SECONDS(2)); 
    
    char psm[64]; snprintf(psm, sizeof(psm), "AT+CPSMS=%s", PSM_SETTINGS);
    if (send_at(psm, K_MSEC(1000))) {
        retained.psm_active = true; 
        printk("PSM enabled: TAU=1h, Active=30s\n");
    } else {
        printk("PSM Config Failed (Ignoring)\n");
    }
    
    char edrx[64]; snprintf(edrx, sizeof(edrx), "AT+CEDRXS=%s", EDRX_SETTINGS);
    send_at(edrx, K_MSEC(1000));

    printk("Waiting for Reg...\n");
    bool registered = false;
    for(int i=0; i<20; i++) {
        if (flag_sbc_handoff_pending) return STATE_SBC_PREP;
        if (send_at("AT+CGREG?", K_MSEC(500))) { 
            if (strstr(response_buffer, ",1") || strstr(response_buffer, ",5")) {
                registered = true;
                break;
            }
        }
        k_sleep(K_SECONDS(2));
        watchdog_feed();
    }
    if (!registered) {
        printk("Reg Timeout\n");
        consecutive_fails++; 
        return STATE_CHECK_MODEM;
    }

    send_at("AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"", K_MSEC(500));
    
    if (!send_at("AT+CNACT=0,1", K_SECONDS(10))) {
        send_at("AT+CNACT?", K_MSEC(500));
        if (strstr(response_buffer, "0,1")) {
            printk("Already Active\n");
        } else {
            printk("Act Failed\n");
            consecutive_fails++; 
            return STATE_CHECK_MODEM; 
        }
    }
    return STATE_HTTPS_SETUP;
}

machine_state_t run_https_setup(void) {
    printk("--- HTTPS SETUP ---\n");
    
    send_at("AT+SHDISC", K_MSEC(500)); 
    
    send_at("AT+CFSINIT", K_MSEC(500));
    char cmd[64]; snprintf(cmd, sizeof(cmd), "AT+CFSWFILE=3,\"%s\",0,%d,5000", CA_CERT_FILE, ca_certificate_length);
    uart_poll_out(uart0, '\r'); k_msleep(100);
    for(int i=0; cmd[i]; i++) uart_poll_out(uart0, cmd[i]);
    uart_poll_out(uart0, '\r');
    k_msleep(500);
    for(int i=0; i<ca_certificate_length; i++) uart_poll_out(uart0, ca_certificate[i]);
    k_msleep(500);
    send_at("AT+CFSTERM", K_MSEC(500));
    snprintf(cmd, sizeof(cmd), "AT+CSSLCFG=\"convert\",2,\"%s\"", CA_CERT_FILE);
    send_at(cmd, K_SECONDS(2));
    send_at("AT+CSSLCFG=\"sslversion\",1,3", K_MSEC(500));
    send_at("AT+CSSLCFG=\"ignorertctime\",1,1", K_MSEC(500));
    char sni[128]; snprintf(sni, sizeof(sni), "AT+CSSLCFG=\"sni\",1,\"%s\"", SERVER_HOST);
    send_at(sni, K_MSEC(500));
    char ssl[64]; snprintf(ssl, sizeof(ssl), "AT+SHSSL=1,\"%s\"", CA_CERT_FILE);
    send_at(ssl, K_MSEC(500));
    char url[128]; snprintf(url, sizeof(url), "AT+SHCONF=\"URL\",\"%s\"", SERVER_URL);
    send_at(url, K_MSEC(500));
    send_at("AT+SHCONF=\"BODYLEN\",1024", K_MSEC(500));
    send_at("AT+SHCONF=\"HEADERLEN\",350", K_MSEC(500));
    
    if (send_at("AT+SHCONN", K_SECONDS(15))) {
        http_state.is_session_active = true;
        consecutive_fails = 0;
        hard_reset_count = 0;
        return STATE_POLLING;
    }

    printk("HTTPS Fail -> Strike\n");
    consecutive_fails++; 
    printk("Power Recovery (5s)...\n");
    k_sleep(K_SECONDS(5)); 
    
    return STATE_CHECK_MODEM;
}

machine_state_t run_polling(void) {
    if (!send_at("AT+SHCHEAD", K_MSEC(500))) {
         consecutive_fails++; 
         return STATE_CHECK_MODEM;
    }
    
    send_at("AT+SHAHEAD=\"User-Agent\",\"nRF52-IoT\"", K_MSEC(500));
    char req[64]; snprintf(req, sizeof(req), "AT+SHREQ=\"/api/poll/%s\",1", DEVICE_ID);
    http_state.last_status_code = 0;
    
    if (send_at(req, K_SECONDS(2))) {
        int64_t start = k_uptime_get();
        while (http_state.last_status_code == 0 && (k_uptime_get() - start) < 15000) {
             watchdog_feed();
             k_msleep(50);
        }
        if (http_state.last_status_code == 200) {
             consecutive_fails = 0; 
             is_cmd_received = false;
             char read[32]; snprintf(read, sizeof(read), "AT+SHREAD=0,%d", http_state.last_data_size);
             send_at(read, K_SECONDS(2));
             if (is_cmd_received) {
                 execute_command(command_data);
                 snprintf(req, sizeof(req), "AT+SHREQ=\"/api/ack/%s/OK\",1", DEVICE_ID);
                 send_at(req, K_SECONDS(2));
             }
        }
    }
    
    // Poll every 8s (fits within 30s active window)
    for(int i=0; i<POLL_INTERVAL_SEC * 10; i++) {
        if (flag_sbc_handoff_pending) return STATE_SBC_PREP;
        k_msleep(100);
        watchdog_feed();
    }
    return STATE_POLLING;
}

machine_state_t run_sbc_prep(void) {
    printk("--- PREPARING SBC HANDOFF ---\n");
    
    if (retained.psm_active) {
        printk("WARNING: PSM still marked active!\n");
        printk("Attempting emergency cleanup...\n");
        
        if (!disable_psm_and_edrx()) {
            printk("ERROR: Final PSM disable failed!\n");
            printk("SBC WILL INHERIT PSM SETTINGS!\n");
            printk("Recommend: SBC should run keepalive script.\n");
        }
    } else {
        printk("PSM confirmed disabled. Clean handoff.\n");
    }
    
    k_sleep(K_SECONDS(1));
    flag_sbc_handoff_pending = false;
    
    return STATE_SBC_OWNED;
}

machine_state_t run_sbc_owned(void) {
    printk("--- SBC OWNED (SYSTEM OFF) ---\n");
    printk("MCU entering deep sleep. Wake on handoff pin LOW.\n");
    
    nrf_gpio_cfg_sense_input(sbc_handoff.pin, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_LOW);
    gpio_pin_set_dt(&led0, 0);
    pm_device_action_run(uart0, PM_DEVICE_ACTION_SUSPEND);
    nrf_power_system_off(NRF_POWER);
    return STATE_RECOVERY; 
}

machine_state_t run_recovery(void) {
    printk("--- RECOVERY (SBC Released Control) ---\n");
    k_sleep(K_MSEC(1000));
    
    printk("Verifying modem state post-handoff...\n");
    return STATE_CHECK_MODEM;
}

void button_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    printk("Button pressed: Starting network\n");
    flag_start_network = true;
}

void sbc_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int pin_state = gpio_pin_get_dt(&sbc_handoff);
    
    if (pin_state == 1 && !flag_sbc_active && !flag_sbc_handoff_pending) {
        printk("\n!!! URGENT: SBC HANDOFF DETECTED !!!\n");
        
        if (current_state != STATE_SBC_OWNED && current_state != STATE_SBC_PREP) {
            printk("Emergency PSM cleanup in progress...\n");
            disable_psm_and_edrx();
        }
        
        flag_sbc_handoff_pending = true;
    } else if (pin_state == 0) {
        flag_sbc_active = false;
    }
}

// ---------------- MAIN ---------------- //

int main(void) {
    if (gpio_is_ready_dt(&trigger_pin)) gpio_pin_configure_dt(&trigger_pin, GPIO_OUTPUT_ACTIVE);
    if (gpio_is_ready_dt(&led0)) gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    if (gpio_is_ready_dt(&pwrkey_pin)) gpio_pin_configure_dt(&pwrkey_pin, GPIO_OUTPUT_INACTIVE);
    
    if (gpio_is_ready_dt(&button)) {
        gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
        static struct gpio_callback btn_cb;
        gpio_init_callback(&btn_cb, button_cb, BIT(button.pin));
        gpio_add_callback(button.port, &btn_cb);
    }
    
    if (gpio_is_ready_dt(&button3)) gpio_pin_configure_dt(&button3, GPIO_INPUT | GPIO_PULL_UP);
    
    if (gpio_is_ready_dt(&sbc_handoff)) {
        gpio_pin_configure_dt(&sbc_handoff, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&sbc_handoff, GPIO_INT_EDGE_BOTH);
        static struct gpio_callback sbc_data;
        gpio_init_callback(&sbc_data, sbc_cb, BIT(sbc_handoff.pin));
        gpio_add_callback(sbc_handoff.port, &sbc_data);
        
        if (gpio_pin_get_dt(&sbc_handoff) == 1) {
            printk("SBC already active on boot\n");
            flag_sbc_active = true;
        }
    }

    uart_callback_set(uart0, uart_cb, NULL);
    uart_rx_enable(uart0, rx_buf, RX_BUF_SIZE, RX_TIMEOUT_DELAY);

    printk("PSM: TAU=1h, Active=30s\n");

    while(1) {
        if (flag_sbc_handoff_pending && 
            current_state != STATE_SBC_OWNED && 
            current_state != STATE_SBC_PREP && 
            current_state != STATE_BOOT_WAIT) {
            printk(">>> Transitioning to SBC handoff\n");
            current_state = STATE_SBC_PREP;
        }

        switch(current_state) {
            case STATE_BOOT_WAIT:   current_state = run_boot_wait(); break;
            case STATE_IDLE:        current_state = run_idle(); break;
            case STATE_INIT:        current_state = run_init(); break;
            case STATE_CHECK_MODEM: current_state = run_check_modem(); break;
            case STATE_NET_SETUP:   current_state = run_net_setup(); break;
            case STATE_HTTPS_SETUP: current_state = run_https_setup(); break;
            case STATE_POLLING:     current_state = run_polling(); break;
            case STATE_SBC_PREP:    current_state = run_sbc_prep(); break;
            case STATE_SBC_OWNED:   current_state = run_sbc_owned(); break;
            case STATE_RECOVERY:    current_state = run_recovery(); break;
            case STATE_HARD_RESET:  current_state = run_hard_reset(); break;
        }
        
        if (!is_skip_wdt && wdt_channel_id == -1 && current_state != STATE_BOOT_WAIT) {
            watchdog_init();
        }
        
        watchdog_feed();
    }
    
    return 0;
}

static void watchdog_init(void) {
    if (is_skip_wdt) {
        printk("WDT DISABLED\n");
        return;
    }
    
    if (!device_is_ready(wdt)) {
        printk("ERROR: Watchdog not ready\n");
        return;
    }
    
    struct wdt_timeout_cfg wdt_config = {
        .window.min = 0U, 
        .window.max = 30000U,  // 30 second timeout
        .flags = WDT_FLAG_RESET_SOC, 
        .callback = NULL
    };
    
    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if (wdt_channel_id < 0) {
        printk("ERROR: Failed to install WDT timeout\n");
        return;
    }
    
    if (wdt_setup(wdt, 0) < 0) {
        printk("ERROR: Failed to setup WDT\n");
        return;
    }
    
    printk("Watchdog initialized: 30s timeout\n");
}

static void watchdog_feed(void) {
    if (!is_skip_wdt && wdt_channel_id >= 0 && device_is_ready(wdt)) {
        wdt_feed(wdt, wdt_channel_id);
    }
}