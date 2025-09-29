/* - successful connection v0.1 - HTTP */
/* note, SHREAD broken a. reply - 29/9 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h> // for atoi

// UART & Button
const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

#define RX_BUF_SIZE 128
static int32_t RX_TIMEOUT_DELAY = 200;
static uint8_t rx_buf[RX_BUF_SIZE];
static char response_buf[RX_BUF_SIZE];
static size_t resp_len = 0;
static bool response_complete = false;
static bool waiting_for_response = false;
static bool last_command_successful = false;
static int last_http_data_size = 0; // NEW: Store HTTP response data size

// Button work
static struct k_work button_work;
static void button_pressed(struct k_work *work);

// Forward decls
void send_at_command(const char *cmd);
void run_http_test(void);
void run_communication_test(void);

// UART callback - Updated to parse HTTP response data size
static void uart_evt_cb(const struct device *dev, struct uart_event *evt, struct uart_event *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    switch (evt->type) {
    case UART_RX_RDY: {
        const uint8_t *p = &evt->data.rx.buf[evt->data.rx.offset];
        size_t l = evt->data.rx.len;

        for (size_t i = 0; i < l; i++) {
            char c = p[i];

            if (c == '\r' || c == '\n') {
                if (resp_len > 0) {
                    response_buf[resp_len] = '\0';
                    printk("<<< %s\n", response_buf);

                    // Check for response indicators
                    if (strcmp(response_buf, "OK") == 0) {
                        response_complete = true;
                        waiting_for_response = false;
                        last_command_successful = true;
                    } else if (strcmp(response_buf, "ERROR") == 0 ||
                               strcmp(response_buf, "NO CARRIER") == 0 ||
                               strcmp(response_buf, "NO DIALTONE") == 0 ||
                               strcmp(response_buf, "NO ANSWER") == 0 ||
                               strcmp(response_buf, "NO") == 0 ||
                               strstr(response_buf, "+CME ERROR:") != NULL ||
                               strstr(response_buf, "+SHREQ:") != NULL && strstr(response_buf, "ERROR") != NULL) {
                        response_complete = true;
                        waiting_for_response = false;
                        last_command_successful = false;
                    } 
                    // NEW: Parse HTTP response data size from SHREQ response
                    else if (strstr(response_buf, "+SHREQ:") != NULL) {
                        // Example: +SHREQ:"POST",200,457
                        char *comma1 = strchr(response_buf, ','); // First comma after method
                        if (comma1) {
                            char *comma2 = strchr(comma1 + 1, ','); // Second comma before data size
                            if (comma2) {
                                // Extract the data size number after the second comma
                                char *data_size_str = comma2 + 1;
                                last_http_data_size = atoi(data_size_str);
                                printk("<<< Parsed HTTP response data size: %d\n", last_http_data_size);
                            }
                        }
                    }

                    resp_len = 0;
                }
            } else {
                if (resp_len < sizeof(response_buf) - 1) {
                    response_buf[resp_len++] = c;
                } else {
                    printk("<<< Buffer overflow, resetting buffer. Received char: '%c'\n", c);
                    resp_len = 0;
                }
            }
        }
        break;
    }
    case UART_RX_DISABLED:
        uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), RX_TIMEOUT_DELAY);
        break;

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
    uart_poll_out(uart0, '\n');
}

void wait_for_response(k_timeout_t timeout) {
    response_complete = false;
    waiting_for_response = true;
    last_http_data_size = 0; // Reset HTTP data size when waiting for new response

    printk("Waiting for response for up to %d seconds...\n", timeout.ticks);

    int64_t start = k_uptime_get();
    while (waiting_for_response && (k_uptime_get() - start) < timeout.ticks * 1000) {
        k_msleep(10);
    }

    if (waiting_for_response) {
        printk("<<< (timeout waiting for response)\n");
        waiting_for_response = false;
        last_command_successful = false;
    } else {
        printk("<<< Response received within timeout.\n");
    }

    if (resp_len > 0) {
        response_buf[resp_len] = '\0';
        printk("<<< Leftover buffer after wait: '%s'\n", response_buf);
        resp_len = 0;
    }
}

static void button_pressed(struct k_work *work)
{
    ARG_UNUSED(work);
    printk("\n--- BUTTON PRESSED: Running Communication Test ---\n");
    run_communication_test();
}

void run_communication_test(void)
{
    printk("--- Starting Communication Test ---\n");

    send_at_command("AT");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("ATI");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CGMR");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CGMI");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("ATE1");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("ATE0");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    printk("--- Communication Test Complete ---\n");
    printk("--- Now Running HTTP Test ---\n");
    run_http_test();
}

void run_http_test(void)
{
    printk("--- Starting HTTP Test ---\n");

    send_at_command("AT");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("ATE0");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CPIN?");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CGREG=1");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CGREG?");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CGATT?");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CSQ");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CPSI?");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"");
    wait_for_response(K_SECONDS(3));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CNACT=0,1");
    wait_for_response(K_SECONDS(10));
    k_sleep(K_SECONDS(2));

    send_at_command("AT+CNACT?");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+CGREG?");
    wait_for_response(K_SECONDS(2));
    k_sleep(K_SECONDS(1));

    send_at_command("AT+SHCONF=\"URL\",\"http://httpbin.org\"");
    wait_for_response(K_SECONDS(2));

    send_at_command("AT+SHCONF=\"BODYLEN\",1024");
    wait_for_response(K_SECONDS(2));

    send_at_command("AT+SHCONF=\"HEADERLEN\",350");
    wait_for_response(K_SECONDS(2));

    printk("Waiting for network to stabilize...\n");
    for (int j = 0; j < 30; j++){
        printk(".");
        k_sleep(K_SECONDS(1));
    }
    printk("\n");

    send_at_command("AT+CGATT?");
    wait_for_response(K_SECONDS(2));
    send_at_command("AT+CNACT?");
    wait_for_response(K_SECONDS(2));

    send_at_command("AT+SHCONN");
    wait_for_response(K_SECONDS(20));
    k_sleep(K_SECONDS(2));

    send_at_command("AT+SHSTATE?");
    wait_for_response(K_SECONDS(2));

    if (last_command_successful) {
        printk("AT+SHCONN successful, proceeding with HTTP request.\n");
        
        send_at_command("AT+SHCHEAD");
        wait_for_response(K_SECONDS(2));

        send_at_command("AT+SHAHEAD=\"User-Agent\",\"nRF52-SIM7080\"");
        wait_for_response(K_SECONDS(2));

        send_at_command("AT+SHAHEAD=\"Cache-control\",\"no-cache\"");
        wait_for_response(K_SECONDS(2));

        send_at_command("AT+SHAHEAD=\"Connection\",\"keep-alive\"");
        wait_for_response(K_SECONDS(2));

        send_at_command("AT+SHAHEAD=\"Accept\",\"*/*\"");
        wait_for_response(K_SECONDS(2));

        send_at_command("AT+SHAHEAD=\"Content-Type\",\"application/json\"");
        wait_for_response(K_SECONDS(2));

        const char *post_data = "{\"msg\":\"Hello from nRF52 via SIM7080!\"}";
        char shbod_cmd[50];
        snprintf(shbod_cmd, sizeof(shbod_cmd), "AT+SHBOD=%d,5000", strlen(post_data));
        send_at_command(shbod_cmd);
        wait_for_response(K_SECONDS(3));

        if (last_command_successful) {
            for (int i = 0; post_data[i] != '\0'; i++) {
                uart_poll_out(uart0, post_data[i]);
            }
            uart_poll_out(uart0, 0x1A);
            printk(">>> Sent HTTP body + CTRL+Z\n");

            send_at_command("AT+SHREQ=\"/post\",3");
            wait_for_response(K_SECONDS(20)); // This will parse the data size
            
            // NEW: Read the response using the actual data size returned by SHREQ
            if (last_command_successful && last_http_data_size > 0) {
                char shread_cmd[30];
                snprintf(shread_cmd, sizeof(shread_cmd), "AT+SHREAD=0,%d", last_http_data_size);
                send_at_command(shread_cmd);
                wait_for_response(K_SECONDS(5));
            } else if (last_command_successful) {
                // Fallback if no size was parsed
                send_at_command("AT+SHREAD=0,500");
                wait_for_response(K_SECONDS(5));
            }
        }

        send_at_command("AT+SHDISC");
        wait_for_response(K_SECONDS(5));
    } else {
        printk("AT+SHCONN failed (last_command_successful=%d), skipping subsequent HTTP commands.\n", last_command_successful);
        
        printk("Checking network status again...\n");
        send_at_command("AT+CGREG?");
        wait_for_response(K_SECONDS(2));
        send_at_command("AT+CGATT?");
        wait_for_response(K_SECONDS(2));
        send_at_command("AT+CNACT?");
        wait_for_response(K_SECONDS(2));
        send_at_command("AT+SHSTATE?");
        wait_for_response(K_SECONDS(2));
    }

    printk("--- HTTP test complete ---\n");
}

static void button_cb(const struct device *dev, struct gpio_callback *cb,
                      uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_work_submit(&button_work);
}

int main(void)
{
    printk("Starting application...\n");

    if (!device_is_ready(uart0)) {
        printk("UART not ready!\n");
        return -1;
    }
    printk("UART device found: %s\n", uart0->name);

    uart_callback_set(uart0, uart_evt_cb, NULL);
    uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), 50);

    if (!device_is_ready(button.port)) {
        printk("Button device not ready\n");
    } else {
        gpio_pin_configure_dt(&button, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
        static struct gpio_callback button_cb_data;
        gpio_init_callback(&button_cb_data, button_cb, BIT(button.pin));
        gpio_add_callback(button.port, &button_cb_data);
        k_work_init(&button_work, button_pressed);
        printk("Press button (SW0) to run Communication Test\n");
    }

    printk("Ready. Monitoring UART...\n");
    while (1) {
        k_msleep(1000);
    }
}

/*

CONFIG_UART_CONSOLE=n
CONFIG_USE_SEGGER_RTT=y
CONFIG_RTT_CONSOLE=y
CONFIG_LOG=y
CONFIG_SERIAL=y
CONFIG_UART_ASYNC_API=y


# NO NETWORKING - just UART
# CONFIG_NETWORKING=n
# CONFIG_MODEM=n
# CONFIG_MODEM_SIM7080=n

# Minimal memory usage:
CONFIG_HEAP_MEM_POOL_SIZE=1024
CONFIG_MAIN_STACK_SIZE=1024

*/