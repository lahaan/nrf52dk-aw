#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <hal/nrf_power.h>
#include <zephyr/drivers/uart.h>

//on board
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); // to know button changes

//external wirings (modem wake, rock sbc trigger, sbc interrupt to wake)
static const struct gpio_dt_spec rock_pin = GPIO_DT_SPEC_GET(DT_ALIAS(trigger0), gpios);  // [P0.11] interrupt sbc
static const struct gpio_dt_spec modem_pin = GPIO_DT_SPEC_GET(DT_ALIAS(trigger1), gpios); // [P0.12] wake modem
static const struct gpio_dt_spec wake_pin = GPIO_DT_SPEC_GET(DT_ALIAS(wakepin), gpios);   // [P0.28] interrupt from sbc

//IoT/modem related (external):
const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0)); // [P0.06 - TX] [P0.08 - RX]


#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

static struct gpio_callback button_cb;
static struct k_work rock_pulse_work;

int turned_on = 0;

/*

dts addon (under button child), LED working w this way:

	gpio_trigger {
		compatible = "gpio-leds";
		trigger_pin: trigger_0 {
			gpios = <&gpio0 21 GPIO_ACTIVE_HIGH>;
			label = "Wakeup Trigger Pin";
		};
	};
*/

// A separate handler for triggering pin (rock sbc) due to ISR/Zepyhr not liking delays/sleeps in it
void rock_pulse_handler(struct k_work *work)
{
    gpio_pin_set_dt(&rock_pin, true);
    k_sleep(K_MSEC(70));
    gpio_pin_set_dt(&rock_pin, false);
}

void go_to_sleep(void)
{
    printk("Going to system off...\n");
    k_sleep(K_MSEC(250));
    nrf_power_system_off(NRF_POWER); //deepsleep func
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) //ISR
{

    printk("STATE %s:\n", turned_on ? "HIGH (1)" : "LOW (0)");
    k_work_submit(&rock_pulse_work); //rock pulse
    gpio_pin_set_dt(&led1, turned_on);
    turned_on = !turned_on;

}

int main(void)
{
    int ret;

    //initialize uart
    if (!device_is_ready(uart0)) {
        printk("UART device not found!\n");
        return 0;
    }
    //sample: uart_poll_out(uart0, 'message');

    // led1 setup

    if (!device_is_ready(led1.port)) {
        printk("LED device %s not ready\n", led1.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure LED: %d\n", ret);
        return 0;
    }

    // rock pin setup

    if (!device_is_ready(rock_pin.port)) {
        printk("Rockpin %s not ready\n", led1.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&rock_pin, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure rockpin: %d\n", ret);
        return 0;
    }


    //gpio_pin_set_dt(&led1, 0);  // Off
    // button setup & interrupt

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);

    if (ret < 0) {
        printk("Failed to configure button button: %d\n", ret);
        return 0;
    }

    gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
    ret = gpio_add_callback(button.port, &button_cb);
    if (ret < 0) {
        printk(" Failed to add callback: %d\n", ret);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk(" Failed to configure interrupt: %d\n", ret);
        return 0;
    }

    k_work_init(&rock_pulse_work, rock_pulse_handler); //rock pulse initialization

    gpio_pin_set_dt(&led1, 1); 
    gpio_pin_set_dt(&rock_pin, 0); 
    turned_on = 1;                 

    return 0;
}