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
static struct gpio_callback wake_cb;
static struct k_work rock_pulse_work;

int turned_on = 0;
int counter = 0;

void send_message(void)
{
    //send message via uart
    const char *msg = "FENT!\n";
    uart_poll_out(uart0, '0' + counter);
    for (int i = 0; msg[i] != '\0'; i++) {
        uart_poll_out(uart0, msg[i]);
    }
    counter++;
    if (counter > 9) counter = 0;
    printk("Sent msg\n");
}

// A separate handler for triggering pin (rock sbc) due to ISR/Zepyhr not liking delays/sleeps in it
void rock_pulse_handler(struct k_work *work)
{
    gpio_pin_set_dt(&rock_pin, false);
    k_sleep(K_MSEC(100));
    gpio_pin_set_dt(&rock_pin, true);
    send_message();
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

void wake_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) //ISR
{
    printk("Voltage has been detected via wake pin!\n");
}

int initialize_pins(void){
    int ret;
    //initialize uart
    if (!device_is_ready(uart0)) {
        printk("UART device not found!\n");
        return ret;
    } //sample: uart_poll_out(uart0, 'message');

    // led1 setup
    if (!device_is_ready(led1.port)) {
        printk("LED device %s not ready\n", led1.port->name);
        return ret;
    }

    ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure LED: %d\n", ret);
        return ret;
    }

    // rock pin setup
    if (!device_is_ready(rock_pin.port)) {
        printk("Rockpin %s not ready\n", rock_pin.port->name);
        return ret;
    }

    ret = gpio_pin_configure_dt(&rock_pin, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure rockpin: %d\n", ret);
        return ret;
    }

    // modem pin setup
    if (!device_is_ready(modem_pin.port)) {
        printk("modem_pin %s not ready\n", modem_pin.port->name);
        return ret;
    }

    ret = gpio_pin_configure_dt(&modem_pin, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure modem_pin: %d\n", ret);
        return ret;
    }

    // wake pin setup
    ret = gpio_pin_configure_dt(&wake_pin, GPIO_INPUT);

    if (ret < 0) {
        printk("Failed to configure wake_pin: %d\n", ret);
        return ret;
    }

    gpio_init_callback(&wake_cb, wake_pressed, BIT(wake_pin.pin));
    ret = gpio_add_callback(wake_pin.port, &wake_cb);
    if (ret < 0) {
        printk("Failed to add callback: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&wake_pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure interrupt: %d\n", ret);
        return ret;
    }

    // button setup
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);

    if (ret < 0) {
        printk("Failed to configure button button: %d\n", ret);
        return ret;
    }

    gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
    ret = gpio_add_callback(button.port, &button_cb);
    if (ret < 0) {
        printk("Failed to add callback: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure interrupt: %d\n", ret);
        return ret;
    }
}

int main(void)
{
    if (initialize_pins() < 0) {
        printk("Failed to initialize pins\n");
        return 0;
    }


    //uart_poll_in(uart0, NULL);
    //uart_poll_out(uart0, 'message');

    k_work_init(&rock_pulse_work, rock_pulse_handler); //rock pulse initialization

    gpio_pin_set_dt(&led1, 1); 
    gpio_pin_set_dt(&rock_pin, 1);
    gpio_pin_set_dt(&modem_pin, 0); 
    turned_on = 1;                 

    return 0;
}


/*
dts (just in case):
under button child: 
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

added to alias (ofc):
		trigger0 = &trigger_pin;
		trigger1 = &trigger_pin2;
		wakepin = &wake_pin;

Kconfig (granted pwm is not needed):
CONFIG_STDOUT_CONSOLE=y
CONFIG_PRINTK=y
CONFIG_PWM=y
CONFIG_LOG=y
CONFIG_LOG_PRINTK=y
CONFIG_LOG_MODE_IMMEDIATE=y
CONFIG_PWM_LOG_LEVEL_DBG=y
CONFIG_PM=y
CONFIG_PM_DEVICE=y
CONFIG_SERIAL=y
CONFIG_UART_CONSOLE=y


*/