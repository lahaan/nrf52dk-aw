#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

/* Button 1 (SW1) */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

/* LED0 for feedback */
static const struct gpio_dt_spec rock_pin = GPIO_DT_SPEC_GET(DT_ALIAS(trigger0), gpios);


static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);



#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

static struct gpio_callback button_cb;

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

//void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
/*{
    printk(" Button was pressed!\n");

    // Turn on LED0
    gpio_pin_set_dt(&led1, turned_on);
    
    // Turn on ROCK ;)
    gpio_pin_set_dt(&rock_pin, turned_on);
    
    turned_on = !turned_on;
}*/

int main(void)
{
    int ret;

    /*ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret < 0) {
        printk(" Failed to configure button pin: %d\n", ret);
        return 0;
    }

    if (!device_is_ready(led1.port)) {
        printk(" LED device %s not ready\n", led1.port->name);
        return 0;
    }*/

    if (!device_is_ready(rock_pin.port)) {
        printk("Rockpin %s not ready\n", led1.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&rock_pin, GPIO_OUTPUT);
    if (ret < 0) {
        printk(" Failed to configure rockpin: %d\n", ret);
        return 0;
    }

    /*ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
    if (ret < 0) {
        printk(" Failed to configure LED: %d\n", ret);
        return 0;
    }*/

    //gpio_pin_set_dt(&led1, 0);  // Off
    gpio_pin_set_dt(&rock_pin, 1);  // on

    // === 3. Setup Interrupt ===
    /*gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
    ret = gpio_add_callback(button.port, &button_cb);
    if (ret < 0) {
        printk(" Failed to add callback: %d\n", ret);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk(" Failed to configure interrupt: %d\n", ret);
        return 0;
    }*/


    return 0;
}
