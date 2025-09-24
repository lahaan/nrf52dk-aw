#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <hal/nrf_power.h>
#include <zephyr/drivers/uart.h>

//networking
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>


#include <ctype.h>



//on board
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); // to know button changes

//external wirings (modem wake, rock sbc trigger, sbc interrupt to wake)
static const struct gpio_dt_spec rock_pin = GPIO_DT_SPEC_GET(DT_ALIAS(trigger0), gpios);  // [P0.11] interrupt sbc
//static const struct gpio_dt_spec modem_pin = GPIO_DT_SPEC_GET(DT_ALIAS(trigger1), gpios); // [P0.12] wake modem [REDACTED]
static const struct gpio_dt_spec wake_pin = GPIO_DT_SPEC_GET(DT_ALIAS(wakepin), gpios);   // [P0.28] interrupt from sbc
//static const struct gpio_dt_spec mdm_pn = GPIO_DT_SPEC_GET(DT_ALIAS(modem), mdm-power-gpios); //Zmodem power pin [P0.02]

//IoT/modem related (external):
const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0)); // [P0.06 - TX] [P0.08 - RX]

#define RX_BUF_SIZE 64
static uint8_t rx_buf[RX_BUF_SIZE];

#define CONNECT_TIMEOUT_MS 15000

static struct net_mgmt_event_callback if_cb;
static volatile bool if_up = false;

static struct k_work_delayable led1_off_work;

static struct gpio_callback button_cb;
static struct gpio_callback wake_cb;
static struct k_work rock_pulse_work;

static struct gpio_callback button2_cb;
static struct gpio_callback button3_cb;
static struct k_work button2_work;
static struct k_work button3_work;

int turned_on = 0;
int counter = 0;


static void iface_event_handler(struct net_mgmt_event_callback *cb,
                                uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_IF_UP || mgmt_event == NET_EVENT_L4_CONNECTED) {
        if_up = true;
    }
}

static int wait_net_up(int timeout_ms)
{
    net_mgmt_init_event_callback(&if_cb, iface_event_handler,
                                 NET_EVENT_IF_UP | NET_EVENT_L4_CONNECTED);
    net_mgmt_add_event_callback(&if_cb);

    int waited = 0;
    while (!if_up && waited < timeout_ms) {
        k_msleep(100);
        waited += 100;
    }
    net_mgmt_del_event_callback(&if_cb);
    return if_up ? 0 : -ETIMEDOUT;
}

static int http_smoke_test(void)
{
    int rc = wait_net_up(CONNECT_TIMEOUT_MS);
    if (rc) {
        printk("Network did not come up (%d)\n", rc);
        return rc;
    }
    printk("Network is up. Trying TCP to example.com:80\n");

    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) {
        printk("socket() failed: %d\n", errno);
        return -errno;
    }

    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port   = htons(80),
        .sin_addr   = { .s_addr = htonl(0x5DB8D822) } // 93.184.216.34
    };

    rc = connect(s, (struct sockaddr *)&dst, sizeof(dst));
    if (rc < 0) {
        printk("connect() failed: %d\n", errno);
        close(s);
        return -errno;
    }

    const char req[] =
        "GET / HTTP/1.0\r\nHost: example.com\r\nUser-Agent: zephyr-test\r\n\r\n";
    rc = send(s, req, sizeof(req) - 1, 0);
    if (rc < 0) {
        printk("send() failed: %d\n", errno);
        close(s);
        return -errno;
    }

    char buf[256];
    rc = recv(s, buf, sizeof(buf) - 1, 0);
    if (rc > 0) {
        buf[rc] = '\0';
        printk("HTTP response (%d bytes):\n%.*s\n", rc, rc, buf);
    } else {
        printk("recv() returned %d (errno=%d)\n", rc, errno);
    }

    close(s);
    return 0;
}


static void led1_off_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    gpio_pin_set_dt(&led1, 0);
}


static void uart_evt_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    switch (evt->type) {
    case UART_RX_RDY: {
        /* Grab the new bytes */
        const uint8_t *p = &evt->data.rx.buf[evt->data.rx.offset];
        size_t l = evt->data.rx.len;

        // blink led1 when rx arrives
        gpio_pin_set_dt(&led1, 1);
        k_work_reschedule(&led1_off_work, K_MSEC(50));

        printk("|\n");
        break;
    }

    case UART_RX_BUF_REQUEST:
        /* OK to ignore in this simple single-buffer setup */
        break;

    case UART_RX_DISABLED:
        /* Re-enable if buffer filled */
        uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), 50);
        break;

    case UART_RX_STOPPED:
        /* Recover on error/stop */
        uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), 50);
        break;

    default:
        break;
    }
}

void sw2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("BUTTON 2 pressed → trying network test\n");
    k_work_submit(&button2_work);
}

void sw3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("BUTTON 3 pressed → network OFF\n");
    k_work_submit(&button3_work);
}

void sw2_work_handler(struct k_work *work)
{
    printk("Bringing up network...\n");
    struct net_if *iface = net_if_get_default();
    net_if_up(iface);
    http_smoke_test();
}

void sw3_work_handler(struct k_work *work)
{
    printk("Bringing network down...\n");
    struct net_if *iface = net_if_get_default();
    net_if_down(iface);
}


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
        return -ENODEV;
    } //sample: uart_poll_out(uart0, 'message');

    // led1 setup
    if (!device_is_ready(led1.port)) {
        printk("LED device %s not ready\n", led1.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure LED: %d\n", ret);
        return ret;
    }

    // rock pin setup
    if (!device_is_ready(rock_pin.port)) {
        printk("Rockpin %s not ready\n", rock_pin.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&rock_pin, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure rockpin: %d\n", ret);
        return ret;
    }

    // modem pin setup
    /*if (!device_is_ready(modem_pin.port)) {
        printk("modem_pin %s not ready\n", modem_pin.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&modem_pin, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Failed to configure modem_pin: %d\n", ret);
        return ret;
    }*/

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

    //button2 setup
    ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);

    if (ret < 0) {
        printk("Failed to configure button2: %d\n", ret);
        return ret;
    }

    gpio_init_callback(&button2_cb, sw2_pressed, BIT(button2.pin));
    ret = gpio_add_callback(button2.port, &button2_cb);
    if (ret < 0) {
        printk("Failed to add callback: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure interrupt: %d\n", ret);
        return ret;
    }

    //button3 setup
    ret = gpio_pin_configure_dt(&button3, GPIO_INPUT);     
    if (ret < 0) {
        printk("Failed to configure button3: %d\n", ret);
        return ret;
    }
    gpio_init_callback(&button3_cb, sw3_pressed, BIT(button3.pin));
    ret = gpio_add_callback(button3.port, &button3_cb);  
    if (ret < 0) {
        printk("Failed to add callback: %d\n", ret);
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {  
        printk("Failed to configure interrupt: %d\n", ret);
        return ret;
    }
}

int main(void)
{
    if (initialize_pins() < 0) {
        printk("Failed to initialize pins\n");
        // Don't return! Go to sleep or loop forever.
        while (1) {
            k_sleep(K_FOREVER);
        }
    }

    k_work_init_delayable(&led1_off_work, led1_off_work_handler);
    uart_callback_set(uart0, uart_evt_cb, NULL);
    uart_rx_enable(uart0, rx_buf, sizeof(rx_buf), 50);

    k_work_init(&rock_pulse_work, rock_pulse_handler);
    k_work_init(&button2_work, sw2_work_handler);
    k_work_init(&button3_work, sw3_work_handler);

    gpio_pin_set_dt(&led1, 1); 
    gpio_pin_set_dt(&rock_pin, 1);
    turned_on = 1;

    while (1) {
        k_sleep(K_FOREVER);  // or do other background tasks
    }
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
	};

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

/*
    MODEM CONF:
    

CONFIG_UART_CONSOLE=n
CONFIG_USE_SEGGER_RTT=y
CONFIG_RTT_CONSOLE=y
CONFIG_LOG=y
CONFIG_LOG_MODE_IMMEDIATE=y

# UART + SIM7080 offloaded sockets driver
CONFIG_SERIAL=y
CONFIG_UART_ASYNC_API=y

#modem
CONFIG_MODEM=y
CONFIG_MODEM_SIM7080=y

CONFIG_NETWORKING=y
CONFIG_NET_SOCKETS=y
CONFIG_NET_SOCKETS_CONNECT_TIMEOUT=15000
CONFIG_MODEM_SIMCOM_SIM7080_APN="internet.telia.ee"
#ip
CONFIG_NET_IPV4=y
CONFIG_NET_IPV6=n
CONFIG_NET_TCP=y
CONFIG_NET_UDP=n
#BUFsizes
CONFIG_NET_PKT_RX_COUNT=2
CONFIG_NET_PKT_TX_COUNT=2
CONFIG_NET_BUF_RX_COUNT=8
CONFIG_NET_BUF_TX_COUNT=8
CONFIG_NET_BUF_DATA_SIZE=128

#misc build/comp bugfixes 
CONFIG_HEAP_MEM_POOL_SIZE=4096
CONFIG_MAIN_STACK_SIZE=4096
CONFIG_NET_NATIVE=n
CONFIG_NET_SOCKETS_OFFLOAD=y
CONFIG_POSIX_API=y

dts:

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
    &rng {
	status = "okay";
};



*/