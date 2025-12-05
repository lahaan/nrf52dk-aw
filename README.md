# nrf52-dk-aw



To develop / get started with an nRF52832DK, I recommend just following official Nordic docs -

meaning nRF Connect, Visual Studio + Nordic SDK.

For USB serial connection (Segger), legacy USB drivers are needed, so in terminal / command

prompt, for example: ```JLink\_Windows\_V864\_x864\_64.exe -InstUSBDriver=```

When developing, I used v3.1.0 SDK/toolchain and Zephyr 4.1.99. For logging, RTT viewer.

Zephyr itself is an RTOS (Real-Time Operating System) that makes it easier to develop hardware

on the Nordic series of MCUs. Similar to the SBC, a device tree has to be written here as well -

and a configuration file.



# Firmware features

Since the MCU has a lot of functionality this point will explain most of the features in it.

- GPIOs - Also known as General Purpose Input Output, these are the general pins to

communicate HIGH or LOW (0/1) states between MCU/SBC or to boot a modem up for

example. Since the SBC will wake up the MCU from sleep, one GPIO will be configured

as a wake source in the device tree.

- UART - Also known as Universal Asynchronous Receival-Transmitter. For general

communication with the modem - set to 921600 baud rate.

- WDT - Watchdog Timer. A timer in the MCU, that detects if the MCU is in a stuck state

or not. If so, it can force the MCU to do an operation (restart).

- RTT - Enabling the use of SWD console logging instead of UART logging, as UART

logging would: 1) prevent sleep, 2) UART is already in use (for the modem)

- PM - Power Management. Enables deep sleep functionality.

- PSM - Power Saving Mode. Ability to set the modem to deep sleep. Directly not an

nRF/MCU/Zephyr feature, but a modem feature.

- “Raw” networking - nRF52832 MCU lacks the RAM to run Zephyr’s official SIM

networking stack, so this had to be built from the ground up.


# Firmware versions overview

- (NA) S2 (Sept 2) - GPIO
     - Testing GPIO, changing .dts. Getting Zephyr RTOS working, waking SBC
- (NA) S3 (Sept 3) - UART\&CONFIG
     - Customizing dts, getting UART, wake handler
- (NA) S4 (Sept 22) - TXRX
     - Testing TXRX UART (working)
- (NA) S5 (Sept 24) - Networking broken
     - Zephyr SIM7080 STACK networking (not enough RAM)
- (NA) S6 (Sept 29) - RAW AT HTTP POST w webhook
     - Using RAW AT commands instead due to hardware limitations
- (NA) O1 (Oct 1) - HTTP GET broken
     - RAW AT command GET
- (NA) O2 (Oct 2) - HTTPS GET broken
     - Changing to HTTPS, need certs
- (NA) O3 (Oct 5) - HTTPS POLLING, Working CERT broken
     - Working cert sending to modem; changing to custom js backend server
- (NA) O4 (Oct 6) - HTTPS POLLING, SHCONN WORKING
     - Almost working, need to build around it.
- V1.0-081025 - HTTPS POLLING connection working
     - LEDs, status, GPIOs working over HTTPS Polling
- V1.1-201025
     - Additional handoff logic, PSM \& eDRX functions unused. (untested)
- V1.1-221025
     - Effectively identical to 201025. Unsuccessful. ISR overload.
- V1.1-241025a
     - Fully stable handoff (MCU-SBC-MCU). No client interrupt (SBC-side) yet (button), tested via systemctl suspend.
- V1.1-241025b
     - Built upon 241025a, but added client interrupt logic. Handoff (normal) somewhat stable, button handoff less-so. Overall unstable. One successful demo.
- V1.1-241025a-R
     - Refactored 241025a, as it is reliable and working. Untested/broken.
- V1.1-241025a-R2
     - Bugfixed, tested \& stable 241025a-R.
- V1.1-221025
     - Merged 241025a-R2 \& 241025b. Tested to be stable (enough, albeit encountered cloudfare block). First actual “early-PoC/demo-ready/system” firmware version!
- V1.1-271025a
     - Additional PWRKEY functionality (modem resets, etc pwrkey-related functionality) \& logging. Untested.
- V1.1-291025
     - Watchdog functionality added, MCU reset functionality in a case of stuck state
- V1.2-311025
     - WDT (Watchdog Timer) + PWRKEY (Modem) + Logging (Stats) functionality, untested
- V1.2-031125
     - Tested WDT+PWRKEY+LOGGIN [no-SBC-side conn], reliable, but slow.
- V1.2-031125a
     - Optimizations (bring-up from WDT / recovery), on-board debugger for wdt (dynamic WDT timer) - UNTESTED REDACTED
- V1.2-041125
     - 031125 fixed - dynamic wdt is not possible unless using software-based wdt and due to potential limitations of software wdt with sleep states \& reliability, opted for on-boot enable/disable wdt button instead.
- V1.2-041125a 
     - bugfix: handoff logic was broken (SBC->MCU) - wdt timeout
- V1.2-051125-LPE-b
     - Trying on sleep states, however it doesn’t work, as the handoff is broken.
- V1.2-071125LPE
     - Non-working version. Half-finished (built upon 041125)
- V1.3-121125LPE - LAST LEGACY ISR VERSION
     - 121125LPE-s1: Sleep working, ram retention, stats, button sleep (manual) working. Testing w/ SBC not tested.
- 121125LPE-s2: Fixed to work with SBC, tested.
- 121125LPE-s3: Optimized version of s2, however, handoff from SBC causes issues (garbage data after SHREQ). The issue is likely: too optimistic, fast re-uptake of networking/modem. K-work of sbc-handoff-handler is redundant legacy code now.
- FSM-v1.0a (20.11.25)
    - Tested version (basic), equivalent-ish to 271025a feature-wise. No sleep implemented yet. WDT doesn’t work.

- FSM-v2.0LPE (21.11.25)

     - Tested. Occasionally dies during polling. WDT doesn’t work properly. Sleep

works, stable up to 2 hours. Feature wise comparable to late-v1.2/v1.3.

- FSM-v2.1LPE (21.11.25)
     - Handoff, true deep sleep, polling, pwrkey works, basic fallback \& wdt, stable, ux logging. Tested. (wdt untested, shconn hang fix unconfirmed, 2h+ stable). Superfast setup, much faster recovery relative to v1.3 (ISR arch).
- FSM-v2.2LPE (24.11.25)
     - Addition of PSM.
- FSM-v2.2b (25.11.25)
     - PSM added, it “works”, however, timing has to be figured out for polling (HTTPS) and handoff for SBC - since even if you reboot the modem, the PSM settings persist. This either has to be a co-operated (SBC+MCU) handoff sequence where MCU reboots the modem whilst SBC sends commands to stop PSM; or add PWRKEY rights to SBC (unnecessary logic). Another fix could be a polling script for the SBC to keep the ppp connection active.
- FSM-v2.2c (04.11.25)
     - Experimental changes to potentially try and fix it. Have not fully tested it (no time). o7.

**For stable versions, refer to:**
- V1.1-241025a / V1.1-241025a-R
- V1.1-
- V1.2-041125a
- FSM-v1.0a
- FSM-v2.1LPE

# FSM Version 2.1 Low-Power Experimental

Here I will go a little more into detail how the last/latest stable version (FSM-v2.1LPE) works.

How the code works on the surface and on a deeper level and importantly, how it communicates

with the modem.



# Surface level understanding and relative changes to ISR-based architecture.

There were two main architectures that the firmware was based upon:



**ISR/IRQ + boolean hybrid architecture** - This was what I used from beginning,

effectively how this works, is that whenever something interrupts or an event happens or

during operation some state changes (hence boolean/FSM) - the firmware will know what

to do. ISR refers to Interrupt Service Routine: say as soon as some pin goes high, or some

tick happens -> a function is executed. But due to the complexity and size of the project

in the later stages, this type of code ended up being extremely confusing and difficult to

work with, hence the abandonment.

**FSM (Finite State Machine)** - Because the firmware was ever-so more leaning towards

booleans and states, making it impossible to graph it on a flowchart (but logically

possible with an FSM diagram) - the code was refactored to FSM architecture. FSM has

set states and the system can be in one of the finite states. Transitions occurring based on

inputs/events. This not only greatly made the code more readable but also simplified it

and removed nearly ⅔ lines of code.



# Initialization

This is the phase where all the pins, controllers etc are initialized. This code also has an optional

3-second phase where one can disable WDT during the boot by pressing a button (button3).

Whenever booting up, the firmware also checks the boot reasoning, to know what state to set

itself into:
```c
machine_state_t run_init(void) {
    // Read reset reason register directly from hardware
    uint32_t rr = nrf_power_resetreas_get(NRF_POWER);
    nrf_power_resetreas_clear(NRF_POWER, 0xFFFFFFFF);
    
    // Logic branching based on wake source
    if (rr & NRF_POWER_RESETREAS_OFF_MASK) {
        printk("Wake from Sleep -> RECOVERY\n");
        return STATE_RECOVERY;
    }
    if (rr & NRF_POWER_RESETREAS_DOG_MASK) {
        printk("WDT Reset -> CHECK MODEM\n");
        // If we crashed, skip idle and try to reconnect immediately
        return STATE_CHECK_MODEM; 
    }
    return STATE_IDLE;
}
```



# Networking

The networking layer manages the SIM7080G by sending AT commands over UART. As said

before, using the Zephyr networking stack was not an option with the nRF52832. The key steps

are to check if the modem is responsive, has connection and set up an APN.

Since the modem is quick-to-respond and it is preferred to get things up-and-running fast, a

“fail-fast” philosophy is preferred here. Here’s a snippet:
```c
machine_state_t run_net_setup(void) {
    // APN Configuration 
    send_at("AT+CGDCONT=1,\"IP\",\"internet.telia.ee\"", K_MSEC(500));
    
    // Activate Network Context
    if (!send_at("AT+CNACT=0,1", K_SECONDS(10))) {
        // Check if already active to prevent false negatives
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
```

# Server communication

Communication is via HTTPS polling. TLS/SSL security, certificate loading is handled by

HTTPS\_SETUP \& POLLING states (Diagram 3.). To keep things simple for now, the certificate

itself is hardcoded (GTS4 CA v1).

CA certificate is first uploaded and then the SSL context is configured. After SHCONN (a very

transient spike inducing step) passes - a command to establish HTTPS connection, the polling

can commence, periodically sending GET requests and executing commands on hardware

(sending ACK, acknowledgement, if done so). The payloads are JSON.
```c
machine_state_t run_polling(void) {
    // Construct dynamic request URL
    char req[64]; 
    snprintf(req, sizeof(req), "AT+SHREQ=\"/api/poll/%s\",1", DEVICE_ID);
    if (send_at(req, K_SECONDS(2))) {
        // Wait for asynchronous +SHREQ URC (Result Code)
        // wait logic    
        if (http_state.last_status_code == 200) {
             // Retrieve body content
             char read[32]; 
             snprintf(read, sizeof(read), "AT+SHREAD=0,%d",      http_state.last_data_size);
             send_at(read, K_SECONDS(2));          
             if (is_cmd_received) {
                 execute_command(command_data);
                 // Send ACK
             }
        }
    }
    return STATE_POLLING;
}
```



# Handoff logic

Whenever the SBC’s pin is found to be HIGH, the MCU will know to wrap things up, and go to

sleep and vice versa - if it is asleep, it will be sensing the pin and if it is set to low, it will wake

up and change state accordingly.
```c
machine_state_t run_sbc_owned(void) {
    printk("--- SBC OWNED (SYSTEM OFF) ---\n");
    // Clean disconnect to free resources
    send_at("AT+SHDISC", K_MSEC(500));
    
    // Configure wake-up trigger for return flow
    nrf_gpio_cfg_sense_input(sbc_handoff.pin, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_LOW);
    
    // Suspend peripherals and enter Deep Sleep
    pm_device_action_run(uart0, PM_DEVICE_ACTION_SUSPEND);
    nrf_power_system_off(NRF_POWER);
    
    return STATE_RECOVERY; // Unreachable code unless sleep fails
}
```

As detailed in the comments, nrf\_power\_system\_off(NRF\_POWER) enters the MCU into a deep

sleep state with the only active part being specifically retained RAM and GPIO (for wake).


# Recovery and watchdog

Systems always can fail, so recovery is important.



WDT (watchdog timer) itself is set to 30-seconds, ensuring full restart, if all goes

critically wrong.

Soft Recovery: The modem stops responding, for example. System transitions to

CHECK\_MODEM state, to attempt AT renegotiation.

Hard Reset: If the modem is unresponsive for more than 3 times, the system will enter a

HARD\_RESET state, which will start toggling the modem’s PWRKEY.
```c
machine_state_t run_hard_reset(void) {
    // Check if we have tried hard resetting too many times
    if (hard_reset_count >= MAX_HARD_RESETS) {
        printk("CRITICAL FAILURE -> SYSTEM REBOOT\n");
        NVIC_SystemReset(); // Reset the MCU itself
    }

    // Physical toggle of the modem power pin
    gpio_pin_set_dt(&pwrkey_pin, 1);
    k_sleep(K_MSEC(PWRKEY_PRESS_MS));
    gpio_pin_set_dt(&pwrkey_pin, 0);
    
    // Wait for modem boot sequence
    for(int i=0; i<MODEM_BOOT_WAIT_SEC; i++) {
        k_sleep(K_SECONDS(1));
        watchdog_feed();
    }
    
    hard_reset_count++; 
    return STATE_NET_SETUP;
}
```

# Low Power
```c
for(int i=0; i<POLL_INTERVAL_SEC * 10; i++) {
    if (flag_sbc_active) return STATE_SBC_OWNED;
    
    // Sleep for 100ms to allow CPU to idle
    k_msleep(100);
    watchdog_feed();
}
```



# Server

This is something basic built for the MCU to connect to with the modem. It was built with JS \&

CSS and is hosted on a third-party website (render.com).



# Future

PSM (Power Saving Mode) is a critical low-power feature of the modem that needs to be added.

Currently due to lack of time this has not been fully implemented, however, it is something that

needs care and thought. There are several solutions.



Currently the system is built with HTTPS polling in mind. But in theory, a hybrid, for example, a

system that can do both MQTT and HTTPS simultaneously could be optimal, at least for power.

Ability to connect to different servers / automatically find the correct server. This would mean

multiple URLs or a different type of system for networking but would be necessary for an

end-product.

Running actual client software on the SBC and testing its functionality

