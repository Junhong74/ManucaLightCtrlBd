#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>        
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include "led_ctrl.h"

#define LOG_LEVEL CONFIG_LED_CTRL_LOG_LEVEL
LOG_MODULE_REGISTER(led_ctrl);

#define SHORT_BLINK_MS     3800
#define LONG_BLINK_MS      (0.25 * 60 * 1000)   /* 15 seconds */
#define BLINK_PERIOD_MS    333 // 1.5 Hz

struct led_spec {
    const struct gpio_dt_spec *led;
    struct k_timer blink_timer;
    struct k_timer duration_timer;
    bool blinking;
    bool led_on;
};

static struct gpio_dt_spec led_left_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_1), gpios, {0});
static struct gpio_dt_spec led_right_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_2), gpios, {0});

// LED states
static struct led_spec left_led = {
    .led = &led_left_spec,
    .blinking = false,
    .led_on = false,
};
static struct led_spec right_led = {
    .led = &led_right_spec,
    .blinking = false,
    .led_on = false,
};

led_mode_t led_mode;

/**
 * @brief Blink timer callback function
 *
 * This function is called when the blink timer expires.
 *
 * @param timer_id The timer ID
 */
static void blink_timer_fn(struct k_timer *timer_id)
{
    struct led_spec *ld = CONTAINER_OF(timer_id, struct led_spec, blink_timer);
    ld->led_on = !ld->led_on;
    gpio_pin_set_dt(ld->led, ld->led_on);
}

/**
 * @brief Duration timer callback function
 *
 * This function is called when the duration timer expires.
 *
 * @param timer_id The timer ID 
 */
static void duration_timer_fn(struct k_timer *timer_id)
{
    struct led_spec *ld = CONTAINER_OF(timer_id, struct led_spec, duration_timer);
    k_timer_stop(&ld->blink_timer);
    ld->blinking = false;
    gpio_pin_set_dt(ld->led, 0); // Turn off LED
}

/**
 * @brief Control the left LED
 *
 * This function controls the state of the left LED.
 *
 * @param event The blink event
 * @return 0 on success, or a negative error code on failure
 */
static int8_t led_ctrl_left(led_event_t event)
{
    if (event == OFF) {
        k_timer_stop(&left_led.blink_timer);
        k_timer_stop(&left_led.duration_timer);
        left_led.blinking = false;
        left_led.led_on = false;
        gpio_pin_set_dt(left_led.led, 0); // Turn off LED
        LOG_INF("Left LED OFF");
        return 0; // Success
    }
    else if (event == ON) {
        left_led.blinking = true;
        left_led.led_on = true;
        gpio_pin_set_dt(left_led.led, 1); // Turn on LED
        k_timer_start(&left_led.blink_timer, K_MSEC(BLINK_PERIOD_MS), K_MSEC(BLINK_PERIOD_MS));
        k_timer_stop(&left_led.duration_timer);
        LOG_INF("Left LED started");
        return 0; // Success
    }   
    else if (event == SHORT) {
        left_led.blinking = true;
        left_led.led_on = true;
        gpio_pin_set_dt(left_led.led, 1); // Turn on LED
        k_timer_start(&left_led.blink_timer, K_MSEC(BLINK_PERIOD_MS), K_MSEC(BLINK_PERIOD_MS));
        k_timer_start(&left_led.duration_timer, K_MSEC(SHORT_BLINK_MS), K_NO_WAIT);
        LOG_INF("Left LED - short duration");
        return 0; // Success
    }
    else if (event == LONG) {
        left_led.blinking = true;
        left_led.led_on = true;
        gpio_pin_set_dt(left_led.led, 1); // Turn on LED
        k_timer_start(&left_led.blink_timer, K_MSEC(BLINK_PERIOD_MS), K_MSEC(BLINK_PERIOD_MS));
        k_timer_start(&left_led.duration_timer, K_MSEC(LONG_BLINK_MS), K_NO_WAIT);
        LOG_INF("Left LED - long duration");
        return 0; // Success
    }
    return -1; // Failure
}

/**
 * @brief Control the right LED.
 *
 * @param event The blink event
 * @return 0 on success, or a negative error code on failure
 */
static int8_t led_ctrl_right(led_event_t event)
{
    if (event == OFF) {
        k_timer_stop(&right_led.blink_timer);
        k_timer_stop(&right_led.duration_timer);
        right_led.blinking = false;
        right_led.led_on = false;
        gpio_pin_set_dt(right_led.led, 0); // Turn off LED
        LOG_INF("Right LED OFF");
        return 0; // Success
    }
    else if (event == ON) {
        right_led.blinking = true;
        right_led.led_on = true;
        gpio_pin_set_dt(right_led.led, 1); // Turn on LED
        k_timer_start(&right_led.blink_timer, K_MSEC(BLINK_PERIOD_MS), K_MSEC(BLINK_PERIOD_MS));
        k_timer_stop(&right_led.duration_timer);
        LOG_INF("Right LED started");
        return 0; // Success
    }    
    else if (event == SHORT) {
        right_led.blinking = true;
        right_led.led_on = true;
        gpio_pin_set_dt(right_led.led, 1); // Turn on LED
        k_timer_start(&right_led.blink_timer, K_MSEC(BLINK_PERIOD_MS), K_MSEC(BLINK_PERIOD_MS));
        k_timer_start(&right_led.duration_timer, K_MSEC(SHORT_BLINK_MS), K_NO_WAIT);
        LOG_INF("Right LED - short duration");
        return 0; // Success
    }
    else if (event == LONG) {
        right_led.blinking = true;
        right_led.led_on = true;
        gpio_pin_set_dt(right_led.led, 1); // Turn on LED
        k_timer_start(&right_led.blink_timer, K_MSEC(BLINK_PERIOD_MS), K_MSEC(BLINK_PERIOD_MS));
        k_timer_start(&right_led.duration_timer, K_MSEC(LONG_BLINK_MS), K_NO_WAIT);
        LOG_INF("Right LED - long duration");
        return 0; // Success
    }
    return -1; // Failure
}

/**
 * @brief Control the hazard LED.
 *
 * @param event The blink event
 * @return 0 on success, or a negative error code on failure
 */
static int8_t led_ctrl_hazard(led_event_t event)
{
    if (event == OFF) {
        led_ctrl_left(OFF);
        led_ctrl_right(OFF);
        LOG_INF("HAZARD LED OFF");
    }
    else {
        led_ctrl_left(ON);
        led_ctrl_right(ON);
        LOG_INF("HAZARD blinker ON");
    }
    return 0; // Success
}

/**
 * @brief Initialize the blinker control module.
 *
 * @return 0 on success, or a negative error code on failure
 */
int8_t led_ctrl_init(void)
{
    int ret;

    if (!gpio_is_ready_dt(left_led.led)) {
        LOG_ERR("LED device %s is not ready", left_led.led->port->name);
        return -1;
    }

    if (!gpio_is_ready_dt(right_led.led)) {
        LOG_ERR("LED device %s is not ready", right_led.led->port->name);
        return -1;
    }

	if (left_led.led->port) {
		ret = gpio_pin_configure_dt(left_led.led, GPIO_OUTPUT);
		if (ret != 0) {
			LOG_ERR("E%d: failed to configure LED device %s pin %d", ret, left_led.led->port->name, left_led.led->pin);
            return -1;
		} 
        else {
			LOG_INF("Set up LED at %s pin %d", left_led.led->port->name, left_led.led->pin);
		}
	}

	if (right_led.led->port) {
		ret = gpio_pin_configure_dt(right_led.led, GPIO_OUTPUT);
		if (ret != 0) {
			LOG_ERR("E%d: failed to configure LED device %s pin %d", ret, right_led.led->port->name, right_led.led->pin);
            return -1;
		} 
        else {
			LOG_INF("Set up LED at %s pin %d", right_led.led->port->name, right_led.led->pin);
		}
	}	

    k_timer_init(&left_led.blink_timer, blink_timer_fn, NULL);
    k_timer_init(&left_led.duration_timer, duration_timer_fn, NULL);

    k_timer_init(&right_led.blink_timer, blink_timer_fn, NULL);
    k_timer_init(&right_led.duration_timer, duration_timer_fn, NULL);

    led_ctrl_left(OFF);
    led_ctrl_right(OFF);

    led_mode = LED_OFF;

    return 0;
} 

/**
 * @brief Set the blink mode for the left and right blinkers.
 *
 * @param mode The blink mode to set
 * @return 0 on success, or a negative error code on failure
 */
int8_t led_ctrl_set_mode(led_mode_t mode)
{
    static bool blinking_active = false;
    static led_mode_t prev_mode = LED_OFF;

    led_mode = mode;

    switch (led_mode) {
        case LED_OFF:
            led_ctrl_left(OFF);
            led_ctrl_right(OFF);
            LOG_INF("BOTH blinkers OFF");
            break;
        case LEFT_BLINK_SHORT:
            led_ctrl_left(SHORT);
            led_ctrl_right(OFF);
            LOG_INF("LEFT blinker ON");
            break;
        case LEFT_BLINK_LONG:
            led_ctrl_left(LONG);
            led_ctrl_right(OFF);
            LOG_INF("LEFT blinker ON (LONG)");
            break;
        case RIGHT_BLINK_SHORT:
            led_ctrl_right(SHORT);
            led_ctrl_left(OFF);
            LOG_INF("RIGHT blinker ON");
            break;
        case RIGHT_BLINK_LONG:
            led_ctrl_right(LONG);
            led_ctrl_left(OFF);
            LOG_INF("RIGHT blinker ON (LONG)");
            break;
        case HAZARD_BLINK:
            // Toggle hazard blinkers
            if (prev_mode != HAZARD_BLINK) {
                blinking_active = true; // Reset blinking state
            }
            else
            {
                blinking_active = !blinking_active; // Toggle state
            }
            
            if (blinking_active) {
                led_ctrl_hazard(ON);
                LOG_INF("Hazard blinkers ON");
            }
            else {
                led_ctrl_hazard(OFF);
                LOG_INF("Hazard blinkers OFF");
            }
            break;
        default:
            return -EINVAL; // Invalid mode
    }

    prev_mode = mode;
    return 0; // Success
}

/**
 * @brief Get the current blink mode for the left and right blinkers.
 *
 * @param mode Pointer to the variable to store the current blink mode
 * @return 0 on success, or a negative error code on failure    
 */
int8_t led_ctrl_get_mode(led_mode_t *mode)
{
    if (mode == NULL) {
        return -EINVAL; // Invalid argument
    }

    *mode = led_mode;
    return 0; // Success
}