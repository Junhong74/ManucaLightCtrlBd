#ifndef LED_CTRL_H
#define LED_CTRL_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>        
#include <zephyr/sys/printk.h>

typedef enum {
	LED_OFF = 0U,
	LEFT_BLINK_SHORT,
    LEFT_BLINK_LONG,
	RIGHT_BLINK_SHORT,
    RIGHT_BLINK_LONG,
	HAZARD_BLINK
} led_mode_t;

typedef enum {
    ON = 0U,
    SHORT,
    LONG,
    OFF,
} led_event_t;

int8_t led_ctrl_init(void);
int8_t led_ctrl_set_mode(led_mode_t mode);
int8_t led_ctrl_get_mode(led_mode_t *mode);

#endif // LED_CTRL_H