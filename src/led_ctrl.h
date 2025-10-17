#ifndef LED_CTRL_H
#define LED_CTRL_H

#include <stdint.h>

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

int led_ctrl_init(void);
int led_ctrl_set_mode(led_mode_t mode);
int led_ctrl_get_mode(led_mode_t *mode);

#endif // LED_CTRL_H