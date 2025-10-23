#ifndef LED_CTRL_H_
#define LED_CTRL_H_

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

int16_t led_ctrl_init(void);
int16_t led_ctrl_set_mode(led_mode_t mode);
int16_t led_ctrl_get_mode(led_mode_t *mode);

#endif // LED_CTRL_H_