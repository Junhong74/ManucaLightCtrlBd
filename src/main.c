#include <zephyr/kernel.h>
#include <inttypes.h>
#include <zephyr/logging/log.h>

#include "led_ctrl.h"

#define LOG_LEVEL CONFIG_MAIN_LOG_LEVEL
LOG_MODULE_REGISTER(main);

#define SLEEP_TIME_MS	100

int main(void)
{
    led_ctrl_init();

    while (1) {
        led_ctrl_set_mode(HAZARD_BLINK);
        LOG_INF("Hazard blinkers ON");
        k_msleep(15000);
        led_ctrl_set_mode(LED_OFF);
        LOG_INF("Hazard blinkers OFF");
        k_msleep(5000);
    }

    return 0;
}
