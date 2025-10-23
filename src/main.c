#include <zephyr/kernel.h>
#include <inttypes.h>
#include <zephyr/logging/log.h>

#include "config.h"
#include "led_ctrl.h"
#include "als_drv.h"

#define LOG_LEVEL CONFIG_MAIN_LOG_LEVEL
LOG_MODULE_REGISTER(main);

#define SLEEP_TIME_MS	100

dev_data_t dev_info;

int main(void)
{
    led_ctrl_init();
    als_init();

    while (1) {
        als_get_data(&dev_info.als);
        
        led_ctrl_set_mode(HAZARD_BLINK);
        LOG_INF("Hazard blinkers ON");
        k_msleep(9000);
        led_ctrl_set_mode(LED_OFF);
        LOG_INF("Hazard blinkers OFF");
        k_msleep(1000);
    }

    return 0;
}
