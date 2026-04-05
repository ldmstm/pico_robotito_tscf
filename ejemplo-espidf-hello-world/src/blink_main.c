

#ifdef RUN_BLINK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN 5

void blink_task(void *arg)
{
    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(300));

        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

void app_main(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);
}

#endif