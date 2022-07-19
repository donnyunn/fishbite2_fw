#include "sleep.h"

void sleep_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = BIT(WAKEUP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

void lightSleep(void)
{
    gpio_wakeup_enable(WAKEUP_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable(5, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    printf("Entering light sleep\n");
    esp_light_sleep_start();
}

void deepSleep(void)
{
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT(WAKEUP_PIN), ESP_GPIO_WAKEUP_GPIO_LOW));
    esp_deep_sleep_start();
}

bool isPushedPwrbtn(void)
{
    if (gpio_get_level(WAKEUP_PIN) != 0) return false;
    else return true; 
}