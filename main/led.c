#include "led.h"

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_GPIO       (0)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (1)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#define LEDC_CH_NUM            (2)
#define LEDC_OFF_DUTY          (256)
#define LEDC_TEST_FADE_TIME    (3000)

static led_indicate_t _state = LED_STANDBY;
static uint8_t _duty = 0;

static void led_task(void *arg)
{
    while (1) {
        switch (_state) {
            case LED_STANDBY:
                break;
            case LED_OFF:
                // green off
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, LEDC_OFF_DUTY);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);
                // red off
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, LEDC_OFF_DUTY);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);

                _state = LED_STANDBY;
                break;
            case LED_RED_ON:
                // green off
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, LEDC_OFF_DUTY);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);
                // red on
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, _duty);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);
                
                _state = LED_STANDBY;
                break;
            case LED_GREEN_ON:
                // red off
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, LEDC_OFF_DUTY);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);
                // green on
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, _duty);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);

                _state = LED_STANDBY;
                break;
            case LED_ADVERTISING:
                // red on
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, _duty);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(500));
                // red off
                ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, LEDC_OFF_DUTY);
                ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void led_init(void)
{
    int ch;
    
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = 1000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_CH_NUM] = {
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
    };
    
    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    led_off();

    xTaskCreate(led_task, "led_task", 2048, 0, 10, NULL);
}

void led_setDuty(uint8_t duty)
{
    _duty = duty;
}

void led_red_on(void)
{
    _state = LED_RED_ON;
}

void led_grn_on(void)
{
    _state = LED_GREEN_ON;
}

void led_off(void)
{
    _state = LED_OFF;
}

void led_advertising(void)
{
    _state = LED_ADVERTISING;
}

void led_indicate_poweron(void)
{
    led_off();
    while (_state != LED_STANDBY);
    // red fade-in
    ledc_set_fade_with_time(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, 0, 1500);
    ledc_fade_start(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, LEDC_FADE_NO_WAIT);
}

void led_indicate_poweroff(void)
{
    led_grn_on();
    while (_state != LED_STANDBY);
    // green fade-out
    ledc_set_fade_with_time(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, LEDC_OFF_DUTY, 1000);
    ledc_fade_start(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, LEDC_FADE_WAIT_DONE);
}
