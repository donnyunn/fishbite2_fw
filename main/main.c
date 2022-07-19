#include "main.h"

static const char * TAG = "main";
bool f_blepairing = false;

void onSensitivityChanged(uint8_t value)
{
    if (value <= 0)
    {
        value = 1;
    }

    //fbd_sensitivity((double)value * 0.01);
    
    ESP_LOGI(TAG, "Sensitivity: %d", value);
}

void onBrightnessChanged(uint8_t value)
{
    if (value <= 0){
        value = 0;
    }   
    if (value > 100){
        value = 100;
    }

    ESP_LOGI(TAG, "Brightness: %d", value);
    //led_set_brightness((double)value);
}

void nvs_init(void)
{
    esp_err_t ret;
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}

void app_main(void)
{
    adxl_evt_t evt_num;
    int cnt_btn = 0;
    uint8_t val[4] = {0,1,2,3};

    sleep_init();
    i2c_init();

    // check power on or not
    while (isPushedPwrbtn()) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        cnt_btn++;
        if (cnt_btn > 30) break;
    }
    if (cnt_btn <= 30) {
        adxl345_sleep();
        deepSleep();
    }
    cnt_btn = 0;

    nvs_init();

    led_init();
    led_indicate_poweron();

    // check long push for ble pairing
    while (isPushedPwrbtn()) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        cnt_btn++;
        if (cnt_btn > 30) {
            led_advertising();

            // wait for btn released
            while (isPushedPwrbtn()) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            // start to advertise
            gatts_init();
            ble_set_onSensitivityChange(onSensitivityChanged);
            ble_set_onBrightnessChanged(onBrightnessChanged);
            int timeout = 300;
            while (--timeout != 0) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                if (isBleConnected()) break;
            }
            esp_ble_gap_stop_advertising();
            led_red_on();
            break;
        }
    }
    cnt_btn = 0;

    if (!adxl345_knock()) {
        ESP_LOGI(TAG, "adxl345 fail");
    }
    adxl345_init();

    while(1) {
        evt_num = adxl345_getEventQueue();
        if(evt_num != ADXL345_EVT_NULL) {
            ESP_LOGI(TAG, "event %d", evt_num);
            if (evt_num == ADXL345_EVT_INACTION) {
                led_grn_on();
            } else if (evt_num >= ADXL345_EVT_ACTION) {
                led_red_on();
                ble_sendIndication(PROFILE_C_APP_ID, val);
            }
        } else {
            printf("null\n");
        }

        while (isPushedPwrbtn()) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            cnt_btn++;
            if (cnt_btn > 30) {
                led_indicate_poweroff();
                adxl345_sleep();
                // wait for release
                while (isPushedPwrbtn()) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                deepSleep();
            }
        }
        cnt_btn = 0;
    }
}
