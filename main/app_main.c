/* ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "app_main.h"
#include "esp_partition.h"
#include "ssd1306.h"
#include "driver/i2c.h"
void gpio_led_init()
{
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pin_bit_mask = 1LL << GPIO_LED_RED;
    gpio_config(&gpio_conf);
    gpio_conf.pin_bit_mask = 1LL << GPIO_LED_WHITE;
    gpio_config(&gpio_conf);

}

void led_task(void *arg)
{
    while (1) {
        switch (g_state) {
        case WAIT_FOR_WAKEUP:
            gpio_set_level(GPIO_LED_RED, 1);
            gpio_set_level(GPIO_LED_WHITE, 0);
            break;

        case WAIT_FOR_CONNECT:
            gpio_set_level(GPIO_LED_WHITE, 0);
            gpio_set_level(GPIO_LED_RED, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_LED_RED, 0);
            break;

        case START_DETECT:
        case START_RECOGNITION:
            gpio_set_level(GPIO_LED_WHITE, 1);
            gpio_set_level(GPIO_LED_RED, 0);
            break;

        case START_ENROLL:
            gpio_set_level(GPIO_LED_WHITE, 1);
            gpio_set_level(GPIO_LED_RED, 1);
            break;

        case START_DELETE:
            gpio_set_level(GPIO_LED_WHITE, 1);
            for (int i = 0; i < 3; i++) {
                gpio_set_level(GPIO_LED_RED, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(GPIO_LED_RED, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            break;

        default:
            gpio_set_level(GPIO_LED_WHITE, 1);
            gpio_set_level(GPIO_LED_RED, 0);
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

en_fsm_state g_state = WAIT_FOR_WAKEUP;
int g_is_enrolling = 0;
int g_is_deleting = 0;

void mssd1306_init()
{
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    /*
    *   default use I2C_NUM_1
    *   SDA --> 21
    *   SCL --> 22
    */
    ssd1306_128x64_i2c_init();
    ssd1306_flipHorizontal(1);
    ssd1306_flipVertical(1);
    ssd1306_clearScreen();
    ssd1306_printFixedN(64 / 2 - 8, 28, "esp-who", STYLE_ITALIC, FONT_SIZE_2X);
    vTaskDelay(30 / portTICK_PERIOD_MS);
}


#define I2C_MASTER_NUM I2C_NUM_1    /*!< I2C port number for master dev */

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int ret; /* Return 0 for Success, non-zero for failure */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_id << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, len, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

#define IP5306_ADDR 0X75
#define IP5306_REG_SYS_CTL0 0x00
bool setPowerBoostKeepOn(int en)
{
    uint8_t data = 0x37; // Set bit1: 1 enable 0 disable boost keep on
    if (!en)
        data = 0x35; // 0x37 is default reg value
    int ret = user_i2c_write(IP5306_ADDR, IP5306_REG_SYS_CTL0, &data, 1);

    ssd1306_clearScreen();
    if (ret == ESP_OK)
        ssd1306_printFixedN(0, 20, "IP5306 keepON PASS", STYLE_NORMAL, FONT_SIZE_2X);
    else
        ssd1306_printFixedN(0, 20, "IP5306 keepON FAIL", STYLE_NORMAL, FONT_SIZE_2X);

    ESP_LOGI("IP5306", "setPowerBoostKeepOn %d\n", ret);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    return true;
}


void pir_task(void *p)
{
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pin_bit_mask = 1LL << GPIO_PIR;
    gpio_config(&gpio_conf);

    int prev = 0;
    while ((1)) {
        int level =  gpio_get_level(GPIO_PIR);
        if (prev != level) {
            if (level) {
                ESP_LOGI("PIR", "pir tirgger \n");
                ssd1306_clearScreen();
                ssd1306_printFixedN(10, 20, "pir tirgger", STYLE_ITALIC, FONT_SIZE_2X);
            } else {
                ssd1306_clearScreen();
            }
            prev = level;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    mssd1306_init();

    app_speech_wakeup_init();

    xTaskCreatePinnedToCore(&pir_task, "blink_task", 2048, NULL, 5, NULL, 0);

    g_state = WAIT_FOR_WAKEUP;

    vTaskDelay(30 / portTICK_PERIOD_MS);
    ESP_LOGI("esp-eye", "Please say 'Hi LeXin' to the board");
    ESP_LOGI("esp-eye", "Version "VERSION);
    while (g_state == WAIT_FOR_WAKEUP)
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    app_wifi_init();
    app_camera_init();
    app_httpserver_init();
    ESP_LOGI("esp-eye", "Version "VERSION" success");
}
