/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "soc/clk_tree_defs.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_DUTY_RES           LEDC_TIMER_7_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095 / 4) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (10000) // Frequency in Hertz. Set frequency at 5 kHz

ledc_channel_t white_channels[4] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3}; 
ledc_channel_t red_channels[4] = {LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7}; 

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    int hpoints[4] = {0, 31, 63, 95};
    int white_pins[4] = {13, 11, 9, 19};
    int red_pins[4] = {12, 10, 20, 8};

    for(int i = 0; i < 4; i++){
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_MODE,
            .channel        = white_channels[i],
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = white_pins[i],
            .duty           = 0, // Set duty to 0%
            .hpoint         = hpoints[i]
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

        ledc_channel.gpio_num = red_pins[i];
        ledc_channel.channel = red_channels[i];
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

void read_and_set_duty(int *input_pins_white, int *input_pins_red, unsigned int max_power){
    unsigned int white_percent = 0;
    unsigned int red_percent = 0;
    for (int i = 0; i < 7; i++){
        white_percent = white_percent << 1;
        red_percent = red_percent << 1;
        white_percent += gpio_get_level(input_pins_white[i]);
        red_percent += gpio_get_level(input_pins_red[i]);
    }
    if (white_percent > 100){
        white_percent = 100;
    }
    if (red_percent > 100){
        red_percent = 100;
    }

    float white_fraction = (float) white_percent / 100.0;
    uint32_t white_duty = (uint32_t) (white_fraction * (float) max_power);
    float red_fraction = (float) red_percent / 100.0;
    uint32_t red_duty = (uint32_t) (red_fraction * (float) max_power);

    printf("White: %d\nRed: %d\n\n", white_percent, red_percent);
    for (int i = 0; i < 4; i++){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, white_channels[i], white_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, white_channels[i]));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, red_channels[i], red_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, red_channels[i]));
    }
}

void app_main(void)
{
    int input_pins_white[7] = {38, 39, 40, 41, 42, 2, 1};
    int input_pins_red[7] = {14, 21, 47, 48, 35, 36, 37};
    unsigned long long inputmask = 0;
    for(int i = 0; i < 7; i++){
        inputmask |= 1ULL << input_pins_white[i];
        inputmask |= 1ULL << input_pins_red[i];
    }

    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = inputmask;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    unsigned int max_power = (1 << 7) - 1;
    ledc_init();
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));

    while(1){
        read_and_set_duty(input_pins_white, input_pins_red, max_power);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
