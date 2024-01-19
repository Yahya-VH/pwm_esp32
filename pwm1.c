#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define TOUCH_PIN_1     GPIO_NUM_22  // Example GPIO pin for touch sensor 1
#define TOUCH_PIN_2     GPIO_NUM_23  // Example GPIO pin for touch sensor 2  // Example GPIO pin for touch sensor 3
#define LED_PIN         GPIO_NUM_5   // Example GPIO pin for LED
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_HIGH_SPEED_MODE
#define PWM_CHANNEL     LEDC_CHANNEL_0
#define PWM_RESOLUTION  LEDC_TIMER_13_BIT

#define TOUCH_THRESH_NO_USE  1000 // Adjust this threshold according to your touch sensor

void init_pwm() {
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = 5000, // PWM frequency in Hz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ledc_conf = {
        .gpio_num = LED_PIN,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_conf);
}

void fade_led(int duty) {
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

void app_main() {
    // Initialize PWM
    init_pwm();

    // Initialize GPIOs for touch sensors
    gpio_config_t touch_gpio_config_1 = {
        .pin_bit_mask = (1ULL << TOUCH_PIN_1),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config_t touch_gpio_config_2 = {
        .pin_bit_mask = (1ULL << TOUCH_PIN_2),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    
    gpio_config(&touch_gpio_config_1);
    gpio_config(&touch_gpio_config_2);
    

    while (1) {
        int touch_value_1 = gpio_get_level(TOUCH_PIN_1);
        int touch_value_2 = gpio_get_level(TOUCH_PIN_2);
       

        if (touch_value_1 == 1) {
            // Touch sensor 1 is touched
            vTaskDelay(pdMS_TO_TICKS(100));  // Debounce delay

            fade_led((10 * (1 << PWM_RESOLUTION)) / 100);
        } else if (touch_value_2 == 1) {
            // Touch sensor 2 is touched
            vTaskDelay(pdMS_TO_TICKS(100));  // Debounce delay

            fade_led((100 * (1 << PWM_RESOLUTION)) / 100);
        } else {
            // No touch sensors are touched, turn off LED
            fade_led(0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // Delay to avoid rapid changes
    }
}
