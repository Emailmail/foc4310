#include "led.h"
#include "bsp_gpio.h"

/**
 * @brief 注册 LED
 */
void led_register(led_t* led, bsp_gpio_t *gpio, uint8_t unset_state) {
    led->gpio = gpio;
    led->unset_state = unset_state;
}

/**
 * @brief 设置 LED 状态
 */
void led_set(led_t* led, uint8_t state) {
    if (state) {
        bsp_gpio_write(led->gpio, !led->unset_state);
    } else {
        bsp_gpio_write(led->gpio, led->unset_state);
    }
}

/**
 * @brief 切换 LED 状态
 */
void led_toggle(led_t* led) {
    bsp_gpio_toggle(led->gpio);
}
