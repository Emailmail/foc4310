#include "bsp_gpio.h"

/**
 * @brief 注册 GPIO
 */
void bsp_gpio_register(bsp_gpio_t* gpio, GPIO_TypeDef* port, uint16_t pin) {
    gpio->port = port;
    gpio->pin = pin;
}

/**
 * @brief 设置 GPIO 模式
 */
void bsp_gpio_set_mode(bsp_gpio_t* gpio, bsp_gpio_mode_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gpio->pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    if (mode == bsp_gpio_mode_input) {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    } else if (mode == bsp_gpio_mode_output) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    }

    HAL_GPIO_Init(gpio->port, &GPIO_InitStruct);
}

/**
 * @brief 读取 GPIO 电平
 */
uint8_t bsp_gpio_read(bsp_gpio_t* gpio) {
    return (uint8_t)(HAL_GPIO_ReadPin(gpio->port, gpio->pin) == GPIO_PIN_SET);
}

/**
 * @brief 设置 GPIO 电平
 */
void bsp_gpio_write(bsp_gpio_t* gpio, uint8_t state) {
    HAL_GPIO_WritePin(gpio->port, gpio->pin, state);
}

/**
 * @brief 翻转 GPIO 电平
 */
void bsp_gpio_toggle(bsp_gpio_t* gpio) {
    HAL_GPIO_TogglePin(gpio->port, gpio->pin);
}
