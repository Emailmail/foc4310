#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"

typedef enum {
    bsp_gpio_mode_input = 0,
    bsp_gpio_mode_output,
} bsp_gpio_mode_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} bsp_gpio_t;

void bsp_gpio_register(bsp_gpio_t* gpio, GPIO_TypeDef* port, uint16_t pin);
void bsp_gpio_set_mode(bsp_gpio_t* gpio, bsp_gpio_mode_t mode);
uint8_t bsp_gpio_read(bsp_gpio_t* gpio);
void bsp_gpio_write(bsp_gpio_t* gpio, uint8_t state);
void bsp_gpio_toggle(bsp_gpio_t* gpio);
#endif
