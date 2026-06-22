#ifndef _LED_H
#define _LED_H
#include "bsp_gpio.h"

typedef struct {
    bsp_gpio_t *gpio;
    uint8_t unset_state; // 0: 低电平时灯灭; 1: 高电平时灯灭
} led_t;

void led_register(led_t* led, bsp_gpio_t *gpio, uint8_t unset_state);
void led_set(led_t* led, uint8_t state);
void led_toggle(led_t* led);
#endif
