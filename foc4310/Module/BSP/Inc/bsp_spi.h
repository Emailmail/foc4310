#ifndef _BSP_SPI_H
#define _BSP_SPI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>

#define DEVICE_SPI_CNT 3     // 最大 SPI 数量
#define SPI_RXBUFF_LIMIT 32  // SPI 接收缓冲区大小

typedef void (*spi_device_callback)(void *device_instance, uint16_t size); // 模块回调函数, 用于解析数据

typedef enum {
    BSP_SPI_TX_BLOCK = 0,
    BSP_SPI_TX_IT,
    BSP_SPI_TX_DMA,
} bsp_spi_tx_mode_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t rxbuff[SPI_RXBUFF_LIMIT];

    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    void *device_instance;       // 模块实例指针, 用于回调函数解析数据时使用
    spi_device_callback device_callback;
} bsp_spi_t;

void bsp_spi_register(bsp_spi_t *spi, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      void *device_instance, spi_device_callback device_callback);
void bsp_spi_set_cs(bsp_spi_t *spi, uint8_t enable);
uint8_t bsp_spi_isidle(bsp_spi_t *spi);
void bsp_spi_transmit(bsp_spi_t *spi, uint8_t *data, uint16_t size, bsp_spi_tx_mode_t mode);
void bsp_spi_receive(bsp_spi_t *spi, uint8_t *data, uint16_t size, bsp_spi_tx_mode_t mode);
void bsp_spi_transmit_receive(bsp_spi_t *spi, uint8_t *tx_data, uint8_t *rx_data, uint16_t size, bsp_spi_tx_mode_t mode);

#ifdef __cplusplus
}
#endif
#endif
