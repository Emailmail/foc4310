#include "bsp_spi.h"

/* spi 数组, 所有注册了的 spi 实例都会保存在这里 */
static uint8_t idx = 0;
static bsp_spi_t *spi_instances[DEVICE_SPI_CNT] = {NULL};

/**
 * @brief 注册 spi 设备实例
 */
void bsp_spi_register(bsp_spi_t *spi, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      void *device_instance, spi_device_callback device_callback) {
    if (idx >= DEVICE_SPI_CNT) {
        return;
    }

    spi->hspi = hspi;
    spi->cs_port = cs_port;
    spi->cs_pin = cs_pin;
    spi->device_instance = device_instance;
    spi->device_callback = device_callback;

    // 初始化 CS 为高电平 (未选中)
    if (cs_port != NULL) {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    }

    spi_instances[idx++] = spi;
}

/**
 * @brief 设置 SPI 片选信号
 * @param enable 0: 拉低选中, 1: 拉高释放
 */
void bsp_spi_set_cs(bsp_spi_t *spi, uint8_t enable) {
    if (spi->cs_port != NULL) {
        HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin,
                          enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief spi 查询是否空闲 (可防止短时间内重复调用发送导致冲突)
 */
uint8_t bsp_spi_isidle(bsp_spi_t *spi) {
    return (spi->hspi->State == HAL_SPI_STATE_READY);
}

/**
 * @brief spi 发送
 */
void bsp_spi_transmit(bsp_spi_t *spi, uint8_t *data, uint16_t size, bsp_spi_tx_mode_t mode) {
    switch (mode) {
        case BSP_SPI_TX_BLOCK:
            HAL_SPI_Transmit(spi->hspi, data, size, HAL_MAX_DELAY);
            break;
        case BSP_SPI_TX_IT:
            HAL_SPI_Transmit_IT(spi->hspi, data, size);
            break;
        case BSP_SPI_TX_DMA:
            HAL_SPI_Transmit_DMA(spi->hspi, data, size);
            break;
    }
}

/**
 * @brief spi 接收
 */
void bsp_spi_receive(bsp_spi_t *spi, uint8_t *data, uint16_t size, bsp_spi_tx_mode_t mode) {
    switch (mode) {
        case BSP_SPI_TX_BLOCK:
            HAL_SPI_Receive(spi->hspi, data, size, HAL_MAX_DELAY);
            break;
        case BSP_SPI_TX_IT:
            HAL_SPI_Receive_IT(spi->hspi, data, size);
            break;
        case BSP_SPI_TX_DMA:
            HAL_SPI_Receive_DMA(spi->hspi, data, size);
            break;
    }
}

/**
 * @brief spi 全双工收发 (同时发送和接收)
 */
void bsp_spi_transmit_receive(bsp_spi_t *spi, uint8_t *tx_data, uint8_t *rx_data, uint16_t size, bsp_spi_tx_mode_t mode) {
    switch (mode) {
        case BSP_SPI_TX_BLOCK:
            HAL_SPI_TransmitReceive(spi->hspi, tx_data, rx_data, size, HAL_MAX_DELAY);
            break;
        case BSP_SPI_TX_IT:
            HAL_SPI_TransmitReceive_IT(spi->hspi, tx_data, rx_data, size);
            break;
        case BSP_SPI_TX_DMA:
            HAL_SPI_TransmitReceive_DMA(spi->hspi, tx_data, rx_data, size);
            break;
    }
}

/**
 * @brief spi 发送完成回调函数, 此处统一管理
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (hspi == spi_instances[i]->hspi) {
            if (spi_instances[i]->device_callback != NULL) {
                spi_instances[i]->device_callback(spi_instances[i]->device_instance, 0);
            }
            return;
        }
    }
}

/**
 * @brief spi 接收完成回调函数, 此处统一管理
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (hspi == spi_instances[i]->hspi) {
            if (spi_instances[i]->device_callback != NULL) {
                spi_instances[i]->device_callback(spi_instances[i]->device_instance,
                                                  spi_instances[i]->hspi->RxXferSize);
            }
            return;
        }
    }
}

/**
 * @brief spi 全双工收发完成回调函数, 此处统一管理
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (hspi == spi_instances[i]->hspi) {
            if (spi_instances[i]->device_callback != NULL) {
                spi_instances[i]->device_callback(spi_instances[i]->device_instance,
                                                  spi_instances[i]->hspi->RxXferSize);
            }
            return;
        }
    }
}

/**
 * @brief spi 错误回调函数, 此处统一管理
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (hspi == spi_instances[i]->hspi) {
            if (spi_instances[i]->device_callback != NULL) {
                spi_instances[i]->device_callback(spi_instances[i]->device_instance, 0);
            }
            return;
        }
    }
}
