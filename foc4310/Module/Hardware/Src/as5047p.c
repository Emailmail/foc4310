#include "as5047p.h"

// ---------------- 协议层 ----------------
/**
 * @brief 计算偶校验位 (bit0~bit14 中 1 的个数为奇数时返回 1)
 */
static uint16_t as5047p_parity(uint16_t data) {
    uint16_t sum = 0;
    while (data) {
        sum ^= data;
        data >>= 1;
    }
    return sum & 0x01;
}

/**
 * @brief 构建 SPI 读命令 (bit14=1 读, bit15=偶校验)
 */
static uint16_t as5047p_build_read_cmd(uint16_t addr) {
    uint16_t cmd = addr | 0x4000;
    if (as5047p_parity(cmd)) {
        cmd |= 0x8000;
    }
    return cmd;
}

/**
 * @brief 通过 SPI 收发一帧 (16-bit)
 */
static uint16_t as5047p_spi_frame(as5047p_t *as5047p, uint16_t txdata) {
    uint16_t rxdata;
    bsp_spi_set_cs(as5047p->spi, 0);
    bsp_spi_transmit_receive(as5047p->spi, (uint8_t *)&txdata, (uint8_t *)&rxdata, 1, BSP_SPI_TX_BLOCK);
    bsp_spi_set_cs(as5047p->spi, 1);
    return rxdata;
}

// ---------------- bsp_spi 回调 ----------------
/**
 * @brief SPI 传输完成回调 — 由 bsp_spi 的 HAL 回调中分发到此
 */
static void as5047p_spi_callback(void *instance, uint16_t size) {
    (void)instance;
    (void)size;
    // 异步模式的数据处理可在此扩展
}

// ---------------- 用户 API ----------------
/**
 * @brief 注册 AS5047P
 */
void as5047p_register(as5047p_t *as5047p, bsp_spi_t *spi) {
    as5047p->spi = spi;
    as5047p->angle = 0.0f;

    // 将自己注册到 bsp_spi 回调链, 使 HAL 中断/DMA 完成时能分发到此模块
    spi->device_instance = as5047p;
    spi->device_callback = as5047p_spi_callback;
}

/**
 * @brief 读取 AS5047P 寄存器 (14-bit)
 * @param addr 寄存器地址
 */
uint16_t as5047p_read(as5047p_t *as5047p, uint16_t addr) {
    // 第 1 帧: 发送读命令
    as5047p_spi_frame(as5047p, as5047p_build_read_cmd(addr));
    // 第 2 帧: 发送 NOP, 时钟移出上一帧的结果
    uint16_t data = as5047p_spi_frame(as5047p, as5047p_build_read_cmd(AS5047P_NOP));
    return data & 0x3FFF;
}

/**
 * @brief 读取 AS5047P 角度 (无滤波，原始值)
 * @return 原始机械角度 [0, 2PI) rad
 * @note   滤波在上层 FOC 中按需进行
 */
float as5047p_read_angle(as5047p_t *as5047p) {
    as5047p->angle = as5047p_read(as5047p, AS5047P_ANGLECOM) * AS5047P_RAW_TO_RAD;
    return as5047p->angle;
}
