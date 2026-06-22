#ifndef AS5047P_H
#define AS5047P_H
#include "bsp_spi.h"
#include <stdint.h>

// AS5047 寄存器地址
#define AS5047P_NOP      0x0000  // 无操作
#define AS5047P_ERRFL    0x0001  // 错误寄存器
#define AS5047P_PROG     0x0003  // 编程寄存器
#define AS5047P_DIAAGC   0x3FFC  // 诊断和 AGC（自动增益控制）
#define AS5047P_MAG      0x3FFD  // CORDIC 幅度
#define AS5047P_ANGLEUNC 0x3FFE  // 读取角度值，无动态角度误差补偿
#define AS5047P_ANGLECOM 0x3FFF  // 读取角度值，有动态角度误差补偿
#define ZPOSM 0x0016             // 零点位置寄存器（高字节）
#define ZPOSL 0x0017             // 零点位置寄存器（低字节）
#define SETTINGS1 0x0018        
#define SETTINGS2 0x0019

// AS5047 角度转换因子
#define AS5047P_RAW_TO_RAD 0.0003834952f
#define AS5047P_RAW_TO_DEG 0.021972656f

typedef struct {
    float alpha;
    float measure;
} as5047p_lowpass_t;

typedef struct {
    bsp_spi_t *spi;
    as5047p_lowpass_t lowpass;
    float angle;
} as5047p_t;

void as5047p_register(as5047p_t *as5047p, bsp_spi_t *spi, float lowpass_alpha);
uint16_t as5047p_read(as5047p_t *as5047p, uint16_t addr);
float as5047p_read_angle(as5047p_t *as5047p);
#endif
