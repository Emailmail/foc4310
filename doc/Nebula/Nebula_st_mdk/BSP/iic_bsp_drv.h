#ifndef __IIC_BSP_DRV_H_
#define __IIC_BSP_DRV_H_

#include "main.h"

typedef struct AuroIIC{
	GPIO_TypeDef* SCL_Port;
    uint16_t SCL_Pin;
	GPIO_TypeDef* SDA_Port;
    uint16_t SDA_Pin;
}AuroIIC;



void IIC_bsp_init( AuroIIC *iic);
void IIC_bsp_write( AuroIIC *iic , uint8_t device , uint8_t registers , uint8_t value );
uint8_t IIC_bsp_read( AuroIIC *iic , uint8_t device , uint8_t registers );
uint8_t IIC_bsp_write_mul( AuroIIC *iic , uint8_t device , uint8_t registers ,uint8_t size, uint8_t *value );
uint8_t IIC_bsp_read_mul( AuroIIC *iic , uint8_t device , uint8_t registers ,uint8_t size, uint8_t *value );

void _IIC_bsp_delay( uint16_t t);
void _IIC_bsp_SDA_out( AuroIIC *iic );
void _IIC_bsp_SDA_in( AuroIIC *iic );
void _IIC_bsp_SDA_H( AuroIIC *iic );
void _IIC_bsp_SDA_L( AuroIIC *iic );
void _IIC_bsp_SCL_H( AuroIIC *iic );
void _IIC_bsp_SCL_L( AuroIIC *iic );
uint8_t _IIC_bsp_SDA_Read( AuroIIC *iic );


void _IIC_bsp_start( AuroIIC *iic );
void _IIC_bsp_stop( AuroIIC *iic );
void _IIC_bsp_write( AuroIIC *iic , uint8_t data);
uint8_t _IIC_bsp_read( AuroIIC *iic );
uint8_t _IIC_bsp_ack( AuroIIC *iic );
uint8_t _IIC_bsp_master_ack( AuroIIC *iic,uint8_t ack );
#endif
