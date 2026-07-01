
#ifndef __LSM6DS3TR_BSP_DRV_H_
#define __LSM6DS3TR_BSP_DRV_H_


#include "iic_bsp_drv.h"


#define LSM6DS3TR_IIC_ADDR 0xD4


typedef struct LSM6DS3TR{
	AuroIIC iic;
}LSM6DS3TR;



void LSM6DS3TR_bsp_init( LSM6DS3TR *lsm6ds3tr);
uint8_t LSM6DS3TR_bsp_read_id( LSM6DS3TR *lsm6ds3tr );






#endif