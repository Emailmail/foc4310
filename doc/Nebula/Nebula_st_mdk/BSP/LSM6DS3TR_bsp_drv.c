#include "LSM6DS3TR_bsp_drv.h"






void LSM6DS3TR_bsp_init( LSM6DS3TR *lsm6ds3tr){


}



uint8_t LSM6DS3TR_bsp_read_id( LSM6DS3TR *lsm6ds3tr ){
	uint8_t data;
	data = IIC_bsp_read( &lsm6ds3tr->iic , LSM6DS3TR_IIC_ADDR , 0x0F);
	return data;

}