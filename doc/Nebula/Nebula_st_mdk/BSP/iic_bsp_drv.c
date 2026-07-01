
#include "iic_bsp_drv.h"




void IIC_bsp_init( AuroIIC *iic){
	_IIC_bsp_SDA_out( iic );
	_IIC_bsp_SDA_H( iic );
	_IIC_bsp_SCL_H( iic );
}
void IIC_bsp_write( AuroIIC *iic , uint8_t device , uint8_t registers , uint8_t value ){
	_IIC_bsp_start( iic );
	_IIC_bsp_write( iic , device );
	_IIC_bsp_ack( iic );
	_IIC_bsp_write( iic , registers );
	_IIC_bsp_ack( iic );
	_IIC_bsp_write( iic , value );
	_IIC_bsp_ack( iic );
	_IIC_bsp_stop( iic );
}

uint8_t IIC_bsp_read( AuroIIC *iic , uint8_t device , uint8_t registers ){
	uint8_t data;
	_IIC_bsp_start( iic );
	_IIC_bsp_write( iic , device );
	_IIC_bsp_ack( iic );
	_IIC_bsp_write( iic , registers );
	_IIC_bsp_ack( iic );
	_IIC_bsp_start( iic );
	_IIC_bsp_write( iic , device + 0x01 );
	_IIC_bsp_ack( iic );
	data = _IIC_bsp_read( iic );
	_IIC_bsp_ack( iic );
	_IIC_bsp_stop( iic );
	return data;
}
uint8_t IIC_bsp_write_mul( AuroIIC *iic , uint8_t device , uint8_t registers ,uint8_t size, uint8_t *value ){
	uint8_t i;
	_IIC_bsp_start( iic );
	_IIC_bsp_write( iic , device );
	_IIC_bsp_ack( iic );
	_IIC_bsp_write( iic , registers );
	_IIC_bsp_ack( iic );
	for( i = 0; i < size ;i++){
		_IIC_bsp_write( iic , value[i] );
		_IIC_bsp_ack( iic );
	}
	_IIC_bsp_stop( iic );
	return 0;
}

uint8_t IIC_bsp_read_mul( AuroIIC *iic , uint8_t device , uint8_t registers ,uint8_t size, uint8_t *value ){
	uint8_t data;
	_IIC_bsp_start( iic );
	_IIC_bsp_write( iic , device );
	_IIC_bsp_ack( iic );
	_IIC_bsp_write( iic , registers );
	_IIC_bsp_ack( iic );
	_IIC_bsp_start( iic );
	_IIC_bsp_write( iic , device + 0x01 );
	_IIC_bsp_ack( iic );
	while(size){
		if( size == 1){
			*value = _IIC_bsp_read(iic);
			_IIC_bsp_master_ack( iic ,1);
		}
		else{
			*value = _IIC_bsp_read(iic);
			_IIC_bsp_master_ack( iic ,0);
		}
		size--;
		value++;
	}
	_IIC_bsp_stop( iic );
	return 0;
}


void _IIC_bsp_delay( uint16_t t){
	volatile int time = t;
    while( time --);
}
void _IIC_bsp_SDA_out( AuroIIC *iic ){
	HAL_GPIO_DeInit( iic->SDA_Port , iic->SDA_Pin );

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin =iic->SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init( iic->SDA_Port , &GPIO_InitStruct );
}
void _IIC_bsp_SDA_in( AuroIIC *iic ){
	HAL_GPIO_DeInit( iic->SDA_Port , iic->SDA_Pin );
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = iic->SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init( iic->SDA_Port , &GPIO_InitStruct );
}
void _IIC_bsp_SDA_H( AuroIIC *iic ){
	HAL_GPIO_WritePin( iic->SDA_Port , iic->SDA_Pin , GPIO_PIN_SET );
}
void _IIC_bsp_SDA_L( AuroIIC *iic ){
	HAL_GPIO_WritePin( iic->SDA_Port , iic->SDA_Pin , GPIO_PIN_RESET );
}
void _IIC_bsp_SCL_H( AuroIIC *iic ){
	HAL_GPIO_WritePin( iic->SCL_Port , iic->SCL_Pin , GPIO_PIN_SET );
}
void _IIC_bsp_SCL_L( AuroIIC *iic ){
	HAL_GPIO_WritePin( iic->SCL_Port , iic->SCL_Pin , GPIO_PIN_RESET );
}
uint8_t _IIC_bsp_SDA_Read( AuroIIC *iic ){
	return HAL_GPIO_ReadPin( iic->SDA_Port , iic->SDA_Pin ) ;
}

void _IIC_bsp_start( AuroIIC *iic ){
	
	_IIC_bsp_SDA_out( iic );
	_IIC_bsp_SDA_H( iic );
	_IIC_bsp_SCL_H( iic );
	_IIC_bsp_delay( 10 );
	_IIC_bsp_SDA_L( iic );
	_IIC_bsp_delay( 10 );
}
void _IIC_bsp_stop( AuroIIC *iic ){
	_IIC_bsp_SDA_out( iic );
	_IIC_bsp_SCL_L( iic );
	_IIC_bsp_delay( 10 );
	_IIC_bsp_SDA_L( iic );
	_IIC_bsp_delay( 10 );
	_IIC_bsp_SCL_H( iic );
	_IIC_bsp_delay( 10 );
	_IIC_bsp_SDA_H( iic );
	_IIC_bsp_delay( 10 );
}
void _IIC_bsp_write( AuroIIC *iic , uint8_t data){
	uint8_t i;
	_IIC_bsp_SDA_out( iic );
	_IIC_bsp_delay( 10 );
    for( i = 0 ; i < 8 ; i++)
    {
        _IIC_bsp_SCL_L( iic );
        _IIC_bsp_delay( 10 );
        if(data & 0x80)
            _IIC_bsp_SDA_H( iic );
        else
            _IIC_bsp_SDA_L( iic );
        data = data << 1;
        _IIC_bsp_delay( 10 );
        _IIC_bsp_SCL_H( iic );
        _IIC_bsp_delay( 10 );
    }
    _IIC_bsp_SCL_L( iic );
}
uint8_t _IIC_bsp_read( AuroIIC *iic ){
	uint8_t i = 0;
    uint8_t data = 0x00;
    _IIC_bsp_SDA_in( iic );
    _IIC_bsp_delay( 10 );
    for( i = 0 ; i < 8 ; i++ )
    {
        data <<= 1;
        _IIC_bsp_SCL_H( iic );
        _IIC_bsp_delay( 10 );
        data |= _IIC_bsp_SDA_Read( iic );
        _IIC_bsp_delay( 10 );
        _IIC_bsp_SCL_L( iic );
        _IIC_bsp_delay( 10 );
    }
    return data;
}
uint8_t _IIC_bsp_ack( AuroIIC *iic ){
    uint8_t time = 0xf;
	_IIC_bsp_SDA_out( iic );
    _IIC_bsp_SCL_L( iic );
    _IIC_bsp_delay( 10 );
    _IIC_bsp_SDA_H( iic );

    _IIC_bsp_SDA_in( iic );
    _IIC_bsp_delay( 10 );
    _IIC_bsp_SCL_H( iic );
    _IIC_bsp_delay( 10 );
    while(time)
    {
        if( _IIC_bsp_SDA_Read( iic ) == 0)   //读取成功
        {
            _IIC_bsp_delay( 10 );
            _IIC_bsp_SCL_L( iic );
        //    usb_printf("iic ack ok\r\n");
            return 1;
        }
        time--;
    }
  //  usb_printf("iic ack fail\r\n");
    _IIC_bsp_SCL_L( iic );
	_IIC_bsp_delay( 10 );
    return 0; //读取错误
}

uint8_t _IIC_bsp_master_ack( AuroIIC *iic,uint8_t ack ){
    uint8_t time = 0xf;
	_IIC_bsp_SDA_out( iic );
    _IIC_bsp_SCL_L( iic );
    _IIC_bsp_delay( 10 );
	if( ack == 0)
		_IIC_bsp_SDA_L( iic );
	else
		_IIC_bsp_SDA_H( iic );
    _IIC_bsp_delay( 10 );
    _IIC_bsp_SCL_H( iic );
    _IIC_bsp_delay( 10 );
    _IIC_bsp_SCL_L( iic );
	_IIC_bsp_SDA_H( iic );
	_IIC_bsp_delay( 10 );
    return 0; //讈取窄铣
}
