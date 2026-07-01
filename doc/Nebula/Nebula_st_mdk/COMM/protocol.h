#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#include "main.h"
#include "link_data.h"

//支持的通信类型
#define SUPPORT_UART			1
#define SUPPORT_USB_CDC			1
#define SUPPORT_RS485			0
#define SUPPORT_CAN				0
#define SUPPORT_NRF24L01		0


#define PACKAGE_SIZE	24
#define PACKAGE_HEAD	0xDE
#define PACKAGE_TAIL	0xED

typedef union float_uint32_int_char{
    char  data[4];
    float f;
    uint32_t u32_data;
    int32_t  i32_data;
}float_uint32_int_char;

typedef volatile struct COMM_MSG_PACKAGE{
    uint8_t head;
    uint8_t func1;
    uint8_t func2;
    uint8_t func3;
    union float_uint32_int_char data1;
    union float_uint32_int_char data2;
    union float_uint32_int_char data3;
    union float_uint32_int_char data4;
    uint16_t motor_id;
    uint8_t  id;
    uint8_t tail;
}COMM_MSG_PACKAGE;

typedef enum COMM_SEND_TYPE{
	uart_type = 0 ,
	usb_cdc_type  ,
	rs485_type    ,
	nrf24l01_type ,
	can_type      ,
}COMM_SEND_TYPE;

typedef enum PROTOCOL_STATUS{
	inst_status,			//指令状态
	file_status,			//文件接收状态
}PROTOCOL_STATUS;

//上报数据类型
typedef enum POLL_PACKAGE_TYPE{
	POLL_PACKAGE_TYPE_NONE = 0,
	POLL_PACKAGE_TYPE_IaIbIc,
	POLL_PACKAGE_TYPE_IqId,
	POLL_PACKAGE_TYPE_Speed,
	POLL_PACKAGE_TYPE_Position,
	
	POLL_PACKAGE_TYPE_Posture,
	POLL_PACKAGE_TYPE_Acc,
	POLL_PACKAGE_TYPE_Gyro,
}POLL_PACKAGE_TYPE;


typedef struct COMM_INFO{
	PROTOCOL_STATUS protocol_status;
	COMM_SEND_TYPE send_type;
	Link link;
	uint16_t len;
	uint8_t  package_buff[24];
	COMM_MSG_PACKAGE package;
	uint8_t  package_curr_len;
}COMM_INFO;

typedef struct PROTOCOL_CTRL{
	#if SUPPORT_UART
	COMM_INFO  uart;
	#endif
	#if SUPPORT_USB_CDC
	COMM_INFO  usb_cdc;
	#endif
	#if SUPPORT_RS485
	COMM_INFO  rs485;
	#endif
	#if SUPPORT_CAN
	COMM_INFO  can;
	#endif
	#if SUPPORT_NRF24L01
	COMM_INFO  nrf24l01;
	#endif
	uint8_t poll_motor_id;
	POLL_PACKAGE_TYPE poll_type;
}PROTOCOL_CTRL;




void package_init(void);

void package_analysis( COMM_INFO  *info , uint8_t data );

void char_to_package( COMM_INFO * info );
void package_to_char( COMM_MSG_PACKAGE package , uint8_t *data );
void package_send( COMM_MSG_PACKAGE package , COMM_SEND_TYPE send_type );

void package_poll_send(void);

extern PROTOCOL_CTRL protocol_ctrl;


#endif
