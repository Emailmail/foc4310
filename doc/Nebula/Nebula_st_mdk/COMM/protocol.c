
#include "protocol.h"
#include "AuroFOCCOMM.h"
#include "usbd_cdc_if.h"
#include "MotorEvent.h"
#include "usart.h"
PROTOCOL_CTRL protocol_ctrl;
void package_init(void){
	uint16_t i;
	#if SUPPORT_UART
		protocol_ctrl.uart.protocol_status = inst_status;
		protocol_ctrl.uart.send_type = uart_type;
		Link_init(&protocol_ctrl.uart.link);
		protocol_ctrl.uart.package_curr_len = 0;
	#endif	
	
	#if SUPPORT_USB_CDC
		protocol_ctrl.usb_cdc.protocol_status = inst_status;
		protocol_ctrl.usb_cdc.send_type = usb_cdc_type;
		Link_init(&protocol_ctrl.usb_cdc.link);
		protocol_ctrl.usb_cdc.package_curr_len = 0;
	#endif
	
	#if SUPPORT_RS485	
		protocol_ctrl.rs485.protocol_status = inst_status;
		protocol_ctrl.rs485.send_type = rs485_type;
		Link_init(&protocol_ctrl.rs485.link);
		protocol_ctrl.rs485.package_curr_len = 0;
	#endif
	#if SUPPORT_CAN
		protocol_ctrl.can.protocol_status = inst_status;
		protocol_ctrl.can.send_type = can_type;
		Link_init(&protocol_ctrl.can.link);
		protocol_ctrl.can.package_curr_len = 0;
	#endif
	#if SUPPORT_NRF24L01
		protocol_ctrl.nrf24l01.protocol_status = inst_status;
		protocol_ctrl.nrf24l01.send_type = nrf24l01_type;
		Link_init(&protocol_ctrl.nrf24l01.link);
		protocol_ctrl.nrf24l01.package_curr_len = 0;	
	#endif
	


	
}
void char_to_package( COMM_INFO * info ){
	info->package.head  		  = info->package_buff[0];
	info->package.func1 	      = info->package_buff[1];
	info->package.func2         = info->package_buff[2];
	info->package.func3         = info->package_buff[3];
	info->package.data1.data[0] = info->package_buff[4];
	info->package.data1.data[1] = info->package_buff[5];
	info->package.data1.data[2] = info->package_buff[6];
	info->package.data1.data[3] = info->package_buff[7];
	info->package.data2.data[0] = info->package_buff[8];
	info->package.data2.data[1] = info->package_buff[9];
	info->package.data2.data[2] = info->package_buff[10];
	info->package.data2.data[3] = info->package_buff[11];
	info->package.data3.data[0] = info->package_buff[12];
	info->package.data3.data[1] = info->package_buff[13];
	info->package.data3.data[2] = info->package_buff[14];
	info->package.data3.data[3] = info->package_buff[15];
	info->package.data4.data[0] = info->package_buff[16];
	info->package.data4.data[1] = info->package_buff[17];
	info->package.data4.data[2] = info->package_buff[18];
	info->package.data4.data[3] = info->package_buff[19];
	info->package.motor_id      = (info->package_buff[20] << 8) + info->package_buff[21];
	info->package.id     	      = info->package_buff[22];
	info->package.tail          = info->package_buff[23];
}
void package_to_char( COMM_MSG_PACKAGE package , uint8_t *data ){
	data[0] = package.head;
	data[1]   = package.func1; 	      
	data[2]   = package.func2;         
	data[3]   = package.func3;         
	data[4]   = package.data1.data[0]; 
	data[5]   = package.data1.data[1]; 
	data[6]   = package.data1.data[2]; 
	data[7]   = package.data1.data[3]; 
	data[8]   = package.data2.data[0]; 
	data[9]   = package.data2.data[1]; 
	data[10]  = package.data2.data[2];
	data[11]  = package.data2.data[3];
	data[12]  = package.data3.data[0];
	data[13]  = package.data3.data[1];
	data[14]  = package.data3.data[2];
	data[15]  = package.data3.data[3];
	data[16]  = package.data4.data[0];
	data[17]  = package.data4.data[1];
	data[18]  = package.data4.data[2];
	data[19]  = package.data4.data[3];
	data[20]  = package.motor_id >> 8;
	data[21]  = package.motor_id & 0xff;
	data[22]  = package.id;
	data[23]  = package.tail;
}


void package_analysis( COMM_INFO  *info ,uint8_t data ){
	uint16_t i;
	Link_insert_ultra( &info->link , data);
	info->package_curr_len += 1;
	if( info->package_curr_len != PACKAGE_SIZE )
		return;
	else
		info->package_curr_len -= 1;
	if( *info->link.head != PACKAGE_HEAD || *info->link.tail != PACKAGE_TAIL )
		return;
	info->package_buff[0] = Link_Read_Data(&info->link , ENABLE);
	for( i = 1 ; i < PACKAGE_SIZE ; i++ )
		info->package_buff[i] = Link_Read_Data(&info->link , DISABLE);
	info->package_curr_len = 0;
	char_to_package( info );
	if( info->protocol_status == inst_status ){
		switch( info->package.func1 ){
			case 0x1A: AuroFOC_Recv_Data( info )	; break;   //FOC inst
//			case 0x2A: AuroCar_Recv_Data( package ) ; break;   //car inst
//			case 0x8A: AuroUI_Recv_Data( package ) ; package_rx_buff_clear( info , info->len );break;   //ui inst
		}
	}
	else if( info->protocol_status == file_status ){
//		AuroUI_Recv_RealData( info->rx_buff , info->len );
	}
}


/* 上报数据 */
void package_poll_send(void){
	motor_poll_send(); //上报数据
}

//package send
uint8_t data[24];
void package_send( COMM_MSG_PACKAGE package , COMM_SEND_TYPE send_type ){
	package_to_char( package , data);
	if( send_type == uart_type ){
		#if SUPPORT_UART
		//	HAL_UART_Transmit_DMA( &huart1 , data , sizeof(data));
			HAL_UART_Transmit( &huart1 , data , sizeof(data) , 0xFFFF);
		#endif
	}
	else if( send_type == usb_cdc_type ){
		#if SUPPORT_USB_CDC
			CDC_Transmit_FS(data, sizeof(data));
		#endif
	
	}
	else if( send_type == rs485_type ){
		#if SUPPORT_RS485
		
		#endif
	}
	else if( send_type == can_type ){	
		#if SUPPORT_CAN
		
		#endif
	}
	else if( send_type == can_type ){	
		#if SUPPORT_NRF24L01
			nrf24l01_send_packet( &nrf24l01 , data);
		#endif
	}
}
