
#include "User_App.h"
#include "MotorEvent.h"
#include "protocol.h"
#include "as5047p_bsp_drv.h"
#include "usart.h"

#include "LSM6DS3TR_bsp_drv.h"
uint8_t uart1_rx_data;
void App_Main(){	
	MotorEvent_Start();             //启动电机    
	package_init();					//初始化通信协议
	
	HAL_TIM_Base_Start_IT(&htim6);  //启动定时器6, 1ms上报一次数据
	HAL_UART_Receive_IT( &huart1 , &uart1_rx_data , 1 );  //启动接收中断 


    LSM6DS3TR t;
	t.iic.SCL_Pin = GPIO_PIN_13;
	t.iic.SDA_Pin = GPIO_PIN_14;
	t.iic.SCL_Port = GPIOC;
	t.iic.SDA_Port = GPIOC;
	uint8_t id;
	id = LSM6DS3TR_bsp_read_id(&t);
	while(1){     
	}                  
}	


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM6 )  //定时器中断 1ms
	{
		package_poll_send();  //上报数据
	}
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart){
	if( huart == &huart1 ){
		package_analysis( &protocol_ctrl.uart , uart1_rx_data );
		HAL_UART_Receive_IT( &huart1 , &uart1_rx_data , 1 );
	}

}