#include "MotorEvent.h"
#include "AuroFOC.h"
#include "AuroPID.h"
#include "as5047p_bsp_drv.h"
#include "adc.h"
#include "AuroFOCCOMM.h"
AuroFOC motorA;


void MotorEvent_Start(void){
	motorA.motorID = 10;
	motorA.htim = &htim1;
	motorA.hadc = &hadc1;
	motorA.as5047p.hspi  = &hspi1;
	motorA.as5047p.CS_Pin = SPI_CSN_Pin;
	motorA.as5047p.CS_Port = SPI_CSN_GPIO_Port;
	AS5047P_bsp_init( &motorA.as5047p);
	motorA.Uq = 1.5;
	motorA.Ud = 0.0;
	motorA.Ts = 1;
	motorA.Udc = 12;
	motorA.K = SQRT_3*motorA.Ts/motorA.Udc;
	motorA.limit.max_uq = 6;
	motorA.limit.max_ud = 6;
	motorA.limit.max_iq = 4;
	motorA.limit.max_id = 4;
	motorA.limit.max_speed = 1000;
	motorA.limit.max_position = 5000;
	motorA.limit.min_uq = -6;
	motorA.limit.min_ud = -6;
	motorA.limit.min_iq = -4;
	motorA.limit.min_id = -4;
	motorA.limit.min_speed = -1000;
	motorA.limit.min_position = -5000;
	motorA.Pole_Pair = 7;
	motorA.motor_rotate_direct = 1;
	motorA.mechanical_offset = 301.464;
	motorA.focmode = stop_loop_mode;
	motorA.aim.aim_iq = 0;
	motorA.aim.aim_id = 0;
	motorA.aim.aim_speed = 0;
	motorA.aim.aim_position = 0;
	PID_Init( &motorA.iq_pid , 2.5f , 0.0045f , 0 , 300);
	PID_Init( &motorA.id_pid , 2.1f , 0.0012f , 0 , 300);
	PID_Init( &motorA.speed_pid , 0.0985f , 0.00185f , 0 , 400);
	PID_Init( &motorA.position_pid , 4.0f , 0.012f , 0 , 200);
	
	foc_start_adc( &motorA );
	foc_start_pwm( &motorA );
}

/** adc注入中断**/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
    if( hadc == &hadc1){
		foc_update_adc( &motorA , hadc1.Instance->JDR1 , hadc1.Instance->JDR2 , 0 );
		foc_control( &motorA );
    }
}
