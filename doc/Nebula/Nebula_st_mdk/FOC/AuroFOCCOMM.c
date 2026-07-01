

#include "AuroFOCCOMM.h"
#include "MotorEvent.h"
#include "protocol.h"

//FOC Parameter recv
COMM_SEND_TYPE AuroFOC_send_type;

void AuroFOC_Recv_Data( COMM_INFO  *info){
	AuroFOC_send_type = info->send_type;
    if((info->package.func2 & (char)0xF0) == (char)0x00 ){
        switch ( info->package.func2) {
            case 0x00: motor_detect_param( info->package ); break;  //send motor id
            case 0x01: motor_work_mode_param( info->package );break; //set motor work mode
			case 0x02: motor_pole_pair_param( info->package );break; //send motor pole pair
        }
    }
    else if((info->package.func2 & (char)0xF0) == (char)0x10 ){
        switch ( info->package.func2) {
            case 0x10: motor_iq_pid_param( info->package ); break;  // set/get iq pid param
            case 0x11: motor_id_pid_param( info->package ); break;  // set/get id pid param
			case 0x12: motor_speed_pid_param( info->package ); break; // set/get speed pid param
			case 0x13: motor_position_pid_param( info->package ); break;// set/get position pid param
			case 0x14: motor_maxmin_iqid_param( info->package ); break; //  set/get iqid max min value
			case 0x15: motor_maxmin_speed_param( info->package ); break; // set/get speed max min value
			case 0x16: motor_maxmin_position_param( info->package ); break;  // set/get position max min value
        }
    }
    else if( (info->package.func2 & (char)0xF0) == (char)0x20 ){
        switch ( info->package.func2) {
            case 0x20: motor_aim_iq_param( info->package ); break;  // set/get iq aim value
            case 0x21: motor_aim_id_param( info->package ); break;  // set/get iq aim value
            case 0x22: motor_aim_speed_param( info->package ); break;  // set/get speed aim value
            case 0x23: motor_aim_position_param( info->package ); break;  // set/get position aim value
            case 0x24: motor_aim_UqUd_param( info->package ); break;  // set/get UqUd aim value
        }
    }
    else if( (info->package.func2 & (char)0xF0) == (char)0x30 ){
        switch ( info->package.func2 ) {
            case 0x30: motor_IaIbIc_param( info->package ); break;  // upload IaIbIc value
            case 0x31: motor_IqId_param( info->package ); break;   // upload IqId value
            case 0x32: motor_speed_param( info->package ); break;  // upload speed value
            case 0x33: motor_position_param( info->package ); break;  //upload position value
			case 0x3A: motor_NONE_param( info->package ); break;  //upload NONE
        }
    }
}

void motor_detect_param( COMM_MSG_PACKAGE package ){
	if( package.func3 == 0x00 ){
		package.data1.u32_data = 0xFFFF0000 + ( motorA.motorID << 0 );
		package.data2.u32_data = 0xFFFFFFFF;
		package.data3.u32_data = 0xFFFFFFFF;
		package.data4.u32_data = 0xFFFFFFFF;	
		package_send( package , AuroFOC_send_type );
	}
}


void motor_work_mode_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			if( package.data1.u32_data == position_loop_mode )
				motorA.as5047p.postion = 0;
			motorA.focmode = (AuroFOCMode)package.data1.u32_data;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.u32_data = motorA.focmode;
		}
		package_send( package , AuroFOC_send_type );
	}
}



void motor_pole_pair_param( COMM_MSG_PACKAGE package ){
	//read
	if( package.func3 == 0x00 ){
		if( package.motor_id == motorA.motorID ){
			package.data1.u32_data = motorA.Pole_Pair;
			package.data2.f = motorA.mechanical_offset;
		}
		package_send( package , AuroFOC_send_type );
	}
}



void motor_iq_pid_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.iq_pid.P = package.data1.f;
			motorA.iq_pid.I = package.data2.f;
			motorA.iq_pid.D = package.data3.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.iq_pid.P;
			package.data2.f = motorA.iq_pid.I;
			package.data3.f = motorA.iq_pid.D;
		}
		package_send( package , AuroFOC_send_type );
	}
}
void motor_id_pid_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.id_pid.P = package.data1.f;
			motorA.id_pid.I = package.data2.f;
			motorA.id_pid.D = package.data3.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.id_pid.P;
			package.data2.f = motorA.id_pid.I;
			package.data3.f = motorA.id_pid.D;
		}
		package_send( package , AuroFOC_send_type );
	}
}
void motor_speed_pid_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.speed_pid.P = package.data1.f;
			motorA.speed_pid.I = package.data2.f;
			motorA.speed_pid.D = package.data3.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.speed_pid.P;
			package.data2.f = motorA.speed_pid.I;
			package.data3.f = motorA.speed_pid.D;
		}
		package_send( package , AuroFOC_send_type );
	}
}
void motor_position_pid_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.position_pid.P = package.data1.f;
			motorA.position_pid.I = package.data2.f;
			motorA.position_pid.D = package.data3.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.position_pid.P;
			package.data2.f = motorA.position_pid.I;
			package.data3.f = motorA.position_pid.D;
		}
		package_send( package , AuroFOC_send_type );
	}
}

void motor_maxmin_iqid_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.limit.max_iq = package.data1.f;
			motorA.limit.min_iq = package.data2.f;
			motorA.limit.max_id = package.data3.f;
			motorA.limit.min_id = package.data4.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.limit.max_iq;
			package.data2.f = motorA.limit.min_iq;
			package.data3.f = motorA.limit.max_id;
			package.data4.f = motorA.limit.min_id;
		}                
		package_send( package , AuroFOC_send_type );
	}
	
}
void motor_maxmin_speed_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.limit.max_speed = package.data1.f;
			motorA.limit.min_speed = package.data2.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			 package.data1.f = motorA.limit.max_speed ;
			 package.data2.f = motorA.limit.min_speed ;
		}               
		package_send( package , AuroFOC_send_type );
	}
}
void motor_maxmin_position_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.limit.max_position = package.data1.f;
			motorA.limit.min_position = package.data2.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.limit.max_position;
			package.data2.f = motorA.limit.min_position;
		}               
		package_send( package , AuroFOC_send_type );
	}
}


void motor_aim_iq_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.aim.aim_iq = package.data1.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.aim.aim_iq;
		}                 
		package_send( package , AuroFOC_send_type );
	}
}
void motor_aim_id_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.aim.aim_id = package.data1.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.aim.aim_id;
		}               
		package_send( package , AuroFOC_send_type );
	}
}
void motor_aim_speed_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.aim.aim_speed = package.data1.f / motorA.as5047p.speed_rpm_param;
			motorA.aim.aim_speeed_rpm = package.data1.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.aim.aim_speed;
		}              
		package_send( package , AuroFOC_send_type );
	}
}
void motor_aim_position_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.aim.aim_position = package.data1.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.aim.aim_position;
		}             
		package_send( package , AuroFOC_send_type );
	}
}
void motor_aim_UqUd_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x01 ){
		if( package.motor_id == motorA.motorID ){
			motorA.Uq = package.data1.f;
			motorA.Ud = package.data2.f;
		}
		else;
		package_send( package , AuroFOC_send_type );
	}
	else{ //read
		if( package.motor_id == motorA.motorID ){
			package.data1.f = motorA.Uq;
			package.data2.f = motorA.Ud;
		}                
		package_send( package , AuroFOC_send_type );
	}

}

void motor_IaIbIc_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x00 ){
		protocol_ctrl.poll_motor_id = package.motor_id;
		protocol_ctrl.poll_type = POLL_PACKAGE_TYPE_IaIbIc;
	//	package_send( package );
	}
}
void motor_IqId_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x00 ){
		protocol_ctrl.poll_motor_id = package.motor_id;
		protocol_ctrl.poll_type = POLL_PACKAGE_TYPE_IqId;
	//	package_send( package );
	}
}
void motor_speed_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x00 ){
		protocol_ctrl.poll_motor_id = package.motor_id;
		protocol_ctrl.poll_type = POLL_PACKAGE_TYPE_Speed;
	//	package_send( package );
	}
}
void motor_position_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x00 ){
		protocol_ctrl.poll_motor_id = package.motor_id;
		protocol_ctrl.poll_type = POLL_PACKAGE_TYPE_Position;
	//	package_send( package );
	}
}
void motor_NONE_param( COMM_MSG_PACKAGE package ){
	//write
	if( package.func3 == 0x00 ){
		protocol_ctrl.poll_motor_id = package.motor_id;
		protocol_ctrl.poll_type = POLL_PACKAGE_TYPE_NONE;
	//	package_send( package );
	}
}


/* 上报电机相关数据 */
void motor_poll_send(void){
    COMM_MSG_PACKAGE package;
	package.head = 0xDE;
	if( protocol_ctrl.poll_type == POLL_PACKAGE_TYPE_NONE ) 
		return;
	else if( protocol_ctrl.poll_type == POLL_PACKAGE_TYPE_IaIbIc ){  
		package.func1 = 0x1A;
		package.func2 = 0x30;
		package.func3 = 0x00;
		if( protocol_ctrl.poll_motor_id == motorA.motorID ){
			package.data1.f = motorA.Ia;
			package.data2.f = motorA.Ib;
			package.data3.f = motorA.Ic;
		}
		else
			return;
	}
	else if( protocol_ctrl.poll_type == POLL_PACKAGE_TYPE_IqId ){  
		package.func1 = 0x1A;
		package.func2 = 0x31;
		package.func3 = 0x00;
		if( protocol_ctrl.poll_motor_id == motorA.motorID ){
			package.data1.f = motorA.Iq;
			package.data2.f = motorA.Id;
			package.data3.f = motorA.aim.aim_iq;
			package.data4.f = motorA.aim.aim_id;
		}
		else
			return;
	}
	else if( protocol_ctrl.poll_type == POLL_PACKAGE_TYPE_Speed ){ 
		package.func1 = 0x1A;
		package.func2 = 0x32;
		package.func3 = 0x00;
		if( protocol_ctrl.poll_motor_id == motorA.motorID ){
			package.data1.f = motorA.as5047p.speed_rpm;
			package.data2.f = motorA.aim.aim_speeed_rpm;
			package.data3.f = motorA.as5047p.angle;
			package.data4.f = motorA.e_angle;
		}
		else
			return;
	}
	else if( protocol_ctrl.poll_type == POLL_PACKAGE_TYPE_Position ){
		package.func1 = 0x1A;
		package.func2 = 0x33;
		package.func3 = 0x00;
		if( protocol_ctrl.poll_motor_id == motorA.motorID ){
			package.data1.f = motorA.as5047p.postion;
			package.data2.f = motorA.aim.aim_position;
		}
		else
			return;
	}
	else
		return;
	package.id = 0x00;
	package.tail = 0xED;
	package_send( package , AuroFOC_send_type );
}
