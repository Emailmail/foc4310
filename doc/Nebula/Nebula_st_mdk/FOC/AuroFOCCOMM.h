
#ifndef __AUROFOCCOMM_H_ 
#define __AUROFOCCOMM_H_

#include "protocol.h"


void AuroFOC_Recv_Data( COMM_INFO  *info );


void motor_detect_param( COMM_MSG_PACKAGE package );
void motor_work_mode_param( COMM_MSG_PACKAGE package );
void motor_pole_pair_param( COMM_MSG_PACKAGE package );

void motor_iq_pid_param( COMM_MSG_PACKAGE package );
void motor_id_pid_param( COMM_MSG_PACKAGE package );
void motor_speed_pid_param( COMM_MSG_PACKAGE package );
void motor_position_pid_param( COMM_MSG_PACKAGE package );

void motor_maxmin_iqid_param( COMM_MSG_PACKAGE package );
void motor_maxmin_speed_param( COMM_MSG_PACKAGE package );
void motor_maxmin_position_param( COMM_MSG_PACKAGE package );





void motor_aim_iq_param( COMM_MSG_PACKAGE package );
void motor_aim_id_param( COMM_MSG_PACKAGE package );
void motor_aim_speed_param( COMM_MSG_PACKAGE package );
void motor_aim_position_param( COMM_MSG_PACKAGE package );
void motor_aim_UqUd_param( COMM_MSG_PACKAGE package );


void motor_IaIbIc_param( COMM_MSG_PACKAGE package );
void motor_IqId_param( COMM_MSG_PACKAGE package );
void motor_speed_param( COMM_MSG_PACKAGE package );
void motor_position_param( COMM_MSG_PACKAGE package );
void motor_NONE_param( COMM_MSG_PACKAGE package );


void motor_poll_send(void);
#endif
