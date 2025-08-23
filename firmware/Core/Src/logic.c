/*
 * logic.c
 *
 *  Created on: Aug 5, 2025
 *      Author: cast
 */
#include <stdio.h>
#ifdef UNIT_TESTING

#include "../../Core/Inc/logic.h"
#include "../../Core/Inc/delay.h"
#include "../../Core/Inc/pid.h"
#include "../../Core/Inc/command_handler.h"
#include "../../Core/Inc/bmp280.h"
#else
#include "logic.h"
#include "delay.h"
#include "pid.h"
#include "bmp280.h"
#include "command_handler.h"
#endif




int  (*can_send)(uint32_t id,uint8_t *data);
void (*read_adc)(uint16_t*value);
void (*set_finger_position)(uint8_t motor , int32_t speed);
void (*set_motor_speed)(uint8_t motor,int32_t speed );


int set_finger_goal_position(uint32_t id , uint8_t*data);
int get_finger_goal_position(uint32_t id , uint8_t*data);
int get_adc_values(uint32_t id , uint8_t*data);
int motor_move(uint32_t id , uint8_t*data);
int stop_all_motors(uint32_t id , uint8_t*data);

pid_element_type pid_element[JOINT_COUNTS];
uint32_t target_position[JOINT_COUNTS];
uint16_t pid_interval=50;
uint8_t pid_enabled=0;

const INT_COMMAND_TYPE command_list[] =
{
{0, set_finger_goal_position},
{1, get_finger_goal_position},
{2, get_adc_values},
{3, motor_move},
{4, stop_all_motors}
};

int motor_move(uint32_t id , uint8_t*data)
{
	int32_t speed;
	 speed  = data[5];
	 speed<<=8;
	 speed |= data[4];
	 speed<<=8;
	 speed |= data[3];
	 speed<<=8;
	 speed |= data[2];
	set_motor_speed(data[1],speed);

	return 0;
}
int stop_all_motors(uint32_t id , uint8_t*data)
{
	pid_enabled=0;
	set_motor_speed(7,0);
	printf("stop\n");

	return 0;
}
int get_adc_values(uint32_t id , uint8_t*data)
{
	uint16_t adc_val[JOINT_COUNTS];
	uint8_t can_data[8];
	read_adc(adc_val);
	if (data[1] >= JOINT_COUNTS) return -1;
	can_data[1] = (adc_val[data[1]] & 0xff);
	can_data[0] = ((adc_val[data[1]] >>  8) & 0xff);
	can_send(id,can_data);
	printf("%d %d %d %d %d %d\n",adc_val[0],adc_val[1],adc_val[2],adc_val[3],adc_val[4],adc_val[5]);
    return 0;
}

int set_finger_goal_position(uint32_t id,uint8_t*data)
{

	int32_t position=0;
	position  = data[5];
	position<<=8;
	position |= data[4];
	position<<=8;
	position |= data[3];
	position<<=8;
	position |= data[2];

	if (data[1] >= JOINT_COUNTS) return -1;
	pid_enabled=1;
	target_position[data[1]]=position;
	printf("set pos[%d] = %d\n",data[1],target_position[data[1]]);
	return 0;
}

int get_finger_goal_position(uint32_t id,uint8_t*data)
{
	printf("get pos\n");
    return 0;
}


void init_pid_elements()
{
    for(int i=0;i<6;i++)
    {
    pid_element[i].dt=10;
    pid_element[i].kd=0;
    pid_element[i].ki=0;
    pid_element[i].kp=1;
    pid_element[i].max_command=3900;
    pid_element[i].min_command=50;
    pid_element[i].max_iterm=1200;
    pid_element[i].min_iterm=-1200;
    pid_element[i].max_output=1900;
    pid_element[i].min_output=-1900;
    }
}

void can_data_received(uint32_t id,uint8_t *data)
{
run_handler(data[0],id,data,command_list,(sizeof(command_list)/sizeof(command_list[0])));
}

void logic_init()
{
	int i;
	init_pid_elements();
	target_position[INDEX_FINGER]=1000;
	target_position[LITTLE_FINGER]=2500;
	target_position[THUMB2_FINGER]=2600;
	uint8_t data[]={1,2,3,4,5,6,7,8};
	printf("send\n");
	can_send(0x281,data);
	for(i=0;i<6;i++)
	{
		bmp280_init(i+1);
		delay_ms(100);
	}

    data[10]=9;
}

void logic_loop()
{
	int i;

	  for(i=0;i<6;i++){
		  printf("sensor num:%d pressure=%f  temp=%f \n",i,bmp280_read_pressure(1+i),bmp280_read_temperature(1+i));
	  	  delay_ms(100);
		  }
	if(!pid_enabled){delay_ms(100);return;}
    uint16_t values[JOINT_COUNTS] = {0};
    read_adc(values);

    delay_ms(pid_interval);
//    command= do_pid(target_position[LITTLE_FINGER],values[LITTLE_FINGER_FEEDBACK_CHANNEL],&pid_element[LITTLE_FINGER]);
//     printf("goal %d adc %d command %d\n",target_position[3],values[LITTLE_FINGER_FEEDBACK_CHANNEL],command);
//    set_finger_position(THUMB_FINGER  , do_pid(target_position[THUMB_FINGER],values[RING_FINGER_FEEDBACK_CHANNEL],&pid_element[INDEX_FINGER]));
//    set_finger_position(THUMB_FINGER2 , do_pid(target_position[THUMB_FINGER2],values[RING_FINGER_FEEDBACK_CHANNEL],&pid_element[INDEX_FINGER]));
//    set_finger_position(INDEX_FINGER  , do_pid(target_position[INDEX_FINGER],values[RING_FINGER_FEEDBACK_CHANNEL],&pid_element[INDEX_FINGER]));
      set_motor_speed(THUMB2_FINGER , do_pid(target_position[THUMB2_FINGER],values[THUMB2_FINGER_FEEDBACK_CHANNEL]   , &pid_element[THUMB2_FINGER]));
//    set_motor_speed(INDEX_FINGER , do_pid(target_position[INDEX_FINGER],values[INDEX_FINGER_FEEDBACK_CHANNEL]   , &pid_element[INDEX_FINGER]));
//    set_motor_speed(MIDDLE_FINGER , do_pid(target_position[MIDDLE_FINGER],values[MIDDLE_FINGER_FEEDBACK_CHANNEL], &pid_element[MIDDLE_FINGER]));
//    set_motor_speed(RING_FINGER   , do_pid(target_position[RING_FINGER]  , values[RING_FINGER_FEEDBACK_CHANNEL]   , &pid_element[RING_FINGER]));
//    set_motor_speed(LITTLE_FINGER , do_pid(target_position[LITTLE_FINGER], values[LITTLE_FINGER_FEEDBACK_CHANNEL] , &pid_element[LITTLE_FINGER]));

}

void logic_register_set_motor_speed(void (*set_motor_speed_callback)(uint8_t motor,int32_t speed ))
{
	set_motor_speed=set_motor_speed_callback;
}

void logic_register_adc(void (*read_adc_callback)(uint16_t*value))
{
    read_adc=read_adc_callback;
}

void logic_register_can_send(int can_send_callback(uint32_t id,uint8_t *data))
{
    can_send=can_send_callback;
}

