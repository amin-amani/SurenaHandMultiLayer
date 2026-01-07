/*
 * logic.c
 *
 *  Created on: Aug 5, 2025
 *      Author: cast
 */
#include <stdio.h>
#include <inttypes.h>


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
#include "command_handler.h"
#endif




int  (*can_send)(uint32_t id,uint8_t *data);
void (*read_adc)(uint16_t*value);
void (*set_finger_position)(uint8_t motor , int32_t speed);
void (*set_motor_speed)(uint8_t motor,int32_t speed );
void (*set_servo_position)(uint8_t  *position);
float (*sensor_read_pressure)(int index);
float (*sensor_read_temperature)(int index);
bool (*sensor_init)(int index);
void (*toggle_test_pin)(void);

int set_finger_goal_position(uint32_t id , uint8_t*data);
int set_pressure_limits(uint32_t id , uint8_t*data);
int set_finger_pid_values(uint32_t id , uint8_t*data);
int get_finger_goal_position(uint32_t id , uint8_t*data);
int get_adc_values(uint32_t id , uint8_t*data);
int motor_move(uint32_t id , uint8_t*data);
int stop_all_motors(uint32_t id , uint8_t*data);
int get_pressure_values(uint32_t id , uint8_t*data);
int set_servo_goal_position(uint32_t id,uint8_t*data);
int set_finger_goal_position_amani(uint32_t id,uint8_t*data);

static pid_element_type pid_element[JOINT_COUNT];
static uint32_t target_position[JOINT_COUNT];
static uint32_t pressure_limits[SENSOR_COUNT];

static uint8_t pid_enabled=0;
static uint8_t control_data=0;
const int error_tolerance = 20;
int max_limits[6]={INDEX_MAX,MIDDLE_MAX,RING_MAX,LITTLE_MAX,THUMB_MAX,THUMB2_MAX};
int min_limits[6]={INDEX_MIN,MIDDLE_MIN,RING_MIN,LITTLE_MIN ,THUMB_MIN,THUMB2_MIN};
#define SENSOR_FILTER_MEM_COUNT (7)
float pressure_offset_mem[SENSOR_COUNT][SENSOR_FILTER_MEM_COUNT];
float pressure_offset[SENSOR_COUNT];
const INT_COMMAND_TYPE command_list[] =
{
    {0, set_finger_goal_position_amani},
    {1, get_finger_goal_position},
    {2, get_adc_values},
    {3, motor_move},
    {4, stop_all_motors},
    {5, get_pressure_values},
    {6, set_servo_goal_position},


    {10, set_finger_goal_position},
    {11, set_pressure_limits},
    {12, set_finger_pid_values},

};
LOGIC_BMP_OUTPUT_TYPE bmp_sensor_value[SENSOR_COUNT];

pid_element_type* get_pid_emelents()
{
 return pid_element;
}

uint32_t* get_target_positions()
{

    return target_position;
}
void logic_init()
{

    init_pid_elements();
    init_servo_positions();
    init_finger_positions();
    init_pressure_sensors();
    send_can_statup_command();

}

void logic_loop()
{

    update_presure_sensors();//33ms or 3ms


//    if(pid_enabled) {
//        run_pid_for_all_fingers();
//    }
}

uint8_t get_pid_status(){
    return pid_enabled;
}

uint8_t get_trigger_status()
{
    return control_data;
}

static void sort_float(float *arr, uint8_t size)
{
    for (uint8_t i = 1; i < size; i++)
    {
        float key = arr[i];
        int8_t j = i - 1;


        while (j >= 0 && (arr[j] - key) > 1e-6f)  // More robust
        {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

static void filter_float_values(float *filtered_value, float sen_raw_value[SENSOR_COUNT][SENSOR_FILTER_MEM_COUNT])
{
    float temp[SENSOR_FILTER_MEM_COUNT];

    for (int ch = 0; ch < SENSOR_COUNT; ch++)
    {

        for (int i = 0; i < SENSOR_FILTER_MEM_COUNT; i++)
        {
            temp[i] = sen_raw_value[ch][i];
        }

        sort_float(temp, SENSOR_FILTER_MEM_COUNT);


        filtered_value[ch] = temp[SENSOR_FILTER_MEM_COUNT / 2];
    }
}

void update_presure_sensors()
{

    static uint8_t sensor_idx = 0;
    static uint8_t sample_idx = 0;
    static uint8_t offset_ready = 0;

    bmp_sensor_value[sensor_idx].pressure = sensor_read_pressure(sensor_idx + 1);

    if (!offset_ready)
    {
        pressure_offset_mem[sensor_idx][sample_idx] = bmp_sensor_value[sensor_idx].pressure;

        sensor_idx++;

        if (sensor_idx >= SENSOR_COUNT)
        {
            sensor_idx = 0;
            sample_idx++;
        }

        if (sample_idx >= SENSOR_FILTER_MEM_COUNT)
        {
            filter_float_values(pressure_offset, pressure_offset_mem);
            printf("pressure offset %8.3f %8.3f %8.3f %8.3f %8.3f \n",
            		pressure_offset[0],
            		pressure_offset[1],
					pressure_offset[2],
					pressure_offset[3],
					pressure_offset[4]
																				   );
            offset_ready = 1;
        }

        return;
    }


    sensor_idx++;
    if (sensor_idx >= SENSOR_COUNT)
        sensor_idx = 0;
}

int motor_move(uint32_t id , uint8_t*data)
{
	if(id!=DEVICE_CAN_ID)return 0;
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
	if(id!=DEVICE_CAN_ID)return 0;
	pid_enabled = 0;
	control_data = 0;
	
	// Stop all motors
	for (int i = 0; i < JOINT_COUNT+1; i++) {
		set_motor_speed(i, 0);
	}
	printf("stop all motors.\n");
	set_motor_speed(7,0);
	return 0;
}

int get_adc_values(uint32_t id , uint8_t*data)
{
	if(id!=DEVICE_CAN_ID)return 0;
	uint16_t adc_val[JOINT_COUNT];
	uint8_t can_data[8];
	read_adc(adc_val);
	if (data[1] >= JOINT_COUNT) return -1;
	can_data[1] = (adc_val[data[1]] & 0xff);
	can_data[0] = ((adc_val[data[1]] >>  8) & 0xff);
	can_send(id,can_data);
	printf("%d %d %d %d %d %d\n",adc_val[0],adc_val[1],adc_val[2],adc_val[3],adc_val[4],adc_val[5]);
    return 0;
}

int get_pressure_values(uint32_t id , uint8_t*data)
{	
	if(id!=DEVICE_CAN_ID)return 0;
	uint8_t can_data[8] = {0};

	memcpy(&can_data[0], &(bmp_sensor_value[data[1]].pressure), sizeof(float));
	can_send(id, can_data);
	return 0;
}

int set_finger_goal_position_amani(uint32_t id,uint8_t*data)
{
	if(id!=DEVICE_CAN_ID)return 0;
    int32_t position=0;
    position  = data[5];
    position<<=8;
    position |= data[4];
    position<<=8;
    position |= data[3];
    position<<=8;
    position |= data[2];

    if (data[1] >= JOINT_COUNT) return -1;
    if(position > max_limits[data[1]]) position = max_limits[data[1]];
    if(position < min_limits[data[1]]) position = min_limits[data[1]];

	target_position[data[1]] = position;

    pid_enabled=1;

    printf("set pos[%d] = %d\n",data[1],target_position[data[1]]);
    return 0;
}
#ifdef RIGHT_HAND
int set_finger_goal_position(uint32_t id,uint8_t*data)
{
	// Data Format: [command_id, motor0, motor1, motor2, motor3, motor4, motor5, control_data]
	// Each motor value represents position for that finger
	if(id!=DEVICE_CAN_ID)return 0;
	uint8_t control_data = data[7];
	
	if (control_data != 0) pid_enabled = 1;
	
	switch (control_data)
	 {
		 case 1:
			 if (data[1] != 0) target_position[INDEX_FINGER]  = INDEX_MIN + data[1] * (INDEX_MAX-INDEX_MIN) / 255;
			 if (data[2] != 0) target_position[MIDDLE_FINGER] = MIDDLE_MIN + data[2] * (MIDDLE_MAX-MIDDLE_MIN) / 255;
			 if (data[3] != 0) target_position[RING_FINGER]   = RING_MIN + data[3] * (RING_MAX-RING_MIN) / 255;
			 if (data[4] != 0) target_position[LITTLE_FINGER] = LITTLE_MIN + data[4] * (LITTLE_MAX-LITTLE_MIN) / 255;
			 if (data[5] != 0) target_position[THUMB_FINGER]  = THUMB_MIN + data[5] * (THUMB_MAX-THUMB_MIN) / 255;
			 if (data[6] != 0) target_position[THUMB2_FINGER] = THUMB2_MIN + data[6] * (THUMB2_MAX-THUMB2_MIN) / 255;
			 break;

		 case 2:
			 target_position[INDEX_FINGER]  = (INDEX_MAX+INDEX_MIN)/2 + (INDEX_MAX-INDEX_MIN)/3;
			 target_position[MIDDLE_FINGER] = (MIDDLE_MAX+MIDDLE_MIN)/2 + (MIDDLE_MAX-MIDDLE_MIN)/6;
			 target_position[RING_FINGER]   = (RING_MAX+RING_MIN)/2;
			 target_position[LITTLE_FINGER] = (LITTLE_MAX+LITTLE_MIN)/2 - (LITTLE_MAX-LITTLE_MIN)/6;
			 target_position[THUMB_FINGER]  = (THUMB_MAX+THUMB_MIN)/2;
			 target_position[THUMB2_FINGER] = (THUMB2_MAX+THUMB2_MIN)/2 + (THUMB2_MAX-THUMB2_MIN)/3;
			 break;

		 case 3:
			 target_position[INDEX_FINGER]  = (INDEX_MAX+INDEX_MIN)/2 - (INDEX_MAX-INDEX_MIN)/6;
			 target_position[MIDDLE_FINGER] = (MIDDLE_MAX+MIDDLE_MIN)/2;
			 target_position[RING_FINGER]   = (RING_MAX+RING_MIN)/2 + (RING_MAX-RING_MIN)/6;
			 target_position[LITTLE_FINGER] = (LITTLE_MAX+LITTLE_MIN)/2 + (LITTLE_MAX-LITTLE_MIN)/3;
			 target_position[THUMB_FINGER]  = (THUMB_MAX+THUMB_MIN)/2;
			 target_position[THUMB2_FINGER] = (THUMB2_MAX+THUMB2_MIN)/2 + (THUMB2_MAX-THUMB2_MIN)/3;
			 break;

		 default:
			 break;
	 }
	
	return 0;
}
#else
int set_finger_goal_position(uint32_t id,uint8_t*data)
{
	// Data Format: [command_id, motor0, motor1, motor2, motor3, motor4, motor5, control_data]
	// Each motor value represents position for that finger
	if(id!=DEVICE_CAN_ID)return 0;
	uint8_t control_data = data[7];

	if (control_data != 0) pid_enabled = 1;

	switch (control_data)
	 {
		 case 1:
			 if (data[1] != 0) target_position[INDEX_FINGER]  = INDEX_MAX - data[1] * (INDEX_MAX-INDEX_MIN) / 255;
			 if (data[2] != 0) target_position[MIDDLE_FINGER] = MIDDLE_MAX - data[2] * (MIDDLE_MAX-MIDDLE_MIN) / 255;
			 if (data[3] != 0) target_position[RING_FINGER]   = RING_MAX - data[3] * (RING_MAX-RING_MIN) / 255;
			 if (data[4] != 0) target_position[LITTLE_FINGER] = LITTLE_MAX - data[4] * (LITTLE_MAX-LITTLE_MIN) / 255;
			 if (data[5] != 0) target_position[THUMB_FINGER]  = THUMB_MIN + data[5] * (THUMB_MAX-THUMB_MIN) / 255;
			 if (data[6] != 0) target_position[THUMB2_FINGER] = THUMB2_MAX - data[6] * (THUMB2_MAX-THUMB2_MIN) / 255;
			 printf("com %d %d %d %d %d %d\n",
					 target_position[INDEX_FINGER],
					 target_position[MIDDLE_FINGER],
					 target_position[RING_FINGER],
					 target_position[LITTLE_FINGER],
					 target_position[THUMB_FINGER],
					 target_position[THUMB2_FINGER]


			 );
			 break;

		 case 2:

			 break;

		 case 3:

			 break;

		 default:
			 break;
	 }

	return 0;
}

#endif

int set_pressure_limits(uint32_t id,uint8_t*data)
{
	// Each sensor value represents target pressure for that sensor
	if(id!=DEVICE_CAN_ID)return 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
        pressure_limits[i] = data[i+1] * 50; // Scale up for better precision
	}
	return 0;
}

int set_finger_pid_values(uint32_t id,uint8_t*data)
{
	// For simplicity, we'll set Kp, Kd, and Ki equal for all motors
	// Data Format: [command_id, Kp, Ki, Kd]
	if(id!=DEVICE_CAN_ID)return 0;
	for (int i = 0; i < JOINT_COUNT; i++) {
        pid_element[i].kp = data[1];
        pid_element[i].ki = data[2];
        pid_element[i].kd = data[3];
	}
	pid_element[0].kp *= -1;
	pid_element[2].kp *= -1;
	pid_element[3].kp *= -1;
	pid_element[4].kp *= -1;
	pid_element[5].kp *= -1;
	
	return 0;
}

int set_servo_goal_position(uint32_t id,uint8_t*data)
{
	if(id!=DEVICE_CAN_ID && id != SHARED_CAN_ID) return 0;
	
	uint8_t position[3];
#ifdef RIGHT_HAND
	position[0] = data[1];
	position[1] = data[2];
	position[2] = data[3];
#else
	position[0] = data[4];
	position[1] = data[5];
	position[2] = data[6];
#endif

	set_servo_position(position);
//	printf("set servo pos %d %d %d (trigger=%d)\n",
//		   position[0], position[1], position[2], trigger);
	return 0;
}


int get_finger_goal_position(uint32_t id,uint8_t*data)
{
	if(id!=DEVICE_CAN_ID)return 0;
	printf("get pos\n");
    return 0;
}

void init_pid_elements()
{
    for(int i=0;i<JOINT_COUNT;i++)
    {
		pid_element[i].dt=2;
		pid_element[i].kd=0;
		pid_element[i].ki=0;
		pid_element[i].kp=100;
		pid_element[i].max_command=3900;
		pid_element[i].min_command=50;
		pid_element[i].max_iterm=1200;
		pid_element[i].min_iterm=-1200;
		pid_element[i].max_output=1900;
		pid_element[i].min_output=-1900;
    }
#ifdef RIGHT_HAND
    pid_element[0].kp *= -1;
    pid_element[2].kp *= -1;
    pid_element[3].kp *= -1;
    pid_element[4].kp *= -1;
    pid_element[5].kp *= -1;
#else
    pid_element[1].kp *= -1;
    pid_element[4].kp *= -1;
    pid_element[5].kp *= -1;
#endif
}

void can_data_received(uint32_t id,uint8_t *data)
{
run_handler(data[0],id,data,command_list,(sizeof(command_list)/sizeof(command_list[0])));
}

void init_finger_positions()
{
	pid_enabled=0;
	control_data=0;

    target_position[INDEX_FINGER]  = (INDEX_MAX+INDEX_MIN)/2 + (INDEX_MAX-INDEX_MIN)/3;
    target_position[MIDDLE_FINGER] = (MIDDLE_MAX+MIDDLE_MIN)/2 + (INDEX_MAX-INDEX_MIN)/6;
    target_position[RING_FINGER]   = (RING_MAX+RING_MIN)/2;
    target_position[LITTLE_FINGER] = (LITTLE_MAX+LITTLE_MIN)/2 - (INDEX_MAX-INDEX_MIN)/6;
    target_position[THUMB_FINGER]  = (THUMB_MAX+THUMB_MIN)/2;
    target_position[THUMB2_FINGER] = (THUMB2_MAX+THUMB2_MIN)/2 + (INDEX_MAX-INDEX_MIN)/3;
    
    for (int i = 0; i < SENSOR_COUNT; i++) { pressure_limits[i] = 0; }
}
void init_servo_positions(void)
{
	uint8_t servo_init_angles[]={90,90,90};
	set_servo_position(servo_init_angles);
}
void init_pressure_sensors()
{

   bool result=0;
	printf("start init sensor ...\n");
    for(int i=0;i<5;i++)
    {
      	printf("start init sensor %d ...\n",i+1);
    	for(int j=0;j<30;j++)
    	{
            result=sensor_init(i+1);
    		if(result==true)
    		{
    			j=30;
    		}


    	}
    	printf("init sensor %d = %d\n",i+1,result);
    }
}

void send_can_statup_command()
{
    uint8_t data[]={8,2,3,4,5,6,7,8};
    printf("send\n");
    can_send(0x281,data);
}

int all_fingers_reached_target(uint16_t * feeback_values,uint32_t *target_position , int threshold)
{
	int i=0;
	for(i=0 ; i<JOINT_COUNT ; i++ )
	{
		if(abs(target_position[i]-feeback_values[i])>threshold)
		return 0;
	}
return 1;

}

int all_finger_position_in_range(uint16_t *current_position)
{

if(current_position[0]<INDEX_MIN || current_position[0] >INDEX_MAX)return -1;
if(current_position[1]<MIDDLE_MIN || current_position[1] >MIDDLE_MAX)return -1;
if(current_position[2]<RING_MIN || current_position[2] >RING_MAX)return -1;
//if(current_position[3]<LITTLE_MIN || current_position[3] >LITTLE_MAX)return -1;
if(current_position[4]<THUMB_MIN || current_position[4] >THUMB_MAX)return -1;
if(current_position[5]<THUMB2_MIN || current_position[5] >THUMB2_MAX)return -1;
return 0;
}
void run_pid_for_all_fingers()
{
	static int step = 0;
	uint16_t values[JOINT_COUNT] = {0};
	//every 1.67ms if with or without return

	if(!pid_enabled)
	{
	return  ;
	}
	read_adc(values);
	toggle_test_pin();
	if(all_fingers_reached_target(values,target_position,error_tolerance))
	{

		pid_enabled= 0;
		set_motor_speed(7,0);
		return;
	}
//	if(all_finger_position_in_range(values)<0)
//	{
//		pid_enabled= 0;
//		set_motor_speed(7,0);
//		return;
//	}

	 int phase = step / 10;

	 switch (phase)
	 {
		 case 0:
			 set_motor_speed(INDEX_FINGER , do_pid(target_position[INDEX_FINGER],values[INDEX_FINGER_FEEDBACK_CHANNEL]   , &pid_element[INDEX_FINGER]));
			 break;
		 case 1:
			 set_motor_speed(MIDDLE_FINGER , do_pid(target_position[MIDDLE_FINGER],values[MIDDLE_FINGER_FEEDBACK_CHANNEL], &pid_element[MIDDLE_FINGER]));
		 break;
		 case 2:
			 set_motor_speed(RING_FINGER   , do_pid(target_position[RING_FINGER]  , values[RING_FINGER_FEEDBACK_CHANNEL]   , &pid_element[RING_FINGER]));
		 break;
		 case 3:
			 set_motor_speed(LITTLE_FINGER , do_pid(target_position[LITTLE_FINGER], values[LITTLE_FINGER_FEEDBACK_CHANNEL] , &pid_element[LITTLE_FINGER]));
		 break;
		 case 4:
			 set_motor_speed(THUMB_FINGER,do_pid(target_position[THUMB_FINGER],values[THUMB_FEEDBACK_CHANNEL],&pid_element[THUMB_FINGER]));
		 break;
		 case 5:
			 set_motor_speed(THUMB2_FINGER , do_pid(target_position[THUMB2_FINGER],values[THUMB2_FINGER_FEEDBACK_CHANNEL]   , &pid_element[THUMB2_FINGER]));
		 break;
		 default:
			 break;
	 }

	 step++;
	 if (step >= 60)
	 {
		 step = 0;
	 }


}

void logic_register_set_servo_position(void set_servo_position_callback(uint8_t  *position))
{
	set_servo_position = set_servo_position_callback;
}
void logic_register_set_motor_speed(void (*set_motor_speed_callback)(uint8_t motor,int32_t speed ))
{
	set_motor_speed = set_motor_speed_callback;
}

void logic_register_adc(void (*read_adc_callback)(uint16_t*value))
{
    read_adc = read_adc_callback;
}

void logic_register_can_send(int can_send_callback(uint32_t id,uint8_t *data))
{
    can_send = can_send_callback;
}

void logic_register_read_pressure(float (*sensor_read_pressure_callback)(int index))
{
	sensor_read_pressure = sensor_read_pressure_callback;
}

void logic_register_read_temperature(float (*sensor_read_temperature_callback)(int index))
{
	sensor_read_temperature=sensor_read_temperature_callback;
}
void  logic_register_sensor_init(bool (*sensor_init_callback)(int index))
{
    sensor_init = sensor_init_callback;
}

void logic_register_hw_pin(void (*toggle_test_pin_callback)(void))
{
	toggle_test_pin = toggle_test_pin_callback;
}

