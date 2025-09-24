/*
 * logic.c
 *
 *  Created on: Aug 5, 2025
 *      Author: cast
 */
#include <stdio.h>
#include <stdbool.h>


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

int set_finger_goal_position(uint32_t id , uint8_t*data);
int set_finger_goal_speed(uint32_t id , uint8_t*data);
int set_pressure_limits(uint32_t id , uint8_t*data);
int set_finger_pid_values(uint32_t id , uint8_t*data);
int get_finger_goal_position(uint32_t id , uint8_t*data);
int get_adc_values(uint32_t id , uint8_t*data);
int move_motor(uint32_t id , uint8_t*data);
int stop_all_motors(uint32_t id , uint8_t*data);
int get_pressure_values(uint32_t id , uint8_t*data);
int set_servo_goal_position(uint32_t id,uint8_t*data);

static void control_logic_loop(void);
static uint8_t is_motor_at_target_position(uint8_t motor_index);
static uint8_t is_pressure_limit_reached(uint8_t sensor_index);

static pid_element_type pid_element[JOINT_COUNT];
static uint32_t target_position[JOINT_COUNT];
static uint32_t target_speed[JOINT_COUNT];
static uint32_t pressure_limits[SENSOR_COUNT];
static uint16_t pid_interval=50;
static uint8_t pid_enabled=0;
static uint8_t control_trigger=0;

const INT_COMMAND_TYPE command_list[] =
{
    {0, set_finger_goal_position},
    {1, set_finger_goal_speed},
    {2, set_pressure_limits},
    {3, set_finger_pid_values},
    {4, get_finger_goal_position},
	{5, get_pressure_values},
    {6, get_adc_values},
    {7, move_motor},
    {8, stop_all_motors},
    {9, set_servo_goal_position},
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

    update_presure_sensors();
    
    if(control_trigger == 1) {
        control_logic_loop();
    }
    
    if(pid_enabled) {
        run_pid_for_all_fingers();
    }
}

void update_presure_sensors()
{
    int i;
    for(i=0;i<SENSOR_COUNT;i++)
    {
    	bmp_sensor_value[i].pressure = sensor_read_pressure(i+1);
    	bmp_sensor_value[i].temperature = sensor_read_temperature(i+1);
        printf("sensor num:%d pressure=%f  temp=%f \n",i,bmp_sensor_value[i].pressure,bmp_sensor_value[i].temperature);
        delay_ms(100);
    }
}

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
	pid_enabled = 0;
	
	// Stop all motors
	for (int i = 0; i < JOINT_COUNT; i++) {
		set_motor_speed(i, 0);
	}
	printf("stop all motors.\n");
	return 0;
}

int get_adc_values(uint32_t id , uint8_t*data)
{
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
	uint8_t can_data[8];
	
	// Send pressure values for all sensors that are requested (non-zero in data[1-6])
	for (int i = 0; i < SENSOR_COUNT; i++) {
		uint32_t pressure = (uint32_t)bmp_sensor_value[i].pressure;
		can_data[0] = (pressure >> 24) & 0xFF; // MSB
		can_data[1] = (pressure >> 16) & 0xFF;
		can_data[2] = (pressure >> 8) & 0xFF;
		can_data[3] = pressure & 0xFF;         // LSB
		// can_data[4] = 0; can_data[5] = 0; can_data[6] = 0; can_data[7] = 0;
		can_send(id, can_data);
	}

    return 0;
}

int set_finger_goal_position(uint32_t id,uint8_t*data)
{
	// Data Format: [command_id, motor0, motor1, motor2, motor3, motor4, motor5, trigger]
	// Each motor value represents position for that finger
	uint8_t trigger = data[7];
	
	for (int i = 0; i < JOINT_COUNT; i++) {
		target_position[i] = data[i+1]; // should be mapped for better precision
		printf("set pos[%d] = %d\n", i, target_position[i]);
	}
	
	control_trigger = trigger;
	if (trigger == 1) {
		pid_enabled = 1;
	} else {
		pid_enabled = 0;
	}
	
	return 0;
}

int set_finger_goal_speed(uint32_t id,uint8_t*data)
{
	// Each motor value represents speed for that finger
	for (int i = 0; i < JOINT_COUNT; i++) {
		target_speed[i] = data[i+1] * 10; // should be mapped for better precision
		printf("set speed[%d] = %d\n", i, target_speed[i]);
	}
	
	return 0;
}

int set_pressure_limits(uint32_t id,uint8_t*data)
{
	// Each sensor value represents target pressure for that sensor
	for (int i = 0; i < SENSOR_COUNT; i++) {
		pressure_limits[i] = data[i+1] * 10; // Scale up for better precision
		printf("set pressure[%d] = %d\n", i, pressure_limits[i]);
	}
	
	return 0;
}

int set_finger_pid_values(uint32_t id,uint8_t*data)
{
	// For simplicity, we'll set Kp, Kd, and Ki equal for all motors
	// Data Format: [command_id, Kp, Ki, Kd]
	for (int i = 0; i < JOINT_COUNT; i++) {
		pid_element[i].kp = data[1] / 10.0f; // should be mapped for better precision
		pid_element[i].ki = data[2] / 10.0f; // should be mapped for better precision
		pid_element[i].kd = data[3] / 10.0f; // should be mapped for better precision
		printf("set PID Kp = %f, Ki = %f, Kd = %f\n", pid_element[i].kp, pid_element[i].ki, pid_element[i].kd);
	}
	
	return 0;
}

int move_motor(uint32_t id,uint8_t*data)
{
	// Set both angle (position) and speed for the target motor
	// Data Format: [command_id, motor, pos, speed]
	
	target_position[data[1]] = data[2];                 // should be mapped for better precision
	for (int i = 0; i < JOINT_COUNT; i++) {
		target_speed[i] = (i==data[1]) ? data[3] : 0;   // should be mapped for better precision
	}
	printf("move motor[%d] pos=%d speed=%d\n", data[1], target_position[data[1]], target_speed[data[1]]);
	
	pid_enabled = 1;
	control_trigger = 1;
	return 0;
}

int set_servo_goal_position(uint32_t id,uint8_t*data)
{
	// Data Format: [command_id, servo0, servo1, servo2, servo3, servo4, servo5, trigger]
	uint8_t trigger = data[7];
	
	uint8_t position[3];
    // right hand
	position[0] = data[1];
	position[1] = data[2];
	position[2] = data[3];
    // left hand
	// position[0] = data[4];
	// position[1] = data[5];
	// position[2] = data[6];

	if (trigger == 1) set_servo_position(position);
	printf("set servo pos %d %d %d (trigger=%d)\n", 
		   position[0], position[1], position[2], trigger);
	return 0;
}


int get_finger_goal_position(uint32_t id,uint8_t*data)
{
	printf("get pos\n");
    return 0;
}

void init_pid_elements()
{
    for(int i=0;i<JOINT_COUNT;i++)
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

void init_finger_positions()
{
	pid_enabled=0;
	control_trigger=0;

    target_position[INDEX_FINGER]=1000;
    target_position[LITTLE_FINGER]=2500;
    target_position[THUMB2_FINGER]=2600;
    
    for (int i = 0; i < JOINT_COUNT; i++) { target_speed[i] = 0; }
    
    for (int i = 0; i < SENSOR_COUNT; i++) { pressure_limits[i] = 0; }
}
void init_servo_positions(void)
{
	uint8_t servo_init_angles[]={90,90,90};
//	set_servo_position(servo_init_angles);
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

void run_pid_for_all_fingers()
{

    uint16_t values[JOINT_COUNT] = {0};
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

static void control_logic_loop(void)
{
    // Check termination conditions for each motor
    for (int i = 0; i < JOINT_COUNT; i++) {
        // Check if motor reached target position
        if (is_motor_at_target_position(i)) {
            set_motor_speed(i, 0);
            printf("Motor %d reached target position\n", i);
        }
        // Check if pressure limit reached
        else if (is_pressure_limit_reached(i)) {
            set_motor_speed(i, 0);
            printf("Motor %d stopped due to pressure limit\n", i);
        }
    }
    
    // Check if all motors have reached their targets or hit pressure limits
    uint8_t all_completed = 1;
    for (int i = 0; i < JOINT_COUNT; i++) {
        if (!is_motor_at_target_position(i) && !is_pressure_limit_reached(i)) {
            all_completed = 0;
            break;
        }
    }
    
    if (all_completed) {
        control_trigger = 0; // Reset trigger when all motors are done
        pid_enabled = 0;
        printf("All motors completed movement\n");
    }
}

static uint8_t is_motor_at_target_position(uint8_t motor_index)
{
    if (motor_index >= JOINT_COUNT) return 0;
    
    uint16_t current_adc_values[JOINT_COUNT];
    read_adc(current_adc_values);
    
    // Get the appropriate feedback channel for this motor
    uint16_t current_position;
    switch(motor_index) {
        case INDEX_FINGER:
            current_position = current_adc_values[INDEX_FINGER_FEEDBACK_CHANNEL];
            break;
        case MIDDLE_FINGER:
            current_position = current_adc_values[MIDDLE_FINGER_FEEDBACK_CHANNEL];
            break;
        case RING_FINGER:
            current_position = current_adc_values[RING_FINGER_FEEDBACK_CHANNEL];
            break;
        case LITTLE_FINGER:
            current_position = current_adc_values[LITTLE_FINGER_FEEDBACK_CHANNEL];
            break;
        case THUMB_FINGER:
            current_position = current_adc_values[THUMB_FEEDBACK_CHANNEL];
            break;
        case THUMB2_FINGER:
            current_position = current_adc_values[THUMB2_FINGER_FEEDBACK_CHANNEL];
            break;
        default:
            return 0;
    }
    
    // Check if within tolerance (e.g., ±50 ADC units)
    int32_t error = (int32_t)target_position[motor_index] - (int32_t)current_position;
    if (error < 0) error = -error; // Absolute value
    
    return (error <= 5); // Tolerance of 50 ADC units
}

static uint8_t is_pressure_limit_reached(uint8_t sensor_index)
{
    if (sensor_index >= SENSOR_COUNT) return 0;
    
    // Check if current pressure exceeds the limit
    float current_pressure = bmp_sensor_value[sensor_index].pressure;
    return (current_pressure >= pressure_limits[sensor_index]);
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
