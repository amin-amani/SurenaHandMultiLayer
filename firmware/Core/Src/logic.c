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
static uint16_t pid_interval=50;
static uint8_t pid_enabled=0;
static uint8_t control_trigger=0;
const int error_tolerance = 50;

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
    control_trigger = 0;

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

uint8_t get_pid_status(){
    return pid_enabled;
}

uint8_t get_trigger_status(){
    return control_trigger;
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

int set_finger_goal_position_amani(uint32_t id,uint8_t*data)
{

    int32_t position=0;
    position  = data[5];
    position<<=8;
    position |= data[4];
    position<<=8;
    position |= data[3];
    position<<=8;
    position |= data[2];

    if (data[1] >= JOINT_COUNT) return -1;
    pid_enabled=1;
    target_position[data[1]]=position;
    printf("set pos[%d] = %d\n",data[1],target_position[data[1]]);
    return 0;
}

int set_finger_goal_position(uint32_t id,uint8_t*data)
{
	// Data Format: [command_id, motor0, motor1, motor2, motor3, motor4, motor5, trigger]
	// Each motor value represents position for that finger
	uint8_t trigger = data[7];
	
	for (int i = 0; i < JOINT_COUNT; i++) {
        target_position[i] = data[i+1] * 4095;
        target_position[i] /= 255;
		printf("set pos[%d] = %d\n", i, target_position[i]);
	}
	
	control_trigger = trigger;
	
	return 0;
}

int set_pressure_limits(uint32_t id,uint8_t*data)
{
	// Each sensor value represents target pressure for that sensor
	for (int i = 0; i < SENSOR_COUNT; i++) {
        pressure_limits[i] = data[i+1] * 1; // Scale up for better precision
		printf("set pressure[%d] = %d\n", i, pressure_limits[i]);
	}
	
	return 0;
}

int set_finger_pid_values(uint32_t id,uint8_t*data)
{
	// For simplicity, we'll set Kp, Kd, and Ki equal for all motors
	// Data Format: [command_id, Kp, Ki, Kd]
	for (int i = 0; i < JOINT_COUNT; i++) {
        pid_element[i].kp = data[1] / 100.0f;
        pid_element[i].ki = data[2] / 100.0f;
        pid_element[i].kd = data[3] / 100.0f;
		printf("set PID Kp = %f, Ki = %f, Kd = %f\n", pid_element[i].kp, pid_element[i].ki, pid_element[i].kd);
	}
	
	return 0;
}

int set_servo_goal_position(uint32_t id,uint8_t*data)
{
	uint8_t trigger = data[7];
	
	uint8_t position[3];
	position[0] = data[1];
	position[1] = data[2];
	position[2] = data[3];

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

static void control_logic_loop()
{
    uint16_t current_adc_values[JOINT_COUNT];
    read_adc(current_adc_values);

    // Check termination conditions for each motor
    for (int i = 0; i < JOINT_COUNT; i++) {
         int32_t pid_speed = do_pid(target_position[i], current_adc_values[i], &pid_element[i]);

        if (is_motor_at_target_position(i,current_adc_values[i]) || is_pressure_limit_reached(i))
        {
            set_motor_speed(i, 0);
            printf("Motor %d reached target position\n", i);
            continue;
        }
        set_motor_speed(i, pid_speed);
    }
    
    // Check if all motors have reached their targets or hit pressure limits
    uint8_t all_completed = 1;
    for (int i = 0; i < JOINT_COUNT; i++) {
        if (!is_motor_at_target_position(i, current_adc_values[i]) && !is_pressure_limit_reached(i)) {
            all_completed = 0;
            break;
        }
    }
    
    if (all_completed) {
        control_trigger = 0; // Reset trigger when all motors are done
        printf("All motors completed movement\n");
    }
}

static uint8_t is_motor_at_target_position(uint8_t motor_index, uint16_t current_adc_value)
{

    int16_t error = (int16_t)target_position[motor_index] - (int16_t)current_adc_value;
    if (error < 0) error = -error; // Absolute value
    
    return (error <= error_tolerance);
}

static uint8_t is_pressure_limit_reached(uint8_t sensor_index)
{
    if (sensor_index >= SENSOR_COUNT) return 0;
    
    // Check if current pressure exceeds the limit
    float current_pressure = (sensor_index<5) ? bmp_sensor_value[sensor_index].pressure : bmp_sensor_value[sensor_index-1].pressure;
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
