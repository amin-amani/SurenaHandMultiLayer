/*
 * logic.h
 *
 *  Created on: Aug 5, 2025
 *      Author: cast
 */

#ifndef INC_LOGIC_H_
#define INC_LOGIC_H_


#include  <inttypes.h>
#include <stdbool.h>

#ifdef UNIT_TESTING
#include "../../Core/Inc/pid.h"
#include "../../Core/Inc/command_handler.h"
#else
#include "pid.h"
#include "bmp280.h"
#endif

//#define RIGHT_HAND
#define LEFT_HAND


#ifdef RIGHT_HAND
#define INDEX_MAX 1700 //open
#define INDEX_MIN 1088  //close need +

#define MIDDLE_MAX 2800 //open
#define MIDDLE_MIN 2320  //close need -

#define RING_MAX 540  //open
#define RING_MIN 250  //close need +

#define LITTLE_MAX 2490 //open need -
#define LITTLE_MIN 2320 //close

#define THUMB_MAX 2100 //open need -
#define THUMB_MIN 1773 //close

#define THUMB2_MAX 3350 //open need -
#define THUMB2_MIN 2470 //close






#define DEVICE_CAN_ID (0x281)
#else

#define INDEX_MIN 1270 //open open with -
#define INDEX_MAX 1950  //close

#define MIDDLE_MIN 1992 //open    open with +
#define MIDDLE_MAX 2790  //close

#define RING_MIN 1641  //open     open with -
#define RING_MAX 2045  //close

#define LITTLE_MIN 2677 //open   open with -
#define LITTLE_MAX 2949 //close

#define THUMB_MAX 3095 //open    open with -
#define THUMB_MIN 2763 //close

#define THUMB2_MIN 2700 //open    open with +
#define THUMB2_MAX 3100 //close

#define DEVICE_CAN_ID (0x282)
#endif

#define SHARED_CAN_ID (0x283)

#define INDEX_FINGER    (0) //correct index ok
#define MIDDLE_FINGER   (1) //ok
#define RING_FINGER     (2) //hardware test pass + open -close
#define LITTLE_FINGER   (3)//hardware test pass + open -close
#define THUMB_FINGER    (4) //ok
#define THUMB2_FINGER   (5) //ok

#define INDEX_FINGER_FEEDBACK_CHANNEL   (0)  //index
#define THUMB_FEEDBACK_CHANNEL          (4)
#define RING_FINGER_FEEDBACK_CHANNEL    (2) //rig
#define LITTLE_FINGER_FEEDBACK_CHANNEL  (3) //hardware test 2800 2200
#define MIDDLE_FINGER_FEEDBACK_CHANNEL  (1) //ok
#define THUMB2_FINGER_FEEDBACK_CHANNEL  (5) //ok adc8

#define JOINT_COUNT 6
#define SENSOR_COUNT 6


// sensor p 0 index finger
// sensor p 2 ring finger
typedef struct
{
	float pressure;
	float temperature;

} LOGIC_BMP_OUTPUT_TYPE;


void logic_init();
void logic_loop();
void init_pid_elements();
void init_finger_positions();
void init_servo_positions(void);
void init_pressure_sensors();
void update_presure_sensors();
void send_can_statup_command();
void run_pid_for_all_fingers();
uint32_t* get_target_positions();
pid_element_type* get_pid_emelents();
void can_data_received(uint32_t id,uint8_t *data);
void logic_register_adc(void (*read_adc_callback)(uint16_t*value));
void logic_register_can_send(int can_send_callback(uint32_t id,uint8_t *data));
void logic_register_set_motor_speed(void (*set_motor_speed_callback)(uint8_t motor,int32_t speed ));
void logic_register_set_servo_position(void set_servo_position_callback(uint8_t  *position));
void logic_register_read_pressure(float (*sensor_read_pressure_callback)(int index));
void logic_register_read_temperature(float (*sensor_read_temperature_callback)(int index));
void logic_register_sensor_init(bool (*sensor_init_callback)(int index));
void logic_register_hw_pin(void (*toggle_test_pin_callback)(void));

uint8_t get_pid_status();
uint8_t get_trigger_status();

#endif /* INC_LOGIC_H_ */
