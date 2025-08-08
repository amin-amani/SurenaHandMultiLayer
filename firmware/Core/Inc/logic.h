/*
 * logic.h
 *
 *  Created on: Aug 5, 2025
 *      Author: cast
 */

#ifndef INC_LOGIC_H_
#define INC_LOGIC_H_

#include  <inttypes.h>

#define THUMB_FINGER    (0)
#define INDEX_FINGER    (1)
#define RING_FINGER     (2)//hardware test pass + open -close
#define LITTLE_FINGER   (3)//hardware test pass + open -close
#define MIDDLE_FINGER   (4)
#define THUMB_FINGER2   (5)

#define THUMB_FEEDBACK_CHANNEL          (0)
#define INDEX_FINGER_FEEDBACK_CHANNEL   (1)
#define RING_FINGER_FEEDBACK_CHANNEL    (2)
#define LITTLE_FINGER_FEEDBACK_CHANNEL  (3) //hardware test 2800 22000
#define MIDDLE_FINGER_FEEDBACK_CHANNEL  (4)
#define THUMB_FINGER2_FEEDBACK_CHANNEL  (5)

#define JOINT_COUNTS 6


void logic_init();
void logic_loop();
void can_data_received(uint32_t id,uint8_t *data);
void logic_register_can_send(int can_send_callback(uint32_t id,uint8_t *data));
void logic_register_adc(void (*read_adc_callback)(uint16_t*value));
void logic_register_set_motor_speed(void (*set_motor_speed_callback)(uint8_t motor,int32_t speed ));
#endif /* INC_LOGIC_H_ */
