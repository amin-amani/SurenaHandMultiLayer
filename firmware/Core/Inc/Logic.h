/*
 * Logic.h
 *
 *  Created on: Jan 21, 2025
 *      Author: amin
 */

#ifndef INC_LOGIC_H_
#define INC_LOGIC_H_
#include <stdint.h>

#define FINGER_1 1
#define FINGER_2 2
#define FINGER_3 3
#define FINGER_4 4
#define FINGER_5 5
#define FINGER_6 6

void register_adc_callback(void (*read_adc_callback)(uint32_t* data, uint32_t length) );
void register_delay(void (*delay_callback)(uint32_t delay));
void register_finger_motros_callback(void (*set_finger_position_callback)(uint8_t motor,int32_t speed));
void logic_start();
void logic_loop();
void can_data_received(uint32_t id,uint8_t*data);
void register_can_send(uint8_t (*can_send_callback)(uint32_t id,uint8_t *data, uint32_t len));
void register_read_pressure(float (*read_pressure_callback)(int index));
#endif /* INC_LOGIC_H_ */
