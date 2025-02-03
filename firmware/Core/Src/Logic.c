/*
 * logic.c
 *
 *  Created on: Jan 21, 2025
 *      Author: amin
 */
#include "Logic.h"
#include "pid.h"

void (*set_finger_position)(uint8_t motor,int32_t speed);
void (*read_adc)(uint32_t* data, uint32_t length);
void (*delay_ms)(uint32_t delay);
float (*read_pressure)(int index);
uint8_t (*can_send)(uint32_t id,uint8_t *data, uint32_t len);
uint32_t adc_values[6];


pid_element_type pid_element[6];


void logic_start()
{

pid_element[4].dt=10;
pid_element[4].kd=0;
pid_element[4].ki=0;
pid_element[4].kp=1;

pid_element[4].max_command=3900;
pid_element[4].min_command=50;
pid_element[4].max_iterm=1200;
pid_element[4].min_iterm=-1200;
pid_element[4].max_output=1900;
pid_element[4].min_output=-1900;


}

void logic_loop()
{
	read_adc(adc_values,6);
    set_finger_position(5, do_pid(1200,adc_values[4],&pid_element[4]));
	delay_ms(10);

}




void register_delay(void (*delay_callback)(uint32_t delay))
{
	delay_ms=delay_callback;
}

void register_finger_motros_callback(void (*set_finger_position_callback)(uint8_t motor,int32_t speed))
{
	set_finger_position=set_finger_position_callback;
}
void register_adc_callback(void (*read_adc_callback)(uint32_t* data, uint32_t length) )
{
	read_adc=read_adc_callback;
}

void  register_can_send(uint8_t (*can_send_callback)(uint32_t id,uint8_t *data, uint32_t len))
{
can_send=can_send_callback;

}
void register_read_pressure(float (*read_pressure_callback)(int index))
{
	read_pressure=read_pressure_callback;
}

void can_data_received(uint32_t id,uint8_t*data)
{

}

