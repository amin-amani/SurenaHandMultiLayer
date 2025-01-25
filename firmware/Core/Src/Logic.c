/*
 * logic.c
 *
 *  Created on: Jan 21, 2025
 *      Author: amin
 */
#include "Logic.h"
void (*set_finger_position)(uint8_t motor,int32_t speed);
void (*read_adc)(uint32_t* data, uint32_t length);
void (*delay_ms)(uint32_t delay);
uint32_t adc_values[6];

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_term;
    float prev_error;
    float dt;
    int max_output;
    int min_output;
    int max_command;
    int min_command;
    int max_iterm;
    int min_iterm;
} pid_element_type;

pid_element_type pid_element[6];

int32_t DoPID(int32_t target_position, int32_t current_position, pid_element_type *pid_elemets)
{
    int error = 0;
    float command;
    float dterm = 0;

    if (current_position < pid_elemets->min_command || current_position > pid_elemets->max_command)
    {
        return 0;
    }

    error = target_position - current_position;
    dterm = (error - pid_elemets->prev_error) / pid_elemets->dt;

    pid_elemets->i_term += (error) * pid_elemets->dt;

    if (pid_elemets->i_term > pid_elemets->max_iterm) pid_elemets->i_term = pid_elemets->max_iterm;
    if (pid_elemets->i_term < pid_elemets->min_iterm) pid_elemets->i_term = pid_elemets->min_iterm;

    command = (pid_elemets->kp * error) + (pid_elemets->kd * dterm) + (pid_elemets->ki * pid_elemets->i_term);

    pid_elemets->prev_error = error;

    if (command > pid_elemets->max_output) command = pid_elemets->max_output;
    if (command < pid_elemets->min_output) command = pid_elemets->min_output;

    return (int32_t)command;
}

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
	set_finger_position(5, DoPID(1200,adc_values[4],&pid_element[4]));
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



void can_data_received(uint32_t id,uint8_t*data)
{

}

