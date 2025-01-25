/*
 * logic.c
 *
 *  Created on: Jan 21, 2025
 *      Author: amin
 */
#include "Logic.h"

uint32_t adc_values[6];
int CurrentPos=0;

int TargetPoition=2048;

void (*set_finger_position)(uint8_t motor,int32_t speed);
void (*read_adc)(uint32_t* data, uint32_t length);
void (*delay_ms)(uint32_t delay);

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

void DoPID()
{
	float ki=0.00,kp=1,kd=0.00;

	int error=0;
	int intcmd;
	int dt=10;
	static int prevError=0;
	float command=1200;
	float Dterm=0;
	static float Iterm=0;

	CurrentPos=adc_values[4];//UpdateEncoder();

	if(CurrentPos<50 || CurrentPos>3900 )
	{
		set_finger_position(7, 0);
		return;
	}

//	printf("adc5=%d\n",CurrentPos);
	error=TargetPoition-CurrentPos;
	Dterm=(error-prevError)/dt;
	Iterm+=(error)*dt;
	if(Iterm>1200)Iterm=1200;
	if(Iterm<-1200)Iterm=-1200;

	command=kp*error+kd*Dterm+ki*Iterm;

	prevError=error;
		if(command>1900)command=1900;
		if(command<-1900)	command=-1900;
		intcmd=command;

	 delay_ms(10);
	//threshold
//	if(command<300)command=300;
//	if(command>2300)command=2300;

		set_finger_position(5, command);


}
void can_data_received(uint32_t id,uint8_t*data)
{

}
void logic_start()
{


}
void logic_loop()
{
	read_adc(adc_values,6);

//	DoPID();
//	 set_finger_position(5,-1900);
}
