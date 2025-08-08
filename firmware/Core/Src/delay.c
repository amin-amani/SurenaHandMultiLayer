/*
 * delay.c
 *
 *  Created on: Jul 24, 2025
 *      Author: amin
 */


#ifdef UNIT_TESTING
#include "../../Core/Inc/delay.h"
#else
#include "delay.h"
#endif

static void (*delay_ms_fn)(uint32_t ms);
void register_delay(void (*delay_callback)(uint32_t ms))
{
	delay_ms_fn = delay_callback;
}
void delay_ms(uint32_t ms)
{
	delay_ms_fn(ms);
}
