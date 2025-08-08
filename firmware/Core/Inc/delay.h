/*
 * delay.h
 *
 *  Created on: Jul 24, 2025
 *      Author: amin
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_
#include <inttypes.h>


void register_delay(void (*delay_callback)(uint32_t ms));
void delay_ms(uint32_t ms);

#endif /* INC_DELAY_H_ */
