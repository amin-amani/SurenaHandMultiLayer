/*
 * command_handler.h
 *
 *  Created on: Jul 18, 2025
 *      Author: cast
 */

#ifndef INC_COMMAND_HANDLER_H_
#define INC_COMMAND_HANDLER_H_
#include <inttypes.h>
typedef struct
{
  int32_t command;
  int (*handler)(uint32_t id, uint8_t *data);
}INT_COMMAND_TYPE;
int run_handler(int32_t command, uint32_t id, uint8_t *data, const INT_COMMAND_TYPE *command_list, int command_list_size);
#endif /* INC_COMMAND_HANDLER_H_ */
