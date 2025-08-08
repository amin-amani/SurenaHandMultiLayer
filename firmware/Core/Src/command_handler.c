/*
 * command_handler.c
 *
 *  Created on: Jul 18, 2025
 *      Author: cast
 */

#ifdef UNIT_TESTING
#include "../../Core/Inc/command_handler.h"


#else
#include "command_handler.h"

#endif

extern  INT_COMMAND_TYPE scommand_list[] ;



int run_handler(int32_t command,uint32_t id, uint8_t *data, const INT_COMMAND_TYPE *command_list, int command_list_size)
{
	uint32_t i=0;
    for ( i = 0; i < command_list_size; i++)
    {
        if (command == command_list[i].command && command_list[i].handler != 0)
        {

            return command_list[i].handler(id,data);
        }
    }
    return -1;
}
