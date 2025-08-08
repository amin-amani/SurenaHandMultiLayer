#ifndef PID_H
#define PID_H
#include <inttypes.h>
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
int32_t do_pid(int32_t target_position, int32_t current_position, pid_element_type *pid_elemets);
#endif // PID_H
