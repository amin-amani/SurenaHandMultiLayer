

#ifdef  UNIT_TESTING
#include "../../Core/Inc/pid.h"
#else
#include "pid.h"
#endif
int32_t do_pid(int32_t target_position, int32_t current_position, pid_element_type *pid_elemets)
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
