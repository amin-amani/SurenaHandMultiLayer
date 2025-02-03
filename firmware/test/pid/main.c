
#include "unity.h"
#include "Logic.h"
#include "string.h"
#include<unistd.h>
#include "pid.h"



void test_do_pid_nominal_case(void)
{
    pid_element_type pid = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .dt = 1.0,
        .prev_error = 0,
        .i_term = 0,
        .max_iterm = 10,
        .min_iterm = -10,
        .max_output = 100,
        .min_output = -100,
        .min_command = -1000,
        .max_command = 1000
    };

    int32_t target_position = 100;
    int32_t current_position = 90;
    int32_t result = do_pid(target_position, current_position, &pid);

    TEST_ASSERT_INT_WITHIN(10, 10, result);
}

void test_do_pid_nominal_smaller_target_case(void)
{
    pid_element_type pid = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .dt = 1.0,
        .prev_error = 0,
        .i_term = 0,
        .max_iterm = 10,
        .min_iterm = -10,
        .max_output = 100,
        .min_output = -100,
        .min_command = -1000,
        .max_command = 1000
    };

    int32_t target_position = 100;
    int32_t current_position =120;
    int32_t result = do_pid(target_position, current_position, &pid);

    TEST_ASSERT_INT_WITHIN(12, -10, result);
}

void test_do_pid_saturation_limits(void)
{
    pid_element_type pid = {
        .kp = 10.0,
        .ki = 5.0,
        .kd = 1.0,
        .dt = 1.0,
        .prev_error = 0,
        .i_term = 0,
        .max_iterm = 10,
        .min_iterm = -10,
        .max_output = 50,
        .min_output = -50,
        .min_command = -1000,
        .max_command = 1000
    };

    int32_t target_position = 1000;
    int32_t current_position = 0;
    int32_t result = do_pid(target_position, current_position, &pid);

    TEST_ASSERT_EQUAL_INT(50, result); // Expect clamping at max_output
}

void test_do_pid_ignore_out_of_bounds(void)
{
    pid_element_type pid = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .dt = 1.0,
        .prev_error = 0,
        .i_term = 0,
        .max_iterm = 10,
        .min_iterm = -10,
        .max_output = 100,
        .min_output = -100,
        .min_command = -50,
        .max_command = 50
    };

    int32_t target_position = 0;
    int32_t current_position = 100; // Out of bounds
    int32_t result = do_pid(target_position, current_position, &pid);

    TEST_ASSERT_EQUAL_INT(0, result);
}

void setUp (void)
{


} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */
int main()
{
    UNITY_BEGIN();


     RUN_TEST(test_do_pid_saturation_limits);
     RUN_TEST(test_do_pid_ignore_out_of_bounds);
     RUN_TEST(test_do_pid_nominal_smaller_target_case);

    RUN_TEST(test_do_pid_nominal_case);
    return UNITY_END();
}
