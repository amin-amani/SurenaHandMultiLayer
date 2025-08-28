#include <unity.h>
#include <unity_fixture.h>

TEST_GROUP_RUNNER(LOGIC_TEST_GROUP)
{

    RUN_TEST_CASE(LOGIC_TEST_GROUP, test_under_const);
    RUN_TEST_CASE(LOGIC_TEST_GROUP, when_log_init_call_pid_elements_is_not_null);
}
