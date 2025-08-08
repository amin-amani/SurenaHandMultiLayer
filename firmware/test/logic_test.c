
#include "unity_fixture.h"
#include "unity.h"
#include "string.h"
#include <unistd.h>

#include "../Core/Inc/logic.h"
#include "../Core/Inc/delay.h"

uint16_t adc_result_mock[6]={0};
uint8_t can_sent_data[8]={0};
int can_sent_id=0;

void delay_fast_mock(uint32_t ms)
{
    for (int i = 0; i < ms; i++)
        usleep(10);
}

void read_adc_mock(uint16_t*value)
{
memcpy(value,adc_result_mock,6);
}

int can_send_mock(uint32_t id,uint8_t *data)
{
can_sent_id=id;
memcpy(can_sent_data,data,6);
return 0;
}

TEST_GROUP(LOGIC_TEST_GROUP);

TEST_SETUP(LOGIC_TEST_GROUP)
{
    register_delay(delay_fast_mock);
    logic_register_adc(read_adc_mock);
    logic_register_can_send(can_send_mock);
}


TEST_TEAR_DOWN(LOGIC_TEST_GROUP)
{
    // Runs after each test
}
TEST(LOGIC_TEST_GROUP, test_under_const)
{

}
