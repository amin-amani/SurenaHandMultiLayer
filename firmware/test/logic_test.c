
#include "unity_fixture.h"
#include "unity.h"

#include "string.h"
#include <unistd.h>

#include "../Core/Inc/logic.h"
#include "../Core/Inc/delay.h"
#include "../Core/Inc/bmp280.h"

uint16_t adc_result_mock[6]={0};
uint8_t can_sent_data[8]={0};
int can_sent_id=0;

void delay_fast_mock(uint32_t ms)
{
    for (int i = 0; i < ms; i++)
        usleep(10);
}
void set_motor_speed_mock(uint8_t motor,int32_t speed )
{

}
void set_servo_position_mock(uint8_t  *position)
{

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
float sesnor_read_pressure_mock(int index)
{
return 0;
}

float sesnor_read_temperature_mock(int index)
{
return 0;
}
bool sensor_init_mock(int index)
{
    return true;
}

TEST_GROUP(LOGIC_TEST_GROUP);

TEST_SETUP(LOGIC_TEST_GROUP)
{
    register_delay(delay_fast_mock);
    logic_register_adc(read_adc_mock);
    logic_register_can_send(can_send_mock);
    logic_register_set_motor_speed(set_motor_speed_mock);
    logic_register_set_servo_position(set_servo_position_mock);
    logic_register_read_pressure(sesnor_read_pressure_mock);
    logic_register_read_temperature(sesnor_read_temperature_mock);
    logic_register_sensor_init(sensor_init_mock);
}


TEST_TEAR_DOWN(LOGIC_TEST_GROUP)
{

    // Runs after each test
}
TEST(LOGIC_TEST_GROUP, when_log_init_call_pid_elements_is_not_null)
{


    logic_init();
    pid_element_type *pid_elenemt=get_pid_emelents();
    uint32_t *pos=get_target_positions();
    for(int i=0; i<JOINT_COUNT;i++)
    UNITY_TEST_ASSERT_NOT_EQUAL_INT(0,pid_elenemt[i].kp,__LINE__,"");

}

TEST(LOGIC_TEST_GROUP, test_under_const)
{


  logic_loop();
}
TEST(LOGIC_TEST_GROUP, when_can_packet_recive_logic_set_motor_speed)
{
    uint32_t id=0x281;
    // uint8_t data[8]={0,1,0x76,0x06,0,0,0,7};
    uint8_t data[8]={0,3,76,114,255,173,97,0};
    can_data_received( id,data);
    logic_loop();
}
