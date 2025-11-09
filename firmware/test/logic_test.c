
#include "unity_fixture.h"
#include "unity.h"
#include <stdbool.h>
#include "string.h"
#include <unistd.h>
#include <inttypes.h>
#include "../Core/Inc/logic.h"
#include "../Core/Inc/delay.h"
#include "../Core/Inc/bmp280.h"

uint16_t adc_result_mock[6]={0};
uint8_t can_sent_data[8]={0};
int can_sent_id=0;
int adc_ratio = 4096/255;
int mock_speed[6];

void delay_fast_mock(uint32_t ms)
{
    for (int i = 0; i < ms; i++)
        usleep(10);
}
void set_motor_speed_mock(uint8_t motor,int32_t speed )
{
    mock_speed[motor] = speed;
}
void set_servo_position_mock(uint8_t  *position)
{

}
void read_adc_mock(uint16_t*value)
{
memcpy(value,adc_result_mock,6* sizeof(uint16_t));
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
float sesnor_read_max_pressure_mock(int index)
{
return 11;
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
    uint32_t id=0x281;
    // uint8_t data[8]={0,1,0x76,0x06,0,0,0,7};

    uint8_t pressure_data[8]={11,10,10,10,10,10,10,0};
    uint8_t pid_data[8]={12,3,76,114,0,0,0,0};
    uint8_t position_data[8]={10,250,76,114,255,173,97,1};

    logic_init();
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"1");
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_trigger_status(),__LINE__,"2");

//    can_data_received( id,pressure_data);
//    can_data_received( id,pid_data);
    can_data_received( id,position_data);
//    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"3");
//    UNITY_TEST_ASSERT_EQUAL_UINT8(1,get_trigger_status(),__LINE__,"4");

//    logic_register_read_pressure(sesnor_read_max_pressure_mock);
for(int i=0;i<1000;i++)
    logic_loop();

//  logic_loop();
}
TEST(LOGIC_TEST_GROUP, when_sensor_reached_max_pressure_trigger_should_change_to_zero)
{
    uint32_t id=0x281;
    // uint8_t data[8]={0,1,0x76,0x06,0,0,0,7};

    uint8_t pressure_data[8]={11,10,10,10,10,10,10,0};
    uint8_t pid_data[8]={12,3,76,114,0,0,0,0};
    uint8_t position_data[8]={10,250,76,114,255,173,97,1};

    logic_init();
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"1");
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_trigger_status(),__LINE__,"2");

    can_data_received( id,pressure_data);
    can_data_received( id,pid_data);
    can_data_received( id,position_data);
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"3");
    UNITY_TEST_ASSERT_EQUAL_UINT8(1,get_trigger_status(),__LINE__,"4");

    logic_register_read_pressure(sesnor_read_max_pressure_mock);

    logic_loop();
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_trigger_status(),__LINE__,"5");
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"6");
}
TEST(LOGIC_TEST_GROUP, when_sensor_reached_max_position_trigger_should_change_to_zero)
{
    uint32_t id=0x281;
    // uint8_t data[8]={0,1,0x76,0x06,0,0,0,7};

    uint8_t pressure_data[8]={11,10,10,10,10,10,10,0};
    uint8_t pid_data[8]={12,3,76,114,0,0,0,0};
    uint8_t position_data[8]={10,250,76,114,40,173,97,1};
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"");
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_trigger_status(),__LINE__,"");

    can_data_received( id,pressure_data);
    can_data_received( id,pid_data);
    can_data_received( id,position_data);
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"");
    UNITY_TEST_ASSERT_EQUAL_UINT8(1,get_trigger_status(),__LINE__,"");

    uint16_t adc_threashold[6] = {250*adc_ratio,76*adc_ratio,114*adc_ratio,40*adc_ratio,173*adc_ratio,97*adc_ratio};
    memcpy(adc_result_mock,adc_threashold,6 * sizeof(uint16_t));
    logic_register_adc(read_adc_mock);

    logic_loop();
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_trigger_status(),__LINE__,"");
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"");
}
TEST(LOGIC_TEST_GROUP, when_sensor_reached_middle_position_motor_speed_is_not_zero)
{
    uint32_t id=0x281;
    // uint8_t data[8]={0,1,0x76,0x06,0,0,0,7};

    uint8_t pressure_data[8]={11,10,10,10,10,10,10,0};
    uint8_t pid_data[8]={12,3,76,114,0,0,0,0};
    uint8_t position_data[8]={10,250,76,114,40,173,97,1};
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"");
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_trigger_status(),__LINE__,"");

    can_data_received( id,pressure_data);
    can_data_received( id,pid_data);
    can_data_received( id,position_data);
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"");
    UNITY_TEST_ASSERT_EQUAL_UINT8(1,get_trigger_status(),__LINE__,"");

    uint16_t adc_threashold[6] = {100*adc_ratio,51*adc_ratio,200*adc_ratio,80*adc_ratio,100*adc_ratio,80*adc_ratio};
    memcpy(adc_result_mock,adc_threashold,6 * sizeof(uint16_t));
    logic_register_adc(read_adc_mock);

    logic_loop();
    UNITY_TEST_ASSERT_EQUAL_UINT8(1,get_trigger_status(),__LINE__,"");
    UNITY_TEST_ASSERT_EQUAL_UINT8(0,get_pid_status(),__LINE__,"");

    UNITY_TEST_ASSERT_NOT_EQUAL_INT(mock_speed[0],0,__LINE__,"0");
    UNITY_TEST_ASSERT_NOT_EQUAL_INT(mock_speed[1],0,__LINE__,"1");
    UNITY_TEST_ASSERT_NOT_EQUAL_INT(mock_speed[2],0,__LINE__,"2");
    UNITY_TEST_ASSERT_NOT_EQUAL_INT(mock_speed[3],0,__LINE__,"3");
    UNITY_TEST_ASSERT_NOT_EQUAL_INT(mock_speed[4],0,__LINE__,"4");
    UNITY_TEST_ASSERT_NOT_EQUAL_INT(mock_speed[5],0,__LINE__,"5");
}
