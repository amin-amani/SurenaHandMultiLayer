
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
void SpiWrite_mock(uint8_t index,uint8_t *data,int len)
{

}

void SPIReadWrite_mock(uint8_t index,uint8_t *txBuffer,uint8_t txLen,uint8_t *rxBuffer,uint8_t rxLen)
{


}
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

BMP280_CALLBACK_STRUCT_TYPE bmp_sensor=
    {
        SPIReadWrite_mock,
        SpiWrite_mock

    };


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
TEST(LOGIC_TEST_GROUP, when_log_init_call_pid_elements_is_not_null)
{

    bmp280_register_callbacks(&bmp_sensor);
    logic_init();
    pid_element_type *pid_elenemt=get_pid_emelents();
    uint32_t *pos=get_target_positions();
    for(int i=0; i<JOINT_COUNT;i++)
    UNITY_TEST_ASSERT_NOT_EQUAL_INT(0,pid_elenemt[i].kp,__LINE__,"");

}

TEST(LOGIC_TEST_GROUP, test_under_const)
{
  bmp280_register_callbacks(&bmp_sensor);

  logic_loop();
}
TEST(LOGIC_TEST_GROUP, when_can_packet_recive_logic_set_motor_speed)
{
    uint32_t id=0x281;
    uint8_t data[8]={0,1,0x76,0x06,0,0,0,7};
    can_data_received( id,data);
    logic_loop();
}
