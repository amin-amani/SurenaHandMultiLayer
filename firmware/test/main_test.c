#include "unity.h"
#include "Logic.h"
#include "string.h"
#include<unistd.h>

void read_adc_mock(uint32_t * data, uint32_t length)
{
    uint16_t ADCResult[6]={1,2,3,4,5,6};
    memcpy(data,ADCResult,length);

}

void test_main_logic()
{

}


void setUp (void)
{
    register_adc_callback(read_adc_mock);

} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */
int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_main_logic);


    return UNITY_END();
}
