#include "unity.h"
#include "../Core/Inc/logic.h"
#include "string.h"
#include<unistd.h>



void test_main_logic()
{

}


void setUp (void)
{


} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */
int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_main_logic);

    return UNITY_END();
}
