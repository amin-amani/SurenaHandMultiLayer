#include "unity.h"
#include "unity_fixture.h"


void runAllTests(void)
{
   RUN_TEST_GROUP(LOGIC_TEST_GROUP);


}

int main()
{
    UnityBegin("test");
    runAllTests();
    return UnityEnd();
}
