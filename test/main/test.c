#include <stdio.h>
#include <esp_system.h>
#include "unity.h"

#include "picc/test_mifare.c"

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    exit(UNITY_END());
}
