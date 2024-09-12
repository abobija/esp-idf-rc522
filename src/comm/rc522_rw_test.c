#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_io.h"
#include "comm/rc522_comm.h"
#include "comm/rc522_rw_test.h"
#include "rc522_registers.h"

static const char *TAG = "rc522_rw_test";

esp_err_t rc522_rw_test(rc522_handle_t rc522, uint8_t test_register, uint8_t times)
{
    uint8_t origin_value;

    ESP_RETURN_ON_ERROR(rc522_read(rc522, test_register, &origin_value),
        TAG,
        "Unable to read origin value of 0x%20X register",
        test_register);

    for (uint8_t val = 0; val < times; val++) {
        ESP_RETURN_ON_ERROR(rc522_write(rc522, test_register, val),
            TAG,
            "Unable to write value %d into 0x%20X register",
            val,
            test_register);

        uint8_t real_value;

        ESP_RETURN_ON_ERROR(rc522_read(rc522, test_register, &real_value),
            TAG,
            "Unable to read value of 0x%20X register",
            test_register);

        ESP_RETURN_ON_FALSE(val == real_value,
            ESP_FAIL,
            TAG,
            "Value %d of register 0x%20X does not match previously written value %d",
            real_value,
            test_register,
            val);
    }

    ESP_RETURN_ON_ERROR(rc522_write(rc522, test_register, origin_value),
        TAG,
        "Unable to restore origin value of 0x%20X register",
        test_register);

    return ESP_OK;
}
