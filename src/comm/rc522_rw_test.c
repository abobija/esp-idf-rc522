#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_pcd.h"
#include "comm/rc522_comm.h"
#include "comm/rc522_rw_test.h"
#include "rc522_registers.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_rw_test(rc522_handle_t rc522)
{
    uint8_t tmp;

    ESP_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_FIFO_LEVEL_REG, &tmp), TAG, "Cannot read FIFO length");

    ESP_RETURN_ON_ERROR(rc522_pcd_fifo_flush(rc522), TAG, "Cannot flush FIFO");

    uint8_t buffer1[] = { 0x13, 0x33, 0x37 };
    const uint8_t buffer_size = sizeof(buffer1);
    uint8_t buffer2[buffer_size];

    ESP_RETURN_ON_ERROR(rc522_pcd_write_n(rc522, RC522_FIFO_DATA_REG, buffer_size, buffer1),
        TAG,
        "Cannot write to FIFO");

    RC522_RETURN_ON_ERROR(rc522_pcd_read(rc522, RC522_FIFO_LEVEL_REG, &tmp));

    ESP_RETURN_ON_FALSE(tmp == buffer_size, ESP_FAIL, TAG, "FIFO length missmatch after write");

    RC522_RETURN_ON_ERROR(rc522_pcd_read_n(rc522, RC522_FIFO_DATA_REG, buffer_size, buffer2));

    bool buffers_content_equal = true;
    for (uint8_t i = 0; i < buffer_size; i++) {
        if (buffer1[i] != buffer2[i]) {
            buffers_content_equal = false;
            break;
        }
    }

    if (!buffers_content_equal) {
        RC522_LOGE("Buffers content missmatch");

        RC522_LOGE("Buffer1: ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer1, buffer_size, ESP_LOG_ERROR);
        RC522_LOGE("Buffer2: ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer2, buffer_size, ESP_LOG_ERROR);

        return ESP_FAIL;
    }

    return ESP_OK;
}
