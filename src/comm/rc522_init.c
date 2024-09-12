#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_io.h"
#include "comm/rc522_comm.h"
#include "comm/rc522_rw_test.h"
#include "rc522_registers.h"

static const char *TAG = "rc522_init";

esp_err_t rc522_init(rc522_handle_t rc522)
{
    // TODO: Implement hard reset via RST pin
    //       and ability to choose between hard and soft reset

    ESP_RETURN_ON_ERROR(rc522_soft_reset(rc522, 150), TAG, "");

    // Reset baud rates
    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_TX_MODE_REG, 0x00), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_RX_MODE_REG, 0x00), TAG, "");

    // Reset modulation width
    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_MOD_WIDTH_REG, RC522_MOD_WIDTH_RESET_VALUE), TAG, "");

    ESP_RETURN_ON_ERROR(rc522_configure_timer(rc522, RC522_T_AUTO, 3390), TAG, "");
    ESP_RETURN_ON_ERROR(rc522_set_timer_reload_value(rc522, 30), TAG, "");

    ESP_RETURN_ON_ERROR(rc522_write(rc522, RC522_TX_ASK_REG, RC522_FORCE_100_ASK), TAG, "");
    ESP_RETURN_ON_ERROR(
        rc522_write(rc522, RC522_MODE_REG, (RC522_TX_WAIT_RF | RC522_POL_MFIN | RC522_CRC_PRESET_6363H)),
        TAG,
        "");

    ESP_RETURN_ON_ERROR(rc522_antenna_on(rc522, RC522_RX_GAIN_43_DB), TAG, "Unable to turn on antenna");

    return ESP_OK;
}
