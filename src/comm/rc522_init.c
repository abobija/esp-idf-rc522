#include <esp_system.h>
#include <esp_check.h>
#include <string.h>

#include "rc522_pcd.h"
#include "comm/rc522_comm.h"
#include "comm/rc522_rw_test.h"
#include "rc522_registers.h"

RC522_LOG_DEFINE_BASE();

esp_err_t rc522_init(rc522_handle_t rc522)
{
    // TODO: Implement hard reset via RST pin
    //       and ability to choose between hard and soft reset

    RC522_RETURN_ON_ERROR(rc522_soft_reset(rc522, 150));

    // Reset baud rates
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TX_MODE_REG, 0x00));
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_RX_MODE_REG, 0x00));

    // Reset modulation width
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_MOD_WIDTH_REG, RC522_MOD_WIDTH_RESET_VALUE));

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.

    // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TIMER_MODE_REG, RC522_T_AUTO));

    // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TIMER_PRESCALER_REG, 0xA9));

    // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TIMER_RELOAD_MSB_REG, 0x03));
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TIMER_RELOAD_LSB_REG, 0xE8));

    // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_TX_ASK_REG, 0x40));

    // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3
    // part 6.2.4)
    RC522_RETURN_ON_ERROR(rc522_write(rc522, RC522_MODE_REG, 0x3D));

    // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
    RC522_RETURN_ON_ERROR(rc522_antenna_on(rc522));

    return ESP_OK;
}
