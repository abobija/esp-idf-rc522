#pragma once

#include "rc522_types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum // RC522_PCD_RF_CFG_REG
{
    RC522_PCD_RX_GAIN_2_BIT = BIT6,
    RC522_PCD_RX_GAIN_1_BIT = BIT5,
    RC522_PCD_RX_GAIN_0_BIT = BIT4,
};

/**
 * Receiver (antenna) gain
 */
typedef enum
{
    RC522_PCD_18_DB_RX_GAIN = (RC522_PCD_RX_GAIN_1_BIT),                                                     /* 18 dB */
    RC522_PCD_23_DB_RX_GAIN = (RC522_PCD_RX_GAIN_1_BIT | RC522_PCD_RX_GAIN_0_BIT),                           /* 23 dB */
    RC522_PCD_33_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT),                                                     /* 33 dB */
    RC522_PCD_38_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT | RC522_PCD_RX_GAIN_0_BIT),                           /* 38 dB */
    RC522_PCD_43_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT | RC522_PCD_RX_GAIN_1_BIT),                           /* 43 dB */
    RC522_PCD_48_DB_RX_GAIN = (RC522_PCD_RX_GAIN_2_BIT | RC522_PCD_RX_GAIN_1_BIT | RC522_PCD_RX_GAIN_0_BIT), /* 48 dB */
} rc522_pcd_rx_gain_t;

esp_err_t rc522_pcd_set_rx_gain(const rc522_handle_t rc522, rc522_pcd_rx_gain_t gain);

#ifdef __cplusplus
}
#endif
