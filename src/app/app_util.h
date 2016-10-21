/**
 ****************************************************************************************
 *
 * @file app_util.h
 *
 * @brief Application Utility API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_UTIL_H_
#define _APP_UTIL_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
// Service setup FLAG
#define BLE_HT_THERMOM_BIT          0x0001
#define BLE_BP_SENSOR_BIT           0x0002
#define BLE_HR_SENSOR_BIT           0x0004
#define BLE_GL_SENSOR_BIT           0x0008
#define BLE_FINDME_TARGET_BIT       0x0010
#define BLE_TIP_SERVER_BIT          0x0020
#define BLE_SP_SERVER_BIT           0x0040
#define BLE_HID_DEVICE_BIT          0x0080
#define BLE_PROX_REPORTER_BIT       0x0100
#define BLE_DIS_SERVER_BIT          0x0200
#define BLE_BATT_SERVER_BIT         0x0400
#define BLE_CSCP_SERVER_BIT         0x0800
#define BLE_PASP_SERVER_BIT         0x1000
#define BLE_AN_SERVER_BIT           0x2000
#define BLE_RSCP_SERVER_BIT         0x4000
#define BLE_QPPS_SERVER_BIT         0x8000

// Advertising data FLAG
#define AD_TYPE_NAME_BIT            0x0001
#define AD_TYPE_16bitUUID_BIT       0x0002

struct app_adv_data
{
    // Advertising data FLAG
    uint16_t flag;
    uint8_t  uuid_num;
    uint8_t  name[ADV_DATA_LEN - 1];  
    uint16_t uuid[ADV_DATA_LEN/2 - 1]; 
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Convert u32 value to array with Hex format
 *
 ****************************************************************************************
 */
void app_i_to_array(uint32_t num, char *array);

/*
 ****************************************************************************************
 * @brief Set local device role
 *
 ****************************************************************************************
 */
void app_set_role(uint8_t role);

/*
 ****************************************************************************************
 * @brief Set local device role
 *
 ****************************************************************************************
 */
uint8_t app_get_role(void);

/*
 ****************************************************************************************
 * @brief Get Device Record Index by connection handle
 *
 ****************************************************************************************
 */
uint8_t app_get_rec_idx_by_conhdl(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Get Connection Handle by Index 
 *
 ****************************************************************************************
 */
uint16_t app_get_conhdl_by_idx(uint8_t idx);

/*
 ****************************************************************************************
 * @brief Get Connection Handle and Index by BD address
 *
 ****************************************************************************************
 */
uint16_t app_get_conhdl_by_bdaddr(struct bd_addr const *addr, uint8_t *idx);

/*
 ****************************************************************************************
 * @brief Get BD Addr by connection handle
 *
 ****************************************************************************************
 */
uint8_t app_get_bd_addr_by_conhdl(uint16_t conhdl, struct bd_addr *addr);

/*
 ****************************************************************************************
 * @brief Get BD Addr by index
 *
 ****************************************************************************************
 */
bool app_get_bd_addr_by_idx(uint8_t idx, struct bd_addr *addr);

/*
 ****************************************************************************************
 * @brief Get link security status by connection handle
 *
 ****************************************************************************************
 */
uint8_t app_get_lk_sec_status(struct bd_addr const *addr);

/*
 ****************************************************************************************
 * @brief Get Connection status by Idx
 *
 ****************************************************************************************
 */
bool app_get_link_status_by_idx(uint8_t idx);

/*
 ****************************************************************************************
 * @brief Get Connection status by Idx
 *
 ****************************************************************************************
 */
bool app_get_link_status_by_idx(uint8_t idx);

/*
 ****************************************************************************************
 * @brief Set Connection status by Idx
 *
 ****************************************************************************************
 */
void app_set_link_status_by_conhdl(uint16_t conhdl, struct gap_link_info const *conn_info, bool conn);

/*
 ****************************************************************************************
 * @brief Get the connection number from the device record
 *
 ****************************************************************************************
 */
uint8_t app_get_link_nb(void);

/*
 ****************************************************************************************
 * @brief Get the bonded number of the device record
 *
 ****************************************************************************************
 */
uint8_t app_get_bond_nb(void);

/*
 ****************************************************************************************
 * @brief Get the bonded staus of the specific device record
 *
 ****************************************************************************************
 */
bool app_get_bond_status(uint8_t idx, uint8_t key_flag);

/**
 ****************************************************************************************
 * @brief Add the bonded device
 *
 ****************************************************************************************
 */
bool app_add_bonded_dev(void *bonded_dev);

/**
 ****************************************************************************************
 * @brief Find the bonded device by device address
 *
 ****************************************************************************************
 */
uint8_t app_find_bonded_dev(struct bd_addr const *addr);

/*
 ****************************************************************************************
 * @brief Check Service setup FLAG and Initiate SMP IRK and CSRK
 *
 ****************************************************************************************
 */
void app_init_local_smp_key(void);

/*
 ****************************************************************************************
 * @brief Get the local service setup inital value
 *
 ****************************************************************************************
 */
uint16_t app_get_local_service_flag(void);

/*
 ****************************************************************************************
 * @brief Clear Service setup FLAG
 *
 ****************************************************************************************
 */
void app_clear_local_service_flag(uint16_t srv_bit);

/*
 ****************************************************************************************
 * @brief Get the remote service UUID enable status from application device record
 *
 ****************************************************************************************
 */
bool app_get_remote_service_status(uint8_t idx, uint16_t uuid);

/**
 ****************************************************************************************
 * @brief Get the client index by connection handle and uuid.
 *
 ****************************************************************************************
 */
uint8_t app_get_client_idx_by_conhdl(uint16_t conhdl, uint16_t uuid);

/*
 ****************************************************************************************
 * @brief Get the remote service UUID enable status from application device record
 *
 ****************************************************************************************
 */
bool app_get_remote_service_status(uint8_t idx, uint16_t uuid);

/*
 ****************************************************************************************
 * @brief Get the client service UUID enable status from application device record
 *
 ****************************************************************************************
 */
uint8_t app_get_client_service_status(uint8_t idx, uint16_t uuid);

/*
 ****************************************************************************************
 * @brief Get the QPP service UUID enable status from application device record
 *
 ****************************************************************************************
 */
uint8_t app_get_qpp_client_service_status(uint8_t idx);

/*
 ****************************************************************************************
 * @brief Set the QPP service enable status from application device record
 *
 ****************************************************************************************
 */
void app_set_qpp_client_service_status(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Set the client service enable status from application device record
 *
 ****************************************************************************************
 */
void app_set_client_service_status(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Create server service database
 *
 ****************************************************************************************
 */
void app_create_server_service_DB(void);

/**
 ****************************************************************************************
 * @brief Enable server service
 *
 ****************************************************************************************
 */
void app_enable_server_service(uint8_t enabled, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Set Advertising data
 *
 ****************************************************************************************
 */
uint8_t app_set_adv_data(uint16_t disc_mode);

/*
 ****************************************************************************************
 * @brief Set Scan response data
 *
 ****************************************************************************************
 */
uint8_t app_set_scan_rsp_data(uint16_t srv_flag);

/*
 ****************************************************************************************
 * @brief Parser Advertising or Scan response data
 *
 ****************************************************************************************
 */
bool app_parser_adv_data(uint8_t *pdata, uint8_t total_len, struct app_adv_data *padv);

/**
 ****************************************************************************************
 * @brief Check Updated Connection Parameters is acceptable or not
 *
 ****************************************************************************************
 */
bool app_check_update_conn_param(struct gap_conn_param_update const *conn_param);

/**
 ****************************************************************************************
 * @brief Configure ble new features
 ****************************************************************************************
 */
void ble_new_feature(void);
#endif

