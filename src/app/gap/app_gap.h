/**
 ****************************************************************************************
 *
 * @file app_gap.h
 *
 * @brief Application GAP API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_GAP_H_
#define _APP_GAP_H_


/**
 ****************************************************************************************
 * @addtogroup APP_GAP_API Generic Access Profile API
 * @ingroup APP_GAP
 * @brief Generic Access Profile API
 *
 * The GAP layer of the BLE Protocol Stack is responsible for handling the device's access 
 * modes and procedures, including device discovery, link establishment, link termination, 
 * initiation of security features, and device configuration.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gap.h"
#include "gap_task.h"
#include "app_gap_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Device name update request
 *
 ****************************************************************************************
 */
void app_gap_set_devname_req(uint8_t const *name, uint8_t len);

/*
 ****************************************************************************************
 * @brief Set security mode request, change this setting before any Adv/Scan/Connect Action
 *
 ****************************************************************************************
 */
void app_gap_set_sec_req(uint8_t sec_lvl);

/*
 ****************************************************************************************
 * @brief Read local version request
 *
 ****************************************************************************************
 */
void app_gap_read_ver_req(void);

/*
 ****************************************************************************************
 * @brief Read local BD addr request
 *
 ****************************************************************************************
 */
void app_gap_read_bdaddr_req(void);

/*
 ****************************************************************************************
 * @brief Set the Device mode
 *
 ****************************************************************************************
 */
void app_gap_set_bondable_mode_req(void);

/*
 ****************************************************************************************
 * @brief Inquiry the nearby BLE devices
 *
 ****************************************************************************************
 */
void app_gap_dev_inq_req(uint8_t inq_type, uint8_t own_addr_type);

/*
 ****************************************************************************************
 * @brief Cancle the inquiry the nearby BLE devices process
 *
 ****************************************************************************************
 */
void app_gap_dev_inq_cancel_req(void);

/*
 ****************************************************************************************
 * @brief scan the nearby BLE devices process
 *
 ****************************************************************************************
 */
void app_gap_scan_req(uint8_t scan_en);

/*
 ****************************************************************************************
 * @brief Set scan mode request from the remote device
 *
 ****************************************************************************************
 */
void app_gap_set_scan_mode_req(uint8_t scan_type, uint16_t scan_intv, uint16_t scan_window);

/*
 ****************************************************************************************
 * @brief get device name request from the remote device
 *
 ****************************************************************************************
 */
void app_gap_name_req(struct bd_addr *p_addr, uint8_t addr_type, uint8_t own_addr_type);

/*
 ****************************************************************************************
 * @brief Bond with the remote BLE device
 *
 ****************************************************************************************
 */
void app_gap_bond_req(struct bd_addr *addr, uint8_t oob, uint8_t auth, uint8_t iocap);

/*
 ****************************************************************************************
 * @brief Bond response to the remote BLE device
 *
 ****************************************************************************************
 */
void app_gap_bond_resp(uint16_t conhdl, uint8_t reject, uint8_t oob, uint8_t auth, uint8_t iocap);

/*
 ****************************************************************************************
 * @brief Unbond with the remote BLE device
 *
 ****************************************************************************************
 */
void app_gap_unpair_req(struct bd_addr *addr, uint8_t nb_bond);

/*
 ****************************************************************************************
 * @brief Initiate a encryption or pairing procedure
 *
 ****************************************************************************************
 */
void app_gap_security_req(struct bd_addr *addr);

/*
 ****************************************************************************************
 * @brief Create connection with the remote BLE device
 *
 ****************************************************************************************
 */
void app_gap_le_create_conn_req(struct bd_addr *addr, uint8_t addr_type, uint8_t own_addr_type,
                                uint16_t conn_intv_min, uint16_t conn_intv_max, uint16_t cnnn_timeout);
/*
 ****************************************************************************************
 * @brief Cancel an existing connection request
 *
 ****************************************************************************************
 */
void app_gap_le_cancel_conn_req(void);

/*
 ****************************************************************************************
 * @brief Disconnect the current connection with the remote BLE device
 *
 ****************************************************************************************
 */
void app_gap_discon_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Start the device to advertising process
 *
 ****************************************************************************************
 */
void app_gap_adv_start_req(uint16_t mode, uint8_t *adv_data, uint8_t adv_data_len, 
                        uint8_t *scan_rsp_data, uint8_t scan_rsp_data_len, uint16_t adv_intv_min, uint16_t adv_intv_max);

/*
 ****************************************************************************************
 * @brief Stop the advertising process
 *
 ****************************************************************************************
 */
void app_gap_adv_stop_req(void);

/*
 ****************************************************************************************
 * @brief Read the white list size
 *
 ****************************************************************************************
 */
void app_gap_le_rd_wlst_size_req(void);

/*
 ****************************************************************************************
 * @brief Add device in the white list
 *
 ****************************************************************************************
 */
void app_gap_le_add_dev_to_wlst_req(uint8_t addr_type, struct bd_addr *addr);

/*
 ****************************************************************************************
 * @brief Remove or clear device(s) in the white list
 *
 ****************************************************************************************
 */
void app_gap_le_rmv_dev_frm_wlst_req(bool all_dev, uint8_t addr_type, struct bd_addr *addr);

/*
 ****************************************************************************************
 * @brief Read remote features
 *
 ****************************************************************************************
 */
void app_gap_le_rd_remote_feat_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Read remote version information
 *
 ****************************************************************************************
 */
void app_gap_rd_rem_ver_info_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Set the random device address
 *
 ****************************************************************************************
 */
void app_gap_set_random_addr_req(struct bd_addr *addr);

/*
 ****************************************************************************************
 * @brief Slave request for connection parameter change
 *
 ****************************************************************************************
 */
void app_gap_param_update_req(uint16_t conhdl, struct gap_conn_param_update *conn_par);

/*
 ****************************************************************************************
 * @brief Master initiates a change in connection parameter
 *
 ****************************************************************************************
 */
void app_gap_change_param_req(uint16_t conhdl, uint16_t result, struct gap_conn_param_update *conn_par);

/*
 ****************************************************************************************
 * @brief Reset the link layer and the host
 *
 ****************************************************************************************
 */
void app_gap_reset_req(void);

/*
 ****************************************************************************************
 * @brief Set Reconnection Address feature, only for central
 *
 ****************************************************************************************
 */
void app_gap_set_recon_addr_req(uint16_t conhdl, uint16_t attrhdl);

/*
 ****************************************************************************************
 * @brief Set the peer Privacy Flag feature, only for central
 *
 ****************************************************************************************
 */
void app_gap_set_ph_privacy_req(uint16_t enable, uint16_t conhdl, uint16_t attrhdl);

/*
 ****************************************************************************************
 * @brief Set privacy preference
 *
 ****************************************************************************************
 */
void app_gap_set_privacy_req(uint8_t priv_flag, uint8_t recon_addr_visible, uint8_t set_to_ll);

/*
 ****************************************************************************************
 * @brief Set or retrieve channel map
 *
 ****************************************************************************************
 */
void app_gap_channel_map_req(bool update_map, uint16_t conhdl, struct le_chnl_map *chmap);

/*
 ****************************************************************************************
 * @brief Read RSSI value
 *
 ****************************************************************************************
 */
void app_gap_read_rssi_req(uint16_t conhdl);

/// @} APP_GAP_API

#endif // APP_GAP_H_

