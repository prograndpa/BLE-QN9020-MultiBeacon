/**
 ****************************************************************************************
 *
 * @file gap_task.h
 *
 * @brief Header file - GAPTASK.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 *
 * @file gap_task.h
 *
 * @brief NXP revised and cut.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef GAP_TASK_H_
#define GAP_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup GAPTASK Task
 * @ingroup GAP
 * @brief Handles ALL messages to/from GAP block.
 *
 * The GAPTASK is responsible for managing the messages coming from
 * the link layer, application layer and host-level layers. The task
 * handling includes device discovery, bonding, connection establishment,
 * link termination and name discovery.
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>
#include <stdint.h>
#include "llm_task.h"
#include "llc_task.h"
#include "smpc.h"
#include "smpm_task.h"
#include "gatt_task.h"
#include "gatt.h"

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */
/// GAP Interface
enum
{
    /// Cancel an existing connection request
    GAP_LE_CANCEL_CONN_REQ = KE_FIRST_MSG(TASK_GAP) + 0,
    /// Read the white list size
    GAP_LE_RD_WLST_SIZE_REQ = KE_FIRST_MSG(TASK_GAP) + 1,
    /// Add device in the white list
    GAP_LE_ADD_DEV_TO_WLST_REQ = KE_FIRST_MSG(TASK_GAP) + 2,
    /// Remove or clear device(s) in the white list
    GAP_LE_RMV_DEV_FRM_WLST_REQ = KE_FIRST_MSG(TASK_GAP) + 3,
    /// Read remote features
    GAP_LE_RD_REMOTE_FEAT_REQ = KE_FIRST_MSG(TASK_GAP) + 4,
    /// Read remote version information
    GAP_RD_REM_VER_INFO_REQ = KE_FIRST_MSG(TASK_GAP) + 5,
    /// Create low energy connection
    GAP_LE_CREATE_CONN_REQ = KE_FIRST_MSG(TASK_GAP) + 6,
    /// Destroy an existing low energy connection
    GAP_DISCON_REQ = KE_FIRST_MSG(TASK_GAP) + 7,
    /// Set device mode
    GAP_SET_MODE_REQ = KE_FIRST_MSG(TASK_GAP) + 8,
    /// Retrieve the name of the remote device
    GAP_NAME_REQ = KE_FIRST_MSG(TASK_GAP) + 9,
    /// Perform device inquiry
    GAP_DEV_INQ_REQ = KE_FIRST_MSG(TASK_GAP) + 10,
    /// Initiate bonding procedure
    GAP_BOND_REQ = KE_FIRST_MSG(TASK_GAP) + 11,
    /// Start or stop the advertising process
    GAP_ADV_REQ = KE_FIRST_MSG(TASK_GAP) + 12,
    /// Start or stop scanning process
    GAP_SCAN_REQ = KE_FIRST_MSG(TASK_GAP) + 13,
    /// Set the random device address
    GAP_SET_RANDOM_ADDR_REQ = KE_FIRST_MSG(TASK_GAP) + 14,
    /// Slave request for connection parameter change
    GAP_PARAM_UPDATE_REQ = KE_FIRST_MSG(TASK_GAP) + 15,
    /// Master initiates a change in connection parameter
    GAP_CHANGE_PARAM_REQ = KE_FIRST_MSG(TASK_GAP) + 16,
    /// Reset the link layer and the host
    GAP_RESET_REQ = KE_FIRST_MSG(TASK_GAP) + 17,
    /// Device name update
    GAP_SET_DEVNAME_REQ = KE_FIRST_MSG(TASK_GAP) + 18,
    /// Read Link and Host layer versions
    GAP_READ_VER_REQ = KE_FIRST_MSG(TASK_GAP) + 19,
    /// Set security mode
    GAP_SET_SEC_REQ = KE_FIRST_MSG(TASK_GAP) + 20,
    /// Read BD address
    GAP_READ_BDADDR_REQ = KE_FIRST_MSG(TASK_GAP) + 21,
    /// Set Reconnection Address feature, only for central
    GAP_SET_RECON_ADDR_REQ = KE_FIRST_MSG(TASK_GAP) + 22,
    /// Set the peer Privacy Flag feature, only for central
    GAP_SET_PH_PRIVACY_REQ = KE_FIRST_MSG(TASK_GAP) + 23,
    /// Set privacy preference
    GAP_SET_PRIVACY_REQ = KE_FIRST_MSG(TASK_GAP) + 24,
    /// Set or retrieve channel map
    GAP_CHANNEL_MAP_REQ = KE_FIRST_MSG(TASK_GAP) + 25,
    /// Read RSSI value
    GAP_READ_RSSI_REQ = KE_FIRST_MSG(TASK_GAP) + 26,
    /// Send bond response
    GAP_BOND_RESP = KE_FIRST_MSG(TASK_GAP) + 27,
    /// Parameter update request indication to Master
    GAP_PARAM_UPDATE_REQ_IND = KE_FIRST_MSG(TASK_GAP) + 28,
    /// Bonding request indication from peer device
    GAP_BOND_REQ_IND = KE_FIRST_MSG(TASK_GAP) + 31,
    /// Parameter update timeout indication
    GAP_PARAM_UPDATE_TO_IND = KE_FIRST_MSG(TASK_GAP) + 34,
    /// Bonding information, indication from above
    GAP_BOND_INFO_IND = KE_FIRST_MSG(TASK_GAP) + 37,
    /// Message from GAP to upper layer, to store known address.
    /// Example, non-resolvable address
    GAP_ADD_KNOWN_ADDR_IND = KE_FIRST_MSG(TASK_GAP) + 38,
    /// Read remote version information complete event
    GAP_RD_REM_VER_INFO_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 40,
    /// Parameter update response
    GAP_PARAM_UPDATE_RESP = KE_FIRST_MSG(TASK_GAP) + 41,
    /// Device inquiry request complete event
    GAP_DEV_INQ_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 42,
    /// Set mode request complete event
    GAP_SET_MODE_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 43,
    /// Name request complete event
    GAP_NAME_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 44,
    /// Bonding request complete event
    GAP_BOND_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 45,
    /// Set random address complete event
    GAP_SET_RANDOM_ADDR_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 46,
    /// Read white list size complete event
    GAP_LE_RD_WLST_SIZE_CMD_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 47,
    /// Add device to white list request complete event
    GAP_LE_ADD_DEV_TO_WLST_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 48,
    /// Remove device(s) from white list complete event
    GAP_LE_RMV_DEV_FRM_WLST_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 49,
    /// Start or stop advertising procedure complete event
    GAP_ADV_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 50,
    /// Start or stop scanning procedure complete event
    GAP_SCAN_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 51,
    /// Disconnection complete event
    GAP_DISCON_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 52,
    /// LE create connection complete event
    GAP_LE_CREATE_CONN_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 53,
    /// Device Inquiry result
    GAP_DEV_INQ_RESULT_EVT = KE_FIRST_MSG(TASK_GAP) + 54,
    /// Advertising report event
    GAP_ADV_REPORT_EVT = KE_FIRST_MSG(TASK_GAP) + 55,
    /// Change parameter request complete event
    GAP_CHANGE_PARAM_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 56,
    /// Read remote feature request complete event
    GAP_LE_RD_REMOTE_FEAT_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 57,
    /// Reset complete event
    GAP_RESET_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 58,
    /// Cancel create connection complete event
    GAP_CANCEL_CONN_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 59,
    /// Device name update complete event
    GAP_SET_DEVNAME_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 60,
    /// Read version request complete event
    GAP_READ_VER_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 61,
    /// Set security mode complete event
    GAP_SET_SEC_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 62,
    /// Read local bd address complete event
    GAP_READ_BDADDR_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 63,
    /// Set Reconnection Address complete event
    GAP_SET_RECON_ADDR_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 64,
    /// Set the privacy feature of peripheral complete event
    GAP_SET_PH_PRIVACY_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 65,
    /// Set privacy feature complete event
    GAP_SET_PRIVACY_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 66,
    /// Known device discovery result complete event
    GAP_KNOWN_DEV_DISC_RESULT_EVT = KE_FIRST_MSG(TASK_GAP) + 67,
    /// Channel map complete event
    GAP_CHANNEL_MAP_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 68,
    /// Read RSSI value complete event
    GAP_READ_RSSI_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 69,
    /// GAP host ready
    GAP_READY_EVT = KE_FIRST_MSG(TASK_GAP) + 70,
    /// Device name changed event
    GAP_DEV_NAME_CHG_EVT = KE_FIRST_MSG(TASK_GAP) + 73,
    /// NP baud rate change request
    GAP_NP_BAUD_CHG_REQ = KE_FIRST_MSG(TASK_GAP) + 74,
    /// NP baud rate change complete event
    GAP_NP_BAUD_CHG_REQ_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 75,
    ///NP read flash program version request
    GAP_NP_READ_FLASH_PROGRAM_VER_REQ = KE_FIRST_MSG(TASK_GAP) + 76,
    ///NP read flash program complete version complete event
    GAP_NP_READ_FLASH_PROGRAM_VER_CMP_EVT = KE_FIRST_MSG(TASK_GAP) + 77,
};
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_GAP_TASK
 * @{
 ****************************************************************************************
 */

/// Connection Parameter update
struct gap_conn_param_update
{
    /// Connection interval minimum
    uint16_t intv_min;
    /// Connection interval maximum
    uint16_t intv_max;
    /// Latency
    uint16_t latency;
    /// Supervision timeout
    uint16_t time_out;
};

/// GAP Connection information @ref GAP_LE_CREATE_CONN_REQ_CMP_EVT message
struct gap_link_info
{
    /// Confirmation status
    uint8_t status;
    /// Connection handle
    uint16_t conhdl;
    /// Peer address type
    uint8_t peer_addr_type;
    /// Peer BT address
    struct bd_addr peer_addr;
    /// Connection interval
    uint16_t con_interval;
    /// Connection latency
    uint16_t con_latency;
    /// Link supervision timeout
    uint16_t sup_to;
    /// Clock accuracy
    uint8_t clk_accuracy;
};

/// Parameters of the @ref GAP_LE_RD_WLST_SIZE_CMD_CMP_EVT message
struct gap_rd_wlst_size_cmd_complete
{
    /// Status of the command reception
    uint8_t status;
    ///White List size
    uint8_t wlst_size;
};

/// Parameters of the @ref GAP_LE_CREATE_CONN_REQ_CMP_EVT message
struct gap_le_create_conn_req_cmp_evt
{
    /// Connection establishment information
    struct gap_link_info conn_info;
};

/// Parameters of the @ref GAP_BOND_REQ_IND message
struct gap_bond_req_ind
{
    /// Device BD Address
    struct bd_addr addr;
    /// Device record index
    uint8_t index;
    /// Authentication Requirements from peer
    uint8_t auth_req;
    ///IO capabilities
    uint8_t io_cap;
    ///Out Of Band Data presence flag
    uint8_t oob_data_flg;
    ///Maximum Encryption Key Size
    uint8_t max_enc_size;
    ///Initiator Key Distribution
    uint8_t ikey_dist;
    ///Responder Key Distribution
    uint8_t rkey_dist;
};

/// Disconnection complete event
struct gap_discon_cmp_evt
{
    /// Reason
    uint8_t reason;
    /// Status
    uint8_t status;
    /// Connection handle
    uint16_t conhdl;
};

/// Parameters of the @ref GAP_RD_REM_VER_INFO_CMP_EVT message
struct gap_rd_rem_ver_info_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///LMP version
    uint8_t vers;
    ///Manufacturer name
    uint16_t compid;
    ///LMP subversion
    uint16_t subvers;
};


/// Parameters of the @ref GAP_SET_RECON_ADDR_REQ_CMP_EVT message
struct gap_set_recon_addr_req_cmp_evt
{
    /// Status
    uint8_t status;
};

/// Parameters of the @ref GAP_SET_PH_PRIVACY_REQ_CMP_EVT message
struct gap_set_ph_privacy_req_cmp_evt
{
    /// Status
    uint8_t status;
};


/// Parameters of the @ref GAP_SET_PRIVACY_REQ_CMP_EVT message
struct gap_set_privacy_req_cmp_evt
{
    /// Status
    uint8_t status;
};

/// Parameters of the @ref GAP_PARAM_UPDATE_RESP message for Slave
struct gap_param_update_resp
{
    /// status
    uint8_t status;
    /// Result
    uint16_t result;
    /// Connection handle
    uint16_t conhdl;
};

/// Parameters of the @ref GAP_PARAM_UPDATE_REQ_IND message for Master
struct gap_param_update_req_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection parameter update request
    struct gap_conn_param_update conn_param;
};

/// Common GAP complete event for higher layer
struct gap_event_common_cmd_complete
{
    /// Command complete status
    uint8_t status;
};

/// Device discovery event result
struct gap_dev_inq_result_evt
{
    ///Number of responses
    uint8_t nb_resp;
    /// advertising report
    struct adv_report adv_rep;
};

/// Device name discovery complete event
struct gap_name_req_cmp_evt
{
    /// status of the name request
    uint8_t status;
    /// Characteristic name
    struct bd_name bdname;
};

/// Bond complete event
struct gap_bond_req_cmp_evt
{
    /// connection handle
    uint16_t conhdl;
    /// record index
    uint8_t idx;
    /// status
    uint8_t status;
    ///Key size for the LTK/STK agreed upon during pairing features exchange)
    uint8_t key_size;
    ///Security properties of the keys
    uint8_t sec_prop;
    ///Bonded status
    uint8_t bonded;
};

/// Event parameter for @ref GAP_LE_RD_REMOTE_FEAT_REQ_CMP_EVT event message
struct gap_le_rd_remote_feat_req_cmp_evt
{
    ///Status of read remote feature request command
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///LE Features used
    struct le_features feats_used;
};

/// Event parameter for @ref GAP_RESET_REQ_CMP_EVT event message
struct gap_reset_req_cmp_evt
{
    ///Status of the reset command
    uint8_t status;
};

/// Event parameter for @ref GAP_CHANGE_PARAM_REQ_CMP_EVT event message
struct gap_change_param_req_cmp_evt
{
    ///Status of the change parameter request
    uint8_t status;
    ///Connection interval value
    uint16_t con_interval;
    ///Connection latency value
    uint16_t con_latency;
    ///Supervision timeout
    uint16_t sup_to;
};

/// Parameters of the @ref GAP_SET_DEVNAME_REQ_CMP_EVT message
struct gap_set_devname_req_cmp_evt
{
    /// status
    uint8_t status;
};

/// Parameters of the @ref GAP_READ_VER_REQ_CMP_EVT message
struct gap_read_ver_req_cmp_evt
{
    /// Status
    uint8_t status;
    /// HCI version
    uint8_t hci_ver;
    /// LMP version
    uint8_t lmp_ver;
    /// Host version
    uint8_t host_ver;
    /// HCI revision
    uint16_t hci_subver;
    /// LMP subversion
    uint16_t lmp_subver;
    /// Host revision
    uint16_t host_subver;
    /// Manufacturer name
    uint16_t manuf_name;
};

/// Parameters of the @ref GAP_SET_SEC_REQ_CMP_EVT message
struct gap_set_sec_req_cmp_evt
{
    /// status
    uint8_t status;
    /// security mode
    uint8_t sec;
};

/// Parameters of the @ref GAP_SET_RANDOM_ADDR_REQ_CMP_EVT message
struct gap_set_random_addr_req_cmp_evt
{
    /// Status
    uint8_t status;
    /// Device BD Address
    struct bd_addr addr;
};

/// Parameters of the @ref GAP_READ_BDADDR_REQ_CMP_EVT message
struct gap_read_bdaddr_req_cmp_evt
{
    /// status
    uint8_t status;
    /// Device BD Address
    struct bd_addr addr;
};

/// Parameters of the @ref GAP_CHANNEL_MAP_CMP_EVT message
struct gap_channel_map_cmp_evt
{
    /// Connection handle
    uint16_t conhdl;
    /// Status
    uint8_t status;
    /// Channel map
    struct le_chnl_map chmap;
};

/// Parameters of the @ref GAP_READ_RSSI_REQ_CMP_EVT message
struct gap_read_rssi_req_cmp_evt
{
    /// Status
    uint8_t status;
    /// RSSI value
    int8_t rssi;
};
/// @} APP_GAP_TASK


/// @cond

/// Advertising information referenced at @ref GAP_SET_MODE_REQ message
struct advertising_info
{
    /// Advertising parameter structure
    struct llm_le_set_adv_param_cmd adv_param;
    /// Advertising data structure
    struct llm_le_set_adv_data_cmd adv_data;
    /// Scan response data structure
    struct llm_le_set_scan_rsp_data_cmd scan_rsp_data;
};

/// Scanning information referenced at @ref GAP_SET_MODE_REQ message
struct scanning_info
{
    /// Scan parameter command structure
    struct llm_le_set_scan_param_cmd set_scan;
    /// Filtering policy
    uint8_t filter_dup;
};

// Generic Access Profile command structures
/// Parameters of the @ref GAP_SET_RANDOM_ADDR_REQ message
struct gap_set_random_addr_req
{
    /// Random Address
    struct bd_addr addr;
};

/// Parameters of the @ref GAP_LE_CREATE_CONN_REQ message
struct gap_le_create_conn_req
{
    /// LE connection command structure
    struct llm_le_create_con_cmd create_cnx;
};

/// Parameters of the @ref GAP_LE_ADD_DEV_TO_WLST_REQ message
struct gap_le_add_dev_to_wlst_req
{
    /// LE add device to white list command structure
    struct llm_le_add_dev_to_wlst_cmd add_dev_to_wlst;
};

/// Parameters of the @ref GAP_LE_RMV_FRM_WLST_REQ message
struct gap_le_rmv_dev_frm_wlst_req
{
    /// Boolean value for remove type command
    bool all_dev;
    /// LE remove device from white list command structure
    struct llm_le_rmv_dev_from_wlst_cmd rmv_dev_from_wlst;
};

/// Parameters of the @ref GAP_LE_RD_REM_USED_FEATS_REQ message
struct gap_le_rd_rem_used_feats_req
{
    /// Read remote used features command structure
    struct llc_le_rd_rem_used_feats_cmd rd_rem_used_feats;
};

/// Parameters of the @ref GAP_SET_PRIVACY_REQ message
struct gap_set_privacy_req
{
    /// Privacy flag:
    uint8_t priv_flag;
    /// Reconnection address: 0x00 hide 0x01 show
    uint8_t recon_addr_visible;
    /// Set to lower layer: 0x00: no set, 0x01: set
    uint8_t set_to_ll;
};

/// Parameters of the @ref GAP_DISCON_REQ message
struct gap_discon_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Disconnection reason
    uint8_t reason;
};

/// Parameters of the @ref GAP_RD_REM_VER_INFO_REQ message
struct gap_rd_rem_ver_info_req
{
    /// Read remote information version command
    struct llc_rd_rem_info_ver_cmd ver;
};


/// Parameters of the @ref GAP_SET_DEVNAME_REQ message
struct gap_set_devname_req
{
    /// BT name
    struct bd_name bdname;
};

/// Parameters of the @ref GAP_SET_MODE_REQ message
struct gap_set_mode_req
{
    /// GAP mode
    uint16_t mode;
    /// Advertising information
    struct advertising_info adv_info;
    /// Scanning information
    struct scanning_info set_scan;
};

/// Parameters of the @ref GAP_ADV_REQ message
struct gap_adv_req
{
    /// Enable or disable advertising
    bool adv_en;
};

/// Parameters of the @ref GAP_SCAN_REQ message
struct gap_scan_req
{
    /// Scan enabling command structure
    struct llm_le_set_scan_en_cmd scan_en;
};

/// Parameters of the @ref GAP_DEV_INQ_REQ message
struct gap_dev_inq_req
{
    /// Inquiry type
    uint8_t inq_type;
    /// Address type
    uint8_t own_addr_type;
    /// Nb of responses
    uint16_t nb_resp;
};

/// Parameters of the @ref GAP_NAME_REQ message
struct gap_name_req
{
    /// LE connection command structure
    struct llm_le_create_con_cmd create_cnx;
};

/// Parameters of the @ref GAP_SET_SEC_REQ message
struct gap_set_sec_req
{
    /// Operation: read = 0x00, write = 0x01
    uint8_t flag;
    /// Security mode
    uint8_t sec;
};

/// Parameters of the @ref GAP_SET_RECON_ADDR_REQ message
struct gap_set_recon_addr_req
{
    /// Connection handle
    uint16_t conhdl;
    /// attribute handle of reconnection address
    uint16_t attrhdl;
};


/// Parameters of the @ref GAP_SET_PH_PRIVACY_REQ message
struct gap_set_ph_privacy_req
{
    /// Enabling value: disable = 0x0000 enable = 0x0001
    uint16_t enable;
    /// Connection handle
    uint16_t conhdl;
    /// attribute handle of reconnection address
    uint16_t attrhdl;
};

/// Parameters of the @ref GAP_BOND_REQ message
struct gap_bond_req
{
    /// BT Address to bond
    struct bd_addr addr;
    /// OOB information
    uint8_t oob;
    /// IO capabilities
    uint8_t iocap;
    /// Authentication requirements
    uint8_t auth;
    /// Encryption key size
    uint8_t key_size;
    ///Initiator key distribution
    uint8_t ikey_dist;
    ///Responder key distribution
    uint8_t rkey_dist;
};

/// Parameters of the @ref GAP_PARAM_UPDATE_REQ message for Slave
struct gap_param_update_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection parameter update request
    struct gap_conn_param_update conn_par;
};

/// Parameters of the @ref GAP_CHANGE_PARAM_REQ message for Master
struct gap_change_param_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Result of connection update request
    uint16_t result;
    /// Connection parameters
    struct gap_conn_param_update conn_par;
};

/// Parameters of the @ref GAP_CHANNEL_MAP_REQ message
struct gap_channel_map_req
{
    /// operation - false: read, true: update
    bool update_map;
    /// Connection handle
    uint16_t conhdl;
    /// Channel map
    struct le_chnl_map chmap;
};

/// Parameters of the @ref GAP_READ_RSSI_REQ message
struct gap_read_rssi_req
{
    /// Connection handle
    uint16_t conhdl;
};

/// Parameters of the @ref GAP_BOND_INFO_IND message
struct gap_bond_info_ind
{
    /// operation: 0x00: added, 0x01: removed
    uint8_t bond_op;
    /// number of bonding info present
    uint8_t nb_bond;
    /// Device BD Address
    struct bd_addr addr;
};

/// Parameters of the @ref GAP_ADD_KNOWN_ADDR_IND message
struct gap_add_known_addr_ind
{
    /// Device BD Address
    struct bd_addr addr;
};

/// Parameters of the @ref GAP_BOND_RESP message
struct gap_bond_resp
{
    /// Connection handle
    uint16_t conhdl;
    /// accept or reject bonding
    uint8_t accept;
    /// IO capabilities
    uint8_t io_cap;
    /// OOB flag
    uint8_t oob;
    /// Authentication Requirements
    uint8_t auth_req;
    /// Max key size
    uint8_t max_key_size;
    /// Initiator key distribution
    uint8_t ikeys;
    /// Responder key distribution
    uint8_t rkeys;
};

/// Advertising event result
struct gap_adv_report_evt
{
    /// status
    uint8_t status;
    /// advertising report
    struct llm_le_adv_report_evt evt;
};

/// Known device discovery event result
struct gap_known_dev_disc_result_evt
{
    /// Resolved BD address
    struct bd_addr res_addr;
    /// Resolved address type
    uint8_t res_addr_type;
    /// Known BD address
    struct bd_addr addr;
    /// Known address type
    uint8_t addr_type;
};

/// Device name changed event
struct gap_dev_name_chg_evt
{
    /// Device name length
    uint8_t length;
    /// Device name
    uint8_t dev_name[1];
};

///read flash program version request
struct gap_np_read_flash_program_ver_req
{
    /// version info length
    int length;
};


// NP baudrate change request
struct gap_baudrate_change_req
{
    int baudrate;
};

/// @endcond


/// @cond

/// @} GAPTASK
/// @endcond
#endif // GAP_TASK_H_
