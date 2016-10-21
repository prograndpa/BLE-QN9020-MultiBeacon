/**
 ****************************************************************************************
 *
 * @file gatt_task.h
 *
 * @brief Header file - GATTTASK.
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
 * @file gatt_task.h
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
#ifndef GATT_TASK_H_
#define GATT_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup GATTTASK Task
 * @ingroup GATT
 * @brief Handles ALL messages to/from GATT block.
 *
 * The GATTTASK is responsible for managing the messages coming from
 * the attribute layer and host-level layers. The task includes services
 * and characteristic discovery, configuration exchanges and attribute
 * value access operations (reading, writing, notification and indication).
 * characteristic value, notification of a characteristic value and indication
 *
 * Messages may originate from @ref ATTM "ATTM", @ref ATTC "ATTC", @ref ATTS "ATTS",
 * @ref GAP "GAP" and Application.
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "attm.h"
#include "gatt.h"
#include "co_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// maximum number of handles returned
#define GATT_MAX_HDL_LIST                            0x08

/// GATT Task messages
enum
{
    // HL -> GATT
    /* ATTRIBUTE CLIENT */
    /// Server configuration request
    GATT_EXC_MTU_REQ = KE_FIRST_MSG(TASK_GATT),
    /*Discover All Services */
    /*Discover Services by Service UUID*/
    /*Find Included Services*/
    /// Service discovery request
    GATT_DISC_SVC_REQ,
    /*Discover Characteristics by UUID*/
    /*Discover All Characteristics of a Service*/
    /// Characteristic discovery request
    GATT_DISC_CHAR_REQ,
    /*Discover All Characteristic Descriptors*/
    /// Characteristic descriptor discovery request
    GATT_DISC_CHAR_DESC_REQ,
    /*Read Characteristic Value*/
    /*Read Signed Characteristic Value*/
    /*Read Using Characteristic UUID*/
    /*Read Long Characteristic Value*/
    /*Read Multiple Characteristic Values*/
    /*Read Characteristic Descriptors*/
    /*Read Long Characteristic Descriptor*/
    /// Read characteristic request
    GATT_READ_CHAR_REQ,
    /*Write without response*/
    /*Write without response with Authentication*/
    /*Write Characteristic Value*/
    /*Signed Write Characteristic Value*/
    /*Write Long Characteristic Value*/
    /*Characteristic Value Reliable Write*/
    /*Write Characteristic Descriptors*/
    /*Write Long Characteristic Descriptors*/
    /// Write characteristic request
    GATT_WRITE_CHAR_REQ,
    /// Write reliable characteristic request
    GATT_WRITE_RELIABLE_REQ,
    /* Cancel / Execute Reliable Write Characteristic*/
    /// Execute write characteristic request
    GATT_EXECUTE_WRITE_CHAR_REQ,
    /* ATTRIBUTE SERVER */
    /*Notify Characteristic, unsigned*/
    /*Notify Characteristic, signed*/
    /// Notification request
    GATT_NOTIFY_REQ,
    /*Indications*/
    /*Indicate Characteristic, signed*/
    /// Indication request
    GATT_INDICATE_REQ,
    /* Attribute Manipulation */
    /// Add or write (update) an attribute to Attribute Server Database
    GATT_WRITE_ATTRIBUTE_REQ,
    /// Remove an attribute to Attribute Server Database
    GATT_CLEAR_ATTRIBUTE_REQ,
    /// Read an attribute to Attribute Server Database
    GATT_READ_ATTRIBUTE_REQ,
    /* GATT -> HL: Events to Upper layer */
    /*Discover All Services*/
    /// Discovery all services complete event
    GATT_DISC_SVC_ALL_CMP_EVT,
    /*Discover All Services by UUID*/
    /// Discover service by UUID complete event
    GATT_DISC_SVC_BY_UUID_CMP_EVT,
    /*Find Included Services*/
    /// Discover included services complete event
    GATT_DISC_SVC_INCL_CMP_EVT,
    /*Discover All Characteristics of a Service*/
    /// Discover all characteristic complete event
    GATT_DISC_CHAR_ALL_CMP_EVT,
    /*Discover Characteristics by UUID*/
    /// Discovery characteristic by UUID complete event
    GATT_DISC_CHAR_BY_UUID_CMP_EVT,
    /*Discover All Characteristic Descriptors*/
    /// Discovery characteristic descriptor complete event
    GATT_DISC_CHAR_DESC_CMP_EVT,
    /*Read Characteristic Value*/
    /*Read Signed Characteristic Value*/
    /*Read Using Characteristic UUID*/
    /// Read characteristic response
    GATT_READ_CHAR_RESP,
    /*Read Multiple Characteristic Values*/
    /// Read characteristic multiple response
    GATT_READ_CHAR_MULT_RESP,
    /*Read Characteristic Descriptors*/
    /// Read characteristic descriptor response
    GATT_READ_CHAR_DESC_RESP,
    /*Read Long Characteristic Descriptor*/
    /// Read characteristic long descriptor response
    GATT_READ_CHAR_LONG_DESC_RESP,
    /*Write Characteristic Value*/
    /// Write characteristic response
    GATT_WRITE_CHAR_RESP,
    /*Characteristic Value Reliable Write*/
    /// Read characteristic reliable response
    GATT_WRITE_CHAR_RELIABLE_RESP,
    /*Cancel Reliable Write Characteristic*/
    /// Cancel write characteristic response
    GATT_CANCEL_WRITE_CHAR_RESP,
    /*Write Characteristic Descriptors*/
    /// Handle Value notification
    GATT_HANDLE_VALUE_NOTIF,
    /// Handle Value indication
    GATT_HANDLE_VALUE_IND,
    /// Confirmation response
    GATT_HANDLE_VALUE_CFM,
    /// Discovery complete event
    GATT_DISC_CMP_EVT,
    /* Default complete event - status, errors */
    /// Generic complete event
    GATT_CMP_EVT,
    /* Attribute manipulation */
    /// Add or write (update) an attribute complete event
    GATT_WRITE_ATTRIBUTE_CMP_EVT,
    /// Remove an attribute to Attribute Server complete event
    GATT_CLEAR_ATTRIBUTE_CMP_EVT,
    /// Read an attribute to Attribute Server complete event
    GATT_READ_ATTRIBUTE_CMP_EVT,
    /// Debug message event
    GATT_SIGN_KEY_EVT,
    /// Write command indication
    GATT_WRITE_CMD_IND,
    /// Register a SVC for indications, notifications or confirms forward
    GATT_SVC_REG2PRF_REQ,
    /// Discovery all services complete event
    GATT_DISC_SVC_ALL_128_CMP_EVT,
    /// Discovery all characteristics complete event
    GATT_DISC_CHAR_ALL_128_CMP_EVT,
    /// Discovery characteristic by UUID complete event
    GATT_DISC_CHAR_BY_UUID_128_CMP_EVT,
    /// Discovery characteristic descriptor complete event
    GATT_DISC_CHAR_DESC_128_CMP_EVT,
    /// Timeout notification
    GATT_TIMEOUT_EVT,
    /// Unregister a SVC for indications, notifications
    GATT_SVC_UNREG2PRF_REQ,
    /// Informs that notification request has been performed
    GATT_NOTIFY_CMP_EVT,
    /// Used to inform upper layers that a peer device request access of database resources
    GATT_RESOURCE_ACCESS_REQ,
    /// When response is received, peer device is able to access attribute database.
    GATT_RESOURCE_ACCESS_RSP,
    /// Send a Service Changed Characteristic Indication to all connected devices or only to a specified device
    GATT_SVC_CHANGED_IND_SEND_REQ,
    /// Inform the application about a new indication configuration
    GATT_SVC_CHANGED_CFG_IND,
    /// Send write response
    GATT_WRITE_RSP,
    /// Read request indication
    GATT_READ_REQ_IND,
    /// read request confirm
    GATT_READ_REQ_CFM,
};

/// Read request indication Structure
struct gatt_read_req_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Attribute handle
    uint16_t hdl;
};
/// Read request confirm Struture
struct gatt_read_req_cfm
{
    /// Connection handle
    uint16_t conhdl;
    /// Attribute handle
    uint16_t atthdl;
    /// size of the value data
    uint16_t val_len;
    /// value data
    uint8_t value[1];
};
/// attribute MTU exchange request Structure
struct gatt_exc_mtu_req
{
    /// connection handle
    uint16_t conhdl;
};

/// Service Discovery Request Structure
struct gatt_disc_svc_req
{
    /// GATT request type
    uint8_t req_type;
    /// connection handle
    uint16_t conhdl;
    /// start handle range
    uint16_t start_hdl;
    /// start handle range
    uint16_t end_hdl;
    /// desired service: if 0x0000, discover all
    struct att_desired_type desired_svc;
};

/// Characteristic Discovery Request Structure
struct gatt_disc_char_req
{
    /// GATT request type
    uint8_t req_type;
    /// connection handle
    uint16_t conhdl;
    /// start handle range
    uint16_t start_hdl;
    /// end handle range
    uint16_t end_hdl;
    /// desired UUID in disc service char
    struct att_desired_type desired_char;
};

/// Characteristic Descriptor Discovery Request Structure
struct gatt_disc_char_desc_req
{
    /// connection handle
    uint16_t conhdl;
    /// start handle range
    uint16_t start_hdl;
    /// end handle range
    uint16_t end_hdl;
};

/// Read Characteristic Values and Descriptor Request Structure
struct gatt_read_char_req
{
    /// request type
    uint8_t req_type;
    /// read offset
    uint16_t offset;
    /// connection handle
    uint16_t conhdl;
    /// start handle range
    uint16_t start_hdl;
    /// end handle range
    uint16_t end_hdl;
    /// number of UUID
    uint16_t nb_uuid;
    /// characteristic UUID
    struct att_uuid_type uuid[ATT_NB_MULT_HDLS];
};

/// Write Reliable Characteristic Request Structure
struct gatt_write_reliable_req
{
    /// number of reliable writes
    uint8_t nb_writes;
    /// automatic execute write
    uint8_t auto_execute;
    /// connection handle
    uint16_t conhdl;
    /// number of reliable
    struct gatt_reliable_write reliable_write[1];
};

/// Write Characteristic Request Structure
struct gatt_write_char_req
{
    /// connection handle
    uint16_t conhdl;
    /// valid characteristic handle
    uint16_t charhdl;
    /// offset to write
    uint16_t wr_offset;
    /// size of the value data
    uint16_t val_len;
    /// request type
    uint8_t req_type;
    /// execute write
    uint8_t auto_execute;
    /// check, maybe union type is required
    uint8_t value[1];
};

/// execute write characteristic request Structure
struct gatt_execute_write_char_req
{
    /// connection handle
    uint16_t conhdl;
    /// flag to describe if write or cancel
    uint8_t exe_wr_ena;
};

/// notify request Structure
struct gatt_notify_req
{
    /// connection handle
    uint16_t conhdl;
    /// connection handle
    uint16_t charhdl;
};

/// Indicate Request Structure
struct gatt_indicate_req
{
    /// connection handle
    uint16_t conhdl;
    /// characteristic handle
    uint16_t charhdl;
};

/* Attribute server manipulation */
/// Add or write (update) an attribute to Attribute Server Database
struct gatt_write_attribute_req
{
    /// size of the data
    uint16_t length;
    /// attribute handle
    uint16_t attr_hdl;
    /// attribute permission
    uint16_t perm;
    /// value to write
    uint8_t val[ATTM_MAX_VALUE];
};

/// Read an attribute to Attribute Server Database
struct gatt_read_attribute_req
{
    /// attribute handle
    uint16_t attr_hdl;
};

///Parameters for @ref GATT_SVC_REG2PRF_REQ message
struct gatt_svc_reg2prf_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Service start handle
    uint16_t svc_shdl;
    /// Service end handle
    uint16_t svc_ehdl;
};

///Parameters for @ref GATT_SVC_UNREG2PRF_REQ message
struct gatt_svc_unreg2prf_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Service start handle
    uint16_t svc_shdl;
    /// Service end handle
    uint16_t svc_ehdl;
};



/// Data list for service result
struct gatt_svc_list
{
    /// start handle
    uint16_t start_hdl;
    /// end handle
    uint16_t end_hdl;
    /// attribute handle
    uint16_t attr_hdl;
};

/// Data list for service result
struct gatt_svc_128_list
{
    /// start handle
    uint16_t start_hdl;
    /// end handle
    uint16_t end_hdl;
    /// attribute handle
    uint8_t attr_hdl[GATT_UUID_16_OCTET_LEN];
};
/// Data list for characteristic result - 16-bit
struct gatt_char_list
{
    /// database element handle
    uint16_t attr_hdl;
    /// properties
    uint8_t prop;
    /// pointer handle to UUID
    uint16_t pointer_hdl;
    /// characteristic UUID
    uint16_t uuid;
};

/// Data list for characteristic result - 128-bit
struct gatt_char_128_list
{
    /// database element handle
    uint16_t attr_hdl;
    /// properties
    uint8_t prop;
    /// pointer handle to UUID
    uint16_t pointer_hdl;
    /// characteristic UUID
    uint8_t uuid[GATT_UUID_16_OCTET_LEN];
};

/// Data list for include result
struct gatt_incl_list
{
    /// element handle
    uint16_t attr_hdl;
    /// start handle
    uint16_t start_hdl;
    /// end handle
    uint16_t end_hdl;
    /// included service UUID
    uint16_t uuid;
};

/// Special data list for include result - 128
struct gatt_incl_128_list
{
    /// element handle
    uint16_t attr_hdl;
    /// start handle
    uint16_t start_hdl;
    /// end handle
    uint16_t end_hdl;
    /// included 128-bit service UUID
    uint8_t uuid[GATT_UUID_16_OCTET_LEN];
};

/// Special data list for descriptor result - 128
struct gatt_char_desc_list
{
    /// database element handle
    uint16_t attr_hdl;
    /// descriptor UUID
    uint16_t desc_hdl;
};

/// Special data list for descriptor result - 128
struct gatt_char_desc_128_list
{
    /// database element handle
    uint16_t attr_hdl;
    /// included 128-bit service UUID
    uint8_t uuid[GATT_UUID_16_OCTET_LEN];
};


/// Discover All Complete Event Structure
struct gatt_disc_svc_all_128_cmp_evt
{
    /// complete event status
    uint8_t status;
    /// number of value pairs
    uint8_t nb_resp;
    /// contain data list
    struct gatt_svc_128_list list;
};

/// service range
struct gatt_svc_range_list
{
    /// start handle
    uint16_t start_hdl;
    /// end handle
    uint16_t end_hdl;
};

/// Discover Service Include Complete Event Structure
struct gatt_disc_svc_incl_cmp_evt
{
    /// number of found includes
    uint8_t nb_entry;
    /// size of the entry
    uint8_t entry_len;
    /// include entries
    union incl_list_tag {
        /// one include with 128-bit entry
        struct gatt_incl_128_list incl;
        /// list of include declarations
        struct gatt_incl_list list[GATT_MAX_HDL_LIST];
    } incl_list;
};

/// Discovery All Characteristic Complete Event Structure
struct gatt_disc_char_all_cmp_evt
{
    /// complete event status
    uint8_t status;
    /// number of entries
    uint8_t nb_entry;
    /// list of found characteristics
    struct gatt_char_list list[GATT_MAX_HDL_LIST];
};

/// Discovery All Characteristic Complete Event Structure
struct gatt_disc_char_all_128_cmp_evt
{
    /// complete event status
    uint8_t status;
    /// number of entries
    uint8_t nb_entry;
    /// list of found characteristics
    struct gatt_char_128_list list;
};

/// Discover Characteristic by UUID Complete Event Structure
struct gatt_disc_char_by_uuid_cmp_evt
{
    /// complete event status
    uint8_t status;
    /// number of entries
    uint8_t nb_entry;
    /// list of found characteristics with UUID
    struct gatt_char_list list[GATT_MAX_HDL_LIST];
};

/// Discover Characteristic by UUID Complete Event Structure
struct gatt_disc_char_by_uuid_128_cmp_evt
{
    /// complete event status
    uint8_t status;
    /// number of entries
    uint8_t nb_entry;
    /// found 128-bit characteristics with UUID
    struct gatt_char_128_list list;
};

/// Discovery Characteristic Descriptor Complete Event Structure
struct gatt_disc_char_desc_cmp_evt
{
    /// number of found includes
    uint8_t nb_entry;
    /// list of include declarations
    struct gatt_char_desc_list list[GATT_MAX_HDL_LIST];
};

/// Discovery Characteristic Descriptor Complete Event Structure
struct gatt_disc_char_desc_128_cmp_evt
{
    /// number of found includes
    uint8_t nb_entry;
    /// include entries
    struct gatt_char_desc_128_list list_128;
};

/// Read Characteristic Response Structure
struct gatt_read_char_resp
{
    /// complete event status
    uint8_t status;
    /// holder of retrieved data
    struct att_info_data data;
};

/// Read Characteristic Long Response Structure
struct gatt_read_char_long_resp
{
    /// complete event status
    uint8_t status;
    /// size of the value data
    uint8_t val_len;
    /// handle
    uint16_t attr_hdl;
    /// actual value pairs
    uint8_t value[ATTM_MAX_LONG_VALUE];
};

/// Query result for multiple responses
struct gatt_query_result
{
    /// length of value
    uint8_t len;
    /// data result from query
    uint8_t value[ATTM_MAX_VALUE];
};

/// Read Characteristic Multiple Responses Structure
struct gatt_read_char_mult_resp
{
    /// complete event status
    uint8_t status;
    /// size of the response data
    uint8_t val_len;
    /// actual value pairs
    struct gatt_query_result data[ATT_NB_MULT_HDLS];
};

/// Read Characteristic Long Descriptor Response Structure
struct gatt_read_char_long_desc_resp
{
    /// complete event status
    uint8_t status;
    /// size of the value data
    uint8_t val_len;
    /// actual value pairs
    uint8_t value[ATTM_MAX_LONG_VALUE];
    /// handle
    uint16_t attr_hdl;
};

/// Write Characteristics Response Structure
struct gatt_write_char_resp
{
    /// complete event status
    uint8_t status;
};

/// Write Reliable Response Structure
struct gatt_write_reliable_resp
{
    /// complete event status
    uint8_t status;
};

/// GATT Complete Event Structure
struct gatt_cmp_evt
{
    /// complete event status
    uint8_t status;
};

/// Add or write (update) an attribute to Attribute Server Database
struct gatt_write_attribute_cmp_evt
{
    /// complete event status
    uint8_t status;
};

/// Read an attribute to Attribute Server Database
struct gatt_read_attribute_cmp_evt
{
    /// attribute handle
    uint16_t elmt_hdl;
    /// length of the data
    uint16_t len;
    /// permission
    uint16_t perm;
    /// complete event status
    uint8_t status;
    /// attribute type
    uint8_t uuid_len;
    /// attribute type
    uint8_t uuid[ATT_UUID_128_LEN];
    /// attribute value
    uint8_t value[GATT_DB_MAX_SIZE];
};

/// Handle value notification
struct gatt_handle_value_notif
{
    /// connection handle
    uint16_t conhdl;
    /// attribute handle
    uint16_t charhdl;
    /// size of data
    uint8_t size;
    /// value
    uint8_t value[ATTM_MAX_VALUE];
};

/// Handle value indication
struct gatt_handle_value_ind
{
    /// connection handle
    uint16_t conhdl;
    /// attribute handle
    uint16_t charhdl;
    /// size of data
    uint8_t size;
    /// value
    uint8_t value[ATTM_MAX_VALUE];
};

/// Handle value confirmation
struct gatt_handle_value_cfm
{
    /// element handle
    uint16_t handle;
    /// status
    uint8_t status;
};

/// Parameters of the @ref GATT_SIGN_KEY_EVT message
struct gatt_sign_key_evt
{
    /// Signature
    uint8_t sign[ATT_SIGNATURE_LEN];
};

/// write command indication
struct gatt_write_cmd_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// element handle
    uint16_t handle;
    /// length
    uint16_t length;
    /// offset
    uint16_t offset;
    /// Flag used to know if peer device shall receive write response
    uint8_t response;
    /// Flag used to know if peer the indication is the latest
    bool last;
    /// value written
    uint8_t value[ATTM_MAX_VALUE];
};

///Parameters for @ref GATT_NOTIFY_CMP_EVT message
struct  gatt_notify_cmp_evt
{
    /// attribute handle
    uint16_t handle;
    /// status
    uint8_t status;
};

/// Peer device request access of device resources
struct gatt_resource_access_req
{
    /// device connection handle
    uint16_t    conhdl;
};

/// @endcond


/**
 ****************************************************************************************
 * @addtogroup APP_GATT_TASK
 * @{
 ****************************************************************************************
 */
 
/// Discover All Complete Event Structure
struct gatt_disc_svc_all_cmp_evt
{
    /// complete event status
    uint8_t status;
    /// number of value pairs
    uint8_t nb_resp;
    /// contain data list
    struct gatt_svc_list list[GATT_MAX_HDL_LIST];
};

/// Discover by UUID Complete Event Structure
struct gatt_disc_svc_by_uuid_cmp_evt
{
    /// complete event status
    uint8_t status;
    /// number of value pairs
    uint8_t nb_resp;
    /// list of found services
    struct gatt_svc_range_list list[GATT_MAX_HDL_LIST];
};

/// GATT Discovery Complete Event Structure
struct gatt_disc_cmp_evt
{
    /// complete event status
    uint8_t status;
};

/// Current device is ready to access peer device requests
struct gatt_resource_access_rsp
{
    /// device connection handle
    uint16_t    conhdl;
};

/// @} APP_GATT_TASK

/// @cond

/// @} GATTTASK
/// @endcond
#endif // GATT_TASK_H_
