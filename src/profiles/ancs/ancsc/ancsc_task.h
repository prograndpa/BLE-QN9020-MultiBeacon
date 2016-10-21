/**
 ****************************************************************************************
 *
 * @file ancsc_task.h
 *
 * @brief Header file - Apple Notification Center Service Notification Consumer (NC) Task.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _ANCSC_TASK_H_
#define _ANCSC_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup ANCSCTASK Apple Notification Center Service Client(NC) Task
 * @ingroup ANCSC
 * @brief Apple Notification Center Service Client
 *
 * The ANCSCTASK is responsible for handling the messages coming in and out of the
 * @ref ANCSC client block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_ANCS_NC)

#include "ke_task.h"
#include "gatt_task.h"
#include "co_error.h"
#include "ancsc.h"
#include "prf_types.h"
#include "prf_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Apple Notification Center Service Client Task
#undef CFG_TASK_ANCSC
#define CFG_TASK_ANCSC          TASK_PRF6

///Maximum number of ANCS Client task instances
#define ANCSC_IDX_MAX           (BLE_CONNECTION_MAX)

///The Apple Notification Center Service UUID
#define ANCS_SERVICE_UUID       "\xD0\x00\x2D\x12\x1E\x4B\x0F\xA4\x99\x4E\xCE\xB5\x31\xF4\x05\x79"
///Characteristics UUID
#define ANCS_NTF_SOURCE_UUID    "\xBD\x1D\xA2\x99\xE6\x25\x58\x8C\xD9\x42\x01\x63\x0D\x12\xBF\x9F"
#define ANCS_CONTROL_POINT_UUID "\xD9\xD9\xAA\xFD\xBD\x9B\x21\x98\xA8\x49\xE1\x45\xF3\xD8\xD1\x69"
#define ANCS_DATA_SOURCE_UUID   "\xFB\x7B\x7C\xCE\x6A\xB3\x44\xBE\xB5\x4B\xD6\x24\xE9\xC6\xEA\x22"

/// Error Codes
#define ANCS_UNKNOWN_COMMAND    (0xA0)
#define ANCS_INVALID_COMMAND    (0xA1)
#define ANCS_INVALID_PARAMETER  (0xA2)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the ANCSC task
enum
{
    /// IDLE state
    ANCSC_IDLE,
    /// Connected state
    ANCSC_CONNECTED,
    /// Procedure is pending
    ANCSC_BUSY,
    // Number of defined states.
    ANCSC_STATE_MAX
};

enum
{
    /// Start the ANCS Client - at connection
    ANCSC_ENABLE_CMD = KE_FIRST_MSG(TASK_ANCSC),

    /// Generic message for configuring the characteristics that can be handled
    ANCSC_CFG_INDNTF_CMD,
    /// Get Notification Attributes Command
    ANCSC_GET_NTF_ATTRIBUTE_CMD,
    /// Get APP Attributes Command
    ANCSC_GET_APP_ATTRIBUTE_CMD,

    /// Indicate the content of the peer device ANCS
    ANCSC_ANCS_CONTENT_IND,
    /// Generic message for Notification Source
    ANCSC_NTF_SOURCE_IND,
    /// Generic message for read responses for APP
    ANCSC_DATA_SOURCE_IND,
    /// Disable profile role - at disconnection
    ANCSC_DISABLE_IND,

    /// Complete Event Information
    ANCSC_CMP_EVT,
};

/// Operation Codes
enum anpc_op_codes
{
    /// Reserved operation code
    ANCSC_RESERVED_OP_CODE  = 0x00,

    /**
     * EXTERNAL OPERATION CODES
     */
    /// Discovery Procedure
    ANCSC_ENABLE_OP_CODE,
    /// Read attribute value Procedure
    ANCSC_CFG_OP_CODE,
    /// Get notification attribute value Procedure
    ANCSC_GET_NTF_ATTRIBUTE_OP_CODE,
    /// Write data attribute value Procedure
    ANCSC_GET_APP_ATTRIBUTE_OP_CODE,
};

/// EventID Values
typedef enum
{
    NOTIFICATION_ADDED = 0,
    NOTIFICATION_MODIFIED = 1,
    NOTIFICATION_REMOVED = 2,
} ancs_event_id;

/// Event Flags
typedef enum
{
    SILENT_FLAG = (1 << 0),
    IMPORTANT_FLAG = (1 << 1),
} ancs_event_flags;

/// CategoryID Values
typedef enum
{
    CATEGORYID_OTHER = 0,
    CATEGORYID_INCOMINGCALL = 1,
    CATEGORYID_MISSEDCALL = 2,
    CATEGORYID_VOICEMAIL = 3,
    CATEGORYID_SOCIAL = 4,
    CATEGORYID_SCHEDULE = 5,
    CATEGORYID_EMAIL = 6,
    CATEGORYID_NEWS = 7,
    CATEGORYID_HEALTH_FITNESS = 8,
    CATEGORYID_BUSINESS_FINANCE = 9,
    CATEGORYID_LOCATION = 10,
    CATEGORYID_ENTERTAINMENT = 11,
} ancs_category_id;

struct ancsc_ntf_source
{
    /// Event ID
    ancs_event_id event_id;
    /// Event flags
    ancs_event_flags event_flags;
    /// Category ID
    uint8_t category_id;
    /// Category Count
    uint8_t category_count;
    /// Notification UID
    uint32_t ntf_uid;
};

/*
 * API MESSAGE STRUCTURES
 ****************************************************************************************
 */

/// Parameters of the @ref ANCSC_ENABLE_CMD message
struct ancsc_enable_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection type
    uint8_t con_type;
    /// Service Change flag 
    uint8_t svc_change;
    /// Existing handle values ANCS
    struct ancsc_content ancs;
};

///Parameters of the @ref ANCSC_CFG_INDNTF_CMD message
struct ancsc_cfg_indntf_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Operation Code
    uint8_t disc_code;
    /// Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

///Parameters of the @ref ANCSC_GET_NTF_ATTRIBUTE_CMD message
struct ancsc_get_ntf_attribute_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// The size of attribute list
    uint8_t attribute_size;
    /// The 32-bit notification UID
    uint32_t notificationUID;
    /// A list of attributes that the NC wants to retrieve 
    uint8_t attribute_list[1];
};

///Parameters of the @ref ANCSC_GET_APP_ATTRIBUTE_CMD message
struct ancsc_get_app_attribute_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// The size of AppId and AttributeID
    uint16_t list_size;
    /// A list of attributes the NC wants to retrieve
    uint8_t id_list[1];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_ANCSC_TASK
 * @{
 ****************************************************************************************
 */

/// Parameters of the @ref ANCSC_ANCS_CONTENT_IND message
struct ancsc_ancs_content_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Characteristic supported
    uint16_t char_feature;
    /// Apple Notification Center Service Content
    struct ancsc_content ancs; 
};

/// Parameters of the @ref ANCSC_NTF_SOURCE_IND message
struct ancsc_ntf_source_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Notification Source Value
    struct ancsc_ntf_source ntf_source; 
};

/// Parameters of the @ref ANCSC_DATA_SOURCE_IND message
struct ancsc_data_source_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Data Size
    uint16_t data_size;
    /// Data Source
    uint8_t data_source[1]; 
};

/// Parameters of the @ref ANCSC_CMP_EVT message
struct ancsc_cmp_evt
{
    /// Connection handle
    uint16_t conhdl;
    /// Operation code
    uint8_t operation;
    /// Status
    uint8_t status; 
};

/// @} APP_ANCSC_TASK
/// @cond

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler ancsc_state_handler[ANCSC_STATE_MAX];
extern const struct ke_state_handler ancsc_default_handler;
extern ke_state_t ancsc_state[ANCSC_IDX_MAX];

extern void task_ancsc_desc_register(void);

#endif // BLE_ANCS_NC

/// @} ANCSCTASK
/// @endcond

#endif // _ANCSC_TASK_H_
