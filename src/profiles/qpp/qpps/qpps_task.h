/**
 ****************************************************************************************
 *
 * @file qpps_task.h
 *
 * @brief Header file - Quintic Private Profile Server Task.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPPS_TASK_H_
#define _QPPS_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup QPPSTASK Task
 * @ingroup QPPS
 * @brief Quintic private profile Task.
 *
 * The QPPSTASK is responsible for handling the messages coming in and out of the
 * @ref QPPS collector block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_QPP_SERVER
#include <stdint.h>
#include "ke_task.h"
#include "qpp_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of Quintic private profile task instances
#define QPPS_IDX_MAX     0x01

#undef TASK_QPPS
#define TASK_QPPS        TASK_PRF8
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the QPPS task
enum
{
    /// Disabled state
    QPPS_DISABLED,
    /// Idle state
    QPPS_IDLE,
    /// Connected state
    QPPS_CONNECTED,

    /// Number of defined states.
    QPPS_STATE_MAX,
};

/// Messages for Quintic private Profile Server
enum
{
    ///Start the Quintic private Profile Server - at connection
    QPPS_ENABLE_REQ = KE_FIRST_MSG(TASK_QPPS),
    ///Disable confirmation with configuration to save after profile disabled
    QPPS_DISABLE_IND,
    ///Error indication to Host
    QPPS_ERROR_IND,
    ///Send value from APP
    QPPS_DATA_SEND_REQ,
    ///Send data confirm to APP if correctly sent.
    QPPS_DATA_SEND_CFM,
    ///Inform APP of new configuration value
    QPPS_CFG_INDNTF_IND,
    ///Client value send to APP
    QPPS_DAVA_VAL_IND,

    ///Add QPPS into the database
    QPPS_CREATE_DB_REQ,
    ///Inform APP about DB creation status
    QPPS_CREATE_DB_CFM,
};

///Parameters of the @ref QPPS_CREATE_DB_REQ message
struct qpps_create_db_req
{
    ///Database configuration
    uint8_t features;
    uint8_t tx_char_num;
};

/// Parameters of the @ref QPPS_ENABLE_REQ message
struct qpps_enable_req
{
    ///Connection handle
    uint16_t conhdl;
    /// security level: b0= nothing, b1=unauthenticated, b2=authenticated, b3=authorized;
    /// b1 or b2 and b3 can go together
    uint8_t sec_lvl;
    ///Type of connection - will someday depend on button press length; can be CFG or DISCOVERY
    uint8_t con_type;

    /// Notification configuration
    uint32_t ntf_en;
};

/////Parameters of the @ref QPPS_DATA_SEND_REQ message
struct qpps_data_send_req
{
    ///Connection handle
    uint16_t conhdl;
    /// Char index
    uint8_t index;
    /// Length
    uint8_t length;
    /// Data
    uint8_t data[1];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_QPPS_TASK
 * @{
 ****************************************************************************************
 */

///Parameters of the @ref QPPS_CREATE_DB_CFM message
struct qpps_create_db_cfm
{
    ///Status
    uint8_t status;
};

///Parameters of the @ref QPPS_DISABLE_IND message
struct qpps_disable_ind
{
    uint16_t conhdl;
    /// Notification configuration
    uint16_t ntf_en;
};

///Parameters of the @ref QPPS_ERROR_IND message
struct qpps_error_ind
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
};

///Parameters of the @ref QPPS_CFG_INDNTF_IND message
struct qpps_cfg_indntf_ind
{
    ///Connection handle
    uint16_t conhdl;
    ///Char index
    uint8_t char_index;
    ///Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

///Parameters of the @ref QPPS_DATA_SEND_CFM message
struct qpps_data_send_cfm
{
    ///Connection handle
    uint16_t conhdl;
    ///Char index
    uint8_t char_index;
    ///Status
    uint8_t status;
};

///Parameters of the @ref QPPS_DAVA_VAL_IND message
struct qpps_data_val_ind
{
    ///Connection handle
    uint16_t conhdl;
    uint8_t length;
    uint8_t data[1];
};
/// @} APP_QPPS_TASK

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

/**
 ****************************************************************************************
 * @brief Initialization of the Quintic private profile module.
 * This function performs all the initializations of the QPPS module.
 ****************************************************************************************
 */
void qpps_init(void);

/**
 ****************************************************************************************
 * @brief Update UUID of the Quintic private profile module.
 * This function should be called before added QPP service into the database.
 ****************************************************************************************
 */
void qpps_set_service_uuid(uint8_t param[ATT_UUID_128_LEN]);

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler qpps_state_handler[QPPS_STATE_MAX];
extern const struct ke_state_handler qpps_default_handler;
extern ke_state_t qpps_state[QPPS_IDX_MAX];

extern void task_qpps_desc_register(void);

#endif /* BLE_QPP_SERVER */

/// @} QPPSTASK
/// @endcond
#endif /* _QPPS_TASK_H_ */
