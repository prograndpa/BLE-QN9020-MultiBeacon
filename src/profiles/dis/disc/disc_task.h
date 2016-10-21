/**
 ****************************************************************************************
 *
 * @file disc_task.h
 *
 * @brief Header file - DISCTASK.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 5767 $
 *
 ****************************************************************************************
 */

#ifndef DISC_TASK_H_
#define DISC_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup DISCTASK Device Information Service Client Task
 * @ingroup DISC
 * @brief Device Information Service Client Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_DIS_CLIENT)
#include "ke_task.h"
#include "gatt_task.h"
#include "disc.h"
#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of Device Information Service Client task instances
#define DISC_IDX_MAX    (BLE_CONNECTION_MAX)

/// Possible states of the DISC task
enum
{
    ///Idle state
    DISC_IDLE,
    ///Discovering State
    DISC_DISCOVERING,
    ///Connected state
    DISC_CONNECTED,

    ///Number of defined states.
    DISC_STATE_MAX
};

enum
{
    /// Start the find me locator profile - at connection
    DISC_ENABLE_REQ = KE_FIRST_MSG(TASK_DISC),
    /// Confirm that cfg connection has finished with discovery results, or that normal cnx started
    DISC_ENABLE_CFM,
    /// Inform APP that the profile client role has been disabled after a disconnection
    DISC_DISABLE_IND,

    /// Generic message to read a DIS characteristic value
    DISC_RD_CHAR_REQ,
    /// Generic message for read responses for APP
    DISC_RD_CHAR_RSP,
};

/// Parameters of the @ref DISC_ENABLE_REQ message
struct disc_enable_req
{
    /// Connection handle
    uint16_t conhdl;
    ///Connection type
    uint8_t con_type;

    /// Existing handle values dis
    struct disc_dis_content dis;
};


///Parameters of the @ref DISC_RD_CHAR_REQ message
struct disc_rd_char_req
{
    ///Connection handle
    uint16_t conhdl;
    ///Characteristic value code
    uint8_t char_code;
};
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_DISC_TASK
 * @{
 ****************************************************************************************
 */

/// Parameters of the @ref DISC_ENABLE_CFM message
struct disc_enable_cfm
{
    /// Connection handle
    uint16_t conhdl;
    ///status
    uint8_t status;

    /// DIS handle values and characteristic properties
    struct disc_dis_content dis;
};



///Parameters of the @ref DISC_RD_CHAR_RSP message
struct disc_rd_char_rsp
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t status;
    /// Char. Code
    uint8_t char_code;
    /// Value Length
    uint16_t val_length;
    /// Value
    uint8_t val[1];
};
/// @} APP_DISC_TASK

/// @cond

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler disc_state_handler[DISC_STATE_MAX];
extern const struct ke_state_handler disc_default_handler;
extern ke_state_t disc_state[DISC_IDX_MAX];

extern void task_disc_desc_register(void);

#endif //BLE_DIS_CLIENT

/// @} DISCTASK
/// @endcond
#endif // DISC_TASK_H_
