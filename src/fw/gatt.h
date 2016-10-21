/**
 ****************************************************************************************
 *
 * @file gatt.h
 *
 * @brief Header file - GATT.
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
 * @file gatt.h
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
#ifndef GATT_H_
#define GATT_H_

/**
 ****************************************************************************************
 * @addtogroup GATT Generic Attribute Profile
 * @ingroup HOST
 * @brief Generic Attribute Profile.
 *
 * The GATT module is responsible for providing an API for all attribute related operations.
 * It is responsible for all the service framework activities using the Attribute protocol
 * for discovering services and for reading and writing characteristic values on a peer device.
 * To achieve this, the GATT interfaces with @ref ATTM "ATTM", @ref ATTC "ATTC" and the
 * @ref ATTS "ATTS".
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
/* kernel task */
#include "ke_task.h"
#include "atts.h"
#include "atts_db.h"
#include "attm.h"
#include "attc_task.h"
/*
 * DEFINES
 ****************************************************************************************
 */
/// size for 16-bit UUID
#define GATT_UUID_2_OCTET_LEN                       0x02

/// size for 128-bit UUID
#define GATT_UUID_16_OCTET_LEN                      0x10

/// Local database maximum size
#define GATT_DB_MAX_SIZE                            0x50

/// Maximum possible attribute handle
#define GATT_MAX_ATTR_HDL                           0xFFFF

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */
/// request type - application interface
enum
{
    /*              Attribute Client Flags              */
    /*      Operation flags for discovery operation     */
    /* ************************************************ */
    /// Discover all services
    GATT_DISC_ALL_SVC           = 0x00,
    /// Discover services by UUID
    GATT_DISC_BY_UUID_SVC,
    /// Discover included services
    GATT_DISC_INCLUDED_SVC,
    /// Discover all characteristics
    GATT_DISC_ALL_CHAR,
    /// Discover characteristic by UUID
    GATT_DISC_BY_UUID_CHAR,
    /// Discover characteristic descriptor
    GATT_DISC_DESC_CHAR,
    /* Operation flags for reading attributes           */
    /* ************************************************ */
    /// Read characteristic
    GATT_READ_CHAR,
    /// Read characteristic by UUID
    GATT_READ_BY_UUID_CHAR,
    /// Read long characteristic
    GATT_READ_LONG_CHAR,
    /// Read multiple long characteristic
    GATT_READ_MULT_LONG_CHAR,
    /// Read descriptor
    GATT_READ_DESC,
    /// Read long descriptor
    GATT_READ_LONG_DESC,
    /* Operation flags for writing/modifying attributes */
    /* ************************************************ */
    /// Write no response
    GATT_WRITE_NO_RESPONSE,
    /// Write signed
    GATT_WRITE_SIGNED,
    /// Write characteristic
    GATT_WRITE_CHAR,
    /// Write long characteristic
    GATT_WRITE_LONG_CHAR,
    /// Write reliable characteristic
    GATT_WRITE_RELIABLE_CHAR,
    /// Write descriptor
    GATT_WRITE_DESC,
    /// Write long descriptor
    GATT_WRITE_LONG_DESC,
    /* Cancel write flag                                */
    /* ************************************************ */
    /// Write cancel characteristic
    GATT_WRITE_CANCEL_CHAR,
    /*              Attribute Server Flags              */
    /*      Operation flags for attribute notification  */
    /* ************************************************ */
    /// Handle notification: signed or unsigned
    GATT_HDL_NTF,
    /*      Operation flag for attribute indication     */
    /* ************************************************ */
    /// Handle indication
    GATT_HDL_IND,
    /// No operation
    GATT_NO_OP_REQ,
    /// Last GATT flag
    GATT_LAST
};

/// GATT Specific Error
enum
{
    /// No Error
    GATT_NO_ERROR             = 0x00,
    /// Invalid parameters set
    GATT_INVALID_PARAM_ERR   = 0x50,
    /// Indication not permitted
    GATT_INDICATE_NOT_ALLOWED,
    /// Notification not permitted
    GATT_NOTIFY_NOT_ALLOWED,
    /// Error in service search
    GATT_INVALID_TYPE_IN_SVC_SEARCH,
    /// No attribute client defined
    GATT_ATTRIBUTE_CLIENT_MISSING,
    /// No attribute server defined
    GATT_ATTRIBUTE_SERVER_MISSING,
    /// Invalid reliable write value
    GATT_RELIABLE_WRITE_ERR,
    /// Signed write error
    GATT_SIGNED_WRITE_ERR,
    /// Notify get data
    GATT_NOTIFY_GET_DATA,
    /// Notify
    GATT_NOTIFY_INSUFFICIENT_SPACE,
};

/*
 * STRUCTURES
 ****************************************************************************************
 */
/// reliable structure
struct gatt_reliable_write
{
    /// characteristic handle
    uint16_t elmt_hdl;
    /// value size
    uint16_t size;
    /// value holder
    uint8_t value[ATTM_RELIABLE_WRITE];
};

/// @} GATT

#endif // GATT_H_
