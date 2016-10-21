/**
 ****************************************************************************************
 *
 * @file atts.h
 *
 * @brief Header file - ATTS.
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
 * @file atts.h
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
#ifndef ATTS_H_
#define ATTS_H_

/**
 ****************************************************************************************
 * @addtogroup ATTS Attribute Server
 * @ingroup ATT
 * @brief Attribute Protocol Server
 *
 * The ATTS module is responsible for handling messages intended for the attribute
 * profile server. It has defined interfaces with @ref ATTM "ATTM".
 *
 *
 * @{
 *
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "ke_task.h"
#include "attm.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// macro to fill an attribute characteristic
#define ATTS_CHAR(prop, handle, type) {(prop),                                           \
                                       {(uint8_t)((handle)), (uint8_t)((handle) >> 8)},  \
                                       {(uint8_t)((type)), (uint8_t)((type) >> 8)}       };

#define ATTS_CHAR128(prop, handle, type) {(prop),                                           \
                                         {(uint8_t)((handle)), (uint8_t)((handle) >> 8)},  \
                                         {(type)}};


/// Macro used to retrieve permission value from access and rights on attribute.
#define PERM(access, right) \
    (((PERM_RIGHT_ ## right) << (PERM_ACCESS_ ## access)) & (PERM_ACCESS_MASK_ ## access))

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */
/// Attribute access types
enum
{
    /// Read Access
    PERM_ACCESS_RD           = 0,
    /// Write Access
    PERM_ACCESS_WR           = 3,
    /// Indication Access
    PERM_ACCESS_IND          = 6,
    /// Notification Access
    PERM_ACCESS_NTF          = 9,
    /// Check Encryption key size
    PERM_ACCESS_EKS          = 12,
    /// Hide/Show attribute
    PERM_ACCESS_HIDE         = 13,
    /// Write Signed Enabled attribute
    PERM_ACCESS_WRITE_SIGNED = 14,
    /// Read indication Enabled attribute
    PERM_ACCESS_RD_IND = 15,
};

/// Attribute access types mask
enum
 {
    /// Read Access Mask
    PERM_ACCESS_MASK_RD           = 0x0007,
    /// Write Access Mask
    PERM_ACCESS_MASK_WR           = 0x0038,
    /// Indication Access Mask
    PERM_ACCESS_MASK_IND          = 0x01C0,
    /// Notification Access Mask
    PERM_ACCESS_MASK_NTF          = 0x0E00,
    /// Check Encryption key size Mask
    PERM_ACCESS_MASK_EKS          = 0x1000,
    /// Hide/Show attribute Mask
    PERM_ACCESS_MASK_HIDE         = 0x2000,
    /// Write Signed Enabled attribute Mask
    PERM_ACCESS_MASK_WRITE_SIGNED = 0x4000,
    /// Read indication Enabled attribute Mask
    PERM_ACCESS_MASK_RD_IND = 0x8000,
 };

/// Service access types
enum
{
    /// Service Access
    PERM_ACCESS_SVC      = 0,
    /// Check Encryption key size for service Access
    PERM_ACCESS_SVC_EKS  = 3,
    /// Hide/Show service Access
    PERM_ACCESS_SVC_HIDE = 4,
};

/// Service access types mask
enum
 {
    /// Service Access Mask
    PERM_ACCESS_MASK_SVC       = 0x07,
    /// Check Encryption key size for service Access Mask
    PERM_ACCESS_MASK_SVC_EKS   = 0x08,
    /// Hide/Show service Access Mask
    PERM_ACCESS_MASK_SVC_HIDE  = 0x10,
 };

/// Attribute & Service access rights
enum
{
    /// Disable access
    PERM_RIGHT_DISABLE  = 0,
    /// Enable access
    PERM_RIGHT_ENABLE   = 1,
    /// Access Requires Unauthenticated link
    PERM_RIGHT_UNAUTH   = 2,
    /// Access Requires Authenticated link
    PERM_RIGHT_AUTH     = 3,
    /// Access Requires authorization
    PERM_RIGHT_AUTHZ    = 4,
};

/// Attribute & Service access rights mask
enum
{
    /// Disable access mask
    PERM_RIGHT_MASK_DISABLE  = 0x3,
    /// Enable access mask
    PERM_RIGHT_MASK_ENABLE   = 0x3,
    /// Access Requires Unauthenticated lin kmask
    PERM_RIGHT_MASK_UNAUTH   = 0x3,
    /// Access Requires Authenticated link mask
    PERM_RIGHT_MASK_AUTH     = 0x3,
    /// Access Requires authorization mask
    PERM_RIGHT_MASK_AUTHZ    = 0x4,
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
/// Attribute length type
typedef uint16_t atts_size_t;

/// Attribute Type
typedef struct
{
    uint8_t uuid_len;    // Length of UUID
    uint8_t *uuid; // Pointer to UUID
} atts_type_t;

/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Attribute element in database
struct atts_elmt
{
	/// Element UUID
    uint16_t uuid;
    /// Attribute permission
    uint16_t perm;
    /// Maximum length of the element
    /// 2 MSB bits are used to know if UUID is a 128 bits UUID and if its a BT UUID
    /// max length = 2^14 bytes
    atts_size_t max_length;
    /// Current length of the element
    atts_size_t length;
    /// Element value array
    uint8_t value[2]; // array of 2 bytes to reuse padding
};

/// Attribute description (used to create database)
struct atts_desc
{
    /// Element UUID
    uint16_t uuid;
    /// Attribute permission
    uint16_t perm;
    /// Maximum length of the element
    atts_size_t max_length;
    /// Current length of the element
    atts_size_t length;
    /// Element value array
    uint8_t* value;
};

/// Attribute description (used to create 128-bit UUID in database)
struct atts_desc_ext
{
    /// Attribute type info
    atts_type_t type;
    /// Attribute permission
    uint16_t perm;
    /// Maximum length of the element
    atts_size_t max_length;
    /// Current length of the element
    atts_size_t length;
    /// Element value array
    uint8_t *value;
};

/// Characteristic Value Descriptor
struct atts_char_desc
{
    /// properties
    uint8_t prop;
    /// attribute handle
    uint8_t attr_hdl[ATT_UUID_16_LEN];
    /// attribute type
    uint8_t attr_type[ATT_UUID_16_LEN];
};

/// 128-bit Characteristic Value Descriptor
struct atts_char128_desc
{
    /// properties
    uint8_t prop;
    /// attribute handle
    uint8_t attr_hdl[ATT_UUID_16_LEN];
    /// attribute type
    uint8_t attr_type[ATT_UUID_128_LEN];
};

/// Service Value Descriptor - 16-bit
typedef uint16_t atts_svc_desc_t;

/// include service entry element
struct atts_incl_desc
{
    /// start handle value of included service
    uint16_t start_hdl;
    /// end handle value of included service
    uint16_t end_hdl;
    /// attribute value UUID
    uint16_t uuid;
};

/// @} ATTS

#endif // ATTS_H_
