/**
 ****************************************************************************************
 *
 * @file smpc.h
 *
 * @brief Header file - SMPC.
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
 * @file smpc.h
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
#ifndef SMPC_H_
#define SMPC_H_
/**
 ****************************************************************************************
 * @addtogroup SMP Security Manager Protocol
 * @ingroup HOST
 * @brief Security Manager Protocol.
 *
 * The SMP is responsible for the over-all security policies of BLE.
 * It defines methods for pairing and key distribution, handles encryption,
 * data signing and privacy features such as random addressing generation and resolution.
 *
 * Pairing is performed to exchange pairing features and generate a short term
 * key for link encryption.
 * A transport specific key distribution is performed to
 * share the keys that can be used to encrypt the link in the future
 * reconnection process, signed data verification and random address
 * resolution.
 *
 * There exist 3 phases in the complete security procedure:
 * 1. Feature exchange (IO capabilities, OOB flags, Authentication Requirements, Key distributions)
 * 2. Short Term Key generation
 *    Generation method depends on exchanged features:
 *     - Just Works - use Temporary key = 0
 *     - PassKey Entry - use Temporary Key = 6-digit provided by user
 *     - Out of Band (OOB) - use Temporary Key = 16-octet key, available form OOB source
 * 3. Transport Specific Key Distribution (TKDP)(LTK+EDIV+RAND_NB, IRK+ADDR, CSRK)
 *---------------------------------------------------------------------
 * @addtogroup SMPC Security Manager Protocol Controller
 * @ingroup SMP
 * @brief Security Manager Protocol Controller.
 *
 * This block handles control of SM procedures for several possible existing connections,
 * for which the security procedure may be conducted simultaneously.
 *
 * It allows flow control for LLM access to encryption and random number generation, used
 * at different moments in the procedure.
 *
 * It handles PDU creation and sending through L2CAP, also their reception from L2CAP
 * and interpretation.
 *
 * Other small utilities such as maximum key size determination and TKDP organization are
 * implemented in SMPC.
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>
#include <stdint.h>

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

/// IO Capability Values
enum
{
    ///Display Only
    SMP_IO_CAP_DISPLAY_ONLY = 0x00,
    ///Display Yes No
    SMP_IO_CAP_DISPLAY_YES_NO,
    ///Keyoard Only
    SMP_IO_CAP_KB_ONLY,
    ///No Input No Output
    SMP_IO_CAP_NO_INPUT_NO_OUTPUT,
    ///Keyboard Display
    SMP_IO_CAP_KB_DISPLAY,
    SMP_IO_CAP_LAST,
};

/// OOB Data Present Flag Values
enum
{
    ///OOB Data not present
    SMP_OOB_AUTH_DATA_NOT_PRESENT = 0x00,
    ///OOB data from remote device present
    SMP_OOB_AUTH_DATA_FROM_REMOTE_DEV_PRESENT,
    SMP_OOB_AUTH_DATA_LAST,
};

/// Authentication Requirements
enum
{
    ///No MITM No Bonding
    SMP_AUTH_REQ_NO_MITM_NO_BOND = 0x00,
    ///No MITM Bonding
    SMP_AUTH_REQ_NO_MITM_BOND,
    ///MITM No Bonding
    SMP_AUTH_REQ_MITM_NO_BOND = 0x04,
    ///MITM and Bonding
    SMP_AUTH_REQ_MITM_BOND,
    SMP_AUTH_REQ_LAST
};

/// Pairing Failed Reasons
enum
{
    ///Reserved error code
    SMP_PAIR_ERR_RESERVED = 0x00,
    ///Passkey entry faild
    SMP_PAIR_ERR_PASSKEY_ENTRY_FAILED,
    ///OOB Not available
    SMP_PAIR_ERR_OOB_NOT_AVAILABLE,
    ///Authentication Requirements
    SMP_PAIR_ERR_AUTH_REQUIREMENTS,
    ///Confirm Value Failed
    SMP_PAIR_ERR_CFM_VAL_FAILED,
    ///Pairing Not Supported
    SMP_PAIR_ERR_PAIRING_NOT_SUPPORTED,
    ///Encryption Key Size
    SMP_PAIR_ERR_ENCRYPTION_KEY_SIZE,
    ///command not Supported
    SMP_PAIR_ERR_CMD_NOT_SUPPORTED,
    ///Unspecified Reason
    SMP_PAIR_ERR_UNSPECIFIED_REASON,
    ///Repeated attempts
    SMP_PAIR_ERR_REPEATED_ATTEMPTS,
    ///Invalid Parameters
    SMP_PAIR_ERR_INVALID_PARAMS,
    ///Code unknown, reserved
    SMP_PAIR_ERR_CODE_RSVD,
    SMP_PAIR_ERR_LAST,
};

///Key Distribution Flags
enum
{
    ///No Keys to distribute
    SMP_KDIST_NONE = 0x00,
    ///Encryption key in distribution
    SMP_KDIST_ENCKEY,
    ///IRK (ID key)in distribution
    SMP_KDIST_IDKEY,
    ///CSRK(Signature key) in distribution
    SMP_KDIST_SIGNKEY= 0x04,
    SMP_KDIST_LAST = 0x08,
};

///Security Properties for distributed keys(all have the issued STK's properties)
enum
{
    ///No security properties
    SMP_KSEC_NONE = 0x00,
    ///Unauthenticated no MITM
    SMP_KSEC_UNAUTH_NO_MITM,
    ///Authenticated with MITM
    SMP_KSEC_AUTH_MITM,
};

/*
 * STRUCTURES
 ****************************************************************************************
 */
///Generic SMP key structure
struct smp_key
{
    ///Key value
    uint8_t key[KEY_LEN];
};

///Maximum Encryption Key size
#define SMPC_MAX_ENC_SIZE_LEN                 0x10

/// @} SMPC

#endif // SMPC_H_
