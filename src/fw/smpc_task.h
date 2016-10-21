/**
 ****************************************************************************************
 *
 * @file smpc_task.h
 *
 * @brief Header file SMPC task.
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
 * @file smpc_task.h
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
#ifndef SMPC_TASK_H_
#define SMPC_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup SMPC_TASK Task
 * @ingroup SMPC
 * @brief Handles ALL messages to/from SMPC block.
 *
 * The SMPC task is responsible for all security protocol and secure connections handling.
 *
 * @{
 ****************************************************************************************
 */

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */
/// SMPC Task Messages
enum
{
    ///Host -> SMPC request to encrypt link
    SMPC_START_ENC_REQ = KE_FIRST_MSG(TASK_SMPC) + 3,
	///SMPC -> Host security procedure over
	SMPC_SEC_STARTED_IND = KE_FIRST_MSG(TASK_SMPC) + 4,
    
    ///SMPC-> Host requests & responses for keys
    SMPC_TK_REQ_IND = KE_FIRST_MSG(TASK_SMPC) + 5,
    SMPC_TK_REQ_RSP = KE_FIRST_MSG(TASK_SMPC) + 6,
    SMPC_LTK_REQ_IND = KE_FIRST_MSG(TASK_SMPC) + 7,
    SMPC_LTK_REQ_RSP = KE_FIRST_MSG(TASK_SMPC) + 8,
    SMPC_IRK_REQ_IND = KE_FIRST_MSG(TASK_SMPC) + 9,
    SMPC_IRK_REQ_RSP = KE_FIRST_MSG(TASK_SMPC) + 10,
    SMPC_CSRK_REQ_IND = KE_FIRST_MSG(TASK_SMPC) + 11,
    SMPC_CSRK_REQ_RSP = KE_FIRST_MSG(TASK_SMPC) + 12,

    ///SMPC->Host key that was received
    SMPC_KEY_IND = KE_FIRST_MSG(TASK_SMPC) + 13,

    ///Ask APP to check if peer with BD address is Unauth/Authen/Authen+Authoriz
    SMPC_CHK_BD_ADDR_REQ_IND = KE_FIRST_MSG(TASK_SMPC) + 26,
    SMPC_CHK_BD_ADDR_REQ_RSP = KE_FIRST_MSG(TASK_SMPC) + 27,

    /// protocol timeout event, sent to source of procedure start
    SMPC_TIMEOUT_EVT = KE_FIRST_MSG(TASK_SMPC) + 29,    
};

/*
 * STRUCTURES
 ****************************************************************************************
 */
 ///SMPC start encryption request
struct smpc_start_enc_req
{
    ///Connection index for which pairing info is requested
    uint8_t idx;
    ///Authentication Requirements - needed if slave
    uint8_t auth_req;
    ///EDIV
    uint16_t ediv;
    ///Random number
    struct rand_nb randnb;
    ///LTK
    struct smp_key ltk;
};

///SMPC TK request response: status, TK value and random value
struct smpc_tk_req_rsp
{
    ///Connection index
    uint8_t idx;
    ///Status of response
    uint8_t status;
    ///TK key, all 0 if status is error
    struct smp_key tk;
};

///Host response to SMPC LTK request
struct smpc_ltk_req_rsp
{
    ///Connection index
    uint8_t idx;
    ///Status of response
    uint8_t status;
    ///Security properties kept in host with the key
    uint8_t sec_prop;
    ///EDIV corresponding to key, or 0 if error status
    uint16_t ediv;
    ///Random Number corresponding to key, or 0 if error status
    struct rand_nb nb;
    ///Found LTK key, or 0 if error status
    struct smp_key ltk;
};

///Host response to SMPC IRK request
struct smpc_irk_req_rsp
{
    ///Connection index
    uint8_t idx;
    ///Status of response
    uint8_t status;
    ///Original BD address associated to this IRK
    struct bd_addr orig_addr;
    ///Found IRK key, or 0 if error status
    struct smp_key irk;
};

///Host response to SMPC CSRK request
struct smpc_csrk_req_rsp
{
    ///Connection index
    uint8_t idx;
    ///Status of response
    uint8_t status;
    /// Security Status
    uint8_t sec_status;
    ///Found CSRK key, or 0 if error status
    struct smp_key csrk;
};

///Check BD address Response
struct smpc_chk_bd_addr_rsp
{
    ///Connection index -may be a free task index
    uint8_t idx;
    ///Status - found or not
    uint8_t found_flag;
    ///Saved link security status from higher layer
    uint8_t lk_sec_status;
    ///Type of address to check
    uint8_t type;
    ///Random address to resolve or Public address to check in APP
    struct bd_addr addr;
};
/// @endcond


/**
 ****************************************************************************************
 * @addtogroup APP_SMP_TASK
 * @{
 ****************************************************************************************
 */


/// SMPC Security started indication to host, with status and keys
struct smpc_sec_started_ind
{
    /// Connection index
    uint8_t idx;
    /// Status (OK or failure status)
    uint8_t status;
    ///Key size for the LTK/STK agreed upon during pairing features exchange)
    uint8_t key_size;
    ///Security properties of the keys
    uint8_t sec_prop;
    ///Bonding status
    uint8_t bonded;
};

///SMPC TK request indication
struct smpc_tk_req_ind
{
    ///Connection index
    uint8_t idx;
    ///key type: OOB 16B or 6 digit
    uint8_t oob_en;
    ///action expected if 6 digit, depending on IOs
    uint8_t disp_en;
};

///SMPC request to Host for LTK (and associated EDIV and Rand values)
struct smpc_ltk_req_ind
{
    ///Connection index
    uint8_t idx;
    ///Authentication Requirements from request
    uint8_t auth_req;
};

///SMPC request to Host for IRK
struct smpc_irk_req_ind
{
    ///Connection index
    uint8_t idx;
};


///SMPC request to Host for CSRK
struct smpc_csrk_req_ind
{
    ///Connection index
    uint8_t idx;
    ///Bd address of device for which bonding info containing CSRK should exist
    struct bd_addr addr;
    ///Signing counter received - to check against last known in APP
    uint32_t signcnt;

};

///SMPC Key indication to Host (with a received key during TKDP)
struct smpc_key_ind
{
    ///Connection index
    uint8_t idx;
    ///Key code - use one of defined values for Key distribution parameters
    uint8_t key_code;
    ///EDIV (=0 if not sending an LTK)
    uint16_t ediv;
    ///Random number (=0 if not sending an LTK)
    struct rand_nb nb;
    ///Key being sent to Host (LTK/IRK/CSRK)
    struct smp_key key;
};

///Check BD address Request
struct smpc_chk_bd_addr_req
{
    ///Connection index -may be a free task index
    uint8_t idx;
    ///Type of address to check
    uint8_t type;
    ///Random address to resolve or Public address to check in APP
    struct bd_addr addr;
};


///Timeout Event
struct smpc_timeout_evt
{
    uint8_t idx;
};

/// @} APP_SMP_TASK

/// @cond

/// @} SMPC_TASK
/// @endcond
#endif // SMPC_TASK_H_
