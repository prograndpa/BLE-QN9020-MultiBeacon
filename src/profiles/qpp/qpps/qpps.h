/**
 ****************************************************************************************
 *
 * @file qpps.h
 *
 * @brief Header file - Quintic Private Profile Server.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPPS_H_
#define _QPPS_H_

/**
 ****************************************************************************************
 * @addtogroup Quintic private profile Server
 * @ingroup QPP
 * @brief Quintic private profile Server
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_QPP_SERVER)
#include "qpp_common.h"
#include "prf_types.h"
#include "attm.h"
#include "atts.h"
#include "atts_db.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define QPPS_MANDATORY_MASK             (0x000f)
#define QPPS_RX_CHAR_UUID              "\x00\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"
#define QPPS_FIRST_TX_CHAR_UUID        "\x01\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"

/*
 * MACROS
 ****************************************************************************************
 */

#define QPPS_IS_SUPPORTED(idx, mask) \
    ((((qpps_env.features >> idx) & mask) == mask))

///Attributes State Machine
enum
{
    QPPS_IDX_SVC,

    QPPS_IDX_RX_DATA_CHAR,
    QPPS_IDX_RX_DATA_VAL,
    QPPS_IDX_RX_DATA_USER_DESP,

    QPPS_IDX_VAL_CHAR,
    QPPS_IDX_VAL,
    QPPS_IDX_VAL_NTF_CFG,

    QPPS_IDX_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Server environment variable
struct qpps_env_tag
{
    ///Application Task Id
    ke_task_id_t appid;
    ///Connection handle
    uint16_t conhdl;

    ///Service Start Handle
    uint16_t shdl;
    ///Database configuration
    uint32_t features;
    ///Notify char number
    uint8_t ntf_char_num;
};


/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct atts_desc_ext qpps_att_db[QPPS_IDX_NB];

///  Service - only one instance for now
extern uint8_t qpps_svc[ATT_UUID_128_LEN];

extern const struct atts_char128_desc qpps_value_char;
extern const struct atts_char128_desc qpps_char_rx_data;

extern struct qpps_env_tag qpps_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Indicate error to higher layers.
 * @param status Error status to send to requester task
 ****************************************************************************************
 */
void qpps_error_ind_send(uint8_t status);

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 ****************************************************************************************
 */
void qpps_disable(void);

#endif /* #if (BLE_QPP_SERVER) */

/// @} QPPS

#endif /* _QPPS_H_ */
