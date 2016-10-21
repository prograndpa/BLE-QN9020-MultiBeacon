/**
 ****************************************************************************************
 *
 * @file qppc.h
 *
 * @brief Header file - Quintic Private Profile Client Role.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPPC_H_
#define _QPPC_H_

/**
 ****************************************************************************************
 * @addtogroup QPPC Quintic Private Profile Client
 * @ingroup QPP
 * @brief Quintic Private Profile Client
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_QPP_CLIENT)
#include "qpp_common.h"
#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct qppc_env_tag **qppc_envs;

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send ATT DB discovery results to QPPC host.
 ****************************************************************************************
 */
void qppc_enable_cfm_send(struct qppc_env_tag *qppc_env, struct prf_con_info *con_info, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send error indication from profile to Host, with proprietary status codes.
 * @param status Status code of error.
 ****************************************************************************************
 */
void qppc_error_ind_send(struct qppc_env_tag *qppc_env, uint8_t status);

/**
 ****************************************************************************************
 * @brief Disable Quintic Private Profile Client Task
 ****************************************************************************************
 */
void qppc_disable(struct qppc_env_tag *qppc_env);

#endif /* (BLE_QPP_CLIENT) */

/// @} QPPC

#endif /* _QPPC_H_ */
