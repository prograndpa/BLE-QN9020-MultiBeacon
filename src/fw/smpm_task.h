/**
 ****************************************************************************************
 *
 * @file smpm_task.h
 *
 * @brief Header file - SMPM TASK
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
 * @file smpm_task.h
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
#ifndef SMPM_TASK_H_
#define SMPM_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup SMPM_TASK Security Manager Protocol Manager Task
 * @ingroup SMPM
 * @brief Handles ALL messages to/from SMPM block.
 *
 * The SMPM task is responsible for all security related functions not related to a
 * specific connection with a peer.
 *
 * @{
 ****************************************************************************************
 */

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */
enum
{
    ///API -> SMPM Generic Set Key Request
    SMPM_SET_KEY_REQ = KE_FIRST_MSG(TASK_SMPM) + 2,
    ///SMPM-> API Generic set Key status message
    SMPM_SET_KEY_CFM = KE_FIRST_MSG(TASK_SMPM) + 3,

};

/*
 * STRUCTURES
 ****************************************************************************************
 */

///Application request for Setting key structure
struct smpm_set_key_req
{
    ///Key code (irk or csrk - use those from Key distribution)
    uint8_t key_code;
    ///Key value, MSB-> LSB format
    struct smp_key new_key;
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_SMP_TASK
 * @{
 ****************************************************************************************
 */

///Application response for Setting key structure
struct smpm_set_key_cfm
{
    ///Key set status
    uint8_t status;
    ///Key code (irk or csrk - use those from Key distribution)
    uint8_t key_code;
};

/// @} APP_SMP_TASK



/// @cond

/// @} SMPM_TASK
/// @endcond
#endif // SMPM_TASK_H_
