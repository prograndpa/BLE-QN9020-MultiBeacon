/**
 ****************************************************************************************
 *
 * @file llc_task.h
 *
 * @brief LLC task header file
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
 * @file llc_task.h
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
#ifndef LLC_TASK_H_
#define LLC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup LLCTASK LLCTASK
 * @ingroup LLC
 * @brief Link Layer Controller Task
 *
 * The LLC task is responsible for managing link layer actions related to a
 * specific connection with a peer (e.g. Encryption setup, Supervision timeout, etc.). It
 * implements the state machine related to these actions.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>

/*
 * STRUCTURES
 ****************************************************************************************
 */
///llc LE read remote used feature command parameters structure
struct llc_le_rd_rem_used_feats_cmd
{
    ///Connection handle
    uint16_t            conhdl;
};

///llc read remote information version command parameters structure
struct llc_rd_rem_info_ver_cmd
{
    ///Connection handle
    uint16_t    conhdl;
};

/// @} LLCTASK

#endif // LLC_TASK_H_
