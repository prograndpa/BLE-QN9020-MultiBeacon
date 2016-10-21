/**
 ****************************************************************************************
 *
 * @file ke_task.h
 *
 * @brief This file contains the definition related to kernel task management.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 24731 $
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 *
 * @file ke_task.h
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
#ifndef _KE_TASK_H_
#define _KE_TASK_H_

/**
 ****************************************************************************************
 * @defgroup TASK Task and Process
 * @ingroup KERNEL
 * @brief Task management module.
 *
 * This module implements the functions used for managing tasks.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>          // standard integer
#include <stdbool.h>         // standard boolean

#include "compiler.h"        // compiler defines, INLINE
#include "ke_msg.h"          // kernel message defines
 
/// Tasks types.
enum
{
    TASK_NONE = (uint8_t) -1,
    // Link Layer Tasks
    TASK_LLM = 0 ,
    TASK_LLC ,
    TASK_LLD ,
    TASK_DBG ,
    // Host tasks   
    TASK_L2CM ,
    TASK_L2CC ,
    TASK_SMPM ,
    TASK_SMPC ,
    TASK_ATTM ,
    TASK_ATTC ,
    TASK_ATTS ,  
    TASK_GATT,   
    TASK_GAP ,   
    // Profile tasks
    TASK_PRF1,   
    TASK_PRF2,   
    TASK_PRF3,  
    TASK_PRF4,  
    TASK_PRF5,   
    TASK_PRF6,     
    TASK_PRF7,
    TASK_PRF8,
    // app task.
    TASK_APP, 
    TASK_USER,
    // This is used to define the last task
    TASK_MAX,
};

/// Build the first message ID of a task.
#define KE_FIRST_MSG(task) ((ke_msg_id_t)((task) << 10))
#define MSG_T(msg)         ((ke_task_id_t)((msg) >> 10))
#define MSG_I(msg)         ((msg) & ((1<<10)-1))

/// Format of a task message handler function
typedef int (*ke_msg_func_t)(ke_msg_id_t const msgid, void const *param,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id);

/// Element of a message handler table.
struct ke_msg_handler
{
    /// Id of the handled message.
    ke_msg_id_t id;
    /// Pointer to the handler function for the msgid above.
    ke_msg_func_t func;
};

/// Element of a state handler table.
struct ke_state_handler
{
    /// Pointer to the message handler table of this state.
    const struct ke_msg_handler *msg_table;
    /// Number of messages handled in this state.
    uint16_t msg_cnt;
};

/// Helps writing the initialization of the state handlers without errors.
#define KE_STATE_HANDLER(hdl) {hdl, sizeof(hdl)/sizeof(struct ke_msg_handler)}

/// Helps writing empty states.
#define KE_STATE_HANDLER_NONE {NULL, 0}

/// Task descriptor grouping all information required by the kernel for the scheduling.
struct ke_task_desc
{
    /// Pointer to the state handler table (one element for each state).
    const struct ke_state_handler* state_handler;
    /// Pointer to the default state handler (element parsed after the current state).
    const struct ke_state_handler* default_handler;
    /// Pointer to the state table (one element for each instance).
    ke_state_t* state;
    /// Maximum number of states in the task.
    uint16_t state_max;
    /// Maximum index of supported instances of the task.
    uint16_t idx_max;
};

/*
 * FUNCTION PROTOTYPES
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve the state of a task.
 *
 * @param[in]  id   Task id.
 *
 * @return          Current state of the task
 ****************************************************************************************
 */
typedef ke_state_t (*p_ke_state_get)(ke_task_id_t const id);
#define ke_state_get ((p_ke_state_get)(_ke_state_get))

/**
 ****************************************************************************************
 * @brief Set the state of the task identified by its Task Id.
 *
 * In this function we also handle the SAVE service: when a task state changes we
 * try to activate all the messages currently saved in the save queue for the given
 * task identifier.
 *
 * @param[in]  id          Identifier of the task instance whose state is going to be modified
 * @param[in]  state_id    New State
 *
 ****************************************************************************************
 */
typedef void (*p_ke_state_set)(ke_task_id_t const id, ke_state_t const state_id);
#define ke_state_set ((p_ke_state_set)(_ke_state_set))

/**
 ****************************************************************************************
 * @brief Generic message handler to consume message without handling it in the task.
 *
 * @param[in] msgid Id of the message received (probably unused)
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id TaskId of the receiving task.
 * @param[in] src_id TaskId of the sending task.
 *
 * @return KE_MSG_CONSUMED
 ****************************************************************************************
 */
typedef int (*p_ke_msg_discard)(ke_msg_id_t const msgid, void const *param,
                   ke_task_id_t const dest_id, ke_task_id_t const src_id);
#define ke_msg_discard ((p_ke_msg_discard)(_ke_msg_discard))

/**
 ****************************************************************************************
 * @brief Generic message handler to consume message without handling it in the task.
 *
 * @param[in] msgid Id of the message received (probably unused)
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id TaskId of the receiving task.
 * @param[in] src_id TaskId of the sending task.
 *
 * @return KE_MSG_CONSUMED
 ****************************************************************************************
 */
typedef int (*p_ke_msg_save)(ke_msg_id_t const msgid, void const *param,
                ke_task_id_t const dest_id, ke_task_id_t const src_id);
#define ke_msg_save ((p_ke_msg_save)(_ke_msg_save))


/**
 ****************************************************************************************
 * @brief Register task description into kernel.
 *
 * @param[in] task_id : the task's id
 * @param[in] task_desc task description.
 *
 * @return None
 ****************************************************************************************
 */
typedef void (*p_task_desc_register)(uint8_t task_id, struct ke_task_desc task_desc);
#define task_desc_register ((p_task_desc_register)(_task_desc_register))

/// @} TASK

#endif // _KE_TASK_H_

