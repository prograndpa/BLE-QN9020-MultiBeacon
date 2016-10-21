/**
 ****************************************************************************************
 *
 * @file app_smp_task.c
 *
 * @brief Application SMP task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_SMP_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Set key confirmation from the SMPM. *//**
 *
 * @param[in] msgid     SMPM_SET_KEY_CFM
 * @param[in] param     Pointer to struct smpm_set_key_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPM
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the SMPM to respond to the application to its SMPM_SET_KEY_REQ,
 *  informing it that saving the key values was done and the other actions related to setting
 *  a new key were initiated.
 * @note  
 *
 *  key_code:
 *  - SMP_KDIST_IDKEY       ///IRK (ID key)in distribution
 *  - SMP_KDIST_SIGNKEY     ///CSRK(Signature key) in distribution
 *
 ****************************************************************************************
 */
int app_smpm_set_key_cfm_handler(ke_msg_id_t const msgid, struct smpm_set_key_cfm const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (APP_INIT == ke_state_get(TASK_APP))
    {
        switch (param->key_code)
        {
        case SMP_KDIST_IDKEY:
            {
#if (QN_CSRK_SUPPORT)
                struct smp_key new_key;
                memcpy(new_key.key, app_env.csrk, KEY_LEN);
                app_smpm_set_key_req(SMP_KDIST_SIGNKEY, &new_key);
#else
                app_gap_set_bondable_mode_req();
#endif
            }
            break;
        case SMP_KDIST_SIGNKEY:
            app_gap_set_bondable_mode_req();
        default:
            break;
        }
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Key indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_KEY_IND
 * @param[in] param     Pointer to struct smpc_key_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This message is sent by SMPC during TKDP to application with the value of received  
 *  bonding information from peer device: either LTK+EDIV+random number, or IRK, or CSRK.  
 *  These values should be retrievable by the application at a later time if the implementation 
 *  chosen allows it. The key_code can have one of the defined value for the presence of a 
 *  key in a key distribution. The ediv and nb will be filled with 0's if it is an IRK or 
 *  CSRK that is being sent to application.
 * @note  
 *
 *  Random number structure refer to struct rand_nb
 *  SMP key structure refer to struct smp_key
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_smpc_key_ind_handler(ke_msg_id_t const msgid, struct smpc_key_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Key %d indication idx is %d.\r\n", param->key_code, param->idx); 

    switch (param->key_code)
    {
    case SMP_KDIST_ENCKEY:
        app_env.dev_rec[param->idx].bonded_info.peer_distribute_keys |= SMP_KDIST_ENCKEY;
        app_env.dev_rec[param->idx].bonded_info.pair_info.ediv = param->ediv;
        memcpy(app_env.dev_rec[param->idx].bonded_info.pair_info.rand_nb.nb, 
            param->nb.nb, RAND_NB_LEN);
        memcpy(app_env.dev_rec[param->idx].bonded_info.pair_info.ltk.key, 
            param->key.key, KEY_LEN);
        break;
    case SMP_KDIST_IDKEY:
        app_env.dev_rec[param->idx].bonded_info.peer_distribute_keys |= SMP_KDIST_IDKEY;
        memcpy(app_env.dev_rec[param->idx].bonded_info.pair_info.irk.key, 
            param->key.key, KEY_LEN);
        break;
#if (QN_CSRK_SUPPORT)
    case SMP_KDIST_SIGNKEY:
        app_env.dev_rec[param->idx].bonded_info.peer_distribute_keys |= SMP_KDIST_SIGNKEY;
        memcpy(app_env.dev_rec[param->idx].bonded_info.pair_info.csrk.key, 
            param->key.key, KEY_LEN);
        break;
#endif
    default:
        break;
    }
    
    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief TK request indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_TK_REQ_IND
 * @param[in] param     Pointer to struct smpc_tk_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This message is sent by SMPC to application during pairing, when the TK is necessary 
 *  for calculations of the security values involved in the procedure. Two flags indicate 
 *  to application whether the necessary TK value is an OOB 16 octet value that the device 
 *  should have, or if it is a simple PIN key, whether it should be input/displayed by 
 *  the user/device. The values of the flags are determined after the pairing features 
 *  exchange stage in the procedure. The status in the response from application will 
 *  reflect whether the requested key was found, and will allow the procedure to continue
 *  or stop indicating the failure reason to the peer.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_smpc_tk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_tk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("TK request indication idx is %d.\r\n", param->idx);
    uint8_t input = false;
    
    if (param->disp_en == 0x01)
    {
        QPRINTF("PASSKEY: %s\r\n", app_env.tk);
    }
    else
    {
        if (app_env.tk_type == 1)
        {
            app_menu_cb(msgid, (void *)param);
            input = true;
        }
        else
        {
            // use KEY stored in NVDS
        }
    }

    if (input == false)
    {
        // Fix Pin Code used or response directly here
        struct smp_key tk = {0};
        uint32_t i;

        i = atoi((char *)app_env.tk);
        app_i_to_array(i, (char *)tk.key);
        app_smpc_tk_req_rsp(param->idx, CO_ERROR_NO_ERROR, &tk);
    }
    
    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief LTK request indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_LTK_REQ_IND
 * @param[in] param     Pointer to struct smpc_ltk_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This message may be sent by SMPC to application on various occasions:
 *
 *  When the SMPC in Slave role receives a LE_LTK_Request event from LLC during encryption
 *  procedure, in which case the needed LTK must be retrieved from application.
 *
 *  When a Master device receives a Security Request PDU from the slave, to check whether an
 *  LTK with the right security properties exists is stored in application in order to encrypt
 *  the link and not pair ¨C this is when the Authentication Requirements parameters received
 *  in the Security Request from the Slave will allow the Application to decide whether it has
 *  encryption information corresponding to the requested level of security requested by Slave.
 *
 *  During TKDP when LTK needs to be distributed, it must be retrieved from application.
 *  In case the key is not retrieved, the response message will have a status reflecting that
 *  and SMPC will decide of the next step in the procedure. If it is, the key will be used
 *  either for encryption or distribution.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_smpc_ltk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_ltk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("LTK request indication idx is %d, auth_req is %d.\r\n", param->idx, param->auth_req);
    
    uint8_t bonded_dev_idx;
    struct smp_key key;
    struct rand_nb rand_nb;

    // Find bonded information index from bonded database
    bonded_dev_idx = app_find_bonded_dev(&(app_env.dev_rec[param->idx].bonded_info.peer_addr));
    if (app_env.bond_flag & APP_OP_BOND)
    {
        // Ask for local LTK, response here
        memcpy(key.key, app_env.ltk, KEY_LEN);
        memcpy(rand_nb.nb, app_env.rand, RAND_NB_LEN);
        app_smpc_ltk_req_rsp(param->idx,
                             CO_ERROR_NO_ERROR,
                             (bonded_dev_idx != GAP_INVALID_CONIDX) ? app_env.bonded_info[bonded_dev_idx].sec_prop : SMP_KSEC_UNAUTH_NO_MITM,
                             app_env.ediv,
                             &rand_nb,
                             &key);
    }
    else
    {
        // Ask for remote LTK, response here
        if (bonded_dev_idx != GAP_INVALID_CONIDX)
        {
            if(GAP_PERIPHERAL_SLV != app_get_role())
            {
            app_smpc_ltk_req_rsp(param->idx,
                                 CO_ERROR_NO_ERROR,
                                 app_env.bonded_info[bonded_dev_idx].sec_prop,
                                 app_env.bonded_info[bonded_dev_idx].pair_info.ediv,
                                 &app_env.bonded_info[bonded_dev_idx].pair_info.rand_nb,
                                 &app_env.bonded_info[bonded_dev_idx].pair_info.ltk);
            }
            else
            {
            app_smpc_ltk_req_rsp(param->idx,
                                 CO_ERROR_NO_ERROR,
                                 app_env.bonded_info[bonded_dev_idx].sec_prop,
                                 app_env.ediv,
                                 (struct rand_nb *)app_env.rand,
                                 (struct smp_key *)app_env.ltk); 
            }
        }
        else
        {
            app_smpc_ltk_req_rsp(param->idx,
                                 CO_ERROR_PIN_MISSING,
                                 0,
                                 0,
                                 NULL,
                                 NULL);  
        }
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief IRK request indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_IRK_REQ_IND
 * @param[in] param     Pointer to struct smpc_irk_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This message is sent by SMPC during the procedure for solving a peer random address.
 *  The application are asked to deliver an IRK as long as they no longer hold any record
 *  of IRKs for the known devices that have bonded with the local device, or until the 
 *  address has been solved.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_smpc_irk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_irk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("IRK request indication idx is %d.\r\n", param->idx);

    uint8_t reject;
    uint8_t bonded_count = app_get_bond_nb();

    if (param->idx == 0xFF)
    {
        // We recognised this device, so update address for looking up correct LTK
        // It is no need to write back to NVDS.
        app_env.bonded_info[app_env.irk_pos - 1].peer_addr = app_env.dev_rec[param->idx].bonded_info.peer_addr;
        app_env.irk_pos = 0;
        return (KE_MSG_CONSUMED);
    }

    for (; app_env.irk_pos < bonded_count; app_env.irk_pos++)
    {    
        if (app_get_bond_status(app_env.irk_pos, SMP_KDIST_IDKEY))   //update get bond function
            break;
    }

    if (bonded_count == app_env.irk_pos)
    {
        reject = 1;
        app_env.irk_pos = 0;
        app_smpc_irk_req_rsp(param->idx, reject, NULL, NULL);
    }
    else
    {
        reject = 0;
        app_smpc_irk_req_rsp(param->idx,
                             reject,
                             &app_env.bonded_info[app_env.irk_pos].peer_addr,
                             &app_env.bonded_info[app_env.irk_pos].pair_info.irk);
        app_env.irk_pos++;
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief CSRK request indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_CSRK_REQ_IND
 * @param[in] param     Pointer to struct smpc_csrk_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This message is sent by SMPC when it needs the CSRK value associated to a device that 
 *  just sent a signed message, in order to verify the signature. SMPC will extract the
 *  sign counter from the received message and also send the request including the BD address
 *  of the device that is sending signed messages. The SMPC_CSRK_REQ_RSP is received with
 *  a status code reflecting whether a CSRK value was found stored for that device or not,
 *  but also whether the Signature counter value is valid (larger than the last stored one
 *  for an existing CSRK).
 *
 ****************************************************************************************
 */
#if (QN_CSRK_SUPPORT)
int app_smpc_csrk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_csrk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("CSRK request indication idx is %d.\r\n", param->idx);
    uint8_t bonded_dev_idx;

    // Find bonded information index from bonded database
    bonded_dev_idx = app_find_bonded_dev(&(app_env.dev_rec[param->idx].bonded_info.peer_addr));

    // response here
    if (bonded_dev_idx != GAP_INVALID_CONIDX)
    {
        app_smpc_csrk_req_rsp(param->idx,
                !app_get_bond_status(bonded_dev_idx, SMP_KDIST_SIGNKEY),
                app_env.bonded_info[bonded_dev_idx].sec_prop,
                &app_env.bonded_info[bonded_dev_idx].pair_info.csrk);
    }
    else
    {
        app_smpc_csrk_req_rsp(param->idx, 1, 0, NULL);
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Check BD address request indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_CHK_BD_ADDR_REQ_IND
 * @param[in] param     Pointer to struct smpc_chk_bd_addr_req
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This message is sent by SMPC to application when a SMPC_SOLVE_ADDR_REQ is received for
 *  a peer address. Application is supposed to check the BD address existence in stored 
 *  records and tell SMPC what properties it has using the SMPC_CHK_BD_ADDR_REQ_RSP. 
 *  Or resolve it if it is a random resolvable address by searching which stored IRK will
 *  match.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_smpc_chk_bd_addr_req_ind_handler(ke_msg_id_t const msgid, struct smpc_chk_bd_addr_req const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t bonded = 0;
    uint8_t lk_sec_status = 0;
    uint8_t idx = param->idx;

    if (idx != GAP_INVALID_CONIDX)
    {
        lk_sec_status = app_get_lk_sec_status(&param->addr);
        if (lk_sec_status != GAP_LK_SEC_NONE)
            lk_sec_status |= GAP_LK_BONDED;
    }
    else
    {
        idx = 0;
    }
    
    app_smpc_chk_bd_addr_req_rsp(idx,
        bonded,
        lk_sec_status,
        param->type,
        (struct bd_addr *)&param->addr);

    QPRINTF("Check BD address response:\ridx: %d, type: %d, link security status: 0x%X, bonded: %d.\r\n", 
        idx,
        param->type,
        lk_sec_status, 
        bonded);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Encryption procedure complete indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_SEC_STARTED_IND
 * @param[in] param     Pointer to struct smpc_sec_started_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 *
 * @description
 *
 * This API is used to inform the application that the status of a security procedure.
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_smpc_sec_started_ind_handler(ke_msg_id_t const msgid, struct smpc_sec_started_ind const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Start encryption complete, idx %d, status %d, key_size %d, sec_prop %d, bonded %d.\r\n", 
                                param->idx, param->status, param->key_size, param->sec_prop, param->bonded);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Timeout indication from the SMPC. *//**
 *
 * @param[in] msgid     SMPC_TIMEOUT_EVT
 * @param[in] param     Pointer to struct smpc_timeout_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SMPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This message is sent by SMPC when Timeout happens in SM procedures if more than 30s 
 *  elapse between sending two commands. All SM PDUs are commands, so every time one is 
 *  sent, a timer starts and will be reset when the next one is sent. It will be stopped
 *  of course when the procedure completes correctly.
 *  This timer is normally sufficient for the PDU exchange, the only vulnerable moment is
 *  PassKey entry if any is required and user is not fast enough. After a timeout, the SM
 *  L2CAP channel must not be reused until link re-establishment, so the SMPC task will 
 *  become FREE and ignore all other SM local or peer requests, it's up to the application
 *  to disconnect when it has finished other actions.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_smpc_timeout_evt_handler(ke_msg_id_t const msgid, struct smpc_timeout_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Time out event idx is %d.\r\n", param->idx);
    app_env.bond_flag &= ~APP_OP_BOND;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}
#endif

/// @} APP_SMP_TASK
