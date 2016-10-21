/**
 ****************************************************************************************
 *
 * @file app_smp.c
 *
 * @brief Application SMP API
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
 * @addtogroup APP_SMP_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief SMPM Set Key request. *//**
 * @param[in] key_code     GATT request type for distinguishing between IRK and CSRK.
 *  - SMP_KDIST_IDKEY      
 *  - SMP_KDIST_SIGNKEY    
 * @param[in] key          This structure contains a 16 octet array (U8[16]) in which 
 *                         the key value from the application is written MSB to LSB 
 *                         from index 0 to 15.
 * @response  SMPM_SET_KEY_CFM
 * @description
 *
 *  This API is used by the Application to set the device keys that are unique for 
 *  the device and not connection dependent. The key_code parameter allows to use one single
 *  API message for setting either the IRK or the CSRK value ¨C 0x02 is used for IRK and 0x04
 *  for CSRK because the definitions made in SMPC used in key distribution formatting are 
 *  reused in SMPM. The value of the keys is sent from Application to SMPM in MSB to LSB format
 *  because these keys are used in the LE_Encrypt command in this format directly. Upon receiving
 *  a CSRK set request, the signature counter, also kept in the SMPM environment, will be reset
 *  to 0, and also the SMPM_GET_SUBKEYS_REQ is sent to SMPC to calculate the K1 and K2 keys needed
 *  in the signature generation algorithm, to save time when a signature will be asked from SM block.
 *  The SMPM_SET_KEY_CFM is sent to the API with a simple status and the key code value for confirming
 *  that the right key was set.
 *
 ****************************************************************************************
 */
void app_smpm_set_key_req(uint8_t key_code, struct smp_key *key)
{
    struct smpm_set_key_req *msg = KE_MSG_ALLOC(SMPM_SET_KEY_REQ, TASK_SMPM, TASK_APP,
                                                smpm_set_key_req);

    ///Key code (irk or csrk - use those from Key distribution)
    msg->key_code = key_code;
    ///Key value, MSB-> LSB format
    msg->new_key = *key; 

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief SMPC TK request response. *//**
 * @param[in] idx           Connection index for which the TK is sent from application.
 * @param[in] status        OK if TK was found and input, not OK if key is not found by
 *                          application.
 * @param[in] tk            A 16 octet array (U8[16]) filled MSB to LSB from [0:15] because
 *                          it is used with the LE_Encrypt command.
 * @response  None
 * @description
 *
 *  This message is used by the application to respond to SMPC_TK_REQ_IND message with either
 *  status OK and the TK value needed, or status not OK and all 0's in the TK parameter space.
 *  After receiving the SMCP_TK_REQ_IND, the application may have had more exchanges with
 *  even higher layers or the User in order to obtain the key, but that is up to the implementation.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_smpc_tk_req_rsp(uint8_t idx, uint8_t status, struct smp_key *tk)
{
    struct smpc_tk_req_rsp *msg = KE_MSG_ALLOC(SMPC_TK_REQ_RSP, TASK_SMPC, TASK_APP,
                                                smpc_tk_req_rsp);

    ///Connection index
    msg->idx = idx;
    ///Status of response
    msg->status = status;
    ///TK key, all 0 if status is error
    if (tk != NULL)
        msg->tk = *tk; 
    
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief SMPC LTK request response. *//**
 * @param[in] idx           Connection index for which the LTK and its associated information
 *                          is being given.
 * @param[in] status        If OK, the information was retrieved by application, if not OK,
 *                          the security procedure will stop.
 * @param[in] sec_prop      Security properties of this LTK.
 * @param[in] ediv          Encryption diversifier associated with the LTK.
 * @param[in] rand_nb       Random number associated with the LTK, 8 octet array filled with
 *                          the random number LSB to MSB [0:7].
 * @param[in] ltk           16 octet array with the LTK value LSB to MSB [0:15].
 * @response  None
 * @description
 *
 *  This message is sent by the application in response to the SMPC_LTK_REQ_IND message. If
 *  the status is OK, then the rest of the parameters will be used for the procedure the SMPC 
 *  task corresponding to the index is currently handling. If the status is not OK (key was
 *  not found), SMPC decides the following steps depending on the current procedure.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_smpc_ltk_req_rsp(uint8_t idx, 
                          uint8_t status,
                          uint8_t sec_prop,
                          uint16_t ediv, 
                          struct rand_nb *rand_nb, 
                          struct smp_key *ltk)
{
    struct smpc_ltk_req_rsp *msg = KE_MSG_ALLOC(SMPC_LTK_REQ_RSP, TASK_SMPC, TASK_APP,
                                                smpc_ltk_req_rsp);

    ///Connection index
    msg->idx = idx;
    ///Status of response
    msg->status = status;
    ///Security properties kept in host with the key
    msg->sec_prop = sec_prop;
    ///EDIV corresponding to key, or 0 if error status
    msg->ediv = ediv;
    ///Random Number corresponding to key, or 0 if error status
    if (rand_nb != NULL)
    {
        msg->nb = *rand_nb;
        ///Found LTK key, or 0 if error status
        msg->ltk = *ltk;
    }

    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief SMPC IRK request response. *//**
 * @param[in] idx           Connection index for which the IRK is being given.
 * @param[in] status        If OK, the information was retrieved by application, if not OK, 
 *                          the security procedure will stop.
 * @param[in] addr          6 octet array with the BD address associated with this IRK, 
 *                          LSB to MSB [0:5].
 * @param[in] irk           16 octet array with the IRK value LSB to MSB [0:15].
 * @response  None
 * @description
 *
 *  This message is sent from application in response to SMPC_IRK_REQ_IND. If the status
 *  is OK, an IRK value was found and it is in the message parameters together with the BD
 *  address to which it was associated. SMPC will use it in trying to solve the random
 *  address it has under study, and if the IRK fails to match, a new one will be requested.
 *  If the status is not OK, and no (more) IRKs exists in application, the SMPC will decide what
 *  the next step in the address solving procedure is.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_smpc_irk_req_rsp(uint8_t idx, uint8_t status, struct bd_addr *addr, struct smp_key *irk)
{
    struct smpc_irk_req_rsp *msg = KE_MSG_ALLOC(SMPC_IRK_REQ_RSP, TASK_SMPC, TASK_APP,
                                                smpc_irk_req_rsp);

    ///Connection index
    msg->idx = idx;
    ///Status of response
    msg->status = status;
    ///Original BD address associated to this IRK
    if (addr != NULL)
    {
        msg->orig_addr = *addr;
        ///Found IRK key, or 0 if error status
        msg->irk = *irk; 
    }

    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief SMPC CSRK request response. *//**
 * @param[in] idx           Connection index for which the CSRK is being given.
 * @param[in] status        If OK, the information was retrieved by application, if not OK, 
 *                          the security procedure will stop.
 * @param[in] sec_status    Security Status with connection index.
 * @param[in] csrk          16 octet array with the CSRK value LSB to MSB [0:15].
 * @response  None
 * @description
 *
 *  This message is sent by application in response to SMPC_CSRK_REQ_IND and it contains 
 *  the status of application search for a CSRK stored associated with the BD address 
 *  indicated in the request, but also having had a last signature counter smaller than 
 *  the one in the request. An appropriate status is sent in the response together with
 *  the found value of the CSRK or all 0's if the status is not OK.
 *
 ****************************************************************************************
 */
#if (QN_CSRK_SUPPORT)
void app_smpc_csrk_req_rsp(uint8_t idx, uint8_t status, uint8_t sec_status, struct smp_key *csrk)
{
    struct smpc_csrk_req_rsp *msg = KE_MSG_ALLOC(SMPC_CSRK_REQ_RSP, TASK_SMPC, TASK_APP,
                                                smpc_csrk_req_rsp);

    ///Connection index
    msg->idx = idx;
    ///Status of response
    msg->status = status;
    ///Security Status
    msg->sec_status = sec_status;
    ///Found CSRK key, or 0 if error status
    if (csrk != NULL)
        msg->csrk = *csrk; 

    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief SMPC Check BD Addr request response. *//**
 * @param[in] idx           Connection index for which the address check is received
 *                          (authorization) may be for a free task.
 * @param[in] found_flag    Found or not.
 * @param[in] sec_status    Authentication, Authorization and Bonded status information
 *                          recovered from application stored information about this BD
 *                          address.
 * @param[in] type          Type of address that was checked.
 * @param[in] addr          6 octet array with the BD address that was checked.
 * @response  None
 * @description
 *
 *  This message is the response for SMPC_CHK_BD_ADDR_REQ_IND, informing SMPC that the bd
 *  address that was requested to be checked has been found or not, and if found, it gives
 *  the link the security property set in lk_sec_status parameter.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_smpc_chk_bd_addr_req_rsp(uint8_t idx, 
                                    uint8_t found_flag,
                                    uint8_t sec_status,
                                    uint8_t type,
                                    struct bd_addr *addr)
{
    struct smpc_chk_bd_addr_rsp *msg = KE_MSG_ALLOC(SMPC_CHK_BD_ADDR_REQ_RSP, TASK_SMPC, TASK_APP,
                                                smpc_chk_bd_addr_rsp);

    ///Connection index -may be a free task index
    msg->idx = idx;
    ///Status - found or not
    msg->found_flag = found_flag;
    ///Saved link security status from higher layer
    msg->lk_sec_status = sec_status;
    ///Type of address to check
    msg->type = type;
    ///Random address to resolve or Public address to check in APP
    msg->addr = *addr;

    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief SMPC encrypt link request. *//**
 * @param[in] idx           Connection index for which application want to start the encryption 
 *                          procedure with an existing LTK.
 * @param[in] auth_req      Security properties of the LTK to be used..
 * @param[in] ediv          Encryption diversifier associated with the LTK.
 * @param[in] rand_nb       Random number associated with the LTK, 8 octet array filled with
 *                          the random number LSB to MSB [0:7].
 * @param[in] ltk           16 octet array with the LTK value LSB to MSB [0:15].
 * @response  None
 * @description
 *
 *  This message can be sent by the Higher Layers to directly encrypt a link with a peer 
 *  using known bonding information from a previous connection when pairing+bonding occurred.
 *  The known information for direct encryption is the LTK and its associated EDIV and Random
 *  Number values. The status of the encryption procedure is returned to the application using 
 *  the SMPC_ENC_STARTED_IND.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_smpc_start_enc_req(uint8_t idx, 
                               uint8_t auth_req,
                               uint16_t ediv,
                               struct rand_nb *rand_nb,
                               struct smp_key *ltk)
{
    struct smpc_start_enc_req *msg = KE_MSG_ALLOC(SMPC_START_ENC_REQ, TASK_SMPC, TASK_APP,
                                                  smpc_start_enc_req);

    ///Connection index
    msg->idx = idx;
    ///Authentication Requirements - needed if slave
    msg->auth_req = auth_req;
    ///EDIV
    msg->ediv = ediv;
    ///Random number
    msg->randnb = *rand_nb;
    ///LTK
    msg->ltk = *ltk;

    ke_msg_send(msg);
}
#endif

/// @} APP_SMP_API

