/**
 ****************************************************************************************
 *
 * @file prf_utils.c
 *
 * @brief Implementation of Profile Utilities
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup PRF_UTILS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_ATTS || BLE_ATTC)

#include <stdint.h>
#include <stdbool.h>
#include "ke_task.h"
#include "co_error.h"
#include "atts_util.h"
#include "gatt_task.h"
#include "prf_utils.h"
#include "ke_mem.h"
#if (BLE_QPP_SERVER)
#include "qpps_task.h"
#endif

#if (BLE_QPP_CLIENT)
#include "qppc_task.h"
#endif

#if (BLE_ACCEL)
#include "accel.h"
#include "accel_task.h"
#endif // (BLE_ACCEL)

#if (BLE_HT_THERMOM)
#include "htpt.h"
#include "htpt_task.h"
#endif // (BLE_HT_THERMOM)

#if (BLE_HT_COLLECTOR)
#include "htpc.h"
#include "htpc_task.h"
#endif // (BLE_HT_COLLECTOR)

#if (BLE_DIS_SERVER)
#include "diss.h"
#include "diss_task.h"
#endif // (BLE_DIS_SERVER)

#if (BLE_DIS_CLIENT)
#include "disc.h"
#include "disc_task.h"
#endif // (BLE_DIS_CLIENT)

#if (BLE_BP_SENSOR)
#include "blps.h"
#include "blps_task.h"
#endif // (BLE_BP_SENSOR)

#if (BLE_BP_COLLECTOR)
#include "blpc.h"
#include "blpc_task.h"
#endif // (BLE_BP_COLLECTOR)

#if (BLE_TIP_SERVER)
#include "tips.h"
#include "tips_task.h"
#endif // (BLE_TIP_SERVER)

#if (BLE_TIP_CLIENT)
#include "tipc.h"
#include "tipc_task.h"
#endif // (BLE_TIP_CLIENT)

#if (BLE_HR_SENSOR)
#include "hrps.h"
#include "hrps_task.h"
#endif // (BLE_HR_SENSOR)

#if (BLE_HR_COLLECTOR)
#include "hrpc.h"
#include "hrpc_task.h"
#endif // (BLE_HR_COLLECTOR)

#if (BLE_FINDME_LOCATOR)
#include "findl.h"
#include "findl_task.h"
#endif // (BLE_FINDME_LOCATOR)

#if (BLE_FINDME_TARGET)
#include "findt.h"
#include "findt_task.h"
#endif // (BLE_FINDME_TARGET)

#if (BLE_PROX_MONITOR)
#include "proxm.h"
#include "proxm_task.h"
#endif // (BLE_PROX_MONITOR)

#if (BLE_PROX_REPORTER)
#include "proxr.h"
#include "proxr_task.h"
#endif // (BLE_PROX_REPORTER)

#if (BLE_SP_CLIENT)
#include "scppc.h"
#include "scppc_task.h"
#endif // (BLE_SP_CLENT)

#if (BLE_SP_SERVER)
#include "scpps.h"
#include "scpps_task.h"
#endif // (BLE_SP_SERVER)

#if (BLE_BATT_CLIENT)
#include "basc.h"
#include "basc_task.h"
#endif // (BLE_BATT_CLIENT)

#if (BLE_BATT_SERVER)
#include "bass.h"
#include "bass_task.h"
#endif // (BLE_BATT_SERVER)

#if (BLE_HID_DEVICE)
#include "hogpd.h"
#include "hogpd_task.h"
#endif // (BLE_HID_DEVICE)

#if (BLE_HID_BOOT_HOST)
#include "hogpbh.h"
#include "hogpbh_task.h"
#endif // (BLE_HID_BOOT_HOST)

#if (BLE_HID_REPORT_HOST)
#include "hogprh.h"
#include "hogprh_task.h"
#endif // (BLE_HID_REPORT_HOST)

#if (BLE_GL_COLLECTOR)
#include "glpc.h"
#include "glpc_task.h"
#endif // (BLE_GL_COLLECTOR)

#if (BLE_GL_SENSOR)
#include "glps.h"
#include "glps_task.h"
#endif // (BLE_GL_SENSOR)

#if (BLE_PAS_CLIENT)
#include "paspc.h"
#include "paspc_task.h"
#endif // (BLE_PAS_CLIENT)

#if (BLE_PAS_SERVER)
#include "pasps.h"
#include "pasps_task.h"
#endif // (BLE_PAS_SERVER)

#if (BLE_AN_CLIENT)
#include "anpc.h"
#include "anpc_task.h"
#endif // (BLE_AN_CLIENT)

#if (BLE_AN_SERVER)
#include "anps.h"
#include "anps_task.h"
#endif // (BLE_AN_SERVER)

#if (BLE_RSC_COLLECTOR)
#include "rscpc.h"
#include "rscpc_task.h"
#endif // (BLE_RSC_COLLECTOR)

#if (BLE_RSC_SENSOR)
#include "rscps.h"
#include "rscps_task.h"
#endif // (BLE_RSC_SENSOR)

#if (BLE_CSC_COLLECTOR)
#include "cscpc.h"
#include "cscpc_task.h"
#endif // (BLE_CSC_COLLECTOR)

#if (BLE_CSC_SENSOR)
#include "cscps.h"
#include "cscps_task.h"
#endif // (BLE_CSC_SENSOR)

#if (BLE_OTA_SERVER)
#include "otas_task.h"
#endif // (BLE_OTA_SERVER)

#if (BLE_ANCS_NC)
#include "ancsc_task.h"
#endif // (BLE_ANCS_NC)

#endif /* (BLE_ATTS || BLE_ATTC) */
/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_ATTC || BLE_TIP_SERVER || BLE_AN_SERVER || BLE_PAS_SERVER)


/**
 ****************************************************************************************
 * @brief This function is used to allocate a memory bloc containing a pool of pointers.
 * (Size = 4 * BLE_CONNECTION_MAX)
 * These pointers are initialize to NULL.
 *
 * @param[in/out] ***p_envs     Pointer to a pool of pointer. It shall be a null pointer.
 *                              Its value will contain the address of the pool.
 *
 * @return  PRF_ERR_OK              If the allocation has been successfully done.
 *          PRF_ERR_REQ_DISALLOWED  If the allocation has failed because of a lack of memory.
 *          PRF_ERR_INVALID_PARAM   If the provided pointer is non NULL
 ****************************************************************************************
 */
static uint8_t prf_client_pool_envs_alloc(prf_env_struct ***p_envs);

/**
 ****************************************************************************************
 * @brief This function is used to free the memory bloc containing a pool of pointers.
 *
 * @param[in/out] ***p_envs     Pointer to a pool of pointer. It shall be a non-null pointer.
 ****************************************************************************************
 */
static void prf_client_pool_envs_free(prf_env_struct ***p_envs);

/**
 ****************************************************************************************
 * @brief This function is used to allocate enough memory to contain the environment of a
 * profile client role.
 *
 * @param[in/out] ***p_envs   Pointer to a pool of pointer. If its value is NULL, a pool will
 * be created.
 * @param[in]     idx         Index of the connection for which the environment is created
 * @param[in]     env_size    Size of the environment structure.
 *
 * @return  PRF_ERR_OK              If the allocation has been successfully done.
 *          PRF_ERR_REQ_DISALLOWED  If the allocation has failed because of a lack of memory.
 *          PRF_ERR_INVALID_PARAM   Either the index is not valid or the profile client role
 *                                  is already enabled for this index.
 ****************************************************************************************
 */
static uint8_t prf_client_env_alloc(prf_env_struct ***p_envs, uint8_t idx, uint16_t env_size);

/**
 ****************************************************************************************
 * @brief This function is used to free the memory bloc containing a profile client role task
 *        environment.
 *
 * @param[in/out] ***p_envs     Pointer to a pool of pointer. It shall be a non-null pointer.
 * @param[in]     idx           Index of the connection for which the environment is freed.
 ****************************************************************************************
 */
static uint8_t prf_client_env_free(prf_env_struct ***p_envs, uint8_t idx);

#endif // #if (BLE_ATTC || BLE_TIP_SERVER || BLE_AN_SERVER || BLE_PAS_SERVER)

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_BATT_SERVER)
void prf_pack_char_pres_fmt(uint8_t *packed_val, const struct prf_char_pres_fmt* char_pres_fmt)
{
    *packed_val =               char_pres_fmt->format;
    *(packed_val + 1) =         char_pres_fmt->exponent;
    co_write16p(packed_val + 2, char_pres_fmt->unit);
    *(packed_val + 4) =         char_pres_fmt->namespace;
    co_write16p(packed_val + 5, char_pres_fmt->description);
}
#endif // (BLE_BATT_SERVER)

#if (BLE_BATT_CLIENT)
void prf_unpack_char_pres_fmt(const uint8_t *packed_val, struct prf_char_pres_fmt* char_pres_fmt)
{

    char_pres_fmt->format       = *packed_val;
    char_pres_fmt->exponent     = *(packed_val + 1);
    char_pres_fmt->unit         = co_read16p(packed_val + 2);
    char_pres_fmt->namespace    = *(packed_val + 4);
    char_pres_fmt->description  = co_read16p(packed_val + 5);
}
#endif // (BLE_BATT_CLIENT)

#if (BLE_ATTC)

void prf_read_char_send(struct prf_con_info* con_info,
                        uint16_t shdl, uint16_t ehdl, uint16_t valhdl)
{
    struct gatt_read_char_req * req  = KE_MSG_ALLOC(GATT_READ_CHAR_REQ,
                                                    TASK_GATT, con_info->prf_id,
                                                    gatt_read_char_req);
    //request type
    req->req_type                       = GATT_READ_CHAR;
    req->offset                         = 0x0000;
    req->conhdl                         = con_info->conhdl;
    req->start_hdl                      = shdl;
    req->end_hdl                        = ehdl;
    // number of UUID
    req->nb_uuid                        = 0x01;
    // handle of char value
    req->uuid[0].expect_resp_size       = ATT_UUID_16_LEN;
    req->uuid[0].value_size             = ATT_UUID_16_LEN;
    co_write16p(&req->uuid[0].value[0], valhdl);

    //send request to GATT
    ke_msg_send(req);
}

void prf_register_atthdl2gatt(struct prf_con_info* con_info, struct prf_svc *svc)
{
    if(svc->shdl != ATT_INVALID_HANDLE)
    {
        //register HTPC task in gatt for indication/notifications
        struct gatt_svc_reg2prf_req * reg = KE_MSG_ALLOC(GATT_SVC_REG2PRF_REQ,
                                                         TASK_GATT, con_info->prf_id,
                                                         gatt_svc_reg2prf_req);
        reg->conhdl = con_info->conhdl;
        reg->svc_shdl = svc->shdl;
        reg->svc_ehdl = svc->ehdl;

        ke_msg_send(reg);
    }
}

void prf_unregister_atthdl2gatt(struct prf_con_info* con_info, struct prf_svc *svc)
{
    if(svc->shdl != ATT_INVALID_HANDLE)
    {
        //register HTPC task in gatt for indication/notifications
        struct gatt_svc_unreg2prf_req * reg = KE_MSG_ALLOC(GATT_SVC_UNREG2PRF_REQ,
                                                           TASK_GATT, con_info->prf_id,
                                                           gatt_svc_unreg2prf_req);
        reg->conhdl = con_info->conhdl;
        reg->svc_shdl = svc->shdl;
        reg->svc_ehdl = svc->ehdl;

        ke_msg_send(reg);
    }
}

#if (BLE_QPP_CLIENT)
void prf_disc_qpp_svc_send(struct prf_con_info* con_info)
{
    struct gatt_disc_svc_req * svc_req = KE_MSG_ALLOC(GATT_DISC_SVC_REQ,
                                                      TASK_GATT, con_info->prf_id,
                                                      gatt_disc_svc_req);

    //gatt request type: by UUID
    svc_req->req_type         = GATT_DISC_BY_UUID_SVC;
    //connection handle
    svc_req->conhdl           = con_info->conhdl;
    //start handle;
    svc_req->start_hdl        = ATTC_1ST_REQ_START_HDL;
    //end handle
    svc_req->end_hdl          = ATTC_1ST_REQ_END_HDL;
    //UUID search
    svc_req->desired_svc.value_size = ATT_UUID_128_LEN;
    //set the 128 bits UUID to the value array
    memcpy(&(svc_req->desired_svc.value[0]), QPP_SVC_PRIVATE_UUID, ATT_UUID_128_LEN);

    //send the message to GATT, which will send back the response when it gets it
    ke_msg_send(svc_req);
}
#endif

void prf_disc_svc_send(struct prf_con_info* con_info, uint16_t uuid)
{
    //send GATT discover primary services by UUID request: find by type request
    struct gatt_disc_svc_req * svc_req = KE_MSG_ALLOC(GATT_DISC_SVC_REQ,
                                                      TASK_GATT, con_info->prf_id,
                                                      gatt_disc_svc_req);

    //gatt request type: by UUID
    svc_req->req_type         = GATT_DISC_BY_UUID_SVC;
    //connection handle
    svc_req->conhdl           = con_info->conhdl;
    //start handle;
    svc_req->start_hdl        = ATTC_1ST_REQ_START_HDL;
    //end handle
    svc_req->end_hdl          = ATTC_1ST_REQ_END_HDL;

    // UUID search
    svc_req->desired_svc.value_size = ATT_UUID_16_LEN;

    //set the first two bytes to the value array, LSB to MSB:Health Thermometer Service UUID first
    co_write16p(&(svc_req->desired_svc.value[0]), uuid);

    //send the message to GATT, which will send back the response when it gets it
    ke_msg_send(svc_req);
}

void prf_disc_incl_svc_send(struct prf_con_info* con_info, struct prf_svc *svc)
{
    // Send GATT discover included services
    struct gatt_disc_svc_req * svc_req = KE_MSG_ALLOC(GATT_DISC_SVC_REQ,
                                                      TASK_GATT, con_info->prf_id,
                                                      gatt_disc_svc_req);

    svc_req->req_type         = GATT_DISC_INCLUDED_SVC;
    // Connection handle
    svc_req->conhdl           = con_info->conhdl;
    // SVC Start handle;
    svc_req->start_hdl        = svc->shdl;
    // SVC End handle
    svc_req->end_hdl          = svc->ehdl;

    // Send the message to GATT, which will send back the response when it gets it
    ke_msg_send(svc_req);
}

void prf_disc_char_all_send(struct prf_con_info *con_info,
                            struct prf_svc *svc)
{
    struct gatt_disc_char_req * char_req  = KE_MSG_ALLOC(GATT_DISC_CHAR_REQ,
                                                         TASK_GATT, con_info->prf_id,
                                                         gatt_disc_char_req);
   char_req->req_type  = GATT_DISC_ALL_CHAR;
   char_req->conhdl    = con_info->conhdl;
   char_req->start_hdl = svc->shdl;
   char_req->end_hdl   = svc->ehdl;

   //send the message to GATT, which will send back the response when it gets it
   ke_msg_send(char_req);
}

void prf_disc_char_desc_send(struct prf_con_info* con_info, struct prf_char_inf * charact)
{
    // Check if there is at least one descriptor to find
    if ((charact->val_hdl + 1) <= (charact->char_hdl + charact->char_ehdl_off - 1))
    {
        // Ask for handles of a certain uuid
        struct gatt_disc_char_desc_req * dsc_req = KE_MSG_ALLOC(GATT_DISC_CHAR_DESC_REQ,
                                                                TASK_GATT, con_info->prf_id,
                                                                gatt_disc_char_desc_req);
        dsc_req->conhdl    = con_info->conhdl;
        dsc_req->start_hdl = charact->val_hdl + 1;
        dsc_req->end_hdl   = charact->char_hdl + charact->char_ehdl_off - 1;

        //send the message to GATT;there should be only one handle response every time
        ke_msg_send(dsc_req);
    }
    else
    {
        // Send a GATT complete event message
        struct gatt_cmp_evt *evt = KE_MSG_ALLOC(GATT_CMP_EVT,
                                                con_info->prf_id, TASK_GATT,
                                                gatt_cmp_evt);

        evt->status = PRF_ERR_OK;

        // Send the message
        ke_msg_send(evt);
    }
}

void prf_gatt_write(struct prf_con_info* con_info,
                    uint16_t handle, uint8_t* value, uint16_t length, uint8_t req_type)
{
    struct gatt_write_char_req *wr_char = KE_MSG_ALLOC_DYN(GATT_WRITE_CHAR_REQ,
                                                           TASK_GATT, con_info->prf_id,
                                                           gatt_write_char_req, length);

    // Connection Handle
    wr_char->conhdl         = con_info->conhdl;
    // Offset
    wr_char->wr_offset      = 0x0000;
    // Write Type
    wr_char->req_type       = req_type;
    // Characteristic Value attribute handle
    wr_char->charhdl        = handle;
    // Value Length
    wr_char->val_len        = length;
    // Auto Execute
    wr_char->auto_execute   = 1;
    // Value
    memcpy(&wr_char->value[0], value, length);

    // Send the message
    ke_msg_send(wr_char);
}

void prf_gatt_write_ntf_ind(struct prf_con_info* con_info, uint16_t handle, uint16_t ntf_ind_cfg)
{
    uint8_t value[2];

    // put value in air format
    co_write16p((&value[0]), ntf_ind_cfg);
    // write value over GATT
    prf_gatt_write(con_info, handle, value, 2, GATT_WRITE_CHAR);
}

uint8_t prf_check_svc_128_char_validity(uint8_t nb_chars,
                                    const struct prf_char_inf* chars,
                                    const struct qpp_char_def* chars_req)
{
    uint8_t status = PRF_ERR_OK;
    uint8_t i;

    for(i = 0; ((i < nb_chars) && (status == PRF_ERR_OK)); i++)
    {
        if (chars[i].char_hdl == ATT_INVALID_HANDLE)
        {
            //If Characteristic is not present, check requirements
            if (chars_req[i].req_flag == ATT_MANDATORY)
            {
                status = PRF_ERR_STOP_DISC_CHAR_MISSING;
            }
        }
        else
        {
            //If Characteristic is present, check properties
            if((chars[i].prop & chars_req[i].prop_mand) != chars_req[i].prop_mand)
            {
                status = PRF_ERR_STOP_DISC_WRONG_CHAR_PROP;
            }
        }
    }

    return status;
}

uint8_t prf_check_svc_char_validity(uint8_t nb_chars,
                                    const struct prf_char_inf* chars,
                                    const struct prf_char_def* chars_req)
{
    uint8_t status = PRF_ERR_OK;
    uint8_t i;

    for(i = 0; ((i < nb_chars) && (status == PRF_ERR_OK)); i++)
    {
        if (chars[i].char_hdl == ATT_INVALID_HANDLE)
        {
            //If Characteristic is not present, check requirements
            if (chars_req[i].req_flag == ATT_MANDATORY)
            {
                status = PRF_ERR_STOP_DISC_CHAR_MISSING;
            }
        }
        else
        {
            //If Characteristic is present, check properties
            if((chars[i].prop & chars_req[i].prop_mand) != chars_req[i].prop_mand)
            {
                status = PRF_ERR_STOP_DISC_WRONG_CHAR_PROP;
            }
        }
    }

    return status;
}


uint8_t prf_check_svc_char_desc_validity(uint8_t descs_size,
                                         const struct prf_char_desc_inf* descs,
                                         const struct prf_char_desc_def* descs_req,
                                         const struct prf_char_inf* chars)
{
    uint8_t status = PRF_ERR_OK;
    uint8_t i;

    for(i = 0; ((i < descs_size) && (status == PRF_ERR_OK)) ; i++)
    {
        if (descs[i].desc_hdl == ATT_INVALID_HANDLE)
        {
            //If Descriptor is missing, check if it is mandatory
            if (descs_req[i].req_flag == ATT_MANDATORY)
            {
                //Check if Char is present
                if (chars[descs_req[i].char_code].char_hdl != ATT_INVALID_HANDLE)
                {
                    //Char. is present and descriptor not, error
                    status = PRF_ERR_STOP_DISC_CHAR_MISSING;
                }
            }
        }
    }

    return status;
}

void prf_search_chars(uint16_t svc_ehdl, uint8_t nb_chars,
                      struct prf_char_inf* chars, const struct prf_char_def* chars_req,
                      const struct gatt_disc_char_all_cmp_evt* param,
                      uint8_t* last_found_char)
{
    // Counters
    uint8_t i, j;
    // Indicate if the read char is correct or not
    uint8_t char_ok;

    //Look over received characteristics
    for (i=0; i<(param->nb_entry); i++)
    {
        // Initialize char_ok value
        char_ok = 0;

        //Look over requested characteristics
        for (j=0; ((j<nb_chars) && (char_ok == 0)); j++)
        {
            /*
             * If the found characteristic belongs to the service
             */
            if(param->list[i].uuid == chars_req[j].uuid)
            {
                // Fount characteristic is correct
                char_ok = 1;

                //Save properties and handles
                chars[j].char_hdl       = param->list[i].attr_hdl;
                chars[j].val_hdl        = param->list[i].pointer_hdl;
                chars[j].prop           = param->list[i].prop;

                //Compute number of attribute in Char. using SVC edhl - Limited to 255
                chars[j].char_ehdl_off    = (uint8_t)(svc_ehdl - chars[j].char_hdl + 1);
            }

            if (char_ok == 1)
            {
                // Check if the last found characteristic was correct
                if (j != *last_found_char)
                {
                    //Update number of attributes for the last found char.
                    chars[*last_found_char].char_ehdl_off
                        = (uint8_t)(param->list[i].attr_hdl - chars[*last_found_char].char_hdl);

                    *last_found_char = j;
                }
            }
            else
            {
                //Update number of attributes for the last found char.
                chars[*last_found_char].char_ehdl_off
                    = (uint8_t)(param->list[i].attr_hdl - chars[*last_found_char].char_hdl);
            }
        }
    }
}

void prf_search_descs(uint8_t nb_descs,
                      struct prf_char_desc_inf* descs, const struct prf_char_desc_def* descs_req,
                      const struct gatt_disc_char_desc_cmp_evt* param,
                      uint8_t last_char_code)
{
    //Counters
    uint8_t i, j;

    //Retrieve characteristic descriptor handle using UUID
    for(i = 0; i<(param->nb_entry); i++)
    {
        for(j = 0; j<nb_descs; j++)
        {
            if ((last_char_code == descs_req[j].char_code)
                        && (param->list[i].desc_hdl == descs_req[j].uuid))
            {
                descs[j].desc_hdl = param->list[i].attr_hdl;
            }
        }
    }
}

void prf_client_disable_ind_send(prf_env_struct ***p_envs, ke_msg_id_t msg_id,
                                 ke_task_id_t task_id, uint8_t state)
{
    // Get the address of the environment
    prf_env_struct *env = prf_client_get_env(*p_envs, task_id);

    ASSERT_WARN(env != NULL);

    if (env != NULL)
    {
        struct prf_client_disable_ind *ind = KE_MSG_ALLOC(msg_id,
                                                          env->con_info.appid, env->con_info.prf_id,
                                                          prf_client_disable_ind);

        ind->conhdl    = env->con_info.conhdl;
        ind->status    = prf_client_disable(p_envs, KE_IDX_GET(env->con_info.prf_id));

        // Send the message
        ke_msg_send(ind);

        // Go to idle state
        ke_state_set(env->con_info.prf_id, state);
    }
}

#endif //(BLE_ATTC)

#if (BLE_ATTC || BLE_TIP_SERVER || BLE_AN_SERVER || BLE_PAS_SERVER)

static uint8_t prf_client_pool_envs_alloc(prf_env_struct ***p_envs)
{
    // Allocation status
    uint8_t status = PRF_ERR_OK;

    // Check if provided pointer is NULL
    if (*p_envs == NULL)
    {
        // Allocates buffer size for BLE_CONNECTION_MAX pointers
        *p_envs = (prf_env_struct **)ke_malloc(BLE_CONNECTION_MAX*sizeof(prf_env_struct *));

        // Check if a non-NULL pointer has been returned
        if(*p_envs == NULL)
        {
            // Not enough memory to allocate the data structure.
            status = PRF_ERR_REQ_DISALLOWED;
        }
        else
        {
            // Initialize all pointers for the pool to NULL
            memset(*p_envs, 0, BLE_CONNECTION_MAX*sizeof(prf_env_struct *));
        }
    }
    else
    {
        // envs should be a NULL pointer
        status = PRF_ERR_INVALID_PARAM;
    }

    return status;
}

static void prf_client_pool_envs_free(prf_env_struct ***p_envs)
{
    // Check if the pool of environments exists
    if (*p_envs != NULL)
    {
        // No environment exists, free the array of environment pointers
        ke_free(*p_envs);
        // Reset the address of the pool
        *p_envs = NULL;
    }
}

static uint8_t prf_client_env_alloc(prf_env_struct ***p_envs, uint8_t idx, uint16_t env_size)
{
    // Status
    uint8_t status = PRF_ERR_OK;
    // Pointer to the environment
    prf_env_struct *env;

    /*
     * Check if the pool of environment already exists. This pool exists only if it contains
     * the address of at least one environment.
     */
    if (*p_envs == NULL)
    {
        // Allocate the pool which contains the address of each profile client role environment.
        status = prf_client_pool_envs_alloc(p_envs);
    }

    // If the pool exists, allocate a new environment
    if (status == PRF_ERR_OK)
    {
        // Check index
        if (idx < BLE_CONNECTION_MAX)
        {
            // Get the current address of the environment
            env = *(*p_envs + idx);

            // Check if the environment matching this index already exists.
            if (env == NULL)
            {
                // Allocate a new environment.
                env = (prf_env_struct *)ke_malloc((uint32_t)env_size);

                // Check if the memory for the environment has been successfully allocated.
                if (env == NULL)
                {
                    // Not enough memory to allocate a new environment.
                    status = PRF_ERR_REQ_DISALLOWED;
                }
                else
                {
                    // Initialize values in the profile environment
                    memset(env, 0, env_size);

                    // Save the address of the environment in the pool.
                    *(*p_envs + idx) = env;
                }
            }
            else
            {
                // The profile client role is already enable for this index.
                status = PRF_ERR_INVALID_PARAM;
            }
        }
        else
        {
            // The provided connection doesn't exist.
            status = PRF_ERR_INVALID_PARAM;
        }
    }

    return status;
}

static uint8_t prf_client_env_free(prf_env_struct ***p_envs, uint8_t idx)
{
    // Status
    uint8_t status = PRF_ERR_OK;
    // Pointer to a profile environment
    prf_env_struct *p_prf_env = NULL;

    // Check that the provided pointer in non-NULL and that idx is OK
    if ((p_envs != NULL) && (idx < BLE_CONNECTION_MAX))
    {
        // Check if the pool of environments exists
        if (*p_envs != NULL)
        {
            // Get the address of the environment for the provided connection
            p_prf_env = *(*p_envs + idx);
        }

        // Check if this environment exists
        if (p_prf_env != NULL)
        {
            // Free the profile environment
            ke_free(p_prf_env);

            // Reset the address of the environment in the pool
            *(*p_envs + idx) = NULL;
        }
        else
        {
            // The environment doesn't exist.
            status = PRF_ERR_INVALID_PARAM;
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;
    }


    return status;
}

uint8_t prf_client_enable(prf_env_struct ***p_envs, struct prf_con_info *p_con_info,
                          void const *p_param, uint16_t env_size)
{
    // Status
    uint8_t status = PRF_ERR_OK;
    // Index
    uint8_t idx;
    // Pointer to an environment structure
    prf_env_struct *env;

    // Get the index matching this connection handle
    idx = gap_get_rec_idx(p_con_info->conhdl);

    // Check if the connection exists
    if (idx != GAP_INVALID_CONIDX)
    {
        // Check if we are in the good task instance
        if (idx != KE_IDX_GET(p_con_info->prf_id))
        {
            // Forward the message to the good instance of the task
            ke_msg_forward(p_param, KE_BUILD_ID(KE_TYPE_GET(p_con_info->prf_id), idx), p_con_info->appid);

            /*
             * Here, the status PRF_ERR_FEATURE_NOT_SUPPORTED is used to inform that the message
             * has been forwarded.
             */
            status = PRF_ERR_FEATURE_NOT_SUPPORTED;
        }
        else
        {
            // Create the environment for the provided connection.
            status = prf_client_env_alloc(p_envs, idx, env_size);

            // Check if the environment has been successfully created.
            if (status == PRF_ERR_OK)
            {
                // Get the profile environment address
                env = *(*p_envs + idx);

                /*
                 * Save the connection information in the environment.
                 * The first parameter of a profile client role environment is:
                 * struct prf_con_info con_info
                 */
                memcpy(&env->con_info, p_con_info, sizeof(struct prf_con_info));
            }
        }
    }
    else
    {
        // The connection doesn't exist
        status = PRF_ERR_INVALID_PARAM;
    }

    return status;
}

uint8_t prf_client_disable(prf_env_struct ***p_envs, uint8_t idx)
{
    // Status
    uint8_t status = PRF_ERR_INVALID_PARAM;
    // Counter
    uint8_t prf_id;
    // Pointer to a profile environment
    prf_env_struct *p_prf_env;

    // Check that the provided pointer in non-NULL and that idx is OK
    if ((p_envs != NULL) && (idx < BLE_CONNECTION_MAX))
    {
        // If the pool of environment doesn't exist, do nothing
        if (*p_envs != NULL)
        {
            status = prf_client_env_free(p_envs, idx);

            /*
             * If the removed environment was the last existing one in the environment pool, remove also
             * the pool.
             */
            if (status == PRF_ERR_OK)
            {
                // Reset the pointer to the environment.
                p_prf_env = NULL;

                // Check if at least one environment exists for the profile client role
                for (prf_id = 0; ((prf_id < BLE_CONNECTION_MAX) && (p_prf_env == NULL)); prf_id++)
                {
                    p_prf_env = *(*p_envs + prf_id);
                }

                if (p_prf_env == NULL)
                {
                    prf_client_pool_envs_free(p_envs);
                }
            }
        }
    }

    return status;
}

void prf_client_reset(prf_env_struct ***p_envs, ke_task_id_t task_type, ke_state_t state)
{
    // Index
    uint8_t idx;

    // Check if the provided pool of environments exists
    if (*p_envs != NULL)
    {
        // Remove all existing environments
        for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
        {
            prf_client_env_free(p_envs, idx);
        }

        // Remove the pool of environments
        prf_client_pool_envs_free(p_envs);
    }

    // Set the state to IDLE for each task instance
    for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
        // Go to IDLE state
        ke_state_set(KE_BUILD_ID(task_type, idx), state);
    }
}

prf_env_struct *prf_client_get_env(prf_env_struct **p_envs, ke_task_id_t task_id)
{
    // Address of the environement
    prf_env_struct *env = NULL;

    // Check if the provided pool of environments has been allocated
    if (p_envs != NULL)
    {
        // Get the address stored in the pool
        env = *(p_envs + KE_IDX_GET(task_id));
    }

    // Return the address of the environment
    return env;
}

void prf_client_enable_error(prf_env_struct ***p_envs, ke_task_id_t prf_task_id,
                             ke_state_t disc_state, ke_state_t idle_state)
{
    /* An error has been raised, three possibilities:
     *      - Either the state is IDLE, no environment has been created, nothing to do,
     *      - Or the state is DISCOVERING, we need to come back to the IDLE state and to remove the
     * environment,
     *      - Or the state is CONNECTED, the profile was already connected for the provided connection,
     * nothing to do
     */
    if (ke_state_get(prf_task_id) == disc_state)
    {
        // Come back to IDLE State
        ke_state_set(prf_task_id, idle_state);

        // Remove the allocated environment
        prf_client_disable(p_envs, KE_IDX_GET(prf_task_id));
    }
}

#endif // (BLE_ATTC) || BLE_TIP_SERVER || BLE_AN_SERVER || BLE_PAS_SERVER)

#if (BLE_ATTS)

void prf_server_error_ind_send(prf_env_struct *p_env, uint8_t status,
                               ke_msg_id_t ind_msg_id, ke_msg_id_t msg_id)
{
    struct prf_server_error_ind *ind = KE_MSG_ALLOC(ind_msg_id,
                                                    p_env->con_info.appid, p_env->con_info.prf_id,
                                                    prf_server_error_ind);

    ind->conhdl    = p_env->con_info.conhdl;
    ind->status    = status;
    ind->msg_id    = msg_id;

    // send the message
    ke_msg_send(ind);
}

#endif //(BLE_ATTS)
#if (BLE_ATTS || BLE_ATTC)

uint8_t prf_pack_date_time(uint8_t *packed_date, const struct prf_date_time* date_time)
{
    co_write16p(packed_date, date_time->year);
    *(packed_date + 2) = date_time->month;
    *(packed_date + 3) = date_time->day;
    *(packed_date + 4) = date_time->hour;
    *(packed_date + 5) = date_time->min;
    *(packed_date + 6) = date_time->sec;

    return 7;
}
uint8_t prf_unpack_date_time(uint8_t *packed_date, struct prf_date_time* date_time)
{
    date_time->year = co_read16p(&(packed_date[0]));
    date_time->month = packed_date[2];
    date_time->day   = packed_date[3];
    date_time->hour  = packed_date[4];
    date_time->min   = packed_date[5];
    date_time->sec   = packed_date[6];

    return 7;
}

void prf_dispatch_disconnect(uint8_t status, uint8_t reason, uint16_t conhdl, uint8_t idx)
{
    #if (BLE_ATTC || BLE_TIP_SERVER || BLE_PAS_SERVER || BLE_AN_SERVER)
    ke_task_id_t prf_task_id;
    #endif //(BLE_ATTC || BLE_TIP_SERVER || BLE_PAS_SERVER || BLE_AN_SERVER)

    //All profiles get this event, they must disable clean
    #if (BLE_QPP_CLIENT)
    prf_task_id = KE_BUILD_ID(TASK_QPPC, idx);
    //(void)prf_task_id; // Just for eliminating compiler warning
    if (ke_state_get(prf_task_id) != QPPC_IDLE)
    {
        gap_send_discon_cmp_evt(status,reason,conhdl,prf_task_id);
    }
    #endif // (BLE_QPP_CLIENT)

    #if (BLE_QPP_SERVER)
    if (ke_state_get(TASK_QPPS) == QPPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status,reason,conhdl,TASK_QPPS);
    }
    #endif // (BLE_QPP_SERVER)

    #if (BLE_ACCEL)
    if (ke_state_get(TASK_ACCEL) == ACCEL_ACTIVE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_ACCEL);
    }
    #endif // (BLE_ACCEL)

    #if (BLE_PROX_REPORTER)
    if (ke_state_get(TASK_PROXR) == PROXR_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_PROXR);
    }
    #endif // #if BLE_PROX_REPORTER

    #if (BLE_PROX_MONITOR)
    prf_task_id = KE_BUILD_ID(TASK_PROXM, idx);

    if (ke_state_get(prf_task_id) != PROXM_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_PROX_MONITOR

    #if (BLE_FINDME_TARGET)
    if (ke_state_get(TASK_FINDT) == FINDT_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_FINDT);
    }
    #endif // #if BLE_PROX_REPORTER

    #if (BLE_FINDME_LOCATOR)
    prf_task_id = KE_BUILD_ID(TASK_FINDL, idx);

    if (ke_state_get(prf_task_id) != FINDL_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_PROX_MONITOR

    #if (BLE_HT_THERMOM)
    if (ke_state_get(TASK_HTPT) == HTPT_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_HTPT);
    }
    #endif // #if BLE_HT_THERMOM

    #if (BLE_HT_COLLECTOR)
    prf_task_id = KE_BUILD_ID(TASK_HTPC, idx);

    if (ke_state_get(prf_task_id) != HTPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_HT_COLLECTOR

    #if (BLE_DIS_SERVER)
    if (ke_state_get(TASK_DISS) == DISS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_DISS);
    }
    #endif // #BLE_DIS_SERVER

    #if (BLE_DIS_CLIENT)
    prf_task_id = KE_BUILD_ID(TASK_DISC, idx);

    if (ke_state_get(prf_task_id) != DISC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_DIS_CLIENT

    #if (BLE_TIP_SERVER)
    prf_task_id = KE_BUILD_ID(TASK_TIPS, idx);

    if (ke_state_get(prf_task_id) == TIPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #BLE_TIP_SERVER

    #if (BLE_TIP_CLIENT)
    prf_task_id = KE_BUILD_ID(TASK_TIPC, idx);

    if (ke_state_get(prf_task_id) != TIPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_TIP_CLIENT

    #if (BLE_BP_SENSOR)
    if (ke_state_get(TASK_BLPS) == BLPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_BLPS);
    }
    #endif // #BLE_BP_SENSOR

    #if (BLE_BP_COLLECTOR)
    prf_task_id = KE_BUILD_ID(TASK_BLPC, idx);

    if (ke_state_get(prf_task_id) != BLPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_BP_COLLECTOR

    #if (BLE_HR_SENSOR)
    if (ke_state_get(TASK_HRPS) == HRPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_HRPS);
    }
    #endif // #BLE_HR_SENSOR

    #if (BLE_HR_COLLECTOR)
    prf_task_id = KE_BUILD_ID(TASK_HRPC, idx);

    if (ke_state_get(prf_task_id) != HRPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_HR_COLLECTOR

    #if (BLE_SP_SERVER)
    if (ke_state_get(TASK_SCPPS) == SCPPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_SCPPS);
    }
    #endif // #BLE_SP_SERVER

    #if (BLE_SP_CLIENT)
    prf_task_id = KE_BUILD_ID(TASK_SCPPC, idx);

    if (ke_state_get(prf_task_id) != SCPPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_SP_CLIENT

    #if (BLE_BATT_SERVER)
    if (ke_state_get(TASK_BASS) == BASS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_BASS);
    }
    #endif // #BLE_SP_SERVER

    #if (BLE_BATT_CLIENT)
    prf_task_id = KE_BUILD_ID(TASK_BASC, idx);

    if (ke_state_get(prf_task_id) != BASC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_BATT_CLIENT

    #if (BLE_HID_DEVICE)
    if (ke_state_get(TASK_HOGPD) == HOGPD_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_HOGPD);
    }
    #endif // #if BLE_HID_DEVICE

    #if (BLE_HID_BOOT_HOST)
    prf_task_id = KE_BUILD_ID(TASK_HOGPBH, idx);

    if (ke_state_get(prf_task_id) != HOGPBH_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_HID_BOOT_HOST

    #if (BLE_HID_REPORT_HOST)
    prf_task_id = KE_BUILD_ID(TASK_HOGPRH, idx);

    if (ke_state_get(prf_task_id) != HOGPRH_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // #if BLE_HID_REPORT_HOST

    #if (BLE_GL_COLLECTOR)
    prf_task_id = KE_BUILD_ID(TASK_GLPC, idx);

    if (ke_state_get(prf_task_id) != GLPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // (BLE_GL_COLLECTOR)

    #if (BLE_GL_SENSOR)
    if (ke_state_get(TASK_GLPS) == GLPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_GLPS);
    }
    #endif // (BLE_GL_SENSOR)

    #if (BLE_PAS_SERVER)
    prf_task_id = KE_BUILD_ID(TASK_PASPS, idx);

    if (ke_state_get(prf_task_id) == PASPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif //(BLE_PAS_SERVER)

    #if (BLE_PAS_CLIENT)
    prf_task_id = KE_BUILD_ID(TASK_PASPC, idx);

    if (ke_state_get(prf_task_id) != PASPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif //(BLE_PAS_CLIENT)

    #if (BLE_AN_SERVER)
    prf_task_id = KE_BUILD_ID(TASK_ANPS, idx);

    if (ke_state_get(prf_task_id) >= ANPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif //(BLE_AN_SERVER)

    #if (BLE_AN_CLIENT)
    prf_task_id = KE_BUILD_ID(TASK_ANPC, idx);

    if (ke_state_get(prf_task_id) != ANPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif //(BLE_AN_CLIENT)

    #if (BLE_RSC_COLLECTOR)
    prf_task_id = KE_BUILD_ID(TASK_RSCPC, idx);

    if (ke_state_get(prf_task_id) != RSCPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // (BLE_RSC_COLLECTOR)

    #if (BLE_RSC_SENSOR)
    if (ke_state_get(TASK_RSCPS) >= RSCPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_RSCPS);
    }
    #endif // (BLE_RSC_SENSOR)

    #if (BLE_CSC_COLLECTOR)
    prf_task_id = KE_BUILD_ID(TASK_CSCPC, idx);

    if (ke_state_get(prf_task_id) != CSCPC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // (BLE_CSC_COLLECTOR)

    #if (BLE_CSC_SENSOR)
    if (ke_state_get(TASK_CSCPS) >= CSCPS_CONNECTED)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, TASK_CSCPS);
    }
    #endif // (BLE_CSC_SENSOR)
    #if (BLE_ANCS_NC)
    prf_task_id = KE_BUILD_ID(TASK_ANCSC, idx);

    if (ke_state_get(prf_task_id) != ANCSC_IDLE)
    {
        gap_send_discon_cmp_evt(status, reason, conhdl, prf_task_id);
    }
    #endif // (BLE_ANCS_NC)
}

void prf_init(void)
{
    #if (BLE_ANCS_NC)
    ancsc_init();
    #endif // (BLE_ANCS_NC)

    #if (BLE_QPP_SERVER)
    qpps_init();
    #endif // (BLE_QPP_SERVER)

    #if (BLE_QPP_CLIENT)
    qppc_init();
    #endif // (BLE_QPP_CLIENT)

    #if (BLE_ACCEL)
    accel_init();
    #endif // (BLE_ACCEL)

    #if (BLE_HT_THERMOM)
    htpt_init();
    #endif // (BLE_HT_THERMOM)

    #if (BLE_HT_COLLECTOR)
    htpc_init();
    #endif // (BLE_HT_COLLECTOR)

    #if (BLE_DIS_SERVER)
    diss_init();
    #endif // (BLE_DIS_SERVER)

    #if (BLE_DIS_CLIENT)
    disc_init();
    #endif // (BLE_DIS_CLIENT)

    #if (BLE_BP_SENSOR)
    blps_init();
    #endif // (BLE_BP_SENSOR)

    #if (BLE_BP_COLLECTOR)
    blpc_init();
    #endif // (BLE_BP_COLLECTOR)

    #if (BLE_TIP_SERVER)
    tips_init();
    #endif // (BLE_TIP_SERVER)

    #if (BLE_TIP_CLIENT)
    tipc_init();
    #endif // (BLE_TIP_CLIENT)

    #if (BLE_HR_SENSOR)
    hrps_init();
    #endif // (BLE_HR_SENSOR)

    #if (BLE_HR_COLLECTOR)
    hrpc_init();
    #endif // (BLE_HR_COLLECTOR)

    #if (BLE_FINDME_LOCATOR)
    findl_init();
    #endif // (BLE_FINDME_LOCATOR)

    #if (BLE_FINDME_TARGET)
    findt_init();
    #endif // (BLE_FINDME_TARGET)

    #if (BLE_PROX_MONITOR)
    proxm_init();
    #endif // (BLE_PROX_MONITOR)

    #if (BLE_PROX_REPORTER)
    proxr_init();
    #endif // (BLE_PROX_REPORTER)

    #if (BLE_SP_SERVER)
    scpps_init();
    #endif // (BLE_SP_SERVER)

    #if (BLE_SP_CLIENT)
    scppc_init();
    #endif // (BLE_SP_CLIENT)

    #if (BLE_BATT_SERVER)
    bass_init();
    #endif // (BLE_BATT_SERVER)

    #if (BLE_BATT_CLIENT)
    basc_init();
    #endif // (BLE_BATT_CLIENT)

    #if (BLE_HID_DEVICE)
    hogpd_init();
    #endif // (BLE_HID_DEVICE)

    #if (BLE_HID_BOOT_HOST)
    hogpbh_init();
    #endif // (BLE_HID_BOOT_HOST)

    #if (BLE_HID_REPORT_HOST)
    hogprh_init();
    #endif // (BLE_HID_REPORT_HOST)

    #if (BLE_GL_COLLECTOR)
    glpc_init();
    #endif // (BLE_GL_COLLECTOR)

    #if (BLE_GL_SENSOR)
    glps_init();
    #endif // (BLE_GL_SENSOR)

    #if (BLE_PAS_CLIENT)
    paspc_init();
    #endif // (BLE_PAS_CLIENT)

    #if (BLE_PAS_SERVER)
    pasps_init();
    #endif // (BLE_PAS_SERVER)

    #if (BLE_AN_CLIENT)
    anpc_init();
    #endif // (BLE_AN_CLIENT)

    #if (BLE_AN_SERVER)
    anps_init();
    #endif // (BLE_AN_SERVER)

    #if (BLE_RSC_COLLECTOR)
    rscpc_init();
    #endif // (BLE_RSC_COLLECTOR)

    #if (BLE_RSC_SENSOR)
    rscps_init();
    #endif // (BLE_RSC_SENSOR)

    #if (BLE_CSC_COLLECTOR)
    cscpc_init();
    #endif // (BLE_CSC_COLLECTOR)

    #if (BLE_CSC_SENSOR)
    cscps_init();
    #endif // (BLE_CSC_SENSOR)

    #if (BLE_OTA_SERVER)
    if(OTA_STATUS_OK != otas_init(OTAS_FW2_ADDRESS, OTA_ENABLE_ENCRYPT, OTAS_DECRYPT_KEY))
    {
        ASSERT_ERR(0);
    }

    // Enable app the ability to control ota start or not.
    // PROFILE_CONTROL :  default parameter, nothing need to do , even you can diable following part.
    // APP_CONTROL : need response ota to start ota or not ,when recevie the ota start indication
    #ifdef ENAB_OTAS_APP_CTRL
    struct otas_ctrl_info ctrl_info = {APP_CONTROL, 0};

    if(ctrl_info.ctrl_flag == APP_CONTROL)
    {
        otas_control(&ctrl_info);
    }
    #endif


    #endif // (BLE_OTA_SERVER)
    
    struct gap_env_tag *gap_env = (struct gap_env_tag*)(_gap_env);
    gap_env->conn_rec.own_addr_type = QN_ADDR_TYPE;
}

#endif /* (BLE_ATTS || BLE_ATTC) */

/// @} PRF_UTILS

