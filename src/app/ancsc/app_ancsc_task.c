/**
 ****************************************************************************************
 *
 * @file app_ancsc_task.c
 *
 * @brief Application ANCSC task implementation
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
 * @addtogroup APP_ANCSC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_ANCS_NC
#include "app_ancsc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_ancsc_env_tag *app_ancsc_env = &app_env.ancsc_ev[0];


/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/*

 ****************************************************************************************
 * @brief Indicate the content of the peer device ANCS. *//**
 *
 * @param[in] idx       the index of the current connection
 * @param[in] status    current status of the operation e
 * @description
 *
 * This handler is called when the status of the sm operation releated need update
 *
 ****************************************************************************************
 */
void app_ancsc_sm_entry(uint8_t idx, uint8_t status)
{
    switch (app_ancsc_env[idx].operation)
    {
        case ANCSC_OP_IDLE:
            app_ancsc_env[idx].operation = ANCSC_OP_CFG_DATA_SOURCE;
            app_ancsc_cfg_indntf_req(ANCSC_DESC_DATA_SOURCE_CL_CFG, PRF_CLI_START_NTF, app_ancsc_env[idx].conhdl);
            break;
        case ANCSC_OP_CFG_DATA_SOURCE:
            if (status == PRF_ERR_OK || status == PRF_ERR_INEXISTENT_HDL)
            {
                app_ancsc_env[idx].operation = ANCSC_OP_CFG_NTF_SOURCE;
                app_ancsc_cfg_indntf_req(ANCSC_DESC_NTF_SOURCE_CL_CFG, PRF_CLI_START_NTF, app_ancsc_env[idx].conhdl);
            }
            else if (status == ATT_ERR_INSUFF_AUTHEN || 
                     status == ATT_ERR_INSUFF_ENC)
            {
                struct bd_addr addr;

                app_ancsc_env[idx].operation = ANCSC_OP_IDLE;
                app_get_bd_addr_by_idx(idx, &addr);
                app_gap_bond_req(&addr, SMP_OOB_AUTH_DATA_NOT_PRESENT,
                                 SMP_AUTH_REQ_NO_MITM_BOND, SMP_IO_CAP_NO_INPUT_NO_OUTPUT);
            }
            else
            {
                ;
            }
            break;
        case ANCSC_OP_CFG_NTF_SOURCE:
            app_ancsc_env[idx].operation = ANCSC_OP_IDLE;
          //  app_ancsc_cfg_indntf_req(ANCSC_DESC_NTF_SOURCE_CL_CFG, PRF_CLI_START_NTF, app_ancsc_env[idx].conhdl);
            break;
        default:
            break;
    }
}

/*
 ****************************************************************************************
 * @brief Indicate the content of the peer device ANCS. *//**
 *
 * @param[in] msgid     ANCSC_ANCS_CONTENT_IND
 * @param[in] param     Pointer to struct ancsc_ancs_content_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANCSC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is called at the end of the discovery procedure performed after the
 * connection establishment. It contains the structure of the peer device ANCS.
 *
 ****************************************************************************************
 */
int app_ancsc_ancs_content_ind_handler(ke_msg_id_t const msgid,
                                       struct ancsc_ancs_content_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    app_ancsc_env[idx].ancs = param->ancs;

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief check and handle the cached notify requests  *//**
 *
 * @param[in] idx       index of current connection
 * @param[in] conhle    current connected handle
 * @description
 *
 * This handler is called when one notification had handled completely
 *
 ****************************************************************************************
 */
void app_ancsc_send_ancs_attribute_req_next(uint8_t idx,uint16_t conhdl)
{
   uint8_t ntf_cnt = app_ancsc_env[idx].ntf_cnt;
   uint8_t rd_cnt = app_ancsc_env[idx].rd_ntf_cnt;
   uint8_t index;
    
   if(rd_cnt < ntf_cnt)
   {     
       // find the ntf index to be handled
       for(index = rd_cnt;index < ntf_cnt;)
       {
            if(app_ancsc_env[idx].ancs_source_record[index].handle_flag == 1) 
                break;
            else
                index++;
       }
        app_ancsc_env[idx].ancs_source_record[index].handle_flag = 0;
        app_ancsc_env[idx].rd_ntf_cnt = (index); 
        app_ancsc_get_ntf_attribute_req(app_ancsc_env[idx].ancs_source_record[index].data_record.ntf_source.ntf_uid, app_ancsc_env[idx].req_cmd_len, app_ancsc_env[idx].req_cmd, conhdl);
        app_ancsc_env[idx].ntf_uid = app_ancsc_env[idx].ancs_source_record[index].data_record.ntf_source.ntf_uid;
       
   }
   else
   {
       memset(app_ancsc_env[idx].ancs_source_record,0,sizeof(struct ancsc_record_data_st)*APP_ANCSC_SOURCE_MAX_RECORD);
       app_ancsc_env[idx].ntf_cnt = 0;
       app_ancsc_env[idx].rd_ntf_cnt = 0;
       app_ancsc_env[idx].rd_ntf_flag = ANCSC_RD_NTF_FINISHED;
       app_ancsc_env[idx].rd_attr_flag = ANCSC_RD_ATTR_FINISHED;
       QPRINTF("\r\nAll NOTIFY info successfully recieved!\r\n");
   }
}
/*
 ****************************************************************************************
 * @brief clear the buffer when connection borken. *//**
 *
 * 
 * @description
 *
 * This handler is called when disconnection happened
 *
 ****************************************************************************************
 */
void app_ancsc_clear_buffer(void)
{
   
    uint8_t idx = KE_IDX_GET(TASK_ANCSC);
    app_ancsc_env[idx].ntf_cnt = 0;
    app_ancsc_env[idx].rd_ntf_cnt = 0;
    app_ancsc_env[idx].rd_ntf_flag = ANCSC_RD_NTF_FINISHED;
    app_ancsc_env[idx].rd_attr_flag = ANCSC_RD_ATTR_FINISHED;
    app_ancsc_env[idx].recv_data_source.length = 0;
    memset(app_ancsc_env[idx].ancs_source_record,0,sizeof(struct ancsc_record_data_st)*APP_ANCSC_SOURCE_MAX_RECORD);
    memset(app_ancsc_env[idx].recv_data_source.data_souce_buffer,0,APP_ANCSC_DATA_SOURCE_BUFFER_LEN);

    
}
/*
 ****************************************************************************************
 * @brief Handler of the error occuration from peer device ANCS. *//**
 *
 * @param[in] idx       the index of the current connection
 * @param[in] status    current status of the operation e
 * @description
 *
 * This handler is called when the error retured from Apple device
 *
 ****************************************************************************************
 */
void app_ancsc_get_ntf_error_handler(uint8_t idx, uint8_t status)
{
   
 
   app_ancsc_env[idx].ancs_source_record[app_ancsc_env[idx].rd_ntf_cnt].handle_flag = 0;
   app_ancsc_env[idx].rd_ntf_cnt++;
   app_ancsc_env[idx].rd_attr_flag = ANCSC_RD_ATTR_FINISHED;
   app_ancsc_env[idx].recv_ntf_att_cnt = 0;
   app_ancsc_env[idx].recv_data_source.length = 0;
   memset(app_ancsc_env[idx].recv_data_source.data_souce_buffer,0,APP_ANCSC_DATA_SOURCE_BUFFER_LEN);
   app_ancsc_send_ancs_attribute_req_next(idx,0);
    
}
/*
 ****************************************************************************************
 * @brief Accept and process of the Notification Source notification. *//**
 *
 * @param[in] msgid     ANCSC_NTF_SOURCE_IND
 * @param[in] param     Pointer to struct ancsc_ntf_source_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANCSC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is called once a Notification Source value has been received from the peer device 
 * upon a notification operation.
 *
 ****************************************************************************************
 */

int app_ancsc_ntf_source_ind_handler(ke_msg_id_t const msgid,
                                     struct ancsc_ntf_source_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    uint8_t notify_idx = app_ancsc_env[idx].ntf_cnt;
    uint8_t tag;
    QPRINTF("EventID: %d, EventFlags: %d, CategoryID: %d, count: %d, NotificationUID: %d\r\n", 
                    param->ntf_source.event_id, 
                    param->ntf_source.event_flags,
                    param->ntf_source.category_id, 
                    param->ntf_source.category_count,
                    param->ntf_source.ntf_uid);
   
    uint8_t cmd[] = {0x00, 0x01, 0x20, 0x00,0x03,0x20,0x00};
    app_ancsc_env[idx].req_cmd_len = sizeof(cmd);
     // reference the ancs spec, here request 2 notify attributes
    // 0x00: APPIDentifier
    // 0x01: Title
    // 0x03: Messeage
    memcpy(app_ancsc_env[idx].req_cmd,cmd,app_ancsc_env[idx].req_cmd_len);
    app_ancsc_env[idx].req_ntf_att_cnt = 0x03;
    if (param->ntf_source.event_id == NOTIFICATION_ADDED)
    {
        if(app_ancsc_env[idx].rd_ntf_flag== ANCSC_RD_NTF_FINISHED)
        { 
            app_ancsc_env[idx].rd_ntf_flag = ANCSC_RD_NTF_ONGOING;
            memset(app_ancsc_env[idx].ancs_source_record,0,sizeof(struct ancsc_record_data_st)*APP_ANCSC_SOURCE_MAX_RECORD);
            memcpy(&app_ancsc_env[idx].ancs_source_record[notify_idx].data_record,param,sizeof(struct ancsc_ntf_source_ind));          
            app_ancsc_env[idx].ntf_cnt = 1;
            app_ancsc_env[idx].rd_ntf_cnt = 0;
            app_ancsc_env[idx].rd_attr_flag = ANCSC_RD_ATTR_FINISHED;
            app_ancsc_get_ntf_attribute_req(param->ntf_source.ntf_uid, sizeof(cmd), cmd, param->conhdl);
            app_ancsc_env[idx].ntf_uid = param->ntf_source.ntf_uid;  
            
        }
        else
        {
            if(app_ancsc_env[idx].ntf_cnt >= APP_ANCSC_SOURCE_MAX_RECORD)
            {
                QPRINTF("\r\nWarning: Notification Buffer overfloat!!\r\n");
            }  
            else
            {
                 QPRINTF("Saved New notification in buffer!\r\n");
                 memcpy(&app_ancsc_env[idx].ancs_source_record[notify_idx].data_record,param,sizeof(struct ancsc_ntf_source_ind)); 
                 app_ancsc_env[idx].ntf_cnt++;
                 app_ancsc_env[idx].ancs_source_record[notify_idx].handle_flag = 1;
            }
           
            
        }
     
    }
    else // if the notification is removed, travers the notify list delete the unhanlded notify
    {
       
        for(tag = 0; tag < app_ancsc_env[idx].ntf_cnt;tag++)
        {
            if(param->ntf_source.ntf_uid == app_ancsc_env[idx].ancs_source_record[tag].data_record.ntf_source.ntf_uid)
            {
                app_ancsc_env[idx].ancs_source_record[tag].handle_flag = 0; 
            }    
        }
    
    }
    return (KE_MSG_CONSUMED);
}
/*
 ****************************************************************************************
 * @brief handle the bufferd data of the uncomplete attribute which are transfered in multi fragment *//**
 *
 * @param[in]  recv_data_st   Pointer of the cached data 
 * @param[out] attr_nbr       Number of complete attribute frame in the buffer 
 * @param[out] tail_id_pos    the last attr id postion in the current package 
 * @param[out] offset         offset of length, from which index the traverse begin  
 * @description 
 *
 * this API used for handling the package data of APP, by traversing the package , attr_nbr value is updated accordingly,
 * based on the tail_id_pos, the ramaind data will stored in the buffer, processing of the nest coming package will be 
 * different.
 *
 ****************************************************************************************
 */
void app_ancsc_data_source_buffer(uint8_t *attr_nbr,uint8_t *tail_id_pos,uint8_t offset,struct ancsc_data_source_ind *param)
{
    uint16_t att_length = 0;
    uint8_t index = 0;
    uint8_t pos = offset;
    for(pos = offset; pos < param->data_size;)
    {
        att_length = co_read16p(&param->data_source[pos+1]);    
        
        pos +=(att_length + APP_ANCSC_DATA_SOURCE_HEAD_LEN);
        index ++;

     }
    
    if(pos > param->data_size)
    {
         pos -= (att_length + APP_ANCSC_DATA_SOURCE_HEAD_LEN);
         index--;   
    }

    *tail_id_pos = pos;
    *attr_nbr = index;        
         

    
  
}
/*
 ****************************************************************************************
 * @brief Accept and process of the Data Source notification. *//**
 *
 * @param[in] msgid     ANCSC_DATA_SOURCE_IND
 * @param[in] param     Pointer to struct ancsc_data_source_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANCSC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is called once a Data Source value has been received from the peer device 
 * upon a notification operation. If the value is larger than the 20 bytes, it is split into 
 * multiple fragments by the NP. The NC must recompose the value by splicing each fragment.
 * The value is complete when the complete tuples for each requested attribute has been received.
 *
 ****************************************************************************************
 */
int app_ancsc_data_source_ind_handler(ke_msg_id_t const msgid,
                                      struct ancsc_data_source_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id); 
    uint8_t complete_attr_nbr = 0;
    uint8_t tail_id_pos = 0;
    uint32_t NotificationUid;
    // current length of the received buffer 
    uint8_t cur_len = app_ancsc_env[idx].recv_data_source.length;
    //one attribute data had not been completely recieved ,so the coming data package is pure data ,we recieved and recompose it 
    if(app_ancsc_env[idx].rd_attr_flag == ANCSC_RD_ATTR_ONGOING)
    {
        
//            QPRINTF("ONGOING ---the param data buffer: cur_len = %d\r\n",cur_len);
//            for (uint8_t i = 0; i < param->data_size; i++)
//                 QPRINTF("%02x ", param->data_source[i]);
//                QPRINTF("\r\n");
        if(cur_len < APP_ANCSC_DATA_SOURCE_HEAD_LEN)
        {
        
            memcpy(&app_ancsc_env[idx].recv_data_source.data_souce_buffer[cur_len],&param->data_source,3-cur_len);
            app_ancsc_env[idx].cur_lack_data_len = co_read16p(&app_ancsc_env[idx].recv_data_source.data_souce_buffer[1]);
            if(param->data_size - (APP_ANCSC_DATA_SOURCE_HEAD_LEN - cur_len) < app_ancsc_env[idx].cur_lack_data_len)
            {
                app_ancsc_env[idx].cur_lack_data_len -= (param->data_size - (APP_ANCSC_DATA_SOURCE_HEAD_LEN - cur_len));
                app_ancsc_env[idx].recv_data_source.length = APP_ANCSC_DATA_SOURCE_HEAD_LEN;
            }
            else
            {
              
                app_ancsc_env[idx].recv_ntf_att_cnt++;
                app_ancsc_data_source_buffer(&complete_attr_nbr,&tail_id_pos,\
                app_ancsc_env[idx].cur_lack_data_len + (APP_ANCSC_DATA_SOURCE_HEAD_LEN-cur_len),param);
                memcpy(app_ancsc_env[idx].recv_data_source.data_souce_buffer,&param->data_source[tail_id_pos],(param->data_size - tail_id_pos));
                app_ancsc_env[idx].cur_lack_data_len = 0;

                app_ancsc_env[idx].recv_data_source.length = param->data_size - tail_id_pos;
            }
        }
        else if(cur_len == APP_ANCSC_DATA_SOURCE_HEAD_LEN)
        {
           
            if( app_ancsc_env[idx].cur_lack_data_len == 0)
            {
                app_ancsc_env[idx].cur_lack_data_len = co_read16p(&app_ancsc_env[idx].recv_data_source.data_souce_buffer[1]);
            }
            if(param->data_size < app_ancsc_env[idx].cur_lack_data_len)
            {
                app_ancsc_env[idx].cur_lack_data_len -= param->data_size;
            }
            else
            {
                app_ancsc_env[idx].recv_ntf_att_cnt++;
                app_ancsc_data_source_buffer(&complete_attr_nbr,&tail_id_pos,app_ancsc_env[idx].cur_lack_data_len,param);
                memcpy(app_ancsc_env[idx].recv_data_source.data_souce_buffer,&param->data_source[tail_id_pos],(param->data_size - tail_id_pos));
                app_ancsc_env[idx].cur_lack_data_len = 0;
                app_ancsc_env[idx].recv_data_source.length = param->data_size - tail_id_pos;
            }
        }
        else // cur_len > APP_ANCSC_DATA_SOURCE_HEAD_LEN
        {
         
            app_ancsc_env[idx].cur_lack_data_len = co_read16p(&app_ancsc_env[idx].recv_data_source.data_souce_buffer[1]);
           
            if( param->data_size + ( app_ancsc_env[idx].recv_data_source.length - APP_ANCSC_DATA_SOURCE_HEAD_LEN) < app_ancsc_env[idx].cur_lack_data_len)
            {         
                app_ancsc_env[idx].cur_lack_data_len -= (param->data_size + ( app_ancsc_env[idx].recv_data_source.length - APP_ANCSC_DATA_SOURCE_HEAD_LEN));
                memset(&app_ancsc_env[idx].recv_data_source.data_souce_buffer[APP_ANCSC_DATA_SOURCE_HEAD_LEN],0,APP_ANCSC_DATA_SOURCE_BUFFER_LEN-APP_ANCSC_DATA_SOURCE_HEAD_LEN);
                app_ancsc_env[idx].recv_data_source.length = APP_ANCSC_DATA_SOURCE_HEAD_LEN;
            }
            else
            {         
               
                app_ancsc_env[idx].recv_ntf_att_cnt++;
                app_ancsc_data_source_buffer(&complete_attr_nbr,&tail_id_pos,\
                app_ancsc_env[idx].cur_lack_data_len - (app_ancsc_env[idx].recv_data_source.length - APP_ANCSC_DATA_SOURCE_HEAD_LEN),param);
                memcpy(app_ancsc_env[idx].recv_data_source.data_souce_buffer,&param->data_source[tail_id_pos],(param->data_size - tail_id_pos));
                app_ancsc_env[idx].cur_lack_data_len = 0;
                app_ancsc_env[idx].recv_data_source.length = param->data_size - tail_id_pos;
            }
        }
    }
    else //start one new attribute list reading
    {
        NotificationUid = co_read32p(&param->data_source[1]);
        QPRINTF("\r\nNotificationUid = %d request attribute numbers = %d \r\n",NotificationUid,app_ancsc_env[idx].req_ntf_att_cnt);
        memset(app_ancsc_env[idx].recv_data_source.data_souce_buffer,0,APP_ANCSC_DATA_SOURCE_BUFFER_LEN);
        if(param->data_source[0] == 0 && NotificationUid == app_ancsc_env[idx].ntf_uid)
        {  
            app_ancsc_data_source_buffer(&complete_attr_nbr,&tail_id_pos,RAW_ANCSC_DATA_SOURCE_HEAD_LEN,param); 
            memcpy(app_ancsc_env[idx].recv_data_source.data_souce_buffer,&param->data_source[tail_id_pos],param->data_size - tail_id_pos);
            app_ancsc_env[idx].recv_data_source.length = param->data_size - tail_id_pos;
           
        }               
    }
    		
    app_ancsc_env[idx].recv_ntf_att_cnt += complete_attr_nbr;
    app_ancsc_env[idx].rd_attr_flag = (app_ancsc_env[idx].recv_ntf_att_cnt == app_ancsc_env[idx].req_ntf_att_cnt)?ANCSC_RD_ATTR_FINISHED:ANCSC_RD_ATTR_ONGOING;
    if( app_ancsc_env[idx].rd_attr_flag == ANCSC_RD_ATTR_FINISHED)
    {
         
            //all the attribute recieved ;
            memset(app_ancsc_env[idx].recv_data_source.data_souce_buffer,0,APP_ANCSC_DATA_SOURCE_BUFFER_LEN);
            app_ancsc_env[idx].recv_data_source.length = 0;
            app_ancsc_env[idx].recv_ntf_att_cnt = 0;
            app_ancsc_env[idx].ancs_source_record[app_ancsc_env[idx].rd_ntf_cnt].handle_flag = 0;
            app_ancsc_env[idx].rd_ntf_cnt++;
            QPRINTF("Total Notification Number [%d]  Current Notification [%d] handle OK\r\n",app_ancsc_env[idx].ntf_cnt,app_ancsc_env[idx].rd_ntf_cnt);
            //check the notify record buffer,if any exist, continue to send control point cmd;
            app_ancsc_send_ancs_attribute_req_next(idx,param->conhdl);
    }  
    
    return (KE_MSG_CONSUMED);

}

/*
 ****************************************************************************************
 * @brief Envent complete information handler. *//**
 *
 * @param[in] msgid     ANCSC_CMP_EVT
 * @param[in] param     Pointer to struct ancsc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANCSC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is called once a envent is completed
 *
 ****************************************************************************************
 */
int app_ancsc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct ancsc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    //QPRINTF("ANCSC Complete Status Value: %d. idx = %d, \r\n", param->status, idx);
    switch (param->operation)
    {
        case ANCSC_ENABLE_OP_CODE:
            if (param->status == PRF_ERR_OK)
            {
                app_ancsc_env[idx].conhdl = param->conhdl;
                app_ancsc_env[idx].enabled = true;
                app_ancsc_env[idx].operation = ANCSC_OP_IDLE;
                app_ancsc_sm_entry(idx, param->status);   
            }
            else
            {
               app_ancsc_env[idx].conhdl = param->conhdl;
               if(app_ancsc_env[idx].enable_count < 10)
                {
                    // start ancsc_supervision_timer, 5000ms
                    ke_timer_set(ANCS_ENABLE_SUVPER_TIMER0+idx, TASK_APP, 500); 
                    app_ancsc_env[idx].enable_count++;
                }
                else
                {
                    app_ancsc_env[idx].enable_count = 0;
                
                }
            }
            QPRINTF("ANCSC enable confirmation status: 0x%X.\r\n", param->status);
            break;
        case ANCSC_CFG_OP_CODE:
            app_ancsc_sm_entry(idx, param->status);
            QPRINTF("ANCSC configure status: 0x%X.\r\n", param->status);
            break;
        case ANCSC_GET_NTF_ATTRIBUTE_OP_CODE:
            // if error , reference handler
            if(param->status != PRF_ERR_OK)
            {
                QPRINTF("GET_NTF_ATTRIBUTE status : %d\r\n",param->status);
                app_ancsc_get_ntf_error_handler(idx, param->status);
            }
            break;
        case ANCSC_GET_APP_ATTRIBUTE_OP_CODE:
            QPRINTF("ANCSC_GET_APP_ATTRIBUTE_OP_CODE\r\n");
            break;
        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     ANCSC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANCSC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Alert Notification Client Role
 * task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_ancsc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    app_ancsc_env[idx].conhdl = 0xFF;
    app_ancsc_env[idx].enabled = false;
    app_ancsc_env[idx].operation = ANCSC_OP_IDLE;
    
    QPRINTF("ANCSC disable indication\r\n");

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_ANCSC_TASK
