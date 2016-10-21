/**
 ****************************************************************************************
 *
 * @file app_ancsc_task.h
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

#ifndef APP_ANCSC_TASK_H_
#define APP_ANCSC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_ANCSC_TASK Apple Notification Center Service NC Role Task API
 * @ingroup APP_ANCSC
 * @brief Apple Notification Center Service NC Role Task API
 *
 * Apple Notification Center Service NC Task APIs are used to handle the message from ANCSC to APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_ANCS_NC
#include "app_ancsc.h"
/// @cond

//NVDS tag for nvds serice handle
#define APP_ANCSC_NVDS_TAG  (150)
#define APP_ANCSC_SOURCE_MAX_RECORD (20)  // buffered notify max number, the first notification are included.
#define APP_ANCSC_RD_ATTR_CMD_NUM_MAX   (3)
#define APP_ANCSC_MAX_REQ_CMD_LEN (50)
#define APP_ANCSC_DATA_SOURCE_BUFFER_LEN (64)
/*  
 *   1  byte   :   attributeID   
 *   2  bytes :   length
 *   so the total length of data source header is 3
*/
#define APP_ANCSC_DATA_SOURCE_HEAD_LEN    3
#define RAW_ANCSC_DATA_SOURCE_HEAD_LEN    5

struct ntf_attr_frame
{
    uint16_t data_len;
    uint8_t atrr_rd_cmp_count;
    uint8_t attr_hdr_state;
    uint16_t attr_remainer;
};
enum
{
    ATTR_HRD_NONE,
    ATTR_ID_CMP,
    ATTR_LEN_LOW_CMP,
    ATTR_HRD_CMP,

};
enum
{
    ANCSC_OP_IDLE,
    ANCSC_OP_SECURITY,
    ANCSC_OP_CFG_NTF_SOURCE,
    ANCSC_OP_CFG_DATA_SOURCE,
    ANCSC_OP_CONTROL_POINT
};
// current read notify status, 

enum
{
    ANCSC_RD_NTF_FINISHED,
    ANCSC_RD_NTF_ONGOING
};

// reading attribute state flags
enum
{
    ANCSC_RD_ATTR_FINISHED, // state--- reading an attribute is complete,so read the next attribute if exist
    ANCSC_RD_ATTR_ONGOING   // state--- reading an attribute is not complete,next package is pure data without package head
};

struct ancsc_service_info
{
    struct bd_addr addr;
    struct ancsc_content ancs;
};

struct ancsc_recv_data_source_st
{
    //the length of the buffer
    uint8_t length;
    //buffer of the uncoplete attibute
    uint8_t data_souce_buffer[APP_ANCSC_DATA_SOURCE_BUFFER_LEN];
};
struct ancsc_record_data_st
{
    struct ancsc_ntf_source_ind  data_record;
    uint8_t handle_flag; // hanlded: 0 ; other: 1
};
/// Apple Notification Center Service NC environment variable
struct app_ancsc_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Current operation
    uint8_t operation;
    /// Connection handle
    uint16_t conhdl;
    /// Notification UID
    uint32_t ntf_uid;
    struct ancsc_content ancs;

    struct ancsc_record_data_st  ancs_source_record[APP_ANCSC_SOURCE_MAX_RECORD];
   
    uint8_t ntf_cnt;
    uint8_t rd_ntf_cnt;
    bool  rd_ntf_flag;
    uint16_t enable_count;
    //uint8_t stop_timer;
    
    
     bool rd_attr_flag; // current state of read attrbute, if one attriute is not complete, the next data package is pure data without head 
    //number of the requested notify attributes , used for recomposing the data source fragments
    uint16_t cur_lack_data_len;
    uint8_t req_ntf_att_cnt;
    //request cmd content buffer, store the cmd
    uint8_t req_cmd[APP_ANCSC_MAX_REQ_CMD_LEN];
    //cmd length
    uint8_t req_cmd_len;
    //attribute number that had been successfully recieved
    uint8_t recv_ntf_att_cnt;
    // store the broken data,eg. some data source may divided into serveral parts then splice the frament.
    struct ancsc_recv_data_source_st recv_data_source;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_ancsc_env_tag *app_ancsc_env;
/// @endcond

/*
 ****************************************************************************************
 * @brief Indicate the content of the peer device ANCS.
 *
 ****************************************************************************************
 */
int app_ancsc_ancs_content_ind_handler(ke_msg_id_t const msgid,
                                       struct ancsc_ancs_content_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Accept and process of the Notification Source notification.
 *
 ****************************************************************************************
 */
int app_ancsc_ntf_source_ind_handler(ke_msg_id_t const msgid,
                                     struct ancsc_ntf_source_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);
                                     
/*
 ****************************************************************************************
 * @brief Accept and process of the Data Source notification.
 *
 ****************************************************************************************
 */
int app_ancsc_data_source_ind_handler(ke_msg_id_t const msgid,
                                      struct ancsc_data_source_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Complete Event Information.
 *
 ****************************************************************************************
 */
int app_ancsc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct ancsc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_ancsc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);
                                  
void app_ancsc_sm_entry(uint8_t idx, uint8_t status);


void app_ancsc_get_ntf_error_handler(uint8_t idx, uint8_t status);                                  
/*
**************************************************************************************
* @brief Handle the data resouce package .
*
****************************************************************************************  
*/     
void app_ancsc_data_source_buffer(uint8_t *attr_nbr,uint8_t *last_attr_id_pos,uint8_t offset,struct ancsc_data_source_ind *param);                                  


void app_ancsc_clear_buffer(void);
#endif // BLE_ANCS_NC

/// @} APP_ANCSC_TASK

#endif // APP_ANCSC_TASK_H_
