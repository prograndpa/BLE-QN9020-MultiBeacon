/**
 ****************************************************************************************
 *
 * @file llm_task.h
 *
 * @brief LLM task header file
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
 * @file llm_task.h
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
#ifndef LLM_TASK_H_
#define LLM_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup LLMTASK LLMTASK
 * @ingroup LLM
 * @brief Link Layer Manager Task
 *
 * The LLM task is responsible for managing link layer actions not related to a
 * specific connection with a peer (e.g. scanning, advertising, etc.). It implements the
 * state machine related to these actions.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "co_bt.h"

#define BLE_ADV_REPORTS_MAX             1

/*
 * STRUCTURES
 ****************************************************************************************
 */
///LLM LE Set Advertising Parameters Command parameters structure
struct llm_le_set_adv_param_cmd
{
    ///Minimum interval for advertising
    uint16_t       adv_intv_min;
    ///Maximum interval for advertising
    uint16_t       adv_intv_max;
    ///Advertising type
    uint8_t        adv_type;
    ///Own address type: public=0x00 /random = 0x01
    uint8_t        own_addr_type;
    ///Direct address type: public=0x00 /random = 0x01
    uint8_t        direct_addr_type;
    ///Direct Bluetooth device address
    struct bd_addr direct_addr;
    ///Advertising channel map
    uint8_t        adv_chnl_map;
    ///Advertising filter policy
    uint8_t        adv_filt_policy;
};

///LLM LE Set Advertising Data Command parameters structure
struct llm_le_set_adv_data_cmd
{
    ///Advertising data length
    uint8_t         adv_data_len;
    ///Advertising data - maximum 31 bytes
    struct adv_data data;
};

///LLM LE Set Scan Response Data Command parameters structure
struct llm_le_set_scan_rsp_data_cmd
{
    ///Scan response data length
    uint8_t              scan_rsp_data_len;
    ///Scan response data - maximum 31 bytes
    struct scan_rsp_data data;
};

///LLM LE Set Scan Parameters Command parameters structure
struct llm_le_set_scan_param_cmd
{
    ///Scan type - 0=passive / 1=active
    uint8_t        scan_type;
    ///Scan interval
    uint16_t       scan_intv;
    ///Scan window size
    uint16_t       scan_window;
    ///Own address type - 0=public, 1=random
    uint8_t        own_addr_type;
    ///Scan filter policy
    uint8_t        scan_filt_policy;
};

///LLM LE Set Scan Enable Command parameters structure
struct llm_le_set_scan_en_cmd
{
    ///Scan enable - 0=disabled, 1=enabled
    uint8_t        scan_en;
    ///Enable for duplicates filtering - 0 =disabled/ 1=enabled
    uint8_t        filter_duplic_en;
};

///LLM LE Create Connection Command parameters structure
struct llm_le_create_con_cmd
{
    ///Scan interval
    uint16_t       scan_intv;
    ///Scan window size
    uint16_t       scan_window;
    ///Initiator filter policy
    uint8_t        init_filt_policy;
    ///Peer address type - 0=public/1=random
    uint8_t        peer_addr_type;
    ///Peer BD address
    struct bd_addr peer_addr;
    ///Own address type - 0=public/1=random
    uint8_t        own_addr_type;
    ///Minimum of connection interval
    uint16_t       con_intv_min;
    ///Maximum of connection interval
    uint16_t       con_intv_max;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout
    uint16_t       superv_to;
    ///Minimum CE length
    uint16_t       ce_len_min;
    ///Maximum CE length
    uint16_t       ce_len_max;
};

///LLM LE Add Device to White List Command parameters structure
struct llm_le_add_dev_to_wlst_cmd
{
    ///Type of address of the device to be added to the White List - 0=public/1=random
    uint8_t        dev_addr_type;
    ///Address of device to be added to White List
    struct bd_addr dev_addr;
};

///LLM LE Remove Device from White List Command parameters structure
struct llm_le_rmv_dev_from_wlst_cmd
{
    ///Type of address of the device to be removed from the White List - 0=public/1=random
    uint8_t        dev_addr_type;
    ///Address of device to be removed from White List
    struct bd_addr dev_addr;
};

///LLM LE advertising report event structure
struct llm_le_adv_report_evt
{
    ///LE Subevent code
    uint8_t             subevent_code;
    ///Number of advertising reports in this event
    uint8_t             nb_reports;
    ///Advertising reports structures array
    struct adv_report   adv_rep[BLE_ADV_REPORTS_MAX];
};

/// @} LLMTASK

#endif // LLM_TASK_H_
