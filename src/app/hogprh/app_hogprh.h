/**
 ****************************************************************************************
 *
 * @file app_hogprh.h
 *
 * @brief Application HID Over GATT Profile Boot Host API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_HOGPRH_H_
#define APP_HOGPRH_H_


/**
 ****************************************************************************************
 * @addtogroup APP_HOGPRH_API HID Over GATT Profile Report Host Role API
 * @ingroup APP_HOGPRH
 * @brief HID Over GATT Profile Report Host Role API
 *
 *  The BLE HOGP Report Host role has been designed to allow a collector to communicate with a HID device. An 
 *  application which would use this role shall support a HID Parser and be able to handle arbitrary format for data 
 *  transfers. 
 *  The table below shown Report Host characteristic requirements: 
 *  - Report Map: Mandatory 
 *  - Report: Mandatory 
 *  - Boot Keyboard Input Report: Excluded 
 *  - Boot Keyboard Output Report: Excluded 
 *  - Boot Mouse Input Report: Excluded 
 *  - HID Information: Mandatory 
 *  - HID Control Point: Mandatory if the Host supports Suspend Mode, otherwise optional. 
 *  - Protocol Mode: Optional 
 *   
 *  Thus, the HOGP Report Host role task will only look for the non-excluded chracteristics during the discovery process. 
 *  Some restrictions have been defined in the BLE HOGP specification and shall be respected by the application designer: 
 *  - A Report Host shall not concurrently be a Boot Host. 
 *  - The Report Host shall use the GAP Central role. 
 *    
 *  As we currently have a static implementation, some firmware limitations have been defined for this role (in the 
 *  hogprh.h file): 
 *  - The maximal number of HID Service instances that can be handled has been limited to 2 
 *  (HOGPBH_NB_HIDS_INST_MAX). 
 *  - The maximal number of Report Characteristics instances that can be handled has been limited to 5 
 *  (HOGPRH_NB_REPORT_INST_MAX). 
 *  - The maximal length of a Report Characteristic value has been limited to 45 bytes. 
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HID_REPORT_HOST
#include "hogprh.h"
#include "hogprh_task.h"
#include "app_hogprh_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the HID Over GATT profile - at connection
 *
 ****************************************************************************************
 */
void app_hogprh_enable_req(uint8_t hids_nb, struct hogprh_hids_content *hids, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Disable profile role - at disconnection
 *
 ****************************************************************************************
 */
void app_hogprh_disable_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a HIDS characteristic/descriptor value
 *
 ****************************************************************************************
 */
void app_hogprh_rd_char_req(uint8_t read_code, uint8_t report_nb, uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message for configuring the characteristics
 *
 ****************************************************************************************
 */
void app_hogprh_cfg_ntf_req(uint8_t report_nb, uint16_t ntf_cfg, uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write peer HID Control Point value request
 *
 ****************************************************************************************
 */
void app_hogprh_hid_ctnl_pt_wr_req(uint8_t ctnl_pt, uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write peer Protocol Mode value request (Report Protocol Mode)
 *
 ****************************************************************************************
 */
void app_hogprh_set_report_proto_mode_req(uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write peer Report value request
 *
 ****************************************************************************************
 */
void app_hogprh_report_wr_req(uint8_t report_nb, uint8_t report_length, 
    uint8_t out_report_type, uint8_t *report, uint8_t hids_nb, uint16_t conhdl);

#endif /* BLE_HID_REPORT_HOST */

/// @} APP_HOGPRH_API

#endif // APP_HOGPRH_H_
