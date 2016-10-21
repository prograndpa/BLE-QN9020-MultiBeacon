/**
 ****************************************************************************************
 *
 * @file app_hogpbh.h
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

#ifndef APP_HOGPBH_H_
#define APP_HOGPBH_H_


/**
 ****************************************************************************************
 * @addtogroup APP_HOGPBH_API HID Over GATT Profile Boot Host Role API
 * @ingroup APP_HOGPBH
 * @brief HID Over GATT Profile Boot Host Role API
 *
 *  The BLE HOGP Boot Host role has been designed to allow a collector to easily commuicate with a HID Boot device 
 *  (Boot Keyboard or Boot Mouse). All data exchanged within this role have a fixed length and each bit in a packet has a 
 *  known meaning. There is no need of a Report descriptor, so no HID Parser is required in the application. 
 *  The table below shown Boot Host characteristic requirements: 
 *  - Report Map: Excluded 
 *  - Report: Excluded 
 *  - Boot Keyboard Input/Output Report: Mandatory to support at least one of these features
 *  - Boot Mouse Input Report: Mandatory to support at least one of these features
 *  - HID Information: Excluded 
 *  - HID Control Point: Excluded 
 *  - Protocol Mode: Mandatory 
 *   
 *  Thus, the HOGP Boot Host role task will only look for the non-excluded chracteristics during the discovery process 
 *  (more details in HOGPBH_ENABLE_REQ and HOGPBH_ENABLE_CFM). 
 *  Some restrictions have been defined in BLE HOGP specification and shall be respected by the application designer: 
 *  - A Boot Host shall not concurrently be a Report Host. 
 *  - The Boot Host shall use the GAP Central role. 
 *   
 *  As we currently have a static implementation, some firmware limitations have been defined for this role (in the 
 *  hogpbh.h file): 
 *  - The maximal number of HID Service instances that can be handled has been limited to 2 
 *  (HOGPBH_NB_HIDS_INST_MAX). 
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HID_BOOT_HOST
#include "hogpbh.h"
#include "hogpbh_task.h"
#include "app_hogpbh_task.h"

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
void app_hogpbh_enable_req(uint8_t hids_nb, struct hids_content *hids, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Disable profile role - at disconnection
 *
 ****************************************************************************************
 */
void app_hogpbh_disable_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a HIDS or DIS or BAS characteristic value
 *
 ****************************************************************************************
 */
void app_hogpbh_rd_char_req(uint8_t char_code, uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message for configuring the characteristics
 *
 ****************************************************************************************
 */
void app_hogpbh_cfg_ntf_req(uint8_t desc_code, uint16_t ntf_cfg, uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write peer Boot Report Char. value
 *
 ****************************************************************************************
 */
void app_hogpbh_boot_report_wr_req(uint8_t wr_type, uint8_t char_code, uint8_t report_length, uint8_t hids_nb, uint8_t *report, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write peer Boot Keyboard Input Report Char. value
 *
 ****************************************************************************************
 */
void app_hogpbh_boot_kb_in_report_wr_req(uint8_t *report, uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write peer Boot Mouse Input Report Char. value
 *
 ****************************************************************************************
 */
void app_hogpbh_boot_mouse_in_report_wr_req(uint8_t length, uint8_t *report, uint8_t hids_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Set Boot Protocol Mode on peer device
 *
 ****************************************************************************************
 */
void app_hogpbh_set_boot_proto_mode_req(uint8_t hids_nb, uint16_t conhdl);

#endif /* BLE_HID_BOOT_HOST */

/// @} APP_HOGPBH_API

#endif // APP_HOGPBH_H_
