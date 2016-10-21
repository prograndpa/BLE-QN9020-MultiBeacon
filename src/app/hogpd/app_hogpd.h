/**
 ****************************************************************************************
 *
 * @file app_hogpd.h
 *
 * @brief Application HOGPD API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_HOGPD_H_
#define APP_HOGPD_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HOGPD_API HID Over Gatt Profile device
 * @ingroup APP_HOGPD
 * @brief HID Over Gatt Profile device
 * This profile defines how a device with Bluetooth low energy wireless communications can
 * support HID services over the Bluetooth low energy protocol stack. 
 * This profile is an adaptation of the USB HID specification to operate over a Bluetooth
 * low energy wireless link.
 * Two entities are defined in the Human Interface Device profile: the host and the device.
 * The device is the entity that directly interacts with a human, such as a keyboard or
 * mouse. A HID device may have more than one instance of the HID Service and in each of
 * these instancesit may have several instances of the Report Characteristic.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HID_DEVICE
#include "hogpd.h"
#include "hogpd_task.h"
#include "app_hogpd_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the hid device database - at initiation        
 *
 ****************************************************************************************
 */
void app_hogpd_create_db(uint8_t hids_nb, struct hogpd_hids_cfg *cfg);

/*
 ****************************************************************************************
 * @brief Set the HID Report Descriptor as Report Map Characteristic value     
 *
 ****************************************************************************************
 */
void app_hogpd_report_map_req(uint16_t report_map_len, uint8_t hids_nb, uint8_t *report_map);

/*
 ****************************************************************************************
 * @brief Start the HID profile - at connection  
 *
 ****************************************************************************************
 */
void app_hogpd_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint8_t hids_nb, struct hogpd_hids_ntf_cfg *ntf_cfg);

/*
 ****************************************************************************************
 * @brief Request sending of a report to the host     
 *
 ****************************************************************************************
 */
void app_hogpd_report_upd_req(uint16_t conhdl, uint8_t hids_nb, uint8_t report_nb, uint8_t report_length, uint8_t *report);

/*
 ****************************************************************************************
 * @brief Request sending of a boot report to the host     
 *
 ****************************************************************************************
 */
void app_hogpd_boot_report_upd_req(uint16_t conhdl, uint8_t hids_nb, uint8_t char_code, uint8_t report_length, uint8_t *boot_report);

#endif // BLE_HID_DEVICE

/// @} APP_HOGPD_API

#endif // APP_HOGPD_H_
