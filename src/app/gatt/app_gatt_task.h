/**
 ****************************************************************************************
 *
 * @file app_gatt_task.h
 *
 * @brief Application GATT task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_GATT_TASK_H_
#define APP_GATT_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_GATT_TASK  Generic Attribute Profile Task API 
 * @ingroup APP_GATT
 * @brief Generic Attribute Profile Task API 
 *
 * GATT Task APIs are used to handle the message from GATT or APP.
 * @{
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief Handles the Discovery all services complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_svc_all_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_all_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Discovery services by UUID complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_svc_by_uuid_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_by_uuid_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Discovery included services by UUID complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_svc_incl_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_incl_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Discovery services with 128 bits UUID complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_svc_all_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_all_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the discover all characteristics complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_all_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_all_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the discover all characteristics by UUID complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_by_uuid_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_by_uuid_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the complete event for discover all 128 bits UUID characteristics.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_all_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_all_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the complete event for discover specific 128 bits UUID characteristics.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_by_uuid_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_by_uuid_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the discover a specified range characteristics descriptors complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_desc_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_desc_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the complete event for discover all 128 bits UUID characteristics.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_desc_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_desc_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the read characteristic complete event.
 *
 ****************************************************************************************
 */
int app_gatt_read_char_resp_handler(ke_msg_id_t const msgid, struct gatt_read_char_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Read multiple characteristic complete event.
 *
 ****************************************************************************************
 */
int app_gatt_read_char_mult_resp_handler(ke_msg_id_t const msgid, struct gatt_read_char_mult_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the write characteristic complete event.
 *
 ****************************************************************************************
 */
int app_gatt_write_char_resp_handler(ke_msg_id_t const msgid, struct gatt_write_char_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the write reliable characteristic complete event.
 *
 ****************************************************************************************
 */
int app_gatt_write_char_reliable_resp_handler(ke_msg_id_t const msgid, struct gatt_write_reliable_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the cancel write characteristic complete event.
 *
 ****************************************************************************************
 */
int app_gatt_cancel_write_char_resp_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the notify complete event.
 *
 ****************************************************************************************
 */
int app_gatt_notify_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_notify_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the value notification.
 *
 ****************************************************************************************
 */
int app_gatt_handle_value_notif_handler(ke_msg_id_t const msgid, struct gatt_handle_value_notif const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles reception of of a peer device indication.
 *
 ****************************************************************************************
 */
int app_gatt_handle_value_ind_handler(ke_msg_id_t const msgid, struct gatt_handle_value_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles reception of a peer device confirmation of a previously sent indication.
 *
 ****************************************************************************************
 */
int app_gatt_handle_value_cfm_handler(ke_msg_id_t const msgid, struct gatt_handle_value_cfm const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Discovery complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
                               
/*
 ****************************************************************************************
 * @brief Handles the GATT operation complete event.
 *
 ****************************************************************************************
 */
int app_gatt_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_cmp_evt const *param,
                          ke_task_id_t const dest_id, ke_task_id_t const src_id);

/// @} APP_GATT_TASK
#endif // APP_GATT_TASK_H_
