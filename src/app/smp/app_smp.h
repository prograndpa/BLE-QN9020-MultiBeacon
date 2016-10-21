/**
 ****************************************************************************************
 *
 * @file app_smp.h
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

#ifndef _APP_SMP_H_
#define _APP_SMP_H_

/**
 ****************************************************************************************
 * @addtogroup APP_SMP_API Security Manager Protocol API
 * @ingroup APP_SMP
 * @brief Security Manager Protocol API
 *
 *  The Security Manager allows two devices to setup a secure relationship either by encrypting
 *  a link, by bonding (exchanging information about each other) or signature use over a plain
 *  link. Please refer to the Bluetooth Core 4.0 specification for the SM requirements and
 *  protocol methods. Further details can be found in the BLE Host Software Functional
 *  Specification document [2]. Due to the possibility that a device be connected to several
 *  devices and that it starts the security procedure at different times simultaneously with some
 *  of them, the SM has been implemented in the format of a SM Manager task with one instance
 *  (SMPM) and a SM Controller task (SMPC) that will handle connection specific security procedures
 *  per connection. The SMPM environment will handle holding the keys that are unique per device
 *  and used for several connections, and act as a multiplexer between the SMPC tasks and the 
 *  uniquely instantiated LLM for the encrypt functionality, or GAP for being informed of connections
 *  and disconnections.
 *
 *  SMP APIs are used by the Application directly especially for requesting keys necessary
 *  during the pairing/encrypting procedures.
 *
 * @{
 ****************************************************************************************
 */

#include "smpc.h"
#include "smpc_task.h"
#include "smpm_task.h"
#include "app_smp_task.h"

// In bonding operation
#define APP_OP_BOND             0x01

/*
 ****************************************************************************************
 * @brief SMPM Set Key request
 *
 ****************************************************************************************
 */
void app_smpm_set_key_req(uint8_t key_code, struct smp_key *key);

/*
 ****************************************************************************************
 * @brief SMPC TK request response
 *
 ****************************************************************************************
 */
void app_smpc_tk_req_rsp(uint8_t idx, uint8_t status, struct smp_key *tk);

/*
 ****************************************************************************************
 * @brief SMPC LTK request response
 *
 ****************************************************************************************
 */
void app_smpc_ltk_req_rsp(uint8_t idx, 
                          uint8_t status,
                          uint8_t sec_prop,
                          uint16_t ediv, 
                          struct rand_nb *rand_nb, 
                          struct smp_key *ltk);

/*
 ****************************************************************************************
 * @brief SMPC IRK request response
 *
 ****************************************************************************************
 */
void app_smpc_irk_req_rsp(uint8_t idx, uint8_t status, struct bd_addr *addr, struct smp_key *irk);

/*
 ****************************************************************************************
 * @brief SMPC CSRK request response
 *
 ****************************************************************************************
 */
void app_smpc_csrk_req_rsp(uint8_t idx, uint8_t status, uint8_t sec_status, struct smp_key *csrk);

/*
 ****************************************************************************************
 * @brief SMPC Check BD Addr request response
 *
 ****************************************************************************************
 */
void app_smpc_chk_bd_addr_req_rsp(uint8_t idx, 
	                                uint8_t found_flag,
	                                uint8_t sec_status,
	                                uint8_t type,
	                                struct bd_addr *addr);
                             
/*
 ****************************************************************************************
 * @brief SMPC encrypt link request
 *
 ****************************************************************************************
 */
void app_smpc_start_enc_req(uint8_t idx, 
                               uint8_t auth_req,
                               uint16_t ediv,
                               struct rand_nb *rand_nb,
                               struct smp_key *ltk);

/// @} APP_SMP_API

#endif // _APP_SMP_H_

