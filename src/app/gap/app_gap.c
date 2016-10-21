/**
 ****************************************************************************************
 *
 * @file app_gap.c
 *
 * @brief Application GAP API
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
 * @addtogroup APP_GAP_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"
#include "lib.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Device name update request.    *//**
 * @param[in] name      Name of the device to set
 * @param[in] len       length for name
 * @response  GAP_SET_DEVNAME_REQ_CMP_EVT
 * @description
 *
 * This function is used to set the device name as seen by remote device.
 *
 ****************************************************************************************
 */
void app_gap_set_devname_req(uint8_t const *name, uint8_t len)
{
    struct gap_set_devname_req *msg = KE_MSG_ALLOC(GAP_SET_DEVNAME_REQ, TASK_GAP, TASK_APP,
                                                   gap_set_devname_req);


    msg->bdname.namelen = len;
    memcpy(msg->bdname.name, name, msg->bdname.namelen);
    
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Set security mode request, change this setting before any Adv/Scan/Connect Action. *//**
 * @param[in] sec_lvl    Security settings to write, possible values are:
 *  - GAP_NO_SEC                     
 *  - GAP_SEC1_NOAUTH_PAIR_ENC       
 *  - GAP_SEC1_AUTH_PAIR_ENC         
 *  - GAP_SEC2_NOAUTH_DATA_SGN       
 *  - GAP_SEC2_AUTH_DATA_SGN         
 * @response  GAP_SET_SEC_REQ_CMP_EVT
 * @description
 *
 * This function is used to set security level of the device. It is advisable to set the
 * security level as soon as the device starts.
 *
 ****************************************************************************************
 */
void app_gap_set_sec_req(uint8_t sec_lvl)
{
    struct gap_set_sec_req *msg = KE_MSG_ALLOC(GAP_SET_SEC_REQ, TASK_GAP, TASK_APP,
                                               gap_set_sec_req);

    /// Operation: read = 0x00, write = 0x01
    msg->flag = 0x01;
    /// Security mode
    msg->sec = sec_lvl;
        
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Read local version request.    *//**
 *
 * @response  GAP_READ_VER_REQ_CMP_EVT
 * @description
 *
 * This function is used to read the version information of the BLE stack.
 *
 ****************************************************************************************
 */
void app_gap_read_ver_req(void)
{
    // Send the message
    ke_msg_send_basic(GAP_READ_VER_REQ, TASK_GAP, TASK_APP);
}

/*
 ****************************************************************************************
 * @brief Read local BD addr request.    *//**
 *
 * @response  GAP_READ_BDADDR_REQ_CMP_EVT
 * @description
 *
 * This function is used to read the Bluetooth Address of the device.
 *
 ****************************************************************************************
 */
void app_gap_read_bdaddr_req(void)
{
    // Send the message
    ke_msg_send_basic(GAP_READ_BDADDR_REQ, TASK_GAP, TASK_APP);
}

/*
 ****************************************************************************************
 * @brief Inquiry the nearby BLE devices.    *//**
 *
 * @param[in] inq_type Inquiry type, possible values are:
 *  - GAP_GEN_INQ_TYPE          
 *  - GAP_LIM_INQ_TYPE          
 *  - GAP_KNOWN_DEV_INQ_TYPE    
 * @param[in] own_addr_type the address type used when inquirying
 *  - ADDR_PUBLIC               
 *  - ADDR_RAND                 
 * @response  GAP_DEV_INQ_REQ_CMP_EVT GAP_DEV_INQ_RESULT_EVT GAP_KNOWN_DEV_DISC_RESULT_EVT
 * @description
 *
 * This function is used to search devices within range.
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_dev_inq_req(uint8_t inq_type, uint8_t own_addr_type)
{
    app_env.inq_idx = 0;
    struct gap_dev_inq_req *msg = KE_MSG_ALLOC(GAP_DEV_INQ_REQ, TASK_GAP, TASK_APP,
                                               gap_dev_inq_req);

    msg->inq_type = inq_type;
    msg->nb_resp = 0;
    msg->own_addr_type = own_addr_type;

    ke_state_set(TASK_APP, APP_SCAN);
    app_task_msg_hdl(GAP_DEV_INQ_REQ, msg);
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Cancle the inquiry the nearby BLE devices process.    *//**
 *
 * @response  GAP_SCAN_REQ_CMP_EVT
 * @description
 *
 * This function is used to stop the current inquiry.
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_dev_inq_cancel_req(void)
{
    struct gap_scan_req *msg = KE_MSG_ALLOC(GAP_SCAN_REQ, TASK_GAP, TASK_APP,
                                                gap_scan_req);

    msg->scan_en.scan_en = SCAN_DIS;
    msg->scan_en.filter_duplic_en = SCAN_FILT_DUPLIC_EN;

    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief scan the nearby BLE devices process.    *//**
 *
 * @param[in] scan_en
 *  - 0 = disabled
 *  - 1 = enabled
 * @response  GAP_ADV_REPORT_EVT GAP_DEV_INQ_REQ_CMP_EVT
 * @description
 *
 * This function is used to start and stop the current inquiry.
 *
 ****************************************************************************************
 */
#if BLE_OBSERVER
void app_gap_scan_req(uint8_t scan_en)
{
    struct gap_scan_req *msg = KE_MSG_ALLOC(GAP_SCAN_REQ, TASK_GAP, TASK_APP,
                                                gap_scan_req);

    msg->scan_en.scan_en = scan_en;
    msg->scan_en.filter_duplic_en = SCAN_FILT_DUPLIC_EN;

    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Set scan mode request from the remote device.    *//**
 *
 * @param[in] scan_type
 *  - 0 = passive
 *  - 1 = active
 * @param[in] scan_intv  scan interval
 * @param[in] scan_window  scan window
 * @response  GAP_SET_MODE_REQ_CMP_EVT
 * @description
 *
 * This function is used to set scan mode.
 *
 ****************************************************************************************
 */
#if BLE_OBSERVER
void app_gap_set_scan_mode_req(uint8_t scan_type, uint16_t scan_intv, uint16_t scan_window)
{
    struct gap_set_mode_req *msg = KE_MSG_ALLOC(GAP_SET_MODE_REQ, TASK_GAP, TASK_APP,
                                                gap_set_mode_req);

    msg->mode = GAP_OBSERVER;
    msg->set_scan.set_scan.own_addr_type = QN_ADDR_TYPE;
    msg->set_scan.set_scan.scan_filt_policy = 0;
    msg->set_scan.set_scan.scan_intv = scan_intv;
    msg->set_scan.set_scan.scan_type = scan_type;
    msg->set_scan.set_scan.scan_window = scan_window;
 
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Get device name request from the remote device.    *//**
 *
 * @param[in] p_addr Pointer to device address of peer
 * @param[in] addr_type Device address type of peer, possible values are:
 *  - ADDR_PUBLIC       
 *  - ADDR_RAND         
 * @param[in] own_addr_type Own address type, possible values are:
 *  - ADDR_PUBLIC       
 *  - ADDR_RAND         
 * @response  GAP_NAME_REQ_CMP_EVT
 * @description
 *
 * This function is used to find out the user friendly name of peer device. 
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_name_req(struct bd_addr *p_addr, uint8_t addr_type, uint8_t own_addr_type)
{
    struct gap_name_req *msg = KE_MSG_ALLOC(GAP_NAME_REQ, TASK_GAP, TASK_APP,
                                               gap_name_req);

    // GAP_CONN_MAX_CE 
    msg->create_cnx.ce_len_max = GAP_CONN_MAX_CE;
    // GAP_CONN_MIN_CE
    msg->create_cnx.ce_len_min = GAP_CONN_MIN_CE;
    // GAP_INIT_CONN_MAX_INTV 
    msg->create_cnx.con_intv_max = GAP_INIT_CONN_MAX_INTV;
    // GAP_INIT_CONN_MIN_INTV 
    msg->create_cnx.con_intv_min = GAP_INIT_CONN_MIN_INTV;
    // GAP_CONN_LATENCY 
    msg->create_cnx.con_latency = GAP_CONN_LATENCY;
    // INIT_FILT_IGNORE_WLST
    msg->create_cnx.init_filt_policy = INIT_FILT_IGNORE_WLST;
    // ADDR_PUBLIC, ADDR_RAND
    msg->create_cnx.own_addr_type = own_addr_type;
    // Address 
    memcpy(msg->create_cnx.peer_addr.addr, p_addr->addr, BD_ADDR_LEN);
    // Address type
    msg->create_cnx.peer_addr_type = addr_type;
    // GAP_SCAN_FAST_INTV 
    msg->create_cnx.scan_intv = GAP_SCAN_FAST_INTV;
    // GAP_SCAN_FAST_WIND
    msg->create_cnx.scan_window = GAP_SCAN_FAST_WIND;
    // GAP_CONN_SUPERV_TIMEOUT, 
    msg->create_cnx.superv_to = GAP_CONN_SUPERV_TIMEOUT;

    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Bond request with the remote BLE device.    *//**
 *
 * @param[in] addr Pointer to device address of peer
 * @param[in] oob Out-Of-Band present flag, possible values are:
 * - SMP_OOB_AUTH_DATA_NOT_PRESENT                  
 * - SMP_OOB_AUTH_DATA_FROM_REMOTE_DEV_PRESENT      
 * @param[in] auth Authentication requirements, possible values are:
 * - SMP_AUTH_REQ_NO_MITM_NO_BOND       
 * - SMP_AUTH_REQ_NO_MITM_BOND          
 * - SMP_AUTH_REQ_MITM_NO_BOND          
 * - SMP_AUTH_REQ_MITM_BOND             
 * @param[in] iocap Input and output capabilities of local device, possible values are:
 * - SMP_IO_CAP_DISPLAY_ONLY            
 * - SMP_IO_CAP_DISPLAY_YES_NO          
 * - SMP_IO_CAP_KB_ONLY                 
 * - SMP_IO_CAP_NO_INPUT_NO_OUTPUT      
 * - SMP_IO_CAP_KB_DISPLAY              
 * @response  GAP_BOND_REQ_CMP_EVT
 * @description
 *
 * This function is used to initiate bonding procedure.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_gap_bond_req(struct bd_addr *addr, uint8_t oob, uint8_t auth, uint8_t iocap)
{
    app_env.bond_flag |= APP_OP_BOND;
    struct gap_bond_req *msg = KE_MSG_ALLOC(GAP_BOND_REQ, TASK_GAP, TASK_APP,
                                               gap_bond_req);

    /// BT Address to bond
    msg->addr = *addr;
    /// OOB information, SMP_OOB_AUTH_DATA_NOT_PRESENT or SMP_OOB_AUTH_DATA_FROM_REMOTE_DEV_PRESENT
    msg->oob = oob;
    /// Authentication Requirements
    msg->auth = auth;
    /// IO capabilities
    msg->iocap = iocap;
    /// Encryption key size
    msg->key_size = SMPC_MAX_ENC_SIZE_LEN;
#if (QN_CSRK_SUPPORT)
    /// Initiator key distribution
    msg->ikey_dist = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY | SMP_KDIST_SIGNKEY;
    /// Responder key distribution
    msg->rkey_dist = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY | SMP_KDIST_SIGNKEY;
#else
    /// Initiator key distribution
    msg->ikey_dist = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY;
    /// Responder key distribution
    msg->rkey_dist = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY;
#endif
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Bond response to the remote BLE device.    *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] reject Decision to accept or reject the bond request. 0x00: accept 0x01: reject
 * @param[in] oob Out-Of-Band present flag, possible values are:
 * - SMP_OOB_AUTH_DATA_NOT_PRESENT                  
 * - SMP_OOB_AUTH_DATA_FROM_REMOTE_DEV_PRESENT      
 * @param[in] auth Authentication requirements, possible values are:
 * - SMP_AUTH_REQ_NO_MITM_NO_BOND       
 * - SMP_AUTH_REQ_NO_MITM_BOND          
 * - SMP_AUTH_REQ_MITM_NO_BOND          
 * - SMP_AUTH_REQ_MITM_BOND             
 * @param[in] iocap Input and output capabilities of local device, possible values are:
 * - SMP_IO_CAP_DISPLAY_ONLY            
 * - SMP_IO_CAP_DISPLAY_YES_NO          
 * - SMP_IO_CAP_KB_ONLY                
 * - SMP_IO_CAP_NO_INPUT_NO_OUTPUT     
 * - SMP_IO_CAP_KB_DISPLAY             
 * @response  None
 * @description
 *
 * This function is used to answer to bond request from peer device.
 *
 ****************************************************************************************
 */
void app_gap_bond_resp(uint16_t conhdl, uint8_t reject, uint8_t oob, uint8_t auth, uint8_t iocap)
{
    struct gap_bond_resp *msg = KE_MSG_ALLOC(GAP_BOND_RESP, TASK_GAP, TASK_APP,
                                               gap_bond_resp);

    /// Connection handle
    msg->conhdl = conhdl;
    /// accept or reject bonding
    msg->accept = reject;
    /// OOB flag
    msg->oob = SMP_OOB_AUTH_DATA_NOT_PRESENT;
    /// Authentication Requirements
    msg->auth_req = auth;
    /// IO capabilities
    msg->io_cap = iocap;
    /// Max key size
    msg->max_key_size = 0x10;
#if (QN_CSRK_SUPPORT)
    /// Initiator key distribution
    msg->ikeys = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY;
    /// Responder key distribution
    msg->rkeys = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY;
#else
    /// Initiator key distribution
    msg->ikeys = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY | SMP_KDIST_SIGNKEY;
    /// Responder key distribution
    msg->rkeys = SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY | SMP_KDIST_SIGNKEY;
#endif
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Unpair with desired BD address.    *//**
 *
 * @param[in] addr Desired BD address to be removed in the bond list
 * @param[in] nb_bond Number of bonded devices
 * @response  None
 * @description
 *
 * This function is used to update the stack's bonding information.
 *
 ****************************************************************************************
 */
void app_gap_unpair_req(struct bd_addr *addr, uint8_t nb_bond)
{
    // Put the device in general discoverable and undirected connectable mode
    struct gap_bond_info_ind *msg = KE_MSG_ALLOC(GAP_BOND_INFO_IND, TASK_GAP, TASK_APP,
                                                      gap_bond_info_ind);
    /// operation: 0x00: added, 0x01: removed
    msg->bond_op = 0x01;
    /// number of bonding info present
    msg->nb_bond = nb_bond;
    /// Device BD Address
    msg->addr = *addr;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Initiate a encryption or pairing procedure.        *//**
 *
 * @param[in] addr Desired BD address to be removed in the bond list
 * @response  GAP_BOND_REQ_CMP_EVT or SMPC_SEC_STARTED_IND
 * @description
 *
 * This function is used to initiate a encryption or pairing procedure. For a unbonded device,
 * this will initiate pairing, or this will initiate encryption.
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_gap_security_req(struct bd_addr *addr)
{
    uint8_t bonded_dev_idx = 0;
    uint8_t idx = 0;

    if (app_get_conhdl_by_bdaddr(addr, &idx) == GAP_INVALID_CONIDX)
        return;

    if ((bonded_dev_idx = app_find_bonded_dev(addr)) == GAP_INVALID_CONIDX)
    {
        app_gap_bond_req(addr, SMP_OOB_AUTH_DATA_NOT_PRESENT,
                         SMP_AUTH_REQ_NO_MITM_BOND, SMP_IO_CAP_NO_INPUT_NO_OUTPUT);
    }
    else
    {
        if (GAP_CENTRAL_MST == app_get_role())
        {
            app_smpc_start_enc_req(idx,
                                   app_env.bonded_info[bonded_dev_idx].sec_prop,
                                   app_env.bonded_info[bonded_dev_idx].pair_info.ediv,
                                   &app_env.bonded_info[bonded_dev_idx].pair_info.rand_nb,
                                   &app_env.bonded_info[bonded_dev_idx].pair_info.ltk);  
        }
        else // use own LTK as slave
        {
            app_smpc_start_enc_req(idx,
                                   app_env.bonded_info[bonded_dev_idx].sec_prop,
                                   app_env.ediv,
                                   (struct rand_nb *)&app_env.rand,
                                   (struct smp_key *)&app_env.ltk);
        }
    }
}
#endif

/*
 ****************************************************************************************
 * @brief Create connection with the remote BLE device.        *//**
 *
 * @param[in] addr The address of the remote device to which the connection will be created
 * @param[in] addr_type The address type of the remote device, possible values are:
 * - ADDR_PUBLIC        
 * - ADDR_RAND          
 * @param[in] own_addr_type Own address type, possible values are:
 * - ADDR_PUBLIC        
 * - ADDR_RAND          
 * @param[in] conn_intv_min  Minimum of connection interval
 * @param[in] conn_intv_max  Maximum of connection interval
 * @param[in] cnnn_timeout   Link supervision timeout
 * @response  GAP_LE_CREATE_CONN_REQ_CMP_EVT
 * @description
 *
 * This function is used to create a Link Layer connection to a connectable device.
 * This is initiated by central device, which will become the master of the link.
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_le_create_conn_req(struct bd_addr *addr, uint8_t addr_type, uint8_t own_addr_type,
                                uint16_t conn_intv_min, uint16_t conn_intv_max, uint16_t cnnn_timeout)
{
    struct gap_le_create_conn_req *msg = KE_MSG_ALLOC(GAP_LE_CREATE_CONN_REQ, TASK_GAP, TASK_APP,
                                                      gap_le_create_conn_req);
    // GAP_CONN_MAX_CE 
    msg->create_cnx.ce_len_max = GAP_CONN_MAX_CE;
    // GAP_CONN_MIN_CE
    msg->create_cnx.ce_len_min = GAP_CONN_MIN_CE;
    // GAP_INIT_CONN_MAX_INTV, 
    msg->create_cnx.con_intv_max = conn_intv_max;
    // GAP_INIT_CONN_MIN_INTV, 
    msg->create_cnx.con_intv_min = conn_intv_min;
    // Connection slave latency 
    msg->create_cnx.con_latency = GAP_CONN_LATENCY;
    // INIT_FILT_IGNORE_WLST
    msg->create_cnx.init_filt_policy = INIT_FILT_IGNORE_WLST;
    // ADDR_PUBLIC, ADDR_RAND
    msg->create_cnx.own_addr_type = own_addr_type;
    // Address 
    msg->create_cnx.peer_addr = *addr;
    // Address type
    msg->create_cnx.peer_addr_type = addr_type;
    // GAP_SCAN_FAST_INTV 
    msg->create_cnx.scan_intv = GAP_SCAN_FAST_INTV;
    // GAP_SCAN_FAST_WIND
    msg->create_cnx.scan_window = GAP_SCAN_FAST_WIND;
    // GAP_CONN_SUPERV_TIMEOUT, 
    msg->create_cnx.superv_to = cnnn_timeout;

    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Cancel an existing connection request.        *//**
 *
 * @response  GAP_CANCEL_CONN_REQ_CMP_EVT
 * @description
 *
 * This function is used to cancel an existing connection request. This function shall only
 * be called after the GAP_LE_CREATE_CONN_REQ message has been issued and before
 * GAP_LE_CREATE_CONN_REQ_CMP_EVT message.
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_le_cancel_conn_req(void)
{
    // Send the message
    ke_msg_send_basic(GAP_LE_CANCEL_CONN_REQ, TASK_GAP, TASK_APP);
}
#endif

/*
 ****************************************************************************************
 * @brief Disconnect the current connection with the remote BLE device.        *//**
 *
 * @param[in] conhdl The connection handle of the connection is to be disconnected.
 * @response  GAP_DISCON_CMP_EVT
 * @description
 *
 * This function is used to disconnect an existing BLE connection.
 *
 ****************************************************************************************
 */
void app_gap_discon_req(uint16_t conhdl)
{
    struct gap_discon_req *msg = KE_MSG_ALLOC(GAP_DISCON_REQ, TASK_GAP, TASK_APP,
                                                gap_discon_req);

    msg->conhdl = conhdl;
    msg->reason = CO_ERROR_REMOTE_USER_TERM_CON;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Set the device to bondable mode.        *//**
 *
 * @response  GAP_SET_MODE_REQ_CMP_EVT
 * @description
 *
 * This function is used to set the device to bondable mode.
 *
 ****************************************************************************************
 */
void app_gap_set_bondable_mode_req()
{
    struct gap_set_mode_req *msg = KE_MSG_ALLOC(GAP_SET_MODE_REQ, TASK_GAP, TASK_APP,
                                                gap_set_mode_req);

    msg->mode = GAP_BONDABLE;
 
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the device to advertising process.        *//**
 *
 * @param[in] mode Device mode to set, possible values are:
 * - GAP_NON_DISCOVERABLE     
 * - GAP_GEN_DISCOVERABLE     
 * - GAP_LIM_DISCOVERABLE     
 * - GAP_NON_CONNECTABLE      
 * - GAP_UND_CONNECTABLE      
 * - GAP_DIR_CONNECTABLE      
 * @param[in] adv_data Pointer to advertising data used in the advertising packets
 * @param[in] adv_data_len The length of advertising data
 * @param[in] scan_rsp_data Pointer to Scan Response data used in the advertising packets
 * @param[in] scan_rsp_data_len The length of Scan Response data
 * @param[in] adv_intv_min Minimum interval for advertising
 * @param[in] adv_intv_max Maximum interval for advertising
 * @response  GAP_SET_MODE_REQ_CMP_EVT
 * @description
 *
 * This function is used to set the device to advertising.
 *
 * @note
 * The stack will keep advertising with new parameters if calling this function in advertising state.
 * The adv_intv_min and adv_intv_max shall not be set to less than 0x00A0(100 ms) if the mode 
 * is GAP_NON_DISCOVERABLE.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL || BLE_BROADCASTER)
void app_gap_adv_start_req(uint16_t mode, uint8_t *adv_data, uint8_t adv_data_len, 
                           uint8_t *scan_rsp_data, uint8_t scan_rsp_data_len,
                           uint16_t adv_intv_min, uint16_t adv_intv_max)
{
    struct gap_set_mode_req *msg = KE_MSG_ALLOC(GAP_SET_MODE_REQ, TASK_GAP, TASK_APP,
                                                gap_set_mode_req);

    msg->mode = mode;
    store_ble_dev_mode_flag(mode);
    if (mode & GAP_UND_CONNECTABLE)
    {
        /* Mode in undirected connectable */
        ///Advertising type
        msg->adv_info.adv_param.adv_type = ADV_CONN_UNDIR;
    }
    else if (mode & GAP_DIR_CONNECTABLE)
    {
        /* Mode in directed connectable */
        ///Advertising type
        msg->adv_info.adv_param.adv_type = ADV_CONN_DIR;
    }
    else
    {
        /* Mode in non-connectable */
        ///Advertising type
        msg->adv_info.adv_param.adv_type = 
            (mode & GAP_NON_DISCOVERABLE) ? ADV_NONCONN_UNDIR : ADV_DISC_UNDIR;
    }

    ///Minimum interval for advertising
    msg->adv_info.adv_param.adv_intv_min = adv_intv_min;
    ///Maximum interval for advertising
    msg->adv_info.adv_param.adv_intv_max = adv_intv_max;
    /// Own address type: public=0x00 /random = 0x01
    msg->adv_info.adv_param.own_addr_type = QN_ADDR_TYPE;
    /// Advertising channel map
    msg->adv_info.adv_param.adv_chnl_map = ADV_ALL_CHNLS_EN;
    /// Advertising filter policy
    msg->adv_info.adv_param.adv_filt_policy = 0;

    /// Advertising data structure
    msg->adv_info.adv_data.adv_data_len = adv_data_len;
    if (adv_data)
        memcpy(&msg->adv_info.adv_data.data, adv_data, adv_data_len);
    /// Scan response data structure
    msg->adv_info.scan_rsp_data.scan_rsp_data_len = scan_rsp_data_len;
    if (scan_rsp_data)
        memcpy(&msg->adv_info.scan_rsp_data.data, scan_rsp_data, scan_rsp_data_len);
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Stop the advertising process.        *//**
 *
 * @response  GAP_ADV_REQ_CMP_EVT
 * @description
 *
 * This function is used to stop advertising.
 *
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL || BLE_BROADCASTER)
void app_gap_adv_stop_req(void)
{
    struct gap_adv_req *msg = KE_MSG_ALLOC(GAP_ADV_REQ, TASK_GAP, TASK_APP, gap_adv_req);

    msg->adv_en = ADV_DIS;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Read the white list size.        *//**
 *
 * @response GAP_LE_RD_WLST_SIZE_CMD_CMP_EVT
 * @description
 *
 * This function is used to read the total number of white list entries that can be stored
 * in the BLE chip.
 *
 ****************************************************************************************
 */
#if (QN_WL_SUPPORT)
void app_gap_le_rd_wlst_size_req(void)
{
    // Send the message
    ke_msg_send_basic(GAP_LE_RD_WLST_SIZE_REQ, TASK_GAP, TASK_APP);
}
#endif

/*
 ****************************************************************************************
 * @brief Add device to white list.        *//**
 *
 * @param[in] addr_type The address type of the stored device, possible values are:
 * - ADDR_PUBLIC       
 * - ADDR_RAND         
 * @param[in] addr Pointer to device address to be stored
 * @response GAP_LE_ADD_DEV_TO_WLST_REQ_CMP_EVT
 * @description
 *
 * This function is used to add a single device to the white list stored in the BLE chip.
 *
 ****************************************************************************************
 */
#if (QN_WL_SUPPORT)
void app_gap_le_add_dev_to_wlst_req(uint8_t addr_type, struct bd_addr *addr)
{
    struct gap_le_add_dev_to_wlst_req *msg = KE_MSG_ALLOC(GAP_LE_ADD_DEV_TO_WLST_REQ, TASK_GAP, TASK_APP,
                                                          gap_le_add_dev_to_wlst_req);

    msg->add_dev_to_wlst.dev_addr_type = addr_type;
    msg->add_dev_to_wlst.dev_addr = *addr;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Remove or clear device(s) in the white list.        *//**
 *
 * @param[in] all_dev Flag to determine if all the devices will be removed from the white 
 * list or only the one specified in the structure
 * @param[in] addr_type The address type of the removed device, possible values are:
 * - ADDR_PUBLIC      
 * - ADDR_RAND        
 * @param[in] addr Pointer to device address to be removed
 * @response GAP_LE_RMV_DEV_FRM_WLST_REQ_CMP_EVT
 * @description
 *
 * This function is used to remove device from white list stored in the BLE chip.
 *
 ****************************************************************************************
 */
#if (QN_WL_SUPPORT)
void app_gap_le_rmv_dev_frm_wlst_req(bool all_dev, uint8_t addr_type, struct bd_addr *addr)
{
    struct gap_le_rmv_dev_frm_wlst_req *msg = KE_MSG_ALLOC(GAP_LE_RMV_DEV_FRM_WLST_REQ, TASK_GAP, TASK_APP,
                                                           gap_le_rmv_dev_frm_wlst_req);

    msg->all_dev = all_dev;
    msg->rmv_dev_from_wlst.dev_addr_type = addr_type;
    msg->rmv_dev_from_wlst.dev_addr = *addr;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Read remote features.        *//**
 *
 * @param[in] conhdl The connection handle of the connection is to be read
 * @response GAP_LE_RD_REMOTE_FEAT_REQ_CMP_EVT
 * @description
 *
 * This function is used to request a list of the used LE features from the remote device.
 *
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
void app_gap_le_rd_remote_feat_req(uint16_t conhdl)
{
    struct gap_le_rd_rem_used_feats_req *msg = KE_MSG_ALLOC(GAP_LE_RD_REMOTE_FEAT_REQ, TASK_GAP, TASK_APP,
                                                            gap_le_rd_rem_used_feats_req);

    msg->rd_rem_used_feats.conhdl = conhdl;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Read version information of peer device.        *//**
 *
 * @param[in] conhdl Specifies which Connection Handles version information to get
 * @response GAP_RD_REM_VER_INFO_CMP_EVT
 * @description
 *
 * This function is used to obtain the values for the version information for the remote
 * device.
 *
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
void app_gap_rd_rem_ver_info_req(uint16_t conhdl)
{
    struct gap_rd_rem_ver_info_req *msg = KE_MSG_ALLOC(GAP_RD_REM_VER_INFO_REQ, TASK_GAP, TASK_APP,
                                                       gap_rd_rem_ver_info_req);

    msg->ver.conhdl = conhdl;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Set the random device address.        *//**
 *
 * @param[in] addr Pointer to device address to be set
 * @response GAP_SET_RANDOM_ADDR_REQ_CMP_EVT
 * @description
 *
 * This function is used to set the Random Device Address in the BLE chip.
 *
 ****************************************************************************************
 */
void app_gap_set_random_addr_req(struct bd_addr *addr)
{
    struct gap_set_random_addr_req *msg = KE_MSG_ALLOC(GAP_SET_RANDOM_ADDR_REQ, TASK_GAP, TASK_APP,
                                                       gap_set_random_addr_req);

    msg->addr = *addr;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Slave request for connection parameter change.        *//**
 *
 * @param[in] conhdl Connection handle to be used to identify a connection
 * @param[in] conn_par Pointer to the stuct gap_conn_param_update containing connection parameters
 * @response
 * - GAP_PARAM_UPDATE_RESP
 * - GAP_CHANGE_PARAM_REQ_CMP_EVT
 * @description
 *
 * This function is used to change a set of new connection parameters.
 * The peripheral is the only one that can send this request.
 *
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
void app_gap_param_update_req(uint16_t conhdl, struct gap_conn_param_update *conn_par)
{
    struct gap_param_update_req *msg = KE_MSG_ALLOC(GAP_PARAM_UPDATE_REQ, TASK_GAP, TASK_APP,
                                                    gap_param_update_req);

    msg->conhdl = conhdl;
    msg->conn_par = *conn_par;

    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Master initiates a change in connection parameter.        *//**
 *
 * @param[in] conhdl Connection handle to be used to identify a connection
 * @param[in] result Result of the connection parameters request, 0x0000: accept 0x0001: reject
 * @param[in] conn_par Pointer to the stuct gap_conn_param_update containing connection parameters
 * @response GAP_CHANGE_PARAM_REQ_CMP_EVT
 * @description
 *
 * This function is used to send parameters update change by master.
 * @note This function is called in two occasions: 1. Message GAP_PARAM_UPDATE_REQ_IND will be 
 * a hint that slave requests connection parameters update. At this time, the parameter of 
 * result is meaningful. 2. Device(master) wants to change the current connection parameters.
 * 
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_change_param_req(uint16_t conhdl, uint16_t result, struct gap_conn_param_update *conn_par)
{
    struct gap_change_param_req *msg = KE_MSG_ALLOC(GAP_CHANGE_PARAM_REQ, TASK_GAP, TASK_APP,
                                                    gap_change_param_req);

    msg->conhdl = conhdl;
    msg->result = result;
    msg->conn_par = *conn_par;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Reset the link layer and the host.        *//**
 *
 * @response GAP_RESET_REQ_CMP_EVT
 * @description
 *
 * This function is used to reset the device and the BLE stack will be initialized.
 * 
 ****************************************************************************************
 */
void app_gap_reset_req(void)
{
    // Send the message
    ke_msg_send_basic(GAP_RESET_REQ, TASK_GAP, TASK_APP);
}

/*
 ****************************************************************************************
 * @brief Set Reconnection Address of the the peripheral device, only for central. *//**
 *
 * @param[in] conhdl Connection handle to be used to identify a connection
 * @param[in] attrhdl Attribute handle of the reconnection address
 * @response GAP_SET_RECON_ADDR_REQ_CMP_EVT
 * @description
 *
 * This function is used to set the reconnection address attribute of the peripheral device
 * by the central device.
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_set_recon_addr_req(uint16_t conhdl, uint16_t attrhdl)
{
    struct gap_set_recon_addr_req *msg = KE_MSG_ALLOC(GAP_SET_RECON_ADDR_REQ, TASK_GAP, TASK_APP,
                                                      gap_set_recon_addr_req);

    msg->conhdl = conhdl;
    msg->attrhdl = attrhdl;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Set the peer Privacy Flag feature, only for central. *//**
 *
 * @param[in] enable Specify to enable or disable the privacy flag in the attribute
 * @param[in] conhdl Connection handle to be used to identify a connection
 * @param[in] attrhdl Attribute handle of the reconnection address
 * @response GAP_SET_PH_PRIVACY_REQ_CMP_EVT
 * @description
 *
 * This function is used to set the privacy settings of the peer peripheral device.
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_gap_set_ph_privacy_req(uint16_t enable, uint16_t conhdl, uint16_t attrhdl)
{
    struct gap_set_ph_privacy_req *msg = KE_MSG_ALLOC(GAP_SET_PH_PRIVACY_REQ, TASK_GAP, TASK_APP,
                                                      gap_set_ph_privacy_req);

    msg->enable = enable;
    msg->conhdl = conhdl;
    msg->attrhdl = attrhdl;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Set privacy preference.    *//**
 *
 * @param[in] priv_flag  privacy flag, possible values are:
 * - DEV_PRIV_DIS          
 * - CT_PRIV_EN            
 * - PH_PRIV_EN            
 * - BCST_PRIV_EN          
 * - OBS_PRIV_EN           
 * - OBS_PRIV_RESOLVE      
 * @param[in] recon_addr_visible Reconnection address visible flag, valid only for Peripheral
 * - 0x00 Hide reconnection address
 * - 0x01 Show/Expose reconnection address
 * @param[in] set_to_ll Flag to set the generated random address to link layer, true or false
 * @response GAP_SET_PRIVACY_REQ_CMP_EVT
 * @description
 *
 * This function is used to enable privacy feature of the local device.
 *
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
void app_gap_set_privacy_req(uint8_t priv_flag, uint8_t recon_addr_visible, uint8_t set_to_ll)
{
    struct gap_set_privacy_req *msg = KE_MSG_ALLOC(GAP_SET_PRIVACY_REQ, TASK_GAP, TASK_APP,
                                                   gap_set_privacy_req);

    msg->priv_flag = priv_flag;
    msg->recon_addr_visible = recon_addr_visible;
    msg->set_to_ll = set_to_ll;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Set or retrieve channel map.    *//**
 *
 * @param[in] update_map Flag to either read the map or update it, false:read, true: update
 * @param[in] conhdl Connection handle to be used to read a channel map
 * @param[in] chmap Pointer to the struct le_chnl_map value which is used to update
 * @response GAP_CHANNEL_MAP_CMP_EVT
 * @description
 *
 * This function is used to read the current channel map or change it with the new channel
 * map in the command parameter.
 *
 * @note
 * The Channel Map shall only be updated when the local device supports the Master role.
 *
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
void app_gap_channel_map_req(bool update_map, uint16_t conhdl, struct le_chnl_map *chmap)
{
    struct gap_channel_map_req *msg = KE_MSG_ALLOC(GAP_CHANNEL_MAP_REQ, TASK_GAP, TASK_APP,
                                                   gap_channel_map_req);

    msg->update_map = update_map;
    msg->conhdl = conhdl;
    msg->chmap = *chmap;

    // Send the message
    ke_msg_send(msg);
}
#endif

/*
 ****************************************************************************************
 * @brief Read RSSI value.    *//**
 *
 * @param[in] conhdl The Handle for the connection for which the RSSI is to be read
 * @response GAP_READ_RSSI_REQ_CMP_EVT
 * @description
 *
 * This function is used to read Received Signal Strength Indication (RSSI) value.
 *
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
void app_gap_read_rssi_req(uint16_t conhdl)
{
    struct gap_read_rssi_req *msg = KE_MSG_ALLOC(GAP_READ_RSSI_REQ, TASK_GAP, TASK_APP,
                                                 gap_read_rssi_req);

    msg->conhdl = conhdl;
    
    // Send the message
    ke_msg_send(msg);
}
#endif

/// @} APP_GAP_API
