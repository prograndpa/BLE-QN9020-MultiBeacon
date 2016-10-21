/**
 ****************************************************************************************
 *
 * @file gap.h
 *
 * @brief Header file - GAP.
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
 * @file gap.h
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
#ifndef GAP_H_
#define GAP_H_
/**
 ****************************************************************************************
 * @addtogroup HOST
 * @ingroup ROOT
 * @brief Bluetooth Low Energy Host
 *
 * The HOST layer of the stack contains the higher layer protocols (@ref ATT "ATT",
 * @ref SMP "SMP") and transport module (@ref L2C "L2C"). It also includes the Generic
 * Access Profile (@ref GAP "GAP"), used for scanning/connection operations.
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @addtogroup GAP Generic Access Profile
 * @ingroup HOST
 * @brief Generic Access Profile.
 *
 * The GAP module is responsible for providing an API to the application in order to
 * configure the device in the desired mode (discoverable, connectable, etc.) and perform
 * required actions (scanning, connection, pairing, etc.). To achieve this, the GAP
 * interfaces with both the @ref SMP "SMP", @ref L2C "L2C" and the @ref CONTROLLER "CONTROLLER"
 *
 * @{
 ****************************************************************************************
 */
#include "ke_task.h"
#include "gap_task.h"
#include "gap_cfg.h"
#include "atts.h"
#include "atts_db.h"
#include "prf_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// GAP Advertising Flags
enum
{
    /// Flag
    GAP_AD_TYPE_FLAGS                      = 0x01,
    /// Use of more than 16 bits UUID
    GAP_AD_TYPE_MORE_16_BIT_UUID,
    /// Complete list of 16 bit UUID
    GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    /// Use of more than 32 bit UUD
    GAP_AD_TYPE_MORE_32_BIT_UUID,
    /// Complete list of 32 bit UUID
    GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID,
    /// Use of more than 128 bit UUID
    GAP_AD_TYPE_MORE_128_BIT_UUID,
    /// Complete list of 128 bit UUID
    GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    /// Shortened device name
    GAP_AD_TYPE_SHORTENED_NAME,
    /// Complete device name
    GAP_AD_TYPE_COMPLETE_NAME,
    /// Transmit power
    GAP_AD_TYPE_TRANSMIT_POWER,
    /// Class of device
    GAP_AD_TYPE_CLASS_OF_DEVICE            = 0x0D,
    /// Simple Pairing Hash C
    GAP_AD_TYPE_SP_HASH_C,
    /// Simple Pairing Randomizer
    GAP_AD_TYPE_SP_RANDOMIZER_R,
    /// Temporary key value
    GAP_AD_TYPE_TK_VALUE,
    /// Out of Band Flag
    GAP_AD_TYPE_OOB_FLAGS,
    /// Slave connection interval range
    GAP_AD_TYPE_SLAVE_CONN_INT_RANGE,
    /// Signed data
    GAP_AD_TYPE_SIGNED_DATA,
    /// Complete list of 16 bit service UUID
    GAP_AD_TYPE_CMPLT_LST_16_BIT_SVC_UUID,
    /// Require 128 bit service UUID
    GAP_AD_TYPE_RQRD_128_BIT_SVC_UUID,
    /// Service data
    GAP_AD_TYPE_SERVICE_DATA,
    /// Manufacturer specific data
    GAP_AD_TYPE_MANU_SPECIFIC_DATA         = 0xFF,
};

/// Device discovery search type
enum
{
    /// General Inquiry
    GAP_GEN_INQ_TYPE    = 0x00,
    /// Limited Inquiry
    GAP_LIM_INQ_TYPE,
    /// Known device Inquiry
    GAP_KNOWN_DEV_INQ_TYPE
};

/// Privacy settings
enum
{
    /// Device privacy disabled
    DEV_PRIV_DIS = 0x00,
    /// Central privacy enabled
    CT_PRIV_EN,
    /// Peripheral privacy enabled
    PH_PRIV_EN,
    /// Broadcaster privacy enabled
    BCST_PRIV_EN,
    /// Observer privacy enabled
    OBS_PRIV_EN,
    /// Observer will resolve address
    OBS_PRIV_RESOLVE,
};

/// Security Defines
enum
{
    /// No security (no authentication and encryption)
    GAP_NO_SEC = 0x00,
    /// Unauthenticated pairing with encryption
    GAP_SEC1_NOAUTH_PAIR_ENC,
    /// Authenticated pairing with encryption
    GAP_SEC1_AUTH_PAIR_ENC,
    /// Unauthenticated pairing with data signing
    GAP_SEC2_NOAUTH_DATA_SGN,
    /// Authentication pairing with data signing
    GAP_SEC2_AUTH_DATA_SGN,
    /// Unrecognized security
    GAP_SEC_UNDEFINED
};

/// Random Address type
enum
{
    /// Static random address
    GAP_STATIC_ADDR = 0x00,
    /// Private non resolvable address
    GAP_NON_RSLV_ADDR,
    /// Private resolvable address
    GAP_RSLV_ADDR,
    /// Reconnection address (peripheral only)
    GAP_RECON_ADDR
};

/// GAP Specific Error
enum
{
    /// Invalid parameters set
    GAP_INVALID_PARAM_ERR   = 0x40,
    /// Automatic establishment error
    GAP_AUTO_EST_ERR,
    /// Selective establishment error
    GAP_SELECT_EST_ERR,
    /// Error setting the reconnection address
    GAP_SET_RECON_ADDR_ERR,
    /// Error setting the privacy feature
    GAP_SET_PRIVACY_FEAT_ERR,
    /// Invalid CSRK, signing counter
    GAP_INVALID_CSRK_ERR,
    /// No attribute client defined
    GAP_ATTRIBUTE_CLIENT_MISSING,
    /// No attribute server defined
    GAP_ATTRIBUTE_SERVER_MISSING,
    /// Ill-formed advertising data
    GAP_BAD_ADV_DATA,
};

/// Link security status. This status represents the authentication/authorization/bonding levels of the connection
//bit encoding for 1 byte size
enum
{
    ///No link security status
    GAP_LK_SEC_NONE        = 0x00,
    ///Bit 0: 1=unauthenticated
    GAP_LK_UNAUTHENTICATED = 0x01,
    ///Bit 1: 1=authenticated (if both bit 0 and 1 are 0, means link is plain)
    GAP_LK_AUTHENTICATED   = 0x02,
    ///Bit 2: 1=authorized, 0=unauthorized
    GAP_LK_AUTHORIZED      = 0x04,
    ///Bit 3: 1=bonded, 0=non bonded
    GAP_LK_BONDED          = 0x08,
};

/****************** GAP Role **********************/
/// Master/Central role
#define GAP_CENTRAL_MST                         0x00
/// Peripheral/Slave role
#define GAP_PERIPHERAL_SLV                      0x01
/// Broadcaster role
#define GAP_BROADCASTER_ADV                     0x10
/// Observer role
#define GAP_OBSERVER_SCA                        0x20

/***** GAP Discover-Bond ability Flag - Bit set *******/
/// Mode in non-discoverable
#define GAP_NON_DISCOVERABLE                    0x0001
/// Mode in general discoverable
#define GAP_GEN_DISCOVERABLE                    0x0002
/// Mode in limited discoverable
#define GAP_LIM_DISCOVERABLE                    0x0004
/// Mode in non-connectable
#define GAP_NON_CONNECTABLE                     0x0010
/// Mode in undirected connectable
#define GAP_UND_CONNECTABLE                     0x0020
/// Mode in directed connectable
#define GAP_DIR_CONNECTABLE                     0x0040
/// Mode in non-bondable
#define GAP_NON_BONDABLE                        0x0100
/// Mode in bondability
#define GAP_BONDABLE                            0x0200
/// Mode in broadcaster
#define GAP_BROADCASTER                         0x0400
/// Mode in observer
#define GAP_OBSERVER                            0x0800
/// Mode in auto connection establishment
#define GAP_AUTO_CONNECT                        0x1000
/// Mode in selective connection establishment
#define GAP_SELECT_CONNECT                      0x2000
/// Mode in non-bondable, non-connectable, non-discoverable
#define GAP_NON_BOND_DISC_CONN                  0x0111
/// Limited discovery flag - AD Flag
#define GAP_LE_LIM_DISCOVERABLE_FLG             0x01
/// General discovery flag - AD Flag
#define GAP_LE_GEN_DISCOVERABLE_FLG             0x02
/// Legacy BT not supported - AD Flag
#define GAP_BR_EDR_NOT_SUPPORTED                0x04
/// Dual mode for controller supported (BR/EDR/LE) - AD Flag
#define GAP_SIMUL_BR_EDR_LE_CONTROLLER          0x08
/// Dual mode for host supported (BR/EDR/LE) - AD Flag
#define GAP_SIMUL_BR_EDR_LE_HOST                0x10

/*********** GAP Miscellaneous Defines *************/

/// Invalid connection index
#define GAP_INVALID_CONIDX                      0xFF
/// Invalid connection handle
#define GAP_INVALID_CONHDL                      0xFFFF

/// Maximum number of discovered addresses
#define GAP_MAX_DISC_ADDR                       BLE_CONNECTION_MAX

/// Default encryption key size
#define GAP_ENC_KEYSIZE_DEFAULT                 0x07

/// Connection request
struct gap_conn_rec
{
    /// BD Address bearer to connect
    struct bd_addr tmp_bdaddr;
    /// Own address type
    uint8_t own_addr_type;
};

/// Device controller version
struct gap_dev_version
{
    /// HCI version
    uint8_t hci_ver;
    /// HCI revision
    uint16_t hci_rev;
    /// HCI LMP version
    uint8_t lmp_ver;
    /// Manufacturing name
    uint16_t manuf_name;
    /// LMP subversion
    uint16_t lmp_subver;
};

/// Random Address Structure
struct random_address
{
    /// Static random address
    struct bd_addr static_addr;
    /// Private non-resolvable address
    struct bd_addr priv_nrsvble;
    /// Private resolvable address
    struct bd_addr priv_rsvble;
    /// Temporary address
    struct bd_addr tmp_bdaddr;
};

/// GAP Local Device Record Structure
struct gap_loc_dev_rec
{
    /// flag for discovery, connectivity
    uint16_t flag;
    /// random device address
    struct random_address random_addr;
    /// public device address
    struct bd_addr public_addr;
    /// device local version
    struct gap_dev_version ver;
    /// present device role
    uint8_t role;
    /// local features
    uint8_t local_feats[LE_FEATS_LEN];
    /// event mask
    uint8_t mask[EVT_MASK_LEN];
    /// LE event mask
    uint8_t le_mask[EVT_MASK_LEN];
};

/// Connected Device Record Structure
struct gap_dev_record
{
    /// peer address
    struct bd_addr peer_addr;
    /// connection handle
    uint16_t conhdl;
    /// Peer address type
    uint8_t peer_addr_type;
    /// Local address type used to connect
    uint8_t own_addr_type;
    /// Local random address type used to connect
    uint8_t own_rand_addr_type;
    /// encryption key size
    uint8_t enc_key_size;
    /// bonding status
    uint8_t lk_sec_stat;
    /// status of the device record
    bool free;
    /// Does device resources are ready for current connection
    bool resource_ready;
};

/// Advertising Settings Structure
struct gap_adv_rec
{
    /// advertising channel power level
    uint8_t adv_pwr_lvl;
    /// minimum advertising interval
    uint16_t adv_intv_min;
    /// maximum advertising interval
    uint16_t adv_intv_max;
    /// advertising type */
    uint8_t adv_type;
    /// advertising channel map
    uint8_t adv_chnl_map;
    /// advertising filter policy
    uint8_t adv_filt_policy;
    /// advertising data length
    uint8_t adv_data_len;
    /// advertising data
    struct adv_data adv_data;
    /// scanning response data length
    uint8_t scan_rsp_data_len;
    /// scanning response data
    struct scan_rsp_data scan_rsp_data;
    /// own address type
    uint8_t own_addr_type;
    /// direct address type
    uint8_t direct_addr_type;
    /// peer address ?
    struct bd_addr peer_addr;
    /// advertising flag: enable or disable
    uint8_t adv_en;
};

/// Scanning Settings Structure
struct gap_scan_rec
{
    /// 0=passive, 1=active
    uint8_t scan_type;
    /// scanning interval
    uint16_t scan_intv;
    /// scanning window
    uint16_t scan_window;
    /// address type
    uint8_t own_addr_type;
    /// scanning filter policy
    uint8_t scan_filt_policy;
    /// scanning flag: enable or disable
    uint8_t scan_en;
    /// filtering duplicate
    uint8_t filter_duplic_en;
};

/// Bonding Settings Structure
struct gap_bond_rec
{
    /// Bonding flag
    uint8_t bondflag;
    /// OOB information
    uint8_t oob;
    /// IO capabilities
    uint8_t iocap;
    /// Authentication requirements
    uint8_t auth;
    /// Encryption key size
    uint8_t key_size;
    ///Initiator key distribution
    uint8_t ikey_dist;
    ///Responder key distribution
    uint8_t rkey_dist;
};

/// Security Settings Structure
struct gap_security
{
    /// Device security mode
    uint8_t dev_sec;
    /// Bonding capabilities
    struct gap_bond_rec pair;
    /// Identity Root Key - local
    uint8_t irk[KEY_LEN];
};

/// Address holder
struct gap_discovered_addr
{
    /// BD address
    struct bd_addr addr;
};

/// Device discovery address holder
struct gap_address_disc_keeper
{
    /// Number of addresses
    uint8_t nb;
    /// Discovered addresses holder
    struct gap_discovered_addr addr_keeper[GAP_MAX_DISC_ADDR];
};

/// GAP Manager environment structure
struct gap_env_tag
{
    /// Record for each connected device
    struct gap_dev_record dev_rec[8];
    /// Local device settings
    struct gap_loc_dev_rec ble_dev;
    /// Advertising settings
    struct gap_adv_rec adv_rec;
    /// Scanning settings
    struct gap_scan_rec scan_rec;
    /// Security settings
    struct gap_security sec_rec;
    /// Connection settings
    struct gap_conn_rec conn_rec;
    /// Discovery Address holder
    struct gap_address_disc_keeper disc_addr;
    /// Response id
    ke_task_id_t rspid;
    /// ATTS service start handle
    uint16_t svc_start_hdl;
    /// Current device operational flag
    uint16_t opflag[BLE_CONNECTION_MAX];
    /// Name request connection handle
    uint16_t name_req_hdl;
    /// Parameter update indication connection handle
    uint16_t param_upd_hdl;
    /// Number of connected devices
    uint16_t device_cnt;
    /// Inquiry type set during discovery process
    uint8_t inq_type;
    /// White list size
    uint8_t whitelist_sz;
    /// Address type
    uint8_t rand_type;
    /// GAP status
    uint8_t status;
    /// Number of bonded device
    uint8_t bond_cnt;
    /// State of device privacy
    uint8_t priv_stat_flag;
    /// Flag for host start
    bool start;
    /// Flag for reset
    bool reset_flag;
    /// Flag that signifies disconnection initiator
    bool disc_init;
    /// Flag that requires address renewal
    bool renew_addr;
    /// Flag allowing/disallowing a peripheral initiated parameter update procedure
    bool param_update_disallowed;
    /// Flag for name request
    bool name_req_con_est;
};



/**
 ****************************************************************************************
 * @brief Sends disconnection complete event to upper layer.
 *
 * @param[in] status        disconnection complete event status
 * @param[in] reason        detach reason
 * @param[in] conhdl        connection handle
 * @param[in] dest_id       destination task id
 *
 ****************************************************************************************
 */
typedef void (*p_gap_send_discon_cmp_evt)(
        uint8_t status, uint8_t reason, uint16_t conhdl, ke_task_id_t dest_id);
#define gap_send_discon_cmp_evt ((p_gap_send_discon_cmp_evt)(_gap_send_discon_cmp_evt))

/**
 ****************************************************************************************
 * @brief Retrieve the device record from GAP information manager
 *
 * @param[in] conhdl Connection Handle of the connection for which the index is needed
 *
 * @return The connection index corresponding to this connection handle. If no matching
 *         index is found, @ref GAP_INVALID_CONIDX is returned
 *
 ****************************************************************************************
 */
typedef uint8_t (*p_gap_get_rec_idx)(uint16_t conhdl);
#define gap_get_rec_idx ((p_gap_get_rec_idx)(_gap_get_rec_idx))

/**
 ****************************************************************************************
 * @brief Retrieve the bonding status of the connection.
 *
 * @param[in] idx   connection record index
 *
 * @return bonding status uint8_t
 *
 ****************************************************************************************
 */
typedef uint8_t (*p_gap_get_lk_sec_status)(uint8_t idx);
#define gap_get_lk_sec_status ((p_gap_get_lk_sec_status)(_gap_get_lk_sec_status))

/**
 ****************************************************************************************
 * @brief Retrieve the encryption key size of the connection
 *
 * @param[in] idx   connection record index
 *
 * @return encryption key size uint8_t (size is 7 - 16 byte range)
 *
 ****************************************************************************************
 */
typedef uint8_t (*p_gap_get_enc_keysize)(uint8_t idx);
#define gap_get_enc_keysize ((p_gap_get_enc_keysize)(_gap_get_enc_keysize))

/**
 ****************************************************************************************
 * @brief Retrieve the connection handle
 *
 * @param[in] idx   connection record index
 *
 * @return uint16_t connection handle
 *
 ****************************************************************************************
 */
typedef uint16_t (*p_gap_get_conhdl)(uint8_t idx);
#define gap_get_conhdl ((p_gap_get_conhdl)(_gap_get_conhdl))

/**
 ****************************************************************************************
 * @brief Get the GAP security setting.
 *
 * @return security mode
 *
 ****************************************************************************************
 */
typedef uint8_t (*p_gap_get_security)(void);
#define gap_get_security ((p_gap_get_security)(_gap_get_security))

/**
 ****************************************************************************************
 * @brief Retrieve the connection handle via BD Address
 *
 * @param[in]   addr  BD address search key for connection handle
 * @param[out]  idx   connection record index associated with the BD address
 *
 * @return uint16_t connection handle
 *
 ****************************************************************************************
 */
__INLINE uint16_t
gap_get_conhdl_par_bdaddr(
        struct bd_addr const *addr, uint8_t *idx)
{
	struct gap_env_tag *gap_env = (struct gap_env_tag*)(_gap_env);
	
	/* retrieve connection handle */
    for (register unsigned int i = 0; i < BLE_CONNECTION_MAX; i++)
    {   /* traverse the record with bd_address comparison */
        if (co_bt_bdaddr_compare(addr, &gap_env->dev_rec[i].peer_addr) == true)
        {   /* the record should not be free */
            ASSERT_ERR(gap_env->dev_rec[i].free==false);
            *idx = i;

            /* return the connection handle */
            return gap_env->dev_rec[i].conhdl;
        }
    }
    /* no record found corresponding to the BD address */
    *idx = 0xFF;
    return 0xFFFF;
}

/**
 ****************************************************************************************
 * @brief Retrieve device record of specific connection handle
 *
 * @param[in] idx           connection index
 * @param[in] rec           pointer to gap_dev_record structure
 *
 ****************************************************************************************
 */
__INLINE void
gap_get_link_info(
        uint8_t idx, struct gap_dev_record *rec)
{
	struct gap_env_tag *gap_env = (struct gap_env_tag*)(_gap_env);
	
	/* retrieve link information */
    ASSERT_ERR(gap_env->dev_rec[idx].free == false);
    rec->peer_addr_type = gap_env->dev_rec[idx].peer_addr_type;
    memcpy(&(rec->peer_addr.addr[0]),
            &gap_env->dev_rec[idx].peer_addr.addr[0], BD_ADDR_LEN);
    rec->conhdl = gap_env->dev_rec[idx].conhdl;
}

// /**
//  ****************************************************************************************
//  * @brief Retrieve local BD Address used in connection: public or private
//  *
//  * @param[out] loc          pointer to local bd address
//  * @param[in]  idx          connection index
//  *
//  ****************************************************************************************
//  */
// __INLINE void
// gap_get_local_addr(
//         struct bd_addr *loc, uint8_t idx)
// {   /* get local address */
//     if (idx != 0xFF)
//     {   /* internal function so idx must be correct */
//         ASSERT_ERR(gap_env.dev_rec[idx].free == false);
//         if (gap_env.dev_rec[idx].own_addr_type == ADDR_RAND)
//         {   /* determine which type of random address */
//             switch(gap_env.dev_rec[idx].own_rand_addr_type)
//             {   /* non resolvable address */
//                 case GAP_NON_RSLV_ADDR:
//                     memcpy(&(loc->addr[0]),
//                             &gap_env.ble_dev.random_addr.priv_nrsvble.addr[0],
//                             BD_ADDR_LEN);
//                     break;

//                 case GAP_RSLV_ADDR: /* resolvable address */
//                     memcpy(&(loc->addr[0]),
//                             &gap_env.ble_dev.random_addr.priv_rsvble.addr[0],
//                             BD_ADDR_LEN);
//                     break;

//                 case GAP_STATIC_ADDR:
//                 default: /* static address */
//                     memcpy(&(loc->addr[0]),
//                             &gap_env.ble_dev.random_addr.static_addr.addr[0],
//                             BD_ADDR_LEN);
//                     break;
//             }
//         }
//         else
//         {   /* return the public address */
//             memcpy(&(loc->addr[0]),
//                     &gap_env.ble_dev.public_addr.addr[0],
//                     BD_ADDR_LEN);
//         }
//     }
//     else
//     {   /* return the public address */
//         memcpy(&(loc->addr[0]),
//                 &gap_env.ble_dev.public_addr.addr[0],
//                 BD_ADDR_LEN);
//     }
// }


// #if (BLE_CENTRAL || BLE_OBSERVER || BLE_PERIPHERAL)
// /**
//  ****************************************************************************************
//  * @brief Processes the address and check if this address is known
//  *
//  * @param[in] addr      BD address used as search key
//  * @param[in] type      type of address - public or private
//  *
//  ****************************************************************************************
//  */
// void
// gap_check_known_dev(
//         struct bd_addr const *addr, uint8_t type);
// #endif /* #if (BLE_CENTRAL || BLE_OBSERVER  || BLE_PERIPHERAL) */

/// @} GAP

#endif // GAP_H_
