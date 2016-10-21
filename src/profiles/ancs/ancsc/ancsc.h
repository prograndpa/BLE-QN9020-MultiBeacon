/**
 ****************************************************************************************
 *
 * @file ancsc.h
 *
 * @brief Header file - Apple Notification Center Service Notification Consumer (NC) Role.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _ANCSC_H_
#define _ANCSC_H_

/**
 ****************************************************************************************
 * @addtogroup ANCSC Apple Notification Center Service Client
 * @ingroup ANCS
 * @brief Apple Notification Center Service Client
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_ANCS_NC)

#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"

#define ANCSC_GET_NTF_ATTRIBUTE_CMD_ID      (0)
#define ANCSC_GET_APP_ATTRIBUTE_CMD_ID      (1)
 
/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/// Apple Notification Center Service Characteristics
enum ancs_chars
{
    /// Control Point
    ANCSC_CHAR_CONTROL_POINT,
    /// Notification Source
    ANCSC_CHAR_NTF_SOURCE,
    /// Data Source
    ANCSC_CHAR_DATA_SOURCE,

    ANCSC_CHAR_MAX,
};

/// Apple Notification Center Service Characteristic Descriptors
enum ancs_descs
{
    /// Notification Source Char. - Client Characteristic Configuration
    ANCSC_DESC_NTF_SOURCE_CL_CFG,
    /// Data Source Char. - Client Characteristic Configuration
    ANCSC_DESC_DATA_SOURCE_CL_CFG,

    ANCSC_DESC_MAX,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Characteristic definition
struct prf_char_def128
{
    /// Characteristic UUID
    uint8_t *uuid;
    /// Requirement Attribute Flag
    uint8_t req_flag;
    /// Mandatory Properties
    uint8_t prop_mand;
};
 
/// Structure containing the characteristics handles, value handles and descriptors
struct ancsc_content
{
    /// Service info
    struct prf_svc svc;

    /// Characteristic info:
    ///  - Notification Source
    ///  - Control Point
    ///  - Data Source
    struct prf_char_inf chars[ANCSC_CHAR_MAX];

    /// Descriptor handles:
    ///  - Notification Source Client Cfg
    ///  - Data Source Client Cfg
    struct prf_char_desc_inf descs[ANCSC_DESC_MAX];
};

/// ANCS Client environment variable
struct ancsc_env_tag
{
    /// Profile Connection Info
    struct prf_con_info con_info;
    /// Last requested
    uint16_t last_req;
    /// Last characteristic code discovered
    uint8_t last_char_code;

    /// Current Operation
    uint8_t operation;

    /// Counter used to check service uniqueness
    uint8_t nb_svc;
    
    /// ANCS characteristics
    struct ancsc_content ancs;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
// Pool of ANCS Client task environments
extern struct ancsc_env_tag **ancsc_envs;

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the ANCSC module.
 * This function performs all the initializations of the ANCSC module.
 ****************************************************************************************
 */
void ancsc_init(void);

/**
 ****************************************************************************************
 * @brief Send a ANCSC_CMP_EVT message to the task which enabled the profile
 ****************************************************************************************
 */
void ancsc_send_no_conn_cmp_evt(uint8_t src_id, uint8_t dest_id, uint16_t conhdl, uint8_t operation);

/**
 ****************************************************************************************
 * @brief Send a ANCSC_CMP_EVT message to the task which enabled the profile
 ****************************************************************************************
 */
void ancsc_send_cmp_evt(struct ancsc_env_tag *ancsc_env, uint8_t operation, uint8_t status);

/**
 ****************************************************************************************
 * @brief Check whether characteristics information is satisfied.
 ****************************************************************************************
 */
void prf_search_chars128(uint16_t svc_ehdl, uint8_t nb_chars,
                         struct prf_char_inf* chars, const struct prf_char_def128* chars_req,
                         const struct gatt_disc_char_all_128_cmp_evt* param,
                         uint8_t* last_found_char);
                         
#endif // BLE_ANCS_NC

/// @} ANCSC

#endif // _ANCSC_H_
