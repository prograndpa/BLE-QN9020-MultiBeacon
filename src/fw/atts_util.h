/**
 ****************************************************************************************
 *
 * @file atts_util.h
 *
 * @brief Header file - ATTSUTIL.
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
 * @file atts_util.h
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
#ifndef ATTS_UTIL_H_
#define ATTS_UTIL_H_

/**
 ****************************************************************************************
 * @addtogroup ATTSUTIL Utility
 * @ingroup ATTS
 * @brief Contains utility functions and macros for ATTS
 *
 * The ATTSUTIL is responsible for providing utility functions for
 * @ref ATTS "ATTS", in order to accomplish defined Attribute Server
 * functionalities.
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "atts.h"
#include "co_error.h"
#include "co_bt.h"
#include "co_utils.h"
#include "gap.h"

/*
 * DEFINES
 ****************************************************************************************
 */
//Characteristic element size
#define ATT_CHAR_ELMT_SIZE      0x05

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Format the Write Response PDU and send it after receiving a Write Request PDU
 * @param[in] conhdl  Connection Handle
 * @param[in] atthdl  Attribute handle for which to send the response
 * @param[in] status  ATT error code
 ****************************************************************************************
 */
typedef void (*p_atts_write_rsp_send)(uint16_t conhdl, uint16_t atthdl, uint8_t status);
#define atts_write_rsp_send ((p_atts_write_rsp_send)(_atts_write_rsp_send))

/**
 ****************************************************************************************
 * @brief Function use to ease service database creation.
 *
 * Use @see attsdb_add_service function of attsdb module to create service database,
 * then use @see attsdb_add_attribute function of attsdb module to create attributes
 * according to database description array given in parameter.
 *
 * @note: database description array shall be const to reduce memory consuption (only ROM)
 * @note: It supports only 16 bits UUIDs
 *
 * @param[in|out] shdl       Service start handle.
 * @param[in]     cfg_flag   Bit mask defines which of attribute is added, the first attribute is specified
 * by the first bit, number of attributes is limited to 64
 * @param[in]     max_nb_att Maximum number of attributes in the database description array
 * @param[out]    att_tbl    Stored the attribute value handle
 * @param[in]     dest_id    Task that manages service
 * @param[in]     att_db     Database description array pointer
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If database creation succeeds.
 *  - @ref ATT_ERR_INVALID_HANDLE: If start_hdl given in parameter + nb of attribute override
 *                            some existing services handles.
 *  - @ref ATT_INSUFF_RESOURCE: There is not enough memory to allocate service buffer.
 *                           or of new attribute cannot be added because all expected
 *                           attributes already added or buffer overflow detected during
 *                           allocation
 ****************************************************************************************
 */
typedef uint8_t (*p_atts_svc_create_db)(uint16_t *shdl, uint8_t *cfg_flag, uint8_t max_nb_att,
                                        uint8_t *att_tbl, ke_task_id_t const dest_id,
                                        const struct atts_desc *att_db);
#define atts_svc_create_db ((p_atts_svc_create_db)(_atts_svc_create_db))

/**
 ****************************************************************************************
 * @brief Create database that has a uuid of variable length.
 *
 * @note: database description array shall be const to reduce memory consuption (only ROM)
 *
 * @param[in|out] shdl       Service start handle.
 * @param[in]     cfg_flag   Bit mask defines which of attribute is added, the first attribute is specified
 * by the first bit, number of attributes is limited to 64
 * @param[in]     max_nb_att Maximum number of attributes in the database description array
 * @param[out]    att_tbl    Stored the attribute value handle
 * @param[in]     dest_id    Task that manages service
 * @param[in]     att_db     Database description array pointer
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If database creation succeeds.
 *  - @ref ATT_ERR_INVALID_HANDLE: If start_hdl given in parameter + nb of attribute override
 *                            some existing services handles.
 *  - @ref ATT_INSUFF_RESOURCE: There is not enough memory to allocate service buffer.
 *                           or of new attribute cannot be added because all expected
 *                           attributes already added or buffer overflow detected during
 *                           allocation
 ****************************************************************************************
 */
#if defined(QN_9020_B2) 
uint8_t atts_svc_create_db_ext(uint16_t *shdl, uint8_t *cfg_flag, uint8_t max_nb_att,
                               uint8_t *att_tbl, ke_task_id_t const dest_id,
                               const struct atts_desc_ext *att_db);
#elif defined(QN_9020_B4)
typedef uint8_t (*p_atts_svc_create_db_ext)(uint16_t *shdl, uint8_t *cfg_flag, uint8_t max_nb_att,  \
                               uint8_t *att_tbl, ke_task_id_t const dest_id,                        \
                               const struct atts_desc_ext *att_db);
#define atts_svc_create_db_ext ((p_atts_svc_create_db_ext)(_atts_svc_create_db_ext))
#endif
                             

/// @} ATTSUTIL

#endif // ATTS_UTIL_H_
