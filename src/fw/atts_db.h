/**
 ****************************************************************************************
 *
 * @file atts_db.h
 *
 * @brief Header file - ATTS.
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
 * @file atts_db.h
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
#ifndef ATTS_DB_H_
#define ATTS_DB_H_

/**
 ****************************************************************************************
 * @addtogroup ATTDB Attribute Database
 * @ingroup ATTS
 * @brief Attribute Protocol Database
 *
 * The ATTDB module is responsible for providing different sets of attribute databases
 * for Attribute Profile server.
 *
 * This module can be tailored by client, to match the requirement of the desired database.
 *
 * @{
 *
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "ke_task.h"
#include "atts.h"
#include "attm_cfg.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Add a service in database.
 *
 * According to service start handle and number of attribute, ATTS DB allocate a set of
 * attribute handles, then using other parameters it allocate a buffer used to describe
 * service, and allocate attributes + their values.
 *
 * If start_hdl = 0, it allocated service using first available handle (start_hdl is
 * modified); else it will allocate service according to given start handle.
 *
 * @param[in|out] start_hdl Service start handle.
 * @param[in]     task_id Task that manages service.
 * @param[in]     nb_att Total number of attributes to allocate.
 * @param[in]     nb_att_uuid_128 Number of attributes using 128 bits UUID.
 * @param[in]     total_size Total payload size expected for attribute.
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If service allocation succeeds.
 *  - @ref ATT_ERR_INVALID_HANDLE: If start_hdl given in parameter + nb of attribute override
 *                            some existing services handles.
 *  - @ref ATT_INSUFF_RESOURCE: There is not enough memory to allocate service buffer.
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_add_service)(uint16_t *start_hdl, uint16_t task_id, uint8_t nb_att,
                                        uint8_t nb_att_uuid_128, uint16_t total_size);
#define attsdb_add_service ((p_attsdb_add_service)(_attsdb_add_service))

/**
 ****************************************************************************************
 * @brief Allocate attribute in service.
 *
 * Called once for each attributes in service database, after allocating service database
 * using @see attsdb_add_service. This function allocates attribute description structure
 * and attribute value into service buffer and update table of handle pointers service
 * description structure. Finally it returns handle of allocated attribute.
 *
 * @param[in] start_hdl Service start handle.
 * @param[in] max_length Maximum attribute value.
 * @param[in] uuid_len  UUID length (2bytes - 16bits or 16bytes - 128bits UUID).
 * @param[in] uuid      UUID value (as an array of bytes).
 * @param[in] perm      Attribute permission
 *
 * @param[out] handle Allocated attribute handle.
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If attribute allocation succeeds.
 *  - @ref ATT_ERR_INVALID_HANDLE: If start handle doesn't match any service
 *  - @ref ATT_INSUFF_RESOURCE: If new attribute cannot be added because all expected
 *                         attributes already added or buffer overflow detected during
 *                         allocation
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_add_attribute)(uint16_t start_hdl, atts_size_t max_length,
                                          uint8_t uuid_len, uint8_t* uuid, uint16_t perm,
                                          uint16_t *handle);
#define attsdb_add_attribute ((p_attsdb_add_attribute)(_attsdb_add_attribute))

/**
 ****************************************************************************************
 *  @brief Search in database from which service attribute handle comes from.
 *
 * @param[in] handle Attribute handle.
 *
 * @return Services that contains attribute handle; NULL if handle not available in
 *         database.
 ****************************************************************************************
 */
typedef struct atts_svc_db * (*p_attsdb_get_service)(uint16_t handle);
#define attsdb_get_service ((p_attsdb_get_service)(_attsdb_get_service))

/**
 ****************************************************************************************
 *  @brief Search in database Attribute pointer using attribute handle.
 *
 * @param[in] handle Attribute handle.
 *
 * @return Attribute element structure pointer; NULL if handle not available in
 *         database.
 ****************************************************************************************
 */
typedef struct atts_elmt* (*p_attsdb_get_attribute)(uint16_t handle);
#define attsdb_get_attribute ((p_attsdb_get_attribute)(_attsdb_get_attribute))

/**
 ****************************************************************************************
 * @brief Retrieve attribute at or after specified handle
 *
 * Retrieve first attribute with handle >= parameter handle.
 * Parameter handle is updated according found attribute.
 *
 * note: Handles in hidden services or hidden attributes are not returned.
 *
 * @param[in|out] handle Attribute handle.
 *
 * @return Attribute element structure pointer; NULL no more attribute available in
 *         database.
 ****************************************************************************************
 */
typedef struct atts_elmt * (*p_attsdb_get_next_att)(uint16_t * handle);
#define attsdb_get_next_att ((p_attsdb_get_next_att)(_attsdb_get_next_att))

/**
 ****************************************************************************************
 * @brief Update attribute value
 *
 * Updating attribute value do not trigger any notification or indication, this shall be
 * handled by GATT task.
 *
 * @param[in] handle Attribute handle.
 * @param[in] length Size of new attribute value
 * @param[in] value  Attribute value payload
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If attribute value update succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 *  - @ref ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN: If new value length exceeds maximum attribute
 *                              value length.
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_set_value)(uint16_t handle, atts_size_t length, uint8_t* value);
#define attsdb_att_set_value ((p_attsdb_att_set_value)(_attsdb_att_set_value))

/**
 ****************************************************************************************
 * @brief Update attribute with value to append using offset
 *
 * Updating attribute value do not trigger any notification or indication, this shall be
 * handled by GATT task.
 *
 * @param[in] handle Attribute handle.
 * @param[in] length Size of new attribute to append
 * @param[in] offset Data offset from where data shall be applied
 * @param[in] value  Attribute value payload to append
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If attribute value update succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 *  - @ref ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN: If new value length exceeds maximum attribute
 *                              value length.
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_update_value)(uint16_t handle, atts_size_t length, atts_size_t offset,
                                             uint8_t* value);
#define attsdb_att_update_value ((p_attsdb_att_update_value)(_attsdb_att_update_value))

/**
 ****************************************************************************************
 * @brief Partially update attribute value with a block value
 *
 * Updating attribute value do not trigger any notification or indication, this shall be
 * handled by GATT task.
 *
 * Block value length + offset shall not exceed attribute length.
 * This function doesn't modify attribute length
 *
 * @param[in] handle Attribute handle.
 * @param[in] offset Data offset from where data shall be apped
 * @param[in] size Size of block value to modify
 * @param[in] blk_value  Block value payload to set in attribute value
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If attribute value update succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 *  - @ref ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN: If block value length + offset exceeds
 *                                            attribute value length.
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_partial_value_update)(uint16_t handle, atts_size_t offset, atts_size_t size,
                                                     uint8_t* blk_value);
#define attsdb_att_partial_value_update ((p_attsdb_att_partial_value_update)(_attsdb_att_partial_value_update))

/**
 ****************************************************************************************
 * @brief Retrieve attribute value
 *
 * @param[in]  handle Attribute handle.
 * @param[out] length Size of attribute value
 * @param[out] value  Pointer to attribute value payload
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If request succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_get_value)(uint16_t handle, atts_size_t* length, uint8_t** value);
#define attsdb_att_get_value ((p_attsdb_att_get_value)(_attsdb_att_get_value))

/**
 ****************************************************************************************
 * @brief Retrieve attribute UUID
 *
 * @param[in]  handle   Attribute handle.
 * @param[out] uuid_len Size of attribute UUID
 * @param[out] uuid     UUID value to update
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If request succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_get_uuid)(uint16_t handle, uint8_t* uuid_len, uint8_t* uuid);
#define attsdb_att_get_uuid ((p_attsdb_att_get_uuid)(_attsdb_att_get_uuid))

/**
 ****************************************************************************************
 * @brief Update attribute permission
 *
 * @param[in] handle Attribute handle.
 * @param[in] perm   New attribute permission
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If request succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_set_permission)(uint16_t handle, uint16_t perm);
#define attsdb_att_set_permission ((p_attsdb_att_set_permission)(_attsdb_att_set_permission))

/**
 ****************************************************************************************
 * @brief Retrieve attribute permission
 *
 * @param[in]  handle Attribute handle.
 * @param[out] perm   Permission value to return
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If request succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_get_permission)(uint16_t handle, uint16_t* perm);
#define attsdb_att_get_permission ((p_attsdb_att_get_permission)(_attsdb_att_get_permission))

/**
 ****************************************************************************************
 * @brief Reset some permissions bit in the Handle passed as parameter.
 *
 * @param[in] att_elmt Attribute element for which permission bits have to be reset
 * @param[in] access_mask Access mask of permission to update
 * @param[in] perm  New value of the permission to update
 *
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If request succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_att_update_perm)(uint16_t handle, uint16_t access_mask, uint16_t perm);
#define attsdb_att_update_perm ((p_attsdb_att_update_perm)(_attsdb_att_update_perm))

/**
 ****************************************************************************************
 * @brief Update attribute service permission
 *
 * @param[in] handle Attribute handle.
 * @param[in] perm   New attribute permission
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If request succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_svc_set_permission)(uint16_t handle, uint8_t perm);
#define attsdb_svc_set_permission ((p_attsdb_svc_set_permission)(_attsdb_svc_set_permission))

/**
 ****************************************************************************************
 * @brief Retrieve attribute service permission
 *
 * @param[in]  handle Attribute handle.
 * @param[out] perm   Permission value to return
 *
 * @return Command status code:
 *  - @ref ATT_ERR_NO_ERROR: If request succeeds
 *  - @ref ATT_ERR_INVALID_HANDLE: If handle doesn't exist in database
 ****************************************************************************************
 */
typedef uint8_t (*p_attsdb_svc_get_permission)(uint16_t handle, uint8_t* perm);
#define attsdb_svc_get_permission ((p_attsdb_svc_get_permission)(_attsdb_svc_get_permission))

#endif // ATTS_DB_H_
