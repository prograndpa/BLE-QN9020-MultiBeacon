/**
 ****************************************************************************************
 *
 * @file app_glpc_task.c
 *
 * @brief Application GLPC task implementation
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
 * @addtogroup APP_GLPC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_GL_COLLECTOR
#include "app_glpc.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// @cond
struct app_glpc_env_tag *app_glpc_env = &app_env.glpc_ev[0];
/// @endcond



/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the GLPC. *//**
 *
 * @param[in] msgid     GLPC_ENABLE_CFM
 * @param[in] param     Pointer to the struct glpc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector to either send the discovery results of GLS on the Glucose 
 *  sensor and confirm enabling of the Collector role, or to simply confirm enabling of Collector role if it is a normal 
 *  connection and the attribute details are already known.
 *
 ****************************************************************************************
 */
int app_glpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct glpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("GLPC enable confirmation status: 0x%X.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);
        
        app_glpc_env[idx].conhdl = param->conhdl;
        app_glpc_env[idx].enabled = true;
        app_glpc_env[idx].op_state = OPERATION_STATE_IDLE;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the register to Glucose sensor notifications - confirmation. *//**
 *
 * @param[in] msgid     GLPC_REGISTER_CFM
 * @param[in] param     Pointer to the struct glpc_register_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application about Glucose sensor event 
 *  registration status.
 *
 ****************************************************************************************
 */
int app_glpc_register_cfm_handler(ke_msg_id_t const msgid,
                      struct glpc_register_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("GLPC register confirmation status: 0x%x.\r\n", param->status);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read Glucose sensor features - response. *//**
 *
 * @param[in] msgid     GLPC_READ_FEATURES_RSP 
 * @param[in] param     Pointer to the struct glpc_read_features_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of received peer Glucose sensor 
 *  features.
 *
 ****************************************************************************************
 */
int app_glpc_read_features_rsp_handler(ke_msg_id_t const msgid,
                      struct glpc_read_features_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("GLPC read features response status: 0x%X, features: 0x%04X.\r\n", 
        param->status, param->features);

    uint8_t idx = KE_IDX_GET(src_id);
    // remember the features value for future usage
    app_glpc_env[idx].features = param->features;
    app_glpc_env[idx].op_state = OPERATION_STATE_IDLE;

#if QN_DBG_TRACE_MORE
    /// Low Battery Detection During Measurement Support Bit
    QPRINTF("Low Battery Detection During Measurement Support: %d.\r\n", 
        0 != (param->features & GLP_FET_LOW_BAT_DET_DUR_MEAS_SUPP));

    /// Sensor Malfunction Detection Support Bit
    QPRINTF("Sensor Malfunction Detection Support: %d.\r\n", 
        0 != (param->features & GLP_FET_SENS_MFNC_DET_SUPP));

    /// Sensor Sample Size Support Bit
    QPRINTF("Sensor Sample Size Support: %d.\r\n", 
        0 != (param->features & GLP_FET_SENS_SPL_SIZE_SUPP));

    /// Sensor Strip Insertion Error Detection Support Bit
    QPRINTF("Sensor Strip Insertion Error Detection Support: %d.\r\n", 
        0 != (param->features & GLP_FET_SENS_STRIP_INSERT_ERR_DET_SUPP));

    /// Sensor Strip Type Error Detection Support Bit
    QPRINTF("Sensor Strip Type Error Detection: %d.\r\n", 
        0 != (param->features & GLP_FET_SENS_STRIP_TYPE_ERR_DET_SUPP));

    /// Sensor Result High-Low Detection Support Bit
    QPRINTF("Sensor Result High-Low Detection Support: %d.\r\n", 
        0 != (param->features & GLP_FET_SENS_RES_HIGH_LOW_DET_SUPP));

    /// Sensor Temperature High-Low Detection Support Bit
    QPRINTF("Sensor Temperature High-Low Detection Support: %d.\r\n", 
        0 != (param->features & GLP_FET_SENS_TEMP_HIGH_LOW_DET_SUPP));

    /// Sensor Read Interrupt Detection Support Bit
    QPRINTF("Sensor Read Interrupt Detection Support: %d.\r\n", 
        0 != (param->features & GLP_FET_SENS_RD_INT_DET_SUPP));

    /// General Device Fault Support Bit
    QPRINTF("General Device Fault Support: %d.\r\n", 
        0 != (param->features & GLP_FET_GEN_DEV_FLT_SUPP));

    /// Time Fault Support Bit
    QPRINTF("Time Fault Support: %d.\r\n", 
        0 != (param->features & GLP_FET_TIME_FLT_SUPP));

    /// Multiple Bond Support Bit
    QPRINTF("Multiple Bond Support: %d.\r\n", 
        0 != (param->features & GLP_FET_MUL_BOND_SUPP));
#endif
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message of record Access Control Point - Resp. *//**
 *
 * @param[in] msgid     GLPC_RACP_RSP
 * @param[in] param     Pointer to the struct glpc_racp_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a status of Record Access 
 *  Control Point Action. It shall contain status of executed request or number of stored measurement records if 
 *  GLP_REQ_REP_NUM_OF_STRD_RECS has been requested.
 *
 ****************************************************************************************
 */
int app_glpc_racp_response_handler(ke_msg_id_t const msgid,
                      struct glpc_racp_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    app_glpc_env[idx].op_state = OPERATION_STATE_IDLE;

    QPRINTF("GLPC RACP response OP code:%d, Operator:%d, status: 0x%x.\r\n", 
        param->racp_rsp.op_code,
        param->racp_rsp.operator,
        param->status);

    switch (param->racp_rsp.op_code)
    {
    case GLP_REQ_NUM_OF_STRD_RECS_RSP:
        QPRINTF("Number of stored records is %d.\r\n", 
            param->racp_rsp.operand.num_of_record);
        break;
    case GLP_REQ_RSP_CODE:
        switch (param->racp_rsp.operand.rsp.op_code_req)
        {
        case GLP_REQ_REP_STRD_RECS:
            QPRINTF("RACP Report stored records finished.\r\n");
            break;
        case GLP_REQ_ABORT_OP:
            QPRINTF("RACP Abort operation finished.\r\n");
            break;
        case GLP_REQ_DEL_STRD_RECS:
        default:
            break;
        }
    default:
        break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for Glucose measurement value received. *//**
 *
 * @param[in] msgid     GLPC_MEAS_IND
 * @param[in] param     Pointer to the struct glpc_meas_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received Glucose 
 *  measurement value. This value should be received within a RACP request (GLP_REQ_REP_STRD_RECS), but it could be 
 *  send out of request by Glucose sensor.
 *
 ****************************************************************************************
 */
int app_glpc_meas_ind_handler(ke_msg_id_t const msgid,
                      struct glpc_meas_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("GLPC measurement value received with sequence number: 0x%04X\r\r\n", param->seq_num);

    /// Base Time
    QPRINTF("Date: %04d:%02d:%02d Time: %02d:%02d:%02d\r\r\n", 
        param->meas_val.base_time.year,
        param->meas_val.base_time.month,
        param->meas_val.base_time.day,
        param->meas_val.base_time.hour,
        param->meas_val.base_time.min,
        param->meas_val.base_time.sec);
    /// Time Offset Present
    if (param->meas_val.flags & GLP_MEAS_TIME_OFF_PRES)
    {
        /// Time Offset
        QPRINTF("Time Offset: +%d\r\r\n", param->meas_val.time_offset);
    }
    /// Glucose Concentration, Type and Sample Location Present
    if (param->meas_val.flags & GLP_MEAS_GL_CTR_TYPE_AND_SPL_LOC_PRES)
    {
        /// Glucose measurement Sample Location
        QPRINTF("Sample Location: ");
        switch (param->meas_val.location)
        {
        case GLP_LOC_FINGER:
            QPRINTF("Finger");
            break;
        case GLP_LOC_ALT_SITE_TEST:
            QPRINTF("Alternate Site Test (AST)");
            break;
        case GLP_LOC_EARLOBE:
            QPRINTF("Earlobe");
            break;
        case GLP_LOC_CTRL_SOLUTION:
            QPRINTF("Control solution");
            break;
        case GLP_LOC_SPL_LOC_VAL_NOT_AVA:
        default:
            QPRINTF("Not available");
            break;
        }
        /// Glucose measurement type
        QPRINTF(", Type: ");
        switch (param->meas_val.type)
        {
        case GLP_TYPE_CAPILLARY_WHOLE_BLOOD:
            QPRINTF("Capillary Whole blood");
            break;
        case GLP_TYPE_CAPILLARY_PLASMA:
            QPRINTF("Capillary Plasma");
            break;
        case GLP_TYPE_VENOUS_WHOLE_BLOOD:
            QPRINTF("Venous Whole blood");
            break;
        case GLP_TYPE_VENOUS_PLASMA:
            QPRINTF("Venous Plasma");
            break;
        case GLP_TYPE_ARTERIAL_WHOLE_BLOOD:
            QPRINTF("Arterial Whole blood");
            break;
        case GLP_TYPE_ARTERIAL_PLASMA:
            QPRINTF("Arterial Plasma");
            break;
        case GLP_TYPE_UNDETERMINED_WHOLE_BLOOD:
            QPRINTF("Undetermined Whole blood");
            break;
        case GLP_TYPE_UNDETERMINED_PLASMA:
            QPRINTF("Undetermined Plasma");
            break;
        case GLP_TYPE_INTERSTITIAL_FLUID:
            QPRINTF("Interstitial Fluid (ISF)");
            break;
        case GLP_TYPE_CTRL_SOLUTION:
            QPRINTF("Control Solution");
            break;
        default:
            QPRINTF("NA");
            break;
        }
        QPRINTF("\r\r\n");
    }
    
    /// Glucose Concentration Units (kg/L, mol/L)
    QPRINTF("Concentration Units: ");
    if (param->meas_val.flags & GLP_MEAS_GL_CTR_UNITS_MOL_L)
        QPRINTF("%d(mol/L)\r\r\n", param->meas_val.concentration);
    else
        QPRINTF("%d(kg/L)\r\r\n", param->meas_val.concentration);
    
    /// Sensor Status Annunciation Present
    if (param->meas_val.flags & GLP_MEAS_SENS_STAT_ANNUN_PRES)
    {
        uint8_t idx = KE_IDX_GET(src_id);

        ///Sensor Status Annunciation
        QPRINTF("Sensor Status Annunciation: ");
        // Depend on the features read before
        for (uint8_t i = 0; i < 12; i++)
        {
            switch (param->meas_val.status & (1 << i))
            {
            case GLP_MEAS_STATE_DEV_BAT_LOW:
                if (app_glpc_env[idx].features & GLP_FET_LOW_BAT_DET_DUR_MEAS_SUPP)
                    QPRINTF("Device battery low at time of measurement\r\n");
                break;
            case GLP_MEAS_STATE_SENS_MFNC_OR_FLTING:
                if (app_glpc_env[idx].features & GLP_FET_SENS_MFNC_DET_SUPP)
                    QPRINTF("Sensor malfunction or faulting at time of measurement\r\n");
                break;
            case GLP_MEAS_STATE_SPL_SIZE_INSUFF:
                if (app_glpc_env[idx].features & GLP_FET_SENS_SPL_SIZE_SUPP)
                    QPRINTF("Sample size for blood or control solution insufficient at time of measurement\r\n");
                break;
            case GLP_MEAS_STATE_STRIP_INSERT_ERR:
                if (app_glpc_env[idx].features & GLP_FET_SENS_STRIP_INSERT_ERR_DET_SUPP)
                    QPRINTF("Strip insertion error\r\n");
                break;
            case GLP_MEAS_STATE_STRIP_TYPE_INCOR_FOR_DEV:
                if (app_glpc_env[idx].features & GLP_FET_SENS_STRIP_TYPE_ERR_DET_SUPP)
                    QPRINTF("Strip type incorrect for device\r\n");
                break;
            case GLP_MEAS_STATE_SENS_RES_HIGHER:
                if (app_glpc_env[idx].features & GLP_FET_SENS_RES_HIGH_LOW_DET_SUPP)
                    QPRINTF("Sensor result higher than the device can process\r\n");
                break;
            case GLP_MEAS_STATE_SENS_RES_LOWER:
                if (app_glpc_env[idx].features & GLP_FET_SENS_RES_HIGH_LOW_DET_SUPP)
                    QPRINTF("Sensor result lower than the device can process\r\n");
                break;
            case GLP_MEAS_STATE_SENS_TEMP_TOO_HIGH:
                if (app_glpc_env[idx].features & GLP_FET_SENS_TEMP_HIGH_LOW_DET_SUPP)
                    QPRINTF("Sensor temperature too high for valid test/result at time of measurement\r\n");
                break;
            case GLP_MEAS_STATE_SENS_TEMP_TOO_LOW:
                if (app_glpc_env[idx].features & GLP_FET_SENS_TEMP_HIGH_LOW_DET_SUPP)
                    QPRINTF("Sensor temperature too low for valid test/result at time of measurement\r\n");
                break;
            case GLP_MEAS_STATE_SENS_RD_INTED:
                if (app_glpc_env[idx].features & GLP_FET_SENS_RD_INT_DET_SUPP)
                    QPRINTF("Sensor read interrupted because strip was pulled too soon at time of measurement\r\n");
                break;
            case GLP_MEAS_STATE_GEN_DEV_FLT:
                if (app_glpc_env[idx].features & GLP_FET_GEN_DEV_FLT_SUPP)
                    QPRINTF("General device fault has occurred in the sensor\r\n");
                break;
            case GLP_MEAS_STATE_TIME_FLT:
                if (app_glpc_env[idx].features & GLP_FET_TIME_FLT_SUPP)
                    QPRINTF("Time fault has occurred in the sensor and time may be inaccurate\r\n");
                break;
            default:
                QPRINTF("NA\r\n");
                break;
            }
        }
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for Glucose measurement context received. *//**
 *
 * @param[in] msgid     GLPC_MEAS_CTX_IND
 * @param[in] param     Pointer to the struct glpc_meas_ctx_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received Glucose 
 *  measurement context value. This value should be received within a RACP request (GLP_REQ_REP_STRD_RECS), but it 
 *  could be send out of request by Glucose sensor. It shall be trigger by Glucose sensor only if corresponding glucose 
 *  measurement previously received has GLP_MEAS_CTX_INF_FOLW in its measurement flag.
 *
 ****************************************************************************************
 */
int app_glpc_meas_ctx_ind_handler(ke_msg_id_t const msgid,
                      struct glpc_meas_ctx_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("GLPC measurement context received with sequence number: 0x%04X\r\r\n", param->seq_num);

    /// Carbohydrate ID And Carbohydrate Present
    if (param->ctx.flags & GLP_CTX_CRBH_ID_AND_CRBH_PRES)
    {
        /// Glucose measurement context Carbohydrate ID
        QPRINTF("Carbohydrate ID: ");
        switch (param->ctx.carbo_id)
        {
        case GLP_CID_BREAKFAST:
            QPRINTF("Breakfast");
            break;
        case GLP_CID_LUNCH:
            QPRINTF("Lunch");
            break;
        case GLP_CID_DINNER:
            QPRINTF("Dinner");
            break;
        case GLP_CID_SNACK:
            QPRINTF("Snack");
            break;
        case GLP_CID_DRINK:
            QPRINTF("Drink");
            break;
        case GLP_CID_SUPPER:
            QPRINTF("Supper");
            break;
        case GLP_CID_BRUNCH:
            QPRINTF("Brunch");
            break;
        default:
            QPRINTF("NA");
            break;
        }
        QPRINTF(", Value: 0x%04X(Kilograms)\r\r\n", param->ctx.carbo_val);
    }
    /// Meal Present
    if (param->ctx.flags & GLP_CTX_MEAL_PRES)
    {
        /// Meal
        QPRINTF("Carbohydrate ID: ");
        switch (param->ctx.meal)
        {
        case GLP_MEAL_PREPRANDIAL:
            QPRINTF("Preprandial (before meal)");
            break;
        case GLP_MEAL_POSTPRANDIAL:
            QPRINTF("Postprandial (after meal)");
            break;
        case GLP_MEAL_FASTING:
            QPRINTF("Fasting");
            break;
        case GLP_MEAL_CASUAL:
            QPRINTF("Casual (snacks, drinks, etc.)");
            break;
        case GLP_MEAL_BEDTIME:
            QPRINTF("Bedtime");
            break;
        default:
            QPRINTF("NA");
            break;
        }
        QPRINTF("\r\r\n");
    }
    /// Tester-Health Present
    if (param->ctx.flags & GLP_CTX_TESTER_HEALTH_PRES)
    {
        /// Tester
        QPRINTF("Tester: ");
        switch (param->ctx.tester)
        {
        case GLP_TESTER_SELF:
            QPRINTF("Self");
            break;
        case GLP_TESTER_HEALTH_CARE_PROFESSIONAL:
            QPRINTF("Health Care Professional");
            break;
        case GLP_TESTER_LAB_TEST:
            QPRINTF("Lab test");
            break;
        case GLP_TESTER_TESTER_VAL_NOT_AVA:
        default:
            QPRINTF("NA");
            break;
        }
        QPRINTF(", Health: ");
        /// Health
        switch (param->ctx.health)
        {
        case GLP_HEALTH_MINOR_HEALTH_ISSUES:
            QPRINTF("Minor health issues");
            break;
        case GLP_HEALTH_MAJOR_HEALTH_ISSUES:
            QPRINTF("Major health issues");
            break;
        case GLP_HEALTH_DUR_MENSES:
            QPRINTF("During menses");
            break;
        case GLP_HEALTH_UNDER_STRESS:
            QPRINTF("Under stress");
            break;
        case GLP_HEALTH_NO_HEALTH_ISSUES:
            QPRINTF("No health issues");
            break;
        case GLP_HEALTH_HEALTH_VAL_NOT_AVA:
        default:
            QPRINTF("NA");
            break;
        }
        QPRINTF("\r\r\n");
    }
    
    /// Exercise Duration And Exercise Intensity Present
    if (param->ctx.flags & GLP_CTX_EXE_DUR_AND_EXE_INTENS_PRES)
    {
        if (param->ctx.exercise_dur < 65535)
            QPRINTF("Exercise Durationin in 0x%X seconds, ", param->ctx.exercise_dur);
        else
            QPRINTF("Exercise Durationin is Overrun, ");
        QPRINTF("Intensity Present: %d%\r\r\n", param->ctx.exercise_intens);
    }
    
    /// Medication ID And Medication Present
    if (param->ctx.flags & GLP_CTX_MEDIC_ID_AND_MEDIC_PRES)
    {
        /// Glucose measurement context medication identification
        QPRINTF("Medication ID: ");
        switch (param->ctx.med_id)
        {
        case GLP_MEDID_RAPID_ACTING_INSULIN:
            QPRINTF("Rapid acting insulin");
            break;
        case GLP_MEDID_SHORT_ACTING_INSULIN:
            QPRINTF("Short acting insulin");
            break;
        case GLP_MEDID_INTER_ACTING_INSULIN:
            QPRINTF("Intermediate acting insulin");
            break;
        case GLP_MEDID_LONG_ACTING_INSULIN:
            QPRINTF("Long acting insulin");
            break;
        case GLP_MEDID_PRE_MIXED_INSULIN:
            QPRINTF("Pre-mixed insulin");
            break;
        default:
            QPRINTF("NA");
            break;
        }
        /// Medication Value Units (kilograms, liters)
        QPRINTF(", Value: ");
        if (param->ctx.flags & GLP_CTX_MEDIC_VAL_UNITS_L)
            QPRINTF("0x%04X(Liters)\r\r\n", param->ctx.med_val);
        else
            QPRINTF("0x%04X(Kilograms)\r\r\n", param->ctx.med_val);
    }
    /// HbA1c Present
    if (param->ctx.flags & GLP_CTX_HBA1C_PRES)
    {
        QPRINTF("HbA1c Present: 0x%04X%\r\r\n", param->ctx.hba1c_val);
    }
    /// Extended Flags Present
    if (param->ctx.flags & GLP_CTX_HBA1C_PRES)
    {
        QPRINTF("Extended Flags Present: %d\r\r\n", param->ctx.ext_flags);
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//*
 *
 * @param[in] msgid     GLPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Glucose collector Client
 * Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_glpc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    app_glpc_env[idx].op_state = OPERATION_STATE_IDLE;
    QPRINTF("GLPC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_GLPC_TASK
