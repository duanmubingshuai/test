/**
    \file sensor_server.c
*/

/*
    Copyright (C) 2019. PHYPLUSINC.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "sensor_server.h"
#include "MS_model_states.h"
#include "MS_generic_default_transition_time_api.h"

/* --------------------------------------------- Global Definitions */
/** -- Sensor  Defined States */


uint8 vol_raw_value_x0[VOL_VALUE_LEN];
uint8 vol_column_w0[VOL_VALUE_LEN];
uint8 vol_raw_value_y0[VOL_VALUE_LEN];

uint8 vol_raw_value_x1[VOL_VALUE_LEN];
uint8 vol_column_w1[VOL_VALUE_LEN];
uint8 vol_raw_value_y1[VOL_VALUE_LEN];

uint8 vol_raw_value_x2[VOL_VALUE_LEN];
uint8 vol_column_w2[VOL_VALUE_LEN];
uint8 vol_raw_value_y2[VOL_VALUE_LEN];


uint8 humity_raw_value_x0[HUMITY_VALUE_LEN];
uint8 humity_column_w0[HUMITY_VALUE_LEN];
uint8 humity_raw_value_y0[HUMITY_VALUE_LEN];

uint8 humity_raw_value_x1[HUMITY_VALUE_LEN];
uint8 humity_column_w1[HUMITY_VALUE_LEN];
uint8 humity_raw_value_y1[HUMITY_VALUE_LEN];

uint8 humity_raw_value_x2[HUMITY_VALUE_LEN];
uint8 humity_column_w2[HUMITY_VALUE_LEN];
uint8 humity_raw_value_y2[HUMITY_VALUE_LEN];

uint8 humity_raw_value_x3[HUMITY_VALUE_LEN];
uint8 humity_column_w3[HUMITY_VALUE_LEN];
uint8 humity_raw_value_y3[HUMITY_VALUE_LEN];

uint8 humity_raw_value_x4[HUMITY_VALUE_LEN];
uint8 humity_column_w4[HUMITY_VALUE_LEN];
uint8 humity_raw_value_y4[HUMITY_VALUE_LEN];



uint8 ACC_raw_value_x0[ACC_VALUE_LEN];
uint8 ACC_column_w0[ACC_VALUE_LEN];
uint8 ACC_raw_value_y0[ACC_VALUE_LEN];

uint8 ACC_raw_value_x1[ACC_VALUE_LEN];
uint8 ACC_column_w1[ACC_VALUE_LEN];
uint8 ACC_raw_value_y1[ACC_VALUE_LEN];


uint8 ACC_raw_value_x2[ACC_VALUE_LEN];
uint8 ACC_column_w2[ACC_VALUE_LEN];
uint8 ACC_raw_value_y2[ACC_VALUE_LEN];



MS_STATE_SENSOR_DESCRIPTOR_STRUCT UI_Sensor_Descrip_Infor[SENSOR_NUM]=
{

//VOLTAGE PID
    {
        VOLTAGE_PID,        //sensor_property_id
        41,             //sensor_positive_tolerance--41/4095*100%=1%
        41,             //sensor_negative_tolerance
        0x01,               //sensor_sampling_function
        81,               //sensor_measurement_period 1.1^(n-64)
        81,               //sensor_update_interval 1.1^(n-64)
        0                   //status
    },
//ACCELERO_METER_PID
    {
        ACCELERO_METER_PID,     //sensor_property_id
        0x0000,                 //sensor_positive_tolerance
        0x0000,                 //sensor_negative_tolerance
        0x00,                   //sensor_sampling_function
        0x00,                   //sensor_measurement_period
        0x00,                   //sensor_update_interval
        0                       //status
    },

//HUMIDITY_PID
    {
        HUMIDITY_PID,       //sensor_property_id
        41,             //sensor_positive_tolerance
        41,             //sensor_negative_tolerance
        0x04,               //sensor_sampling_function
        81,             //sensor_measurement_period
        81,             //sensor_update_interval
        0                   //status
    }
};







MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Vol_Sensor_series_column[VOL_COLUMN_NUM]=
{
    {
        VOLTAGE_PID,
        vol_raw_value_x0,
        sizeof(vol_raw_value_x0),

        vol_column_w0,
        sizeof(vol_column_w0),

        vol_raw_value_y0,
        sizeof(vol_raw_value_y0),

        0
    },

    {
        VOLTAGE_PID,
        vol_raw_value_x1,
        sizeof(vol_raw_value_x1),

        vol_column_w1,
        sizeof(vol_column_w1),

        vol_raw_value_y1,
        sizeof(vol_raw_value_y1),

        0
    },

    {
        VOLTAGE_PID,
        vol_raw_value_x2,
        sizeof(vol_raw_value_x2),

        vol_column_w2,
        sizeof(vol_column_w2),

        vol_raw_value_y2,
        sizeof(vol_raw_value_y2),

        0
    }


};




MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Humity_Sensor_series_column[HUMITY_COLUMN_NUM]=
{
    {
        HUMIDITY_PID,
        humity_raw_value_x0,
        sizeof(humity_raw_value_x0),

        humity_column_w0,
        sizeof(humity_column_w0),

        humity_raw_value_y0,
        sizeof(humity_raw_value_y0),

        0
    },

    {
        HUMIDITY_PID,
        humity_raw_value_x1,
        sizeof(humity_raw_value_x1),

        humity_column_w1,
        sizeof(humity_column_w1),

        humity_raw_value_y1,
        sizeof(humity_raw_value_y1),

        0
    },
    {
        HUMIDITY_PID,
        humity_raw_value_x2,
        sizeof(humity_raw_value_x2),

        humity_column_w2,
        sizeof(humity_column_w2),

        humity_raw_value_y2,
        sizeof(humity_raw_value_y2),

        0
    },
    {
        HUMIDITY_PID,
        humity_raw_value_x3,
        sizeof(humity_raw_value_x3),

        humity_column_w3,
        sizeof(humity_column_w3),

        humity_raw_value_y3,
        sizeof(humity_raw_value_y3),

        0
    },
    {
        HUMIDITY_PID,
        humity_raw_value_x4,
        sizeof(humity_raw_value_x4),

        humity_column_w4,
        sizeof(humity_column_w4),

        humity_raw_value_y4,
        sizeof(humity_raw_value_y4),

        0
    },

};


MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_ACC_Sensor_series_column[ACC_COLUMN_NUM]=
{
    {
        ACCELERO_METER_PID,
        ACC_raw_value_x0,
        sizeof(ACC_raw_value_x0),

        ACC_column_w0,
        sizeof(ACC_column_w0),

        ACC_raw_value_y0,
        sizeof(ACC_raw_value_y0),

        0
    },

    {
        ACCELERO_METER_PID,
        ACC_raw_value_x1,
        sizeof(ACC_raw_value_x1),

        ACC_column_w1,
        sizeof(ACC_column_w1),

        ACC_raw_value_y1,
        sizeof(ACC_raw_value_y1),

        0
    },

    {
        ACCELERO_METER_PID,
        ACC_raw_value_x1,
        sizeof(ACC_raw_value_x1),

        ACC_column_w1,
        sizeof(ACC_column_w1),

        ACC_raw_value_y1,
        sizeof(ACC_raw_value_y1),

        0
    }

};

//===========================cadence setting start===========================

uint8 voltage_trigger_delta_down=50; //50mv
uint8 voltage_trigger_delta_up=50;   //50mv
uint8 voltage_fast_cadence_low[VOL_CADENCE_LEN]= {(uint8)(2800&0XFF),(uint8)(2800>>8)};    //2800mv
uint8 voltage_fast_cadence_high[VOL_CADENCE_LEN]= {(uint8)(4300&0XFF),(uint8)(4300>>8)};  //4300mv


uint8 humity_trigger_delta_down=10; //10--1%
uint8 humity_trigger_delta_up=10; //10--1%
uint8 humity_fast_cadence_low[HUMITY_CADENCE_LEN]= {0};
uint8 humity_fast_cadence_high[HUMITY_CADENCE_LEN]= {0};


MS_STATE_SENSOR_CADENCE_STRUCT Sensor_CadenceSetting[SENSOR_CADENCE_NUMBER]=
{
    {
        VOLTAGE_PID,                            //property id
        0X02,                                   //divisor
        1,                                      //trigger type
        &voltage_trigger_delta_down,            //trigger delta down
        sizeof(voltage_trigger_delta_down),     //trigger delta down len
        &voltage_trigger_delta_up,              //trigger delta up
        sizeof(voltage_trigger_delta_up),       //trigger delta up len
        0,                                      // status minimum intervel
        &voltage_fast_cadence_low[0],              // fast cadence low
        VOL_CADENCE_LEN,                        // fast cadence low len
        &voltage_fast_cadence_high[0],             // fast cadence high
        VOL_CADENCE_LEN,                        // fast cadence high len
        0                                       // not use
    },
    {
        HUMIDITY_PID,                             //property id
        0X02,                                     //divisor
        1,                                        //trigger type
        &humity_trigger_delta_down,               //trigger delta down
        sizeof(humity_trigger_delta_down),        //trigger delta down len
        &humity_trigger_delta_up,                 //trigger delta up
        sizeof(humity_trigger_delta_up),          //trigger delta up len
        0,                                        // status minimum intervel
        &humity_fast_cadence_low[0],              // fast cadence low
        HUMITY_CADENCE_LEN,                       // fast cadence low len
        &humity_fast_cadence_high[0],             // fast cadence high
        HUMITY_CADENCE_LEN,                       // fast cadence high len
        0                                         // not use
    }
};


//===========================cadence setting end================================

//===========================settings && setting start ===========================

uint8 humity_setting_raw0=0x55;
uint8 humity_setting_raw1=0xAA;
static uint16 settings_num[SENSOR_SETTINGS_NUM]=
{
    0x0001,
    0x0002
};


MS_STATE_SENSOR_SETTING_STRUCT humity_sensor_setting[2]=
{
    {
        HUMIDITY_PID,
        0x0001,
        Sensor_Access_read,
        &humity_setting_raw0,//just for testing
        1,
        0
    },
    {
        HUMIDITY_PID,
        0x0002,
        Sensor_Access_read,
        &humity_setting_raw1,//just for testing
        1,
        0
    }
};

MS_STATE_SENSOR_SETTINGS_STRUCT humity_sensor_settings=
{
    HUMIDITY_PID,
    settings_num,
    SENSOR_SETTINGS_NUM
};







//===========================settings && setting end ===========================




/* --------------------------------------------- Static Global Variables */

static DECL_CONST UINT32 sensor_server_opcode_list[] =
{
    MS_ACCESS_SENSOR_DESCRIPTOR_GET_OPCODE,
    MS_ACCESS_SENSOR_DESCRIPTOR_STATUS_OPCODE,
    MS_ACCESS_SENSOR_GET_OPCODE,
    MS_ACCESS_SENSOR_STATUS_OPCODE,
    MS_ACCESS_SENSOR_COLUMN_GET_OPCODE,
    MS_ACCESS_SENSOR_COLUMN_STATUS_OPCODE,
    MS_ACCESS_SENSOR_SERIES_GET_OPCODE,
    MS_ACCESS_SENSOR_SERIES_STATUS_OPCODE
};

static DECL_CONST UINT32 sensor_setup_server_opcode_list[] =
{
    MS_ACCESS_SENSOR_CADENCE_GET_OPCODE,
    MS_ACCESS_SENSOR_CADENCE_SET_OPCODE,
    MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_SENSOR_CADENCE_STATUS_OPCODE,
    MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE,
    MS_ACCESS_SENSOR_SETTINGS_STATUS_OPCODE,
    MS_ACCESS_SENSOR_SETTING_GET_OPCODE,
    MS_ACCESS_SENSOR_SETTING_SET_OPCODE,
    MS_ACCESS_SENSOR_SETTING_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_SENSOR_SETTING_STATUS_OPCODE
};

static MS_SENSOR_SERVER_CB       sensor_server_appl_cb;
static MS_ACCESS_MODEL_HANDLE   sensor_server_model_handle;


/* --------------------------------------------- External Global Variables */

extern void appl_dump_bytes(UCHAR* buffer, UINT16 length);


/* --------------------------------------------- Function */

/**
    \brief API to send reply or to update state change

    \par Description
    This is to send reply for a request or to inform change in state.

    \param [in] ctx                     Context of the message.
    \param [in] current_state_params    Model specific current state parameters.
    \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
    \param [in] remaining_time          Time from current state to target state (0: to be ignored).
    \param [in] ext_params              Additional parameters (NULL: to be ignored).

    \return API_SUCCESS or an error code indicating reason for failure
*/
UINT8 flasg;

API_RESULT MS_sensor_server_state_update(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    API_RESULT rslt = API_FAILURE;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker   = 0;
    UINT32     opcode;
    MS_IGNORE_UNUSED_PARAM(target_state_params);
    MS_IGNORE_UNUSED_PARAM(remaining_time);
    MS_IGNORE_UNUSED_PARAM(ext_params);
    printf(
        "[SENSOR_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    case MS_STATE_SENSOR_DESCRIPTOR_T:
    case MS_STATE_SENSOR_PROPERTY_ID_T:
    {
        MS_STATE_SENSOR_DESCRIPTOR_STRUCT* param_p;
        UINT32      tolerance;
        param_p = (MS_STATE_SENSOR_DESCRIPTOR_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->sensor_property_id);
        marker += 2;

        if(param_p->sensor_property_id != 0xffff)
        {
            tolerance = (UINT32)(param_p->sensor_negative_tolerance);
            tolerance = (tolerance << 12) | (param_p->sensor_positive_tolerance & 0xfff);
            MS_PACK_LE_3_BYTE_VAL(&buffer[marker], tolerance);
            marker += 3;
            buffer[marker] = param_p->sensor_sampling_function;
            marker += 1;
            buffer[marker] = param_p->sensor_measurement_period;
            marker += 1;
            buffer[marker] = param_p->sensor_update_interval;
            marker += 1;
        }

        /* Set Opcode */
        opcode = MS_ACCESS_SENSOR_DESCRIPTOR_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_DATA_T:
    case MS_STATE_SENSOR_DATA_PROPERTY_ID_T:
    {
        MS_STATE_SENSOR_DATA_STRUCT* param_pdata;
        param_pdata = (MS_STATE_SENSOR_DATA_STRUCT*)current_state_params->state;

        if(param_pdata->property_id_1 != 0xffff)
        {
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_pdata->property_id_1);
            marker += 2;
            #if 1
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], 0x1234);
            marker += 2;
            #endif
        }
        else
        {
            buffer[marker] = 0xFF;
            marker +=1;
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_pdata->property_id_1);
            marker += 2;
        }

        /* Set Opcode */
        opcode = MS_ACCESS_SENSOR_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_SERIES_T:
    {
        MS_STATE_SENSOR_SERIES_COLUMN_STRUCT* param_series;
        param_series = (MS_STATE_SENSOR_SERIES_COLUMN_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_series->sensor_property_id);
        marker += 2;
        buffer[marker] = param_series->sensor_raw_value_x[0];
        marker += 1;
        buffer[marker] = param_series->sensor_column_width[0];
        marker += 1;
        buffer[marker] = param_series->sensor_raw_value_y[0];
        marker += 1;
        /* Set Opcode */
        opcode = MS_ACCESS_SENSOR_SERIES_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_COLUMN_STATUS_T:
    {
        printf("MS_STATE_SENSOR_COLUMN_STATUS_T\n");
        MS_STATE_SENSOR_SERIES_COLUMN_STRUCT* param_series2;
        param_series2 = (MS_STATE_SENSOR_SERIES_COLUMN_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_series2->sensor_property_id);
        marker += 2;
        buffer[marker] = param_series2->sensor_raw_value_x[0];
        marker += 1;
        buffer[marker] = param_series2->sensor_column_width[0];
        marker += 1;
        buffer[marker] = param_series2->sensor_raw_value_y[0];
        marker += 1;
        /* Set Opcode */
        opcode = MS_ACCESS_SENSOR_COLUMN_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_CADENCE_T:
    {
        printf("MS_STATE_SENSOR_CADENCE_T\n");
        MS_STATE_SENSOR_CADENCE_STRUCT* param_cadence;
        param_cadence = (MS_STATE_SENSOR_CADENCE_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_cadence->sensor_property_id);
        marker += 2;

        if(param_cadence->sensor_property_id != 0xffff)
        {
            buffer[marker] = (param_cadence->fast_cadence_period_divisor & 0x7f)|(param_cadence->status_trigger_type << 7);
            marker += 1;
            buffer[marker] = param_cadence->status_trigger_delta_down[0];
            marker += 1;
            buffer[marker] = param_cadence->status_trigger_delta_up[0];
            marker += 1;
            buffer[marker] = param_cadence->status_min_interval;
            marker += 1;
            buffer[marker] = param_cadence->fast_cadence_low[0];
            marker += 1;
            buffer[marker] = param_cadence->fast_cadence_high[0];
            marker += 1;
        }

        /* Set Opcode */
        opcode = MS_ACCESS_SENSOR_CADENCE_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_SETTINGS_T:
    {
        printf("MS_STATE_SENSOR_SETTINGS_T\n");
        MS_STATE_SENSOR_SETTINGS_STRUCT* param_setting;
        param_setting = (MS_STATE_SENSOR_SETTINGS_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_setting->sensor_property_id);
        marker += 2;
        /* Set Opcode */
        opcode = MS_ACCESS_SENSOR_SETTINGS_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_SETTING_T:
    {
        printf("MS_STATE_SENSOR_SETTINGS_T\n");
        MS_STATE_SENSOR_SETTING_STRUCT* param_setting1;
        param_setting1 = (MS_STATE_SENSOR_SETTING_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_setting1->sensor_property_id);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_setting1->sensor_setting_property_id);
        marker += 2;
        buffer[marker] = 0x01;
        marker += 1;
        /* Set Opcode */
        opcode = MS_ACCESS_SENSOR_SETTING_STATUS_OPCODE;
    }
    break;

    default:
    {
        printf(
            "Invalid State Type: 0x%02X\n", current_state_params->state_type);
        return rslt;
    }
    }

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    rslt = MS_access_reply
           (
               &ctx->handle,
               ctx->daddr,
               ctx->saddr,
               ctx->subnet_handle,
               ctx->appkey_handle,
               ACCESS_INVALID_DEFAULT_TTL,
               opcode,
               pdu_ptr,
               marker
           );
    return rslt;
}

/**
    \brief Access Layer Application Asynchronous Notification Callback.

    \par Description
    Access Layer calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] saddr         16 bit Source Address.
    \param [in] daddr         16 bit Destination Address.
    \param [in] appkey_handle AppKey Handle.
    \param [in] subnet_handle Subnet Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT sensor_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT         req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW             req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T               req_type;
    MS_ACCESS_MODEL_EXT_PARAMS*               ext_params_p;
    MS_ACCESS_MODEL_STATE_PARAMS            state_params;
    MS_STATE_SENSOR_DESCRIPTOR_STRUCT       param;
    MS_STATE_SENSOR_SERIES_COLUMN_STRUCT    param_series;
    MS_STATE_SENSOR_CADENCE_STRUCT          param_cadence;
    MS_STATE_SENSOR_SETTINGS_STRUCT         param_setting;
    MS_STATE_SENSOR_SETTING_STRUCT          param_setting1;
    UINT16  marker = 0;
//    UINT16        marker;
    API_RESULT    retval;
    retval = API_SUCCESS;
    ext_params_p = NULL;
    /* Request Context */
    req_context.handle = *handle;
    req_context.saddr  = saddr;
    req_context.daddr  = daddr;
    req_context.subnet_handle = subnet_handle;
    req_context.appkey_handle = appkey_handle;
    /* Request Raw */
    req_raw.opcode = opcode;
    req_raw.data_param = data_param;
    req_raw.data_len = data_len;
    printf(
        "[SENSOR_SERVER] Callback. Opcode 0x%06X\n", opcode);
    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_SENSOR_DESCRIPTOR_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_DESCRIPTOR_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

        if(data_len==0)
        {
            param.sensor_property_id = 0;
            state_params.state_type = MS_STATE_SENSOR_DESCRIPTOR_T;// update the special porperty id information
        }
        else if(data_len==2)
        {
            MS_UNPACK_LE_2_BYTE(&param.sensor_property_id,&data_param[marker]);
            marker += 2;

            if(param.sensor_property_id == 0)
            {
                return API_FAILURE;
            }

            state_params.state_type = MS_STATE_SENSOR_PROPERTY_ID_T;//update all information
        }

        state_params.state = &param;
    }
    break;

    case MS_ACCESS_SENSOR_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_GET_OPCODE\n");
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

        if(data_len==0)
        {
            param.sensor_property_id = 0;
            state_params.state_type = MS_STATE_SENSOR_DATA_T;// update the special porperty id information
        }
        else if(data_len==2)
        {
            MS_UNPACK_LE_2_BYTE(&param.sensor_property_id,&data_param[marker]);
            marker += 2;

            if(param.sensor_property_id == 0)
            {
                return API_FAILURE;
            }

            state_params.state_type = MS_STATE_SENSOR_DATA_PROPERTY_ID_T;//update all information
        }

        state_params.state = &param;
    }
    break;

    case MS_ACCESS_SENSOR_COLUMN_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_COLUMN_GET_OPCODE\n");
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        MS_UNPACK_LE_2_BYTE(&param_series.sensor_property_id,&data_param[marker]);
        marker += 2;

        if(param_series.sensor_property_id == 0)
        {
            return API_FAILURE;
        }

        state_params.state_type=MS_STATE_SENSOR_COLUMN_STATUS_T;//get column status
        state_params.state = &param_series;
    }
    break;

    case MS_ACCESS_SENSOR_SERIES_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_SERIES_GET_OPCODE\n");
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        MS_UNPACK_LE_2_BYTE(&param_series.sensor_property_id,&data_param[marker]);
        marker += 2;

        if(param_series.sensor_property_id == 0)
        {
            return API_FAILURE;
        }

        state_params.state_type=MS_STATE_SENSOR_SERIES_T;//get column status
        state_params.state = &param_series;
    }
    break;

    case MS_ACCESS_SENSOR_CADENCE_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_CADENCE_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        MS_UNPACK_LE_2_BYTE(&param_cadence.sensor_property_id,&data_param[marker]);
        marker += 2;

        if(param_cadence.sensor_property_id == 0)
        {
            return API_FAILURE;
        }

        state_params.state_type=MS_STATE_SENSOR_CADENCE_T;
        state_params.state = &param_cadence;
    }
    break;

    case MS_ACCESS_SENSOR_CADENCE_SET_OPCODE:
    case MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_CADENCE_SET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;

        if(opcode == MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE)
        {
            req_type.to_be_acked = 0x00;
        }
        else
        {
            req_type.to_be_acked = 0x01;
        }

        MS_UNPACK_LE_2_BYTE(&param_cadence.sensor_property_id,&data_param[marker]);
        marker += 2;
        param_cadence.fast_cadence_period_divisor = data_param[marker]&0x7f;
        param_cadence.status_trigger_type = data_param[marker]>>7;
        marker += 1;

        if(param_cadence.fast_cadence_period_divisor > 15)
        {
            return API_FAILURE;
        }

        MS_UNPACK_LE_N_BYTE(&param_cadence.status_trigger_delta_down[0],&data_param[marker],5);
        marker += 5;
        state_params.state_type=MS_STATE_SENSOR_CADENCE_T;
        state_params.state = &param_cadence;
    }
    break;

    case MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        MS_UNPACK_LE_2_BYTE(&param_setting.sensor_property_id,&data_param[marker]);
        marker += 2;

        if(param_setting.sensor_property_id == 0)
        {
            return API_FAILURE;
        }

        state_params.state_type=MS_STATE_SENSOR_SETTINGS_T;
        state_params.state = &param_setting;
    }
    break;

    case MS_ACCESS_SENSOR_SETTING_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        MS_UNPACK_LE_2_BYTE(&param_setting1.sensor_property_id,&data_param[marker]);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&param_setting1.sensor_setting_property_id,&data_param[marker]);
        marker += 2;

        if(param_setting1.sensor_property_id == 0)
        {
            return API_FAILURE;
        }

        state_params.state_type=MS_STATE_SENSOR_SETTING_T;
        state_params.state = &param_setting1;
    }
    break;

    case MS_ACCESS_SENSOR_SETTING_SET_OPCODE:
    {
        printf(
            "MS_ACCESS_SENSOR_SETTING_SET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;
        MS_UNPACK_LE_2_BYTE(&param_setting1.sensor_property_id,&data_param[marker]);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&param_setting1.sensor_setting_property_id,&data_param[marker]);
        marker += 2;
        param_setting1.sensor_setting_raw = 0x00;
        marker += 1;

        if(param_setting1.sensor_property_id == 0)
        {
            return API_FAILURE;
        }

        state_params.state_type=MS_STATE_SENSOR_SETTING_T;
        state_params.state = &param_setting1;
    }
    break;

    default:
        break;
    }

    /* Application callback */
    if (NULL != sensor_server_appl_cb)
    {
        sensor_server_appl_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
    }

    return retval;
}



/**
    \brief API to initialize MS_Sensor_server_init Server model

    \par Description
    This is to initialize Sensor_Example_1 Server model and to register with Acess layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] UI_cb    Application Callback to be used by the Vendor_Example_1 Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_sensor_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*       sensor_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*       sensor_setup_model_handle,
    /* IN */    MS_SENSOR_SERVER_CB         appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    SENSOR_SERVER_TRC(
        "[sensor server] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_SENSOR_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callback */
    model.cb = sensor_server_cb;
    model.pub_cb = sensor_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = sensor_server_opcode_list;
    model.num_opcodes = sizeof(sensor_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 sensor_model_handle
             );
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_SENSOR_SETUP_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = sensor_server_cb;
    model.pub_cb = sensor_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = sensor_setup_server_opcode_list;
    model.num_opcodes = sizeof(sensor_setup_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model(
                 node_id,
                 &model,
                 sensor_setup_model_handle);
    /* Save Application Callback */
    sensor_server_appl_cb = appl_cb;
    sensor_server_model_handle = *sensor_model_handle;
    return retval;
}

/**
    \brief Access Layer Model Publication Timeout Callback.

    \par Description
    Access Layer calls the registered callback to indicate Publication Timeout
    for the associated model.

    \param [in]  handle        Model Handle.
    \param [out] blob          Blob if any or NULL.
*/
API_RESULT sensor_server_publish_timout_cb(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    return API_FAILURE;
}

API_RESULT MS_sensor_data_pubish
(
    UINT32 value
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    retval = API_FAILURE;
    marker = 0;
    buffer[marker] = 0x01;
    marker +=1;
    MS_PACK_LE_2_BYTE_VAL(&buffer[marker], 0x42);
    marker += 2;
    buffer[marker] = value;
    marker++;

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    retval = MS_access_publish
             (
                 &sensor_server_model_handle,
                 MS_ACCESS_SENSOR_STATUS_OPCODE,
                 pdu_ptr,
                 marker,
                 MS_TRUE
             );
    return retval;
}


