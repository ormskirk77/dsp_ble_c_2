/*
 * main.c
 *
 *  Created on: 29 Aug 2019 TODAY
 *      Author: timfernandez
 */



 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/event_groups.h"
 #include "esp_system.h"
 #include "esp_log.h"
 #include "nvs_flash.h"
 #include "esp_bt.h"


#include <string.h>

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"

#include "esp_gatt_common_api.h"

#include "dsp_ble.h"
#include "coefficient_utils.h"
#include "SigmaStudioFW.h"
#include "SigmaStudioFiles/Design 1_IC_1.h"

#define DSP_TABLE_TAG "DSP_TAG"

#define SDA_PIN 21
#define SCL_PIN 22

#define VOL_PARAM_ADDR			0x19
#define BIQUAD_PARAM_BASE_ADDR	0x14

#define PROFILE_NUM                 1
#define PROFILE_IDX        		    0
#define DSP_APP_ID                  0x55
#define DEVICE_NAME          "CHILTERN AUDIO"
#define SVC_INST_ID                 0


#define CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)



uint16_t dsp_control_handle_table[NUM_TABLE_ELEMENTS];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;


static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};


static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};










static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x00FF;
static const uint16_t GATTS_VOL_UUID_TEST_A        = 0xFF01;
static const uint16_t GATTS_Q_UUID				   = 0xFF02;
static const uint16_t GATTS_GAIN_UUID			   = 0xFF03;
static const uint16_t GATTS_CUTFREQ_UUID		   = 0xFF04;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;

ADI_REG_TYPE DSP_VOLUME_LEVEL[4] 	= {0x00, 0x80, 0x00, 0x00};
ADI_REG_TYPE DSP_Q_LEVEL[4] 		= {0x00, 0x80, 0x00, 0x00};
ADI_REG_TYPE DSP_GAIN_LEVEL[4]		= {0x00, 0x80, 0x00, 0x00};
ADI_REG_TYPE DSP_CUTFREQ_LEVEL[4]	= {0x00, 0x80, 0x00, 0x00};

float Q = 0.7071;
float gain = 0.0;
float cutFreq = 300;


static const esp_gatts_attr_db_t dsp_db[NUM_TABLE_ELEMENTS] =
{		//DSP CONTROLLER SERVICE
				[IDX_SVC] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&primary_service_uuid,
						ESP_GATT_PERM_READ,
						sizeof(uint16_t),
						sizeof(GATTS_SERVICE_UUID_TEST),
						(uint8_t *)&GATTS_SERVICE_UUID_TEST}},

// ========================= VOLUME ========================================

				[IDX_CHAR_VOL] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&character_declaration_uuid,
						ESP_GATT_PERM_READ,
						CHAR_DECLARATION_SIZE,
						CHAR_DECLARATION_SIZE,
						(uint8_t *)&char_prop_read_write}},

//				//DSP CONTROLLER VOLUME LEVEL
				[IDX_CHAR_VOL_VAL] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&GATTS_VOL_UUID_TEST_A,
						ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
						CHAR_VAL_LEN_MAX,
						sizeof(DSP_VOLUME_LEVEL),
						(uint8_t *)&DSP_VOLUME_LEVEL}},
					    /* Characteristic Declaration */

// ========================= QUALITY ========================================

				[IDX_CHAR_Q] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&character_declaration_uuid,
						ESP_GATT_PERM_READ,
						CHAR_DECLARATION_SIZE,
						CHAR_DECLARATION_SIZE,
						(uint8_t *)&char_prop_read_write}},

//				//DSP CONTROLLER VOLUME LEVEL
				[IDX_CHAR_Q_VAL] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&GATTS_Q_UUID,
						ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
						CHAR_VAL_LEN_MAX,
						sizeof(DSP_Q_LEVEL),
						(uint8_t *)&DSP_Q_LEVEL}},

// ========================= GAIN ========================================

				[IDX_CHAR_GAIN] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&character_declaration_uuid,
						ESP_GATT_PERM_READ,
						CHAR_DECLARATION_SIZE,
						CHAR_DECLARATION_SIZE,
						(uint8_t *)&char_prop_read_write}},

//				//DSP CONTROLLER VOLUME LEVEL
				[IDX_CHAR_GAIN_VAL] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&GATTS_GAIN_UUID,
						ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
						CHAR_VAL_LEN_MAX,
						sizeof(DSP_GAIN_LEVEL),
						(uint8_t *)&DSP_GAIN_LEVEL}},


// ========================= CUT OFF FREQ ========================================
				[IDX_CHAR_CUTFREQ] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&character_declaration_uuid,
						ESP_GATT_PERM_READ,
						CHAR_DECLARATION_SIZE,
						CHAR_DECLARATION_SIZE,
						(uint8_t *)&char_prop_read_write}},

//				//DSP CONTROLLER VOLUME LEVEL
				[IDX_CHAR_CUTFREQ_VAL] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16,
						(uint8_t *)&GATTS_CUTFREQ_UUID,
						ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
						CHAR_VAL_LEN_MAX,
						sizeof(DSP_CUTFREQ_LEVEL),
						(uint8_t *)&DSP_CUTFREQ_LEVEL}},

};






















static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{

	switch(event) {

		case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
			esp_ble_gap_start_advertising(&adv_params);
		break;
		case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
			esp_ble_gap_start_advertising(&adv_params);
		break;
		case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
			printf("ESP_GAP_BLE_ADV_START_COMPLETE_EVT called.\n");
			if(param->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS){
				ESP_LOGE(DSP_TABLE_TAG, "Advertising start failed. \n");
			}
		break;

		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
			break;
		case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(DSP_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
			break;
		default :
			break;
	}

}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst dsp_profile_tab[NUM_TABLE_ELEMENTS] = {
    [PROFILE_IDX] = {
        .gatts_cb = gatts_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch(event) {
			case ESP_GATTS_REG_EVT:
				esp_ble_gap_set_device_name(DEVICE_NAME);

				dsp_profile_tab[PROFILE_IDX].gatts_if = gatts_if;

				esp_ble_gatts_create_attr_tab(dsp_db, gatts_if, NUM_TABLE_ELEMENTS, SVC_INST_ID);

	            esp_ble_gap_config_adv_data(&adv_data);
	            esp_ble_gap_config_adv_data(&scan_rsp_data);
			break;
			case ESP_GATTS_CREATE_EVT:
				break;
			case ESP_GATTS_READ_EVT:
				break;
			case ESP_GATTS_WRITE_EVT:
				if(dsp_control_handle_table[IDX_CHAR_VOL_VAL] == param->write.handle){
					float newVol = *param->write.value;
					newVol = newVol/100;
					process_coefficient_for_i2c(newVol, DSP_VOLUME_LEVEL);
					SIGMA_SAFELOAD_SINGLE(0x34, VOL_PARAM_ADDR, DSP_VOLUME_LEVEL);


				} else if (dsp_control_handle_table[IDX_CHAR_Q_VAL] == param->write.handle){
					float newQ = *param->write.value;
					newQ = newQ/25;
					float *ptr = calculate_coefficients(low_pass, gain, newQ, cutFreq);
					SIGMA_SAFELOAD_BIQUAD(0x34, BIQUAD_PARAM_BASE_ADDR, ptr);

				} else if (dsp_control_handle_table[IDX_CHAR_CUTFREQ_VAL] == param->write.handle) {
					float newCutFreq = *param->write.value *10;
					printf("New cut-off frequency value: %f\n", newCutFreq);
					printf("Q: %f  |  newCutFreq: %f\n", Q, newCutFreq);
					float *ptr = calculate_coefficients(low_pass, gain, Q, newCutFreq);
				 	SIGMA_SAFELOAD_BIQUAD(0x34, BIQUAD_PARAM_BASE_ADDR, ptr);

				} else if (dsp_control_handle_table[IDX_CHAR_GAIN_VAL] == param->write.handle){
					float newGain = *param->write.value ;
					printf("New gain value: %f\n", gain);
					printf("Q: %f  |  newCutFreq: %f\n", Q, newGain);
					float *ptr = calculate_coefficients(low_pass, newGain, Q, cutFreq);
				 	SIGMA_SAFELOAD_BIQUAD(0x34, BIQUAD_PARAM_BASE_ADDR, ptr);
				}


				break;
			case ESP_GATTS_EXEC_WRITE_EVT:
				break;
			case ESP_GATTS_CONF_EVT:
				break;
			case ESP_GATTS_MTU_EVT:
				break;
			case ESP_GATTS_START_EVT:
				break;
			case ESP_GATTS_CONNECT_EVT:
			      esp_log_buffer_hex(DSP_TABLE_TAG, param->connect.remote_bda, 6);
					            esp_ble_conn_update_params_t conn_params = {0};
					            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
					            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
					            conn_params.latency = 0;
					            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
					            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
					            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
					            //start sent the update connection parameters to the peer device.
					            esp_ble_gap_update_conn_params(&conn_params);



				break;
			case ESP_GATTS_DISCONNECT_EVT:
				printf("gatts_profile_event_handler: ");
				printf("ESP_GATTS_DISCONNECT_EVT called.\n");
	            esp_ble_gap_config_adv_data(&adv_data);
	            esp_ble_gap_config_adv_data(&scan_rsp_data);
				break;
			case ESP_GATTS_CREAT_ATTR_TAB_EVT:
					printf("ESP_GATTS_CREAT_ATTR_TAB_EVT.\n");

	                memcpy(dsp_control_handle_table, param->add_attr_tab.handles, sizeof(dsp_control_handle_table));
	                esp_ble_gatts_start_service(dsp_control_handle_table[IDX_SVC]);
	                ESP_LOGI(DSP_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);

				break;
	        case ESP_GATTS_STOP_EVT:
	        	break;
	        case ESP_GATTS_OPEN_EVT:
	        	break;
	        case ESP_GATTS_CANCEL_OPEN_EVT:
	        	break;
	        case ESP_GATTS_CLOSE_EVT:
	        	break;
	        case ESP_GATTS_LISTEN_EVT:
	        	break;
	        case ESP_GATTS_CONGEST_EVT:
	        	break;
	        case ESP_GATTS_UNREG_EVT:
	        	break;
	        case ESP_GATTS_DELETE_EVT:
	        	break;
	        default:
	            break;
			}

}



void app_main()
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(DSP_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(DSP_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(DSP_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(DSP_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(DSP_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(DSP_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(DSP_APP_ID);
    if (ret){
        ESP_LOGE(DSP_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(DSP_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

 //   default_download_IC_1();
}
