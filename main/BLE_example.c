#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"


#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

enum
{
    service_AppId                = 0x77,
};

enum
{
    gattServerPrimaryServiceUuid               = 0x2800,
    gattServerCharDeclarationUuid              = 0x2803,
    gattServerClientCharCfgUuid                = 0x2902,

    service_shortUuid                    = 0x1234, //MAKE THIS WHAT YOU NEED
};


enum
{
    txDataAttr_maxLen    = 20,
    rxDataAttr_maxLen    = 20,

    attrIndex_gattServer_service = 0,

    attrIndex_gattServer_incomingData,
    attrIndex_incomingData,

    attrIndex_gattServer_txData,
    attrIndex_txData,
    attrIndex_gattCharConfig_txData,

    attrIndex_num
};


/////////////////////////////////////////////////////////////////////////////
// Advertising data and params

static uint8_t service_data[16] = {
    /* LSB <--------------------------------------------------> MSB */
       //16 bytes
};


esp_ble_adv_data_t gatt_adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,
        .min_interval = 0, //
        .max_interval = 0, //
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data =  0,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 16,
        .p_service_uuid = service_data,
        .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
    };

esp_ble_adv_params_t gatt_adv_params = {
        .adv_int_min        = 0x20,
        .adv_int_max        = 0x40,
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
    //    .peer_addr          =
    //    .peer_addr_type     = BLE_ADDR_TYPE_PUBLIC,
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };



/////////////////////////////////////////////////////////////////////////////
// Service and characteristic table
static const uint16_t service_uuid = service_shortUuid;

static const uint16_t primary_service_uuid          = gattServerPrimaryServiceUuid;
static const uint16_t character_declaration_uuid    = gattServerCharDeclarationUuid;
static const uint16_t character_client_config_uuid  = gattServerClientCharCfgUuid;

static const uint8_t char_prop_notify       = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_notify  = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read         = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write        = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write   = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

static uint8_t gattcharConfig_outgoingData[2] = {0x00, 0x00};
static uint8_t attrValue_rxData[rxDataAttr_maxLen];
static uint8_t attrValue_txData[txDataAttr_maxLen];


static uint8_t incomingData_uuid[ESP_UUID_LEN_128] = {
        //16 bytes of what you want your UUID to be
};

static uint8_t outgoingData_uuid[ESP_UUID_LEN_128] = {
        //16 bytes of what you want your UUID to be
};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t gatt_db[attrIndex_num] = {

    // Service (primary service)
    [attrIndex_gattServer_service] = {  {ESP_GATT_AUTO_RSP}
                                                     , {  ESP_UUID_LEN_16
                                                        , (uint8_t*)&primary_service_uuid
                                                        , ESP_GATT_PERM_READ
                                                        , sizeof(uint16_t)
                                                        , sizeof(service_uuid)
                                                        , (uint8_t*)&service_uuid
                                                       }
                                                    },
    // Incoming Data characteristic declaration
    [attrIndex_gattServer_rxData]        = {  {ESP_GATT_AUTO_RSP}
                                                     , {  ESP_UUID_LEN_16
                                                        , (uint8_t*)&character_declaration_uuid
                                                        , ESP_GATT_PERM_READ
                                                        , CHAR_DECLARATION_SIZE
                                                        , CHAR_DECLARATION_SIZE
                                                        , (uint8_t*)&char_prop_write
                                                       }
                                                    },
    // Incoming Data Characteristic Value
    [attrIndex_rxData]                   = {  {ESP_GATT_RSP_BY_APP}
                                                     , {  ESP_UUID_LEN_128
                                                        , (uint8_t*)incomingData_uuid
                                                        , ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE
                                                        , incomingDataAttr_maxLen
                                                        , 0
                                                        , NULL
                                                       }
                                                   },
    // Outgoing Data Characteristic Declaration
    [attrIndex_gattServer_txData]        = {  {ESP_GATT_AUTO_RSP}
                                                     , {  ESP_UUID_LEN_16
                                                        , (uint8_t*)&character_declaration_uuid
                                                        , ESP_GATT_PERM_READ
                                                        , CHAR_DECLARATION_SIZE
                                                        , CHAR_DECLARATION_SIZE
                                                        , (uint8_t*)&char_prop_read_notify
                                                       }
                                                    },
    // Outgoing Data Characteristic Value
    [attrIndex_txData]                   = {  {ESP_GATT_RSP_BY_APP}
                                                     , {  ESP_UUID_LEN_128
                                                        , (uint8_t*)outgoingData_uuid
                                                        , ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE
                                                        , outgoingDataAttr_maxLen
                                                        , 0
                                                        , NULL
                                                       }
                                                    },
    // Outgoing Data Characteristic Value - Client Characteristic Configuration Descriptor
    [attrIndex_gattCharConfig_txData]    = {  {ESP_GATT_AUTO_RSP}
                                                     , {  ESP_UUID_LEN_16
                                                        , (uint8_t*)&character_client_config_uuid
                                                        , ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE
                                                        , sizeof(uint16_t)
                                                        , sizeof(gattcharConfig_outgoingData)
                                                        , (uint8_t *)&gattcharConfig_outgoingData
                                                       }
                                                    },
};

uint16_t handle_table[attrIndex_num];

/////////////////////////////////////////////////////////////////////////////
// Other variables

static uint16_t getAttributeIndexByHandle(uint16_t attributeHandle)
{
    uint16_t attrIndex = 0;
    uint16_t index;

    for(index = 0; index < attrIndex_num; ++index)
    {
        if( handle_table[index] == attributeHandle )
        {
            attrIndex = index;
            break;
        }
    }

    return attrIndex;
}



/////////////////////////////////////////////////////////////////////
// Functions to handle gatt events

static void handleDisconnectEvent(void* p)
{
    REQUIRE(param);

    TRACE(("BLE GATT DISCONNECT_EVT , conn_id %x, is_conn %d\n", p->disconnect.conn_id, p->disconnect.is_connected ));
}



static void handleConnectEvent(void* p)
{
    REQUIRE(param);

    TRACE(("BLE GATT CONNECT_EVT, conn_id %x, is_conn %d\n", p->connect.conn_id, p->connect.is_connected));
}



static void handleGattReadEvent(esp_gatt_if_t gatts_if, void *param)
{
    REQUIRE(param);

    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;

    int attrIndex = getAttributeIndexByHandle(param->read.handle);

    switch( attrIndex )
    {       
    case addAttrIndex_txData:
        TRACE(("Read outgoingData\n"));
        memcpy(rsp.attr_value.value, txData, txDataAttr_maxLen);
        rsp.attr_value.len = txDataAttr_maxLen;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    default:
        TRACE(("Error - attribute read has invalid handle: 0x%X\n", p->read.handle));
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_READ_NOT_PERMIT, NULL);
        break;
    }
}



static void handleGattWriteEvent(esp_gatt_if_t gatts_if, void *param)
{
    REQUIRE(param);

    esp_ble_gatts_cb_param_t *callbackParam = (esp_ble_gatts_cb_param_t *)param;
    esp_gatt_status_t status = ESP_GATT_OK;
    
    int attrIndex = getAttributeIndexByHandle(param->write.handle)
    
    switch( attrIndex )
    {
    case attrIndex_rxData:
        xTRACE(("Rx BLE Data\n"));
        handleBleRxData(...); //However you decide to implement
        break;

    default:
        TRACE(("Error: Attribute not writeable\n"));
        break;
    }

 
}



static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p = (esp_ble_gatts_cb_param_t *)param;

    switch (event) {
    case ESP_GATTS_REG_EVT:
        connectionInfo.gatt_if = gatts_if;
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, attrIndex_num, 0);
        main_postEvent( state_event_bleInterface_ready );
        break;
    case ESP_GATTS_READ_EVT:
        handleGattReadEvent(gatts_if, param);
        break;
    case ESP_GATTS_WRITE_EVT:
        handleGattWriteEvent(gatts_if, param);
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        break;
    case ESP_GATTS_MTU_EVT:
    	TRACE(("MTU Size: %d, Connection ID: 0x%X\n", p->mtu.mtu, p->mtu.conn_id));
        break;
    case ESP_GATTS_CONF_EVT:    //When receive confirm, the event comes
        xTRACE(("CONFIRM EVENT\n"));
        break;
    case ESP_GATTS_UNREG_EVT:
        TRACE(("UNREG EVENT\n"));
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        TRACE(("SERVICE_START_EVT, status %d, service_handle %d\n", p->start.status, p->start.service_handle));
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        handleConnectEvent(param);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        handleDisconnectEvent(param);
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if(param->add_attr_tab.num_handle == attrIndex_num)
        {
            #ifdef BLEINTERFACE_GATT_TRACE_VERBOSE
                print("HANDLES:\n");
                for(int i = 0; i < attrIndex_num; ++i)
                {
                    print("  0x%04X\n", param->add_attr_tab.handles[i]);
                }
            #endif

            memcpy(handle_table, param->add_attr_tab.handles, attrIndex_num*2);
            esp_ble_gatts_start_service(handle_table[attrIndex_gattServer_server]);
        }
        break;
    default:
        break;
    }
}



esp_err_t bleInterface_gatt_sendNotification(size_t len, uin8_t *data, uint16_t attributeHandle)
{

    while( len > 0 )
    {
        uint16_t sendSize = len > maxNotificationSize ? maxNotificationSize : len;
        
        esp_err_t error = esp_ble_gatts_send_indicate(  gatts_if
                                                      , conn_id
                                                      , attributeHandle
                                                      , sendSize
                                                      , data
                                                      , false
                                                     );
        if( error != ESP_OK )
        {
            printf("Send notification failed!\n");
            return error;
        }
        
        len -= sendSize;
    }

    return ESP_OK;
}



void bleInterface_gatt_start(void)
{
    const char* localName = "gattservername";

    esp_ble_gap_set_device_name((const char*)localName);
    esp_ble_gap_config_adv_data(&gatt_adv_data);
}



void bleInterface_gatt_init(void)
{
	//Need to make some other calls here as well. Look at esp-idf gatt_server_service_table example for that


    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(service_AppId);
}