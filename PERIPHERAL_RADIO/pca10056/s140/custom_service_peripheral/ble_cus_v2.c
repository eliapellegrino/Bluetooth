#include "sdk_common.h"
#include "ble_cus_v2.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
 
static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
 
static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_DISCONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_cus_evt_t evt;
    // Custom Value Characteristic Written to.
    // Quando la caratteristica viene scritta, si entra qua dentro e viene eseguito questo pezzo di codice.
    // Per far si che il l'evento sia gestibile dall'applicazione, mi comporto come negli altri casi: ovvero creo un evento associato che potrò richiamare nell'applicazione e gestirlo da li
    if (p_evt_write->handle == p_cus->custom_value_handles.value_handle)
    {
        evt.evt_type = BLE_CUS_EVT_WRITE_SUCCESS;
        /*
        CONTENUTO DELL'ESEMPIO
        nrf_gpio_pin_toggle(LED_4);
        /*
        if(*p_evt_write->data == 0x01)
        {
            nrf_gpio_pin_clear(20); 
        }
        else if(*p_evt_write->data == 0x02)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
          //Do nothing
        }
        */
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_cus->custom_value_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {
        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {
            

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            
        }
    }
    // Call the application event handler.
    p_cus->evt_handler(p_cus, &evt);

}
 // Questa funzione viene richiamata ad ogni evento BLE. Quando scriviamo il main, questa è però trasparente, nel senso che i nostri eventi li propaghiamo alla applicazione tramite typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_bas, ble_cus_evt_t * p_evt);
 // In questo modo possiamo "nascondere" ciò che avviene qua dentro, e organizzare il tutto da dentro la nostra applicazione dichiarando una una funzione con il prototipo sopra e associarlo alla struttura usata per fare l'init
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_t * p_cus = (ble_cus_t *) p_context;
    
    //NRF_LOG_INFO("BLE event received. Event type = %d\r\n", p_ble_evt->header.evt_id); 
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
    
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cus, p_ble_evt);
            break;
    
        case BLE_GATTS_EVT_WRITE:
            on_write(p_cus, p_ble_evt);
            break;
/* Handling this event is not necessary
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.\r\n");
            break;
*/
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_value_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    /*
    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);

    
    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles);

    */
    // Si può usare ache characteristic_add (è lo stesso, ma è ad un livello sopra e non permette alcune operazioni che si può fare con sd_ble_gatts_characteristic_add)
    ble_add_char_params_t  add_char_params;
    memset(&add_char_params, 0, sizeof(add_char_params));

    uint8_t* initial_value = p_cus_init->initial_custom_value;

    add_char_params.uuid              = CUSTOM_VALUE_CHAR_UUID;
    add_char_params.max_len           = MAX_PACKET_SIZE*SIZE_OF_ARRAY;
    add_char_params.init_len          = MAX_PACKET_SIZE*SIZE_OF_ARRAY;
    add_char_params.p_init_value      = initial_value;
    add_char_params.char_props.notify = 1;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;
    add_char_params.char_props.indicate = 1;

    // SEC_OPEN: nessuna sicurezza per leggere o scrivere il CCCD
    add_char_params.cccd_write_access = SEC_OPEN;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access       = SEC_OPEN;


    err_code = characteristic_add(p_cus->service_handle,
                                  &add_char_params,
                                  &(p_cus->custom_value_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Custom Value characteristic
    return custom_value_char_add(p_cus, p_cus_init);
}

// Cambiato da uint8_t a uint8_t*
uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t* custom_value)
{
    int8_t i;
    //NRF_LOG_INFO("In ble_cus_custom_value_update. \r\n"); 
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;

    ble_gatts_value_t gatts_value;
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = MAX_PACKET_SIZE*SIZE_OF_ARRAY;
    gatts_value.offset  = OFFSET;
    gatts_value.p_value = custom_value;
    /*
    Avrei potuto usare anche sd_ble_gatts_hvx: la differenza tra  i due è che:
    - sd_ble_gatts_hvx: cambia lo stato dell'attributo e lo notifica a tutti i device che sono connessi
    - sd_ble_gatts_value_set: setta semplicemente il nuovo valore dell'attributo: i device connessi non sono notificati del cambio valore, ma se decidono di leggerlo vedranno solamente l'ultimo valore cambiato
    Questa funzione oltre a cambiare l'attributo scelto, lo notifica anche in caso siamo in una connessione valida
    In pratica la prima fa anche il lavoro della seconda
    */
    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->custom_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;
        do {
        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        }while (err_code == NRF_ERROR_RESOURCES);
        //APP_ERROR_CHECK(err_code);
        //NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
        //NRF_LOG_INFO("PACKET SENT: %x", hvx_params.p_data[0]); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }


    return err_code;
}

uint8_t* cut_uint32(uint32_t* custom_value)
{
    int8_t i;
    uint8_t payload[MAX_PACKET_SIZE*SIZE_OF_ARRAY];
    // Spezzo il custom_value lungo 32 bit
    for (i = 0; i < MAX_PACKET_SIZE; i++)
    {
        payload[0+i*MAX_PACKET_SIZE] = (custom_value[i] & 0x000000ff);
        payload[1+i*MAX_PACKET_SIZE] = (custom_value[i] & 0x0000ff00) >> 8;
        payload[2+i*MAX_PACKET_SIZE] = (custom_value[i] & 0x00ff0000) >> 16;
        payload[3+i*MAX_PACKET_SIZE] = (custom_value[i] & 0xff000000) >> 24; 
    }
}