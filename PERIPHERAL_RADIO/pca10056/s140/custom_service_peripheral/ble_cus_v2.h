#ifndef BLE_CUS_H__
#define BLE_CUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"

/**@brief   Macro for defining a ble_cus instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 Serve per registrare ble_cus_on_ble_evt a SoftDevice
 */
#define BLE_CUS_DEF(_name)                                                                          \
static ble_cus_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_cus_on_ble_evt, &_name)



// CUSTOM_SERVICE_UUID_BASE 1f740325-75a9-4f6d-8de2-8fcc12ecdb81 

#define CUSTOM_SERVICE_UUID_BASE         {0x81, 0xDB, 0xEC, 0x12, 0xCC, 0x8F, 0xE2, 0x8D, \
                                          0x6D, 0x4F, 0xA9, 0x75, 0x25, 0x03, 0x74, 0x1F}

#define CUSTOM_SERVICE_UUID               0xA975
#define CUSTOM_VALUE_CHAR_UUID            0xA976
/* SPOSTATO IN sdk_config.h
//Define for characteristic
#define MAX_PACKET_SIZE                   4        /*Numero di celle nell'array del payload
#define SIZE_OF_ARRAY                     1        //Dimensione in byte di ogni cella nell'array (4: uint32_t, 1: uint8_t)
#define OFFSET                            0        //Offset utilizzato per mandare i byte
*/																				
/**@brief Custom Service event type. */
typedef enum
{
    BLE_CUS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. Quando viene scritto 1 nel CCCD*/ 
    BLE_CUS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. Quando viene scritto 0 nel CCCD */
    BLE_CUS_EVT_WRITE_SUCCESS,                                    /**< Quando la caratteristica viene scritta.*/    
    BLE_CUS_EVT_DISCONNECTED,
    BLE_CUS_EVT_CONNECTED
} ble_cus_evt_type_t;

/**@brief Custom Service event. */
typedef struct
{
    ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
} ble_cus_evt_t;

// Forward declaration of the ble_cus_t type.
typedef struct ble_cus_s ble_cus_t;


/**@brief Custom Service event handler type. */
typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_bas, ble_cus_evt_t * p_evt);

/**@brief Battery Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint32_t*                     initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_s
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      custom_value_handles;           /**< Handles related to the Custom Value characteristic. It is assigned by SoftDevice in automatic! */ 
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Custom Service.
 *
 * @note 
 *
 * @param[in]   p_cus      Custom Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_bas          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t* custom_value);

#endif // BLE_CUS_H__
