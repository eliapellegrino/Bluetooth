/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@file
 *
 * @defgroup ble_cus_c Heart Rate Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Heart Rate Service Client module.
 *
 * @details  This module contains the APIs and types exposed by the Heart Rate Service Client
 *           module. The application can use these APIs and types to perform the discovery of
 *           Heart Rate Service at the peer and to interact with it.
 *
 * @warning  Currently, this module only supports the Heart Rate Measurement characteristic. This
 *           means that it is able to enable notification of the characteristic at the peer and
 *           is able to receive Heart Rate Measurement notifications from the peer. It does not
 *           support the Body Sensor Location and the Heart Rate Control Point characteristics.
 *           When a Heart Rate Measurement is received, this module decodes only the
 *           Heart Rate Measurement value field (both 8-bit and 16-bit) and provides it to
 *           the application.
 *
 * @note    The application must register this module as the BLE event observer by using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_cus_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_HRS_C_BLE_OBSERVER_PRIO,
 *                                   ble_cus_c_on_ble_evt, &instance);
 *          @endcode
 */

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "sdk_config.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"

/* SPOSTATE IN SDK_CONFIG.h
#define MAX_PACKET_SIZE 4
#define SIZE_OF_ARRAY 1
*/
#define BLE_CUS_C_BLE_OBSERVER_PRIO 2
/**@brief   Macro for defining a ble_cus_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CUS_C_DEF(_name)                                                                        \
static ble_cus_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_CUS_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_cus_c_on_ble_evt, &_name)

/** @brief Macro for defining multiple ble_cus_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_CUS_C_ARRAY_DEF(_name, _cnt)                 \
static ble_cus_c_t _name[_cnt];                          \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
                      BLE_CUS_C_BLE_OBSERVER_PRIO,       \
                      ble_cus_c_on_ble_evt, &_name, _cnt)


#define CUSTOM_SERVICE_UUID_BASE         {0x81, 0xDB, 0xEC, 0x12, 0xCC, 0x8F, 0xE2, 0x8D, \
                                          0x6D, 0x4F, 0xA9, 0x75, 0x25, 0x03, 0x74, 0x1F}

#define CUSTOM_SERVICE_UUID               0xA975
#define CUSTOM_VALUE_CHAR_UUID            0xA976
/**
 * @defgroup cus_c_enums Enumerations
 * @{
 */

/**@brief Cus Client event type. */
typedef enum
{
    BLE_CUS_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the Custom Service was discovered at the peer. */
    BLE_CUS_C_EVT_NOTIFICATION         /**< Event indicating that a notification of the Heart Rate Measurement characteristic was received from the peer. */
} ble_cus_c_evt_type_t;

/** @} */

/**
 * @defgroup cus_c_structs Structures
 * @{
 */



/**@brief Structure containing the handles related to the Heart Rate Service found on the peer. */
typedef struct
{
    uint16_t cus_cccd_handle;  /**< Handle of the CCCD of the Heart Rate Measurement characteristic. */
    uint16_t cus_handle;       /**< Handle of the Heart Rate Measurement characteristic, as provided by the SoftDevice. */
} cus_db_t;

/**@brief Heart Rate Event structure. */
typedef struct
{
    ble_cus_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the Heart Rate service was discovered on the peer device..*/
    union {
    uint8_t 			*p_data;	  /* Dati ricevuti*/		
    cus_db_t            peer_db;
    } params;
} ble_cus_c_evt_t;

/** @} */

/**
 * @defgroup cus_c_types Types
 * @{
 */

// Forward declaration of the ble_cus_t type.
typedef struct ble_cus_c_s ble_cus_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that is to be provided by the application
 *          of this module to receive events.
 */
typedef void (* ble_cus_c_evt_handler_t) (ble_cus_c_t * p_ble_cus_c, ble_cus_c_evt_t * p_evt);

/** @} */

/**
 * @addtogroup cus_c_structs
 * @{
 */

/**@brief Heart Rate Client structure.
 */
struct ble_cus_c_s
{
    uint16_t                conn_handle;   /**< Connection handle, as provided by the SoftDevice. */
    cus_db_t                peer_cus_db;   /**< Handles related to HRS on the peer. */
    ble_cus_c_evt_handler_t evt_handler;   /**< Application event handler to be called when there is an event related to the Heart Rate Service. */
    ble_srv_error_handler_t error_handler; /**< Function to be called in case of an error. */
    nrf_ble_gq_t          * p_gatt_queue;  /**< Pointer to the BLE GATT Queue instance. */
    uint8_t                   uuid_type;     /**< UUID type. */
};

/**@brief Heart Rate Client initialization structure.
 */
typedef struct
{
    ble_cus_c_evt_handler_t   evt_handler;   /**< Event handler to be called by the Heart Rate Client module when there is an event related to the Heart Rate Service. */
    ble_srv_error_handler_t   error_handler; /**< Function to be called in case of an error. */
    nrf_ble_gq_t            * p_gatt_queue;  /**< Pointer to the BLE GATT Queue instance. */
} ble_cus_c_init_t;

/** @} */


/**
 * @defgroup cus_c_functions Functions
 * @{
 */

/**@brief     Function for initializing the Heart Rate Client module.
 *
 * @details   This function registers with the Database Discovery module for the Heart Rate Service.
 *		   	  The module looks for the presence of a Heart Rate Service instance at the peer
 *            when a discovery is started.
 *
 * @param[in] p_ble_cus_c      Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_cus_c_init Pointer to the Heart Rate initialization structure that contains
 *                             the initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. 
 * @retval    err_code    Otherwise, this function propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_cus_c_init(ble_cus_c_t * p_ble_cus_c, ble_cus_c_init_t * p_ble_cus_c_init);


/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function handles the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the Heart Rate Client module, the function uses the event's data to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the Heart Rate Client structure.
 */
void ble_cus_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for requesting the peer to start sending notification of Heart Rate
 *          Measurement.
 *
 * @details This function enables notification of the Heart Rate Measurement at the peer
 *          by writing to the CCCD of the Heart Rate Measurement characteristic.
 *
 * @param   p_ble_cus_c Pointer to the Heart Rate Client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice is requested to write to the CCCD of the peer.
 * @retval	err_code	Otherwise, this function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_cus_c_notif_enable(ble_cus_c_t * p_ble_cus_c);


/**@brief     Function for handling events from the Database Discovery module.
 *
 * @details   Call this function when you get a callback event from the Database Discovery module.
 *            This function handles an event from the Database Discovery module and determines
 *            whether it relates to the discovery of Heart Rate Service at the peer. If it does, the function 
 *            calls the application's event handler to indicate that the Heart Rate Service was
 *            discovered at the peer. The function also populates the event with service-related
 *            information before providing it to the application.
 *
 * @param[in] p_ble_cus_c Pointer to the Heart Rate Client structure instance for associating the link.
 * @param[in] p_evt Pointer to the event received from the Database Discovery module.
 *
 */
void ble_cus_on_db_disc_evt(ble_cus_c_t * p_ble_cus_c, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning handles to an instance of cus_c.
 *
 * @details   Call this function when a link has been established with a peer to
 *            associate the link to this instance of the module. This association makes it
 *            possible to handle several links and associate each link to a particular
 *            instance of this module. The connection handle and attribute handles are
 *            provided from the discovery event @ref BLE_HRS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_cus_c        Pointer to the Heart Rate Client structure instance for associating the link.
 * @param[in] conn_handle        Connection handle to associate with the given Heart Rate Client Instance.
 * @param[in] p_peer_cus_handles Attribute handles for the HRS server you want this HRS_C client to
 *                               interact with.
 */
uint32_t ble_cus_c_handles_assign(ble_cus_c_t *    p_ble_cus_c,
                                  uint16_t         conn_handle,
                                  const cus_db_t * p_peer_cus_handles);

/** @} */ // End tag for Function group.


/**@brief Function for writing 
 *
 * @param[in] p_ble_lbs_c Pointer to the LED Button client structure.
 * @param[in] status      LED status to send.
 *
 * @retval NRF_SUCCESS If the status was sent successfully.
 * @retval err_code    Otherwise, this API propagates the error code returned by function
 *                     @ref nrf_ble_gq_conn_handle_register.
 */
uint32_t ble_cus_write(ble_cus_c_t * p_ble_cus_c, uint8_t * p_data);


/*
      DISABILITA LE NOTIFICHE
*/
uint32_t ble_cus_c_notif_disable(ble_cus_c_t * p_ble_cus_c);


