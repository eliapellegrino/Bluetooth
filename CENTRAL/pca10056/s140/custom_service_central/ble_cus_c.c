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
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */
#include "sdk_common.h"
#include "ble_cus_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_gattc.h"

#define NRF_LOG_MODULE_NAME ble_cus_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**@brief Function for interception of the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    ble_cus_c_t * p_ble_cus_c = (ble_cus_c_t *)p_ctx;

    NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (p_ble_cus_c->error_handler != NULL)
    {
        p_ble_cus_c->error_handler(nrf_error);
    }
}


/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function uses the Handle Value Notification received from the SoftDevice
 *            and checks whether it is a notification of the heart rate measurement from the peer. If
 *            it is, this function decodes the heart rate measurement and sends it to the
 *            application.
 *
 * @param[in] p_ble_cus_c Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_cus_c_t * p_ble_cus_c, const ble_evt_t * p_ble_evt)
{
    // Check if the event is on the link for this instance.
    if (p_ble_cus_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        NRF_LOG_INFO("Received HVX on link 0x%x, not associated to this instance. Ignore.",
                      p_ble_evt->evt.gattc_evt.conn_handle);
        return;
    }
	
    // Check if this is a heart rate notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_cus_c->peer_cus_db.cus_handle)
    {
        ble_cus_c_evt_t ble_cus_c_evt;
	ret_code_t err_code = NRF_SUCCESS;
        ble_cus_c_evt.evt_type                    = BLE_CUS_C_EVT_NOTIFICATION;
        ble_cus_c_evt.conn_handle                 = p_ble_cus_c->conn_handle;
        
        //__ALIGN(4) uint8_t buffer[MAX_PACKET_SIZE*SIZE_OF_ARRAY+4];
        uint8_t buffer[MAX_PACKET_SIZE*SIZE_OF_ARRAY];
        /*
        uint16_t len = (uint16_t)sizeof(buffer);
        uint8_t i;
        for (i = 0; i < MAX_PACKET_SIZE; i++)
        {
            buffer[i] = p_ble_evt->evt.gattc_evt.params.hvx.data[i];
            //NRF_LOG_INFO("BUFFER: %x", buffer[i]);
        }
        */
        memcpy(buffer, p_ble_evt->evt.gattc_evt.params.hvx.data,p_ble_evt->evt.gattc_evt.params.hvx.len);
        ble_cus_c_evt.params.p_data               = buffer;
        p_ble_cus_c->evt_handler(p_ble_cus_c, &ble_cus_c_evt);
    }
}


/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function checks whether the disconnect event is happening on the link
 *            associated with the current instance of the module. If the event is happening, the function sets the instance's
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_cus_c Pointer to the Heart Rate Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_cus_c_t * p_ble_cus_c, const ble_evt_t * p_ble_evt)
{
    if (p_ble_cus_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_cus_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
        p_ble_cus_c->peer_cus_db.cus_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_cus_c->peer_cus_db.cus_handle      = BLE_GATT_HANDLE_INVALID;
    }
}

// Devo inserire l'UUID del servizio custom che ho creato io e il tipo di servizio.
// Questa funzione deve essere richiamata nel main, nell'handler di eventi per il database discovery
void ble_cus_on_db_disc_evt(ble_cus_c_t * p_ble_cus_c, const ble_db_discovery_evt_t * p_evt)
{
    // Check if the Heart Rate Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == CUSTOM_SERVICE_UUID)
    {
        // Find the CCCD Handle
        uint32_t i;

        ble_cus_c_evt_t evt;

        evt.evt_type    = BLE_CUS_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                CUSTOM_VALUE_CHAR_UUID)
            {
                // Found Heart Rate characteristic. Store CCCD handle and break.
                evt.params.peer_db.cus_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.peer_db.cus_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                break;
            }
        }

        NRF_LOG_DEBUG("Heart Rate Service discovered at peer.");
        //If the instance has been assigned prior to db_discovery, assign the db_handles.
        if (p_ble_cus_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_cus_c->peer_cus_db.cus_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_cus_c->peer_cus_db.cus_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_cus_c->peer_cus_db = evt.params.peer_db;
            }
        }


        p_ble_cus_c->evt_handler(p_ble_cus_c, &evt);
    }
}


uint32_t ble_cus_c_init(ble_cus_c_t * p_ble_cus_c, ble_cus_c_init_t * p_ble_cus_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    cus_uuid;
    ble_uuid128_t cus_base_uuid = {CUSTOM_SERVICE_UUID_BASE};

    VERIFY_PARAM_NOT_NULL(p_ble_cus_c);
    VERIFY_PARAM_NOT_NULL(p_ble_cus_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_cus_c_init->evt_handler);
    VERIFY_PARAM_NOT_NULL(p_ble_cus_c_init->p_gatt_queue);

    p_ble_cus_c->peer_cus_db.cus_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_cus_c->peer_cus_db.cus_handle      = BLE_GATT_HANDLE_INVALID;
    p_ble_cus_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
    p_ble_cus_c->evt_handler                    = p_ble_cus_c_init->evt_handler;
    p_ble_cus_c->p_gatt_queue                   = p_ble_cus_c_init->p_gatt_queue;
    p_ble_cus_c->error_handler                  = p_ble_cus_c_init->error_handler;

    err_code = sd_ble_uuid_vs_add(&cus_base_uuid, &p_ble_cus_c->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

    cus_uuid.type = p_ble_cus_c->uuid_type;
    cus_uuid.uuid = CUSTOM_SERVICE_UUID;

    return ble_db_discovery_evt_register(&cus_uuid);
}

void ble_cus_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_c_t * p_ble_cus_c = (ble_cus_c_t *)p_context;

    if ((p_ble_cus_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_cus_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_cus_c, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(ble_cus_c_t * p_ble_cus_c, bool enable)
{
    NRF_LOG_INFO("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
                  p_ble_cus_c->peer_cus_db.cus_cccd_handle,
                  p_ble_cus_c->conn_handle);

    nrf_ble_gq_req_t cus_c_req;
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    uint16_t         cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : BLE_GATT_HVX_INDICATION;

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    memset(&cus_c_req, 0, sizeof(cus_c_req));

    cus_c_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cus_c_req.error_handler.cb            = gatt_error_handler;
    cus_c_req.error_handler.p_ctx         = p_ble_cus_c;
    cus_c_req.params.gattc_write.handle   = p_ble_cus_c->peer_cus_db.cus_cccd_handle;
    cus_c_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
    cus_c_req.params.gattc_write.p_value  = cccd;
    cus_c_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    // Aggiunge una nuova richiesta alla GATT queue
    return nrf_ble_gq_item_add(p_ble_cus_c->p_gatt_queue, &cus_c_req, p_ble_cus_c->conn_handle);
}


uint32_t ble_cus_c_notif_enable(ble_cus_c_t * p_ble_cus_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_cus_c);

    return cccd_configure(p_ble_cus_c, true);
}

uint32_t ble_cus_c_notif_disable(ble_cus_c_t * p_ble_cus_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_cus_c);

    return cccd_configure(p_ble_cus_c, false);
}


uint32_t ble_cus_c_handles_assign(ble_cus_c_t    * p_ble_cus_c,
                                  uint16_t         conn_handle,
                                  const cus_db_t * p_peer_cus_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_cus_c);

    p_ble_cus_c->conn_handle = conn_handle;
    if (p_peer_cus_handles != NULL)
    {
        p_ble_cus_c->peer_cus_db = *p_peer_cus_handles;
    }

    return nrf_ble_gq_conn_handle_register(p_ble_cus_c->p_gatt_queue, conn_handle);
}



uint32_t ble_cus_write(ble_cus_c_t * p_ble_cus_c, uint8_t * p_data)
{
    VERIFY_PARAM_NOT_NULL(p_ble_cus_c);

    if (p_ble_cus_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    NRF_LOG_INFO("Writing things");

    nrf_ble_gq_req_t write_req;

    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    write_req.error_handler.cb            = gatt_error_handler;
    write_req.error_handler.p_ctx         = p_ble_cus_c;
    write_req.params.gattc_write.handle   = p_ble_cus_c->peer_cus_db.cus_handle;
    write_req.params.gattc_write.len      = sizeof(*p_data);
    write_req.params.gattc_write.p_value  = p_data;
    write_req.params.gattc_write.offset   = 0;
    write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD; 

    return nrf_ble_gq_item_add(p_ble_cus_c->p_gatt_queue, &write_req, p_ble_cus_c->conn_handle);
}