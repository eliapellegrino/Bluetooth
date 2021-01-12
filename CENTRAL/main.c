/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */
 /*
Se flashiamo questo su un micro, esso si comporterà da master/central: quindi funzionerà come uno scanner.
Se vediamo il main, infatti non c'è niente che coinvolga advertising, questo perchè non deve funzionare come un advertiser
ma come uno scanner
*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_cus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_timer.h"
#include "nrf.h"
#include "nrf_uarte.h"
#include "nrf_drv_uart.h"
#include "app_fifo.h"
#include "nrf_drv_rtc.h"

// GPIO
#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1                     /**< Connected LED will be on when the device is connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                     /**< LED to indicate a change of state of the the Button characteristic on the peer. */
#define PIN_OUT                         NRF_GPIO_PIN_MAP(1,1)                   /*Definisco il PIN P1.15 come output per la ricostruzione del pacchetto*/
#define PA_PIN                          NRF_GPIO_PIN_MAP(1,3) /*PA_PIN: toggle del PIN quando viene trasmesso*/
#define LNA_PIN                         NRF_GPIO_PIN_MAP(1,4) /*LNA_PIN: toggle del PIN quando viene ricevuto qualcosa*/
#define PIN_OUT_DEBUG                   NRF_GPIO_PIN_MAP(1,5)                   /*Definisco il PIN P1.15 come output per la PWM*/

#define START_NOTIFICATION_BUTTON       BSP_BUTTON_0
#define STOP_NOTIFICATION_BUTTON        BSP_BUTTON_1
#define WRITE_BUTTON                    BSP_BUTTON_2                        /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define RX_POWER                        -20                                   /*Power that peer will be used if not set by it*/
// BLE SCANNING PARAMETER
#define SCAN_INTERVAL                   160                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     80                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */
// BLE CONNECTION PARAMETER
#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */
#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

// AQUISITION LENGTH
#define WINDOW_LENGTH                   7.5                                        /*Time in ms della finestra di output*/



//TIMER1 usato per la finestra di tot secondi
const nrf_drv_timer_t TIMER_TIMESTAMP = NRF_DRV_TIMER_INSTANCE(1);
const nrf_drv_timer_t TIMER_OUTPUT = NRF_DRV_TIMER_INSTANCE(4);
//RTC PER LA PORTA (UP circa 300us)
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
BLE_CUS_C_DEF(m_ble_cus_c);                                     /**< Main structure used by the cus client module. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static char const m_target_periph_name[] = "ATC_TX";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
int8_t index = 0;           /*Indice per il toggle del PIN*/
uint8_t received[MAX_PACKET_SIZE*SIZE_OF_ARRAY];
static uint8_t tx_uart_buff[20];
uint8_t begin = 0;           /*Se la finestra di acquisizione non è esattamente lunga come l'intervallo di connessione, il begin deve essere impostato uguale a HEX(CONN_INTERVAL)-HEX(WINDOW_LENGTH) e tutto il pacchetto ricevuto deve essere shiftato di begin*/
uint32_t conn_count = 0;
uint32_t mask_int = 0x1UL;
bool stop_timer = false;
bool uart_busy = false;
bool last_packet = false;

///////////////////////////////////
bool enable_uart = false;  /*Abilitare o meno la scirttura su seriale*/
bool enable_pin_out_debug = false; /*Abilitare o meno il pin_out per il delay*/
bool enable_reconstruction = false; /*Abilitare o meno la ricostruzione del pacchetto*/

/*UART DEFINITION*/
/*DEVONO ENTRAMBI ESSERE UNA POTENZA DI 2!!!!!*/
#define RX_BUFF_SIZE 0
#define TX_BUFF_SIZE MAX_PACKET_SIZE

// Istanza per la seriale (istanza = 0)
static nrf_drv_uart_t uart_inst = NRF_DRV_UART_INSTANCE(0);

// Init parametri di scan
/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active        = 0x00,
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    //.filter_policy = BLE_GAP_SCAN_FP_WHITELIST,       /*QUESTA RIGA BUGGAAAA*/
    .timeout       = SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
};

// Init parametri di connessione
static ble_gap_conn_params_t const m_conn_params = 
{
      .min_conn_interval = MIN_CONNECTION_INTERVAL,
      .max_conn_interval = MAX_CONNECTION_INTERVAL,
      .slave_latency = SLAVE_LATENCY,
      .conn_sup_timeout = CONN_SUP_TIMEOUT,
};

///////////////////////////////////////////////// OTHER FUNCTIONS /////////////////////////////////////////////////////
/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}
static void ble_options_set(void)
{
  ret_code_t err_code;
  ble_opt_t opt;
  memset(&opt, 0, sizeof(ble_opt_t));
  // Common PA/LNA config
  opt.common_opt.pa_lna.gpiote_ch_id  = 0;        // GPIOTE channel
  opt.common_opt.pa_lna.ppi_ch_id_clr = 1;            // PPI channel for pin clearing
  opt.common_opt.pa_lna.ppi_ch_id_set = 0;            // PPI channel for pin setting
  // PA config
  opt.common_opt.pa_lna.pa_cfg.active_high = 1;                // Set the pin to be active high
  opt.common_opt.pa_lna.pa_cfg.enable      = 1;                // Enable toggling
  opt.common_opt.pa_lna.pa_cfg.gpio_pin    = PA_PIN;      // The GPIO pin to toggle

  // LNA config
  opt.common_opt.pa_lna.lna_cfg.active_high  = 1;              // Set the pin to be active high
  opt.common_opt.pa_lna.lna_cfg.enable       = 1;              // Enable toggling
  opt.common_opt.pa_lna.lna_cfg.gpio_pin     = LNA_PIN;   // The GPIO pin to toggle

  err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &opt);
  APP_ERROR_CHECK(err_code);

  // Set power of antenna
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, 0, (int8_t) RX_POWER);
  APP_ERROR_CHECK(err_code);
}
/////////////////////////////////////////////// END OTHER FUNCTIONS ///////////////////////////////////////////////////////

///////////////////////////////////////////////// ERROR HANDLER //////////////////////////////////////////////////////
/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the LED Button Service client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void cus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
///////////////////////////////////////////////// END ERROR HANDLER //////////////////////////////////////////////////////

//////////////////////////////////////////////////// HANDLER ///////////////////////////////////////////////////////////
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
  
  if (int_type == NRFX_RTC_INT_TICK)
  {
    TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
    NRF_LOG_INFO("OFF: %d", TIMER_TIMESTAMP.p_reg->CC[0]);
    nrf_gpio_pin_clear(PIN_OUT);
    rtc.p_reg->TASKS_STOP = 1;
    //rtc.p_reg->TASKS_CLEAR = 1;

  }
}
void uart_event_handler (nrf_drv_uart_event_t * p_event, void * p_context)
{
   // Nothing to do
   memset(tx_uart_buff, 0xff, sizeof(tx_uart_buff));
   uart_busy = false;
   if (last_packet)
   {
    last_packet = false;
    nrf_drv_uart_tx(&uart_inst, tx_uart_buff, sizeof(tx_uart_buff));
   }
}
void timer_timestamp_event_handler(nrf_timer_event_t event_type, void* p_context)
{
//NOTHING TO DO
}
void timer_output_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    ret_code_t err_code;
    nrf_gpio_pin_set(PIN_OUT);
    rtc.p_reg->TASKS_START = 1;
    //NRF_LOG_INFO("PIN OUT settatoo\n");
    TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
    NRF_LOG_INFO("event: %d ON at: %d", event_type, TIMER_TIMESTAMP.p_reg->CC[0]);
    if (received[index] != 0)
    {
      TIMER_OUTPUT.p_reg->CC[(event_type-320)/4] = received[index];
      TIMER_OUTPUT.p_reg->INTENSET = mask_int << 16UL+ (event_type-320)/4;
      index++;
    } else
    {
      TIMER_OUTPUT.p_reg->INTENCLR = mask_int << 16UL+(event_type-320)/4;
      stop_timer = true;
    }
    if (stop_timer && TIMER_OUTPUT.p_reg->INTENCLR == 0)
    {
      TIMER_OUTPUT.p_reg->TASKS_STOP = 1;
      TIMER_OUTPUT.p_reg->TASKS_CLEAR = 1;
      NRF_LOG_INFO("STOPPED");
    }
}
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case WRITE_BUTTON:
            {
                if (button_action)
                {
                      NRF_LOG_INFO("PREMUTO IL BOTTONE DI SCRITURA");
                      uint8_t tmp[4] = {0x00,0x00,0x00,0x01};
                      //nrf_gpio_pin_toggle(LED_4);
                      ble_cus_write(&m_ble_cus_c, tmp);
                }
            }
            break;

        case START_NOTIFICATION_BUTTON:
            {
                if (button_action)
                {
                      NRF_LOG_INFO("START NOTIFICHE");
                      err_code = ble_cus_c_notif_enable(&m_ble_cus_c);
                      bsp_board_led_on(BSP_BOARD_LED_2);
                      APP_ERROR_CHECK(err_code);
                      NRF_LOG_INFO("ERROR CODE NOTIFICATION: %x", err_code);
                }
            }
            break;
         case STOP_NOTIFICATION_BUTTON:
            {
                if (button_action)
                {
                      NRF_LOG_INFO("STOP NOTIFICHE");
                      err_code = ble_cus_c_notif_disable(&m_ble_cus_c);
                      APP_ERROR_CHECK(err_code);
                      bsp_board_led_off(BSP_BOARD_LED_2);
                      APP_ERROR_CHECK(err_code);
                      NRF_LOG_INFO("ERROR CODE NOTIFICATION: %x", err_code);
                      memset(tx_uart_buff, 0xff, sizeof(tx_uart_buff));
                err_code = nrf_drv_uart_tx(&uart_inst, tx_uart_buff, sizeof(tx_uart_buff));
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("SENDING LAST PACKET");
                }
                
            }
            break;
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

/**@brief Handles events coming from the CUS service
    Viene richiamato qualsiasi volta c'è di messo il custom service. Se per esempio arriva una notifica viene richiamato @ref BLE_CUS_C_EVT_NOTIFICATION, mentre se uk servizio è stato scoperto sul peripheral @ref BLE_CUS_C_EVT_DISCOVERY_COMPLETE
 */
static void cus_c_evt_handler(ble_cus_c_t * p_cus_c, ble_cus_c_evt_t * p_cus_c_evt)
{
    switch (p_cus_c_evt->evt_type)
    {
        case BLE_CUS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            err_code = ble_cus_c_handles_assign(&m_ble_cus_c,
                                                p_cus_c_evt->conn_handle,
                                                &p_cus_c_evt->params.peer_db);
            NRF_LOG_INFO("Custom service discovered on conn_handle 0x%x.", p_cus_c_evt->conn_handle);
            /*
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            */
            //Questo pezzo è stato associato al bottone 2
            // LED Button service discovered. Enable notification of Button.
            /*
            err_code = ble_cus_c_notif_enable(p_cus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("ERROR CODE NOTIFICATION: %x", err_code);
            */
        } break; // BLE_cus_C_EVT_DISCOVERY_COMPLETE

        case BLE_CUS_C_EVT_NOTIFICATION:
        {
            // FUNZIONANTE!!!!!!!!!!!!!!!!!
            //TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
            //NRF_LOG_INFO("Packet received at : %d", TIMER_TIMESTAMP.p_reg->CC[0]);
                if (enable_reconstruction)
    {
    TIMER_OUTPUT.p_reg->TASKS_STOP = 1;
    TIMER_OUTPUT.p_reg->TASKS_CLEAR = 1;
    }
            conn_count++;
            int8_t i;
            ret_code_t err_code = NRF_SUCCESS;
            //uint8_t received[MAX_PACKET_SIZE*SIZE_OF_ARRAY];
            memset(received, 0, sizeof(received));
            memcpy(received,p_cus_c_evt->params.p_data, MAX_PACKET_SIZE);
            //strcpy(received, p_cus_c_evt->params.p_data);
            //uint32_t count_trans = (received[0] << 8 ) | (received[1]);
            //NRF_LOG_INFO("%d %d", received[0],received[1]);
            //NRF_LOG_INFO("%d = %d ?",conn_count, count_trans);
            if (enable_pin_out_debug){
              nrf_gpio_pin_toggle(PIN_OUT_DEBUG);
            }
            if (enable_uart)
            {
              memcpy(tx_uart_buff,received,20);
              NRF_LOG_INFO("UART TX");
              nrf_drv_uart_tx(&uart_inst, received, (uint8_t) 20);
              uart_busy = true;
            }
            if (enable_reconstruction)
    {
    // Ricostruzione onda su PIN_OUT (setto i valori nei compare e cerco un modo di settare quelli nuovi)
    index = 0; //Contiene l'indice dell'ultima cella da puntare nel pacchetto ricevuto
    stop_timer = false;
      for (i = 0; i < 6; i++) // i < 6: uso il timer che può contenere fino a 6 copmare (TIMER 3 o 4)
      { 
        if (received[i] != 0)
        {
          TIMER_OUTPUT.p_reg->CC[i] = received[i];
          TIMER_OUTPUT.p_reg->INTENSET = (uint32_t)mask_int << 16UL+i;
          index = i+1;
        } else
        {
          // Se trovo uno zero esco dal ciclo (tutti i compare sono stati settati)
          break;
        }
      }
      if (index != 0)
      {
         TIMER_OUTPUT.p_reg->TASKS_START = 1;// Start the timer
      }
    }
        } break; // BLE_cus_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    //NRF_LOG_INFO("BLE EVT TYPE(HEX): %x", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");
            err_code = ble_cus_c_handles_assign(&m_ble_cus_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            bsp_board_led_off(CENTRAL_SCANNING_LED);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
            conn_count = 0;
            scan_start();
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            // Update dei parametri PHY a seguito di una richiesta da parte del PEER
            NRF_LOG_INFO("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = p_gap_evt->params.phy_update_request.peer_preferred_phys.rx_phys,
                .tx_phys = p_gap_evt->params.phy_update_request.peer_preferred_phys.tx_phys,
                //.rx_phys = BLE_GAP_PHY_AUTO,
                //.tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
            NRF_LOG_INFO("MAX TX OCTECT: %d", p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets);
            NRF_LOG_INFO("MAX RX OCTECT: %d", p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets);
            NRF_LOG_INFO("MAX TX TIME: %d", p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_time_us);
            NRF_LOG_INFO("MAX RX TIME: %d", p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_time_us);

        default:
            // No implementation needed.
            break;
    }
}
/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_INFO("DATABASE EVT TYPE: %x", p_evt->evt_type);
    ble_cus_on_db_disc_evt(&m_ble_cus_c, p_evt);
}
/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;
        default:
          break;
    }
}
///////////////////////////////////////////////// END HANDLER //////////////////////////////////////////////////////

//////////////////////////////////////////////////// UART INITIALIZATION ////////////////////////////////////////////////
void init_uart(){
  ret_code_t err_code;
  // UART CONFIGURATION
  static nrf_drv_uart_config_t uart_configuration = {
  .pseltxd = TX_PIN_NUMBER,
  .pselrxd = RX_PIN_NUMBER,
  .pselcts = CTS_PIN_NUMBER,
  .pselrts = RTS_PIN_NUMBER,
  .hwfc = NRF_UART_HWFC_DISABLED,
  .parity = NRF_UART_PARITY_EXCLUDED,
  .baudrate = NRF_UART_BAUDRATE_1000000,
  .interrupt_priority = _PRIO_APP_LOW,
  };
  err_code = nrf_drv_uart_init(&uart_inst, &uart_configuration, uart_event_handler);
  APP_ERROR_CHECK(err_code);
}
//////////////////////////////////////////////////// END UART INITIALIZATION ////////////////////////////////////////////////

//////////////////////////////////////////////// GPIO INITIALIZATION //////////////////////////////////////////////
/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}
/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {WRITE_BUTTON, false, BUTTON_PULL, button_event_handler},
        {START_NOTIFICATION_BUTTON, false, BUTTON_PULL, button_event_handler},
        {STOP_NOTIFICATION_BUTTON, false, BUTTON_PULL, button_event_handler}

    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    NRF_LOG_INFO("INIT BUTTON OK! %x", err_code)
    APP_ERROR_CHECK(err_code);
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}
void pins_init()
{
      ret_code_t err_code;
      // OUTPUT PIN
      nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
      err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
      APP_ERROR_CHECK(err_code);
      // OUTPUT PIN DEBUG
      err_code = nrf_drv_gpiote_out_init(PIN_OUT_DEBUG, &out_config);
      APP_ERROR_CHECK(err_code);
}
/////////////////////////////////////////////////// END GPIO INITIALIZATION /////////////////////////////////////////

////////////////////////////////////////////////// BLE INITIALIZATION //////////////////////////////////////////////
/**@brief LED Button client initialization.
 */
static void cus_c_init(void)
{
    ret_code_t       err_code;
    ble_cus_c_init_t cus_c_init_obj;

    cus_c_init_obj.evt_handler   = cus_c_evt_handler;
    cus_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;
    cus_c_init_obj.error_handler = cus_error_handler;

    err_code = ble_cus_c_init(&m_ble_cus_c, &cus_c_init_obj);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the scan module.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    // Init dei parametri di connesione 
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param = &m_scan_params;
    init_scan.p_conn_param = &m_conn_params;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // Setting filters for scanning.
    // Abilita la whitelist (per NOME) per lo scanner
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
    // Setta la whitelist (m_target_periph_name) allo scanner
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}
///////////////////////////////////////////// END BLE INITIALIZATION //////////////////////////////////////////////////

//////////////////////////////////////////// OTHER INITIALIZATION ////////////////////////////////////////////////////
/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}
///////////////////////////////////////////// END OTHER INITIALIZATION /////////////////////////////////////////////////

///////////////////////////////////////////// TIMER INITIALIZATION ///////////////////////////////////////////////////
/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    uint32_t time_ticks;
    uint32_t time_ticks2;
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    //Timer1: usato per la finestra in cui fare output (uguale a quella sul peripheral)
    nrf_drv_timer_config_t timer_cfg;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    timer_cfg.mode = NRF_TIMER_MODE_TIMER;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_8;
    timer_cfg.interrupt_priority = 6;
    err_code = nrf_drv_timer_init(&TIMER_OUTPUT, &timer_cfg, timer_output_event_handler);
    APP_ERROR_CHECK(err_code);

    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&TIMER_TIMESTAMP, &timer_cfg, timer_timestamp_event_handler);
    APP_ERROR_CHECK(err_code);
    TIMER_TIMESTAMP.p_reg->TASKS_START = 1;

}
/////////////////////////////////////////////////// END TIMER INITIALIZATION ////////////////////////////////////////////
/////////////////////////////////////////////////// RTC INITIALIZATION ////////////////////////////////////////////
static void rtc_init(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 8; /*f = 32768/(pr+1)*/
    config.interrupt_priority = 6;  /*Con priorità = 3 bugga, = 4 FATAL ERROR!!*/
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt (di default è disabilitato)
    nrf_drv_rtc_tick_enable(&rtc,true);
    // Tick event (low power) freq = 32768/(prescaler+1) = 3641Hz 
}
/////////////////////////////////////////////////// END RTC INITIALIZATION ////////////////////////////////////////////

////////////////////////////////////////////////// MAIN ////////////////////////////////////////////////////////////////
int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    ble_stack_init();
    rtc_init();
    ble_options_set();
    leds_init();
    buttons_init();
    pins_init();
    power_management_init();
    init_uart();
    scan_init();
    gatt_init();
    db_discovery_init();
    cus_c_init();

    // Start execution.
    NRF_LOG_INFO("Custom example CENTRAL started.");
    scan_start();
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
////////////////////////////////////////////////// END MAIN ////////////////////////////////////////////////////////////////