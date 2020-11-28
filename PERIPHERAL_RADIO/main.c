/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
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
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_radio_notification.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_cus_v2.h"
#include "app_gpiote.h"
#include "nrfx_timer.h"
#include "nrf_drv_timer.h"
#include "nrf_uarte.h"
#include "nrf_drv_uart.h"
#include "app_fifo.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"

// GPIO
#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define NOTIFICATION_LED                BSP_BOARD_LED_2
#define TIMER_LED                       BSP_BOARD_LED_3
#define PIN_IN                          NRF_GPIO_PIN_MAP(1,1)
#define PIN_OUT_OSC                     NRF_GPIO_PIN_MAP(1,2)
#define PA_PIN                          NRF_GPIO_PIN_MAP(1,3) /*PA_PIN: toggle del PIN quando viene trasmesso*/
#define LNA_PIN                         NRF_GPIO_PIN_MAP(1,4) /*LNA_PIN: toggle del PIN quando viene ricevuto qualcosa*/
#define PIN_OUT_PRE                     NRF_GPIO_PIN_MAP(1,5)
#define PIN_OUT_POST                    NRF_GPIO_PIN_MAP(1,6)

//BLE
#define DEVICE_NAME                     "DAJE_ELO"                         /**< Name of device. Will be included in the advertising data. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */
// BLE ADVERTISING PARAMETERS
#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The 0.625tising time-out (in units of seconds). When set to 0, we will never time out. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(2000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

//OTHERS
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// ACQUISITION LENGTH
#define WINDOW_LENGTH                   7.5             /*Lunghezza in ms della finestra di acquisizione: alla fine di essa verrà mandato il payload al central*/

// SIMULAZIONE INTERRUPT
#define NOTIFICATION_TIME               1        /*Tempo in ms: ogni qual volta il timer 2 scade, viene inviata una notifica ai vari client*/

//Prescaler for RTC
#define RTC_PRESCALER                   0

#define CONN_INT                        245

//Con queste macro definisco l'istanza per il LED button service, GATT and Queue
//Le macro servono solo per istanziare (creare) quegli oggetti: dovranno poi essere usate le varie funzioni nelle API
// per inizializzare, e fare cose su sti oggetti
// MODULE INSTANCE
BLE_CUS_DEF(m_cus);
NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr); /**< Context for the Queued Write module.*/


// TIMER1 usato per la finestra di tot secondi
const nrf_drv_timer_t TIMER_ACQUISITION = NRF_DRV_TIMER_INSTANCE(1);
// TIMER 3 (usato solo per debug, da non usare nella definitiva)
const nrf_drv_timer_t TIMER_TIMESTAMP = NRF_DRV_TIMER_INSTANCE(3);
// RTC
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
uint32_t compare_value = 245; /*WINDOW_LENGTH*0.001*(32768/(RTC_PRESCALER+1)): colpi di clock. Per difetto: 7,47ms*/

// UART (istanza 0)
static nrf_drv_uart_t uart_inst = NRF_DRV_UART_INSTANCE(0);

// Inizializzo payload
uint8_t packet[MAX_PACKET_SIZE];
uint8_t int_event[MAX_PACKET_SIZE]; // Si usa in in_pin_handler e si copia in packet prima dell'invio
uint8_t tx_uart_buff[20];

uint8_t count = 0; 
uint16_t int_counter = 0;
uint32_t count_sent = 0;
uint32_t radio_on_pre = 0;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
uint32_t tmp = 0;
uint8_t current_index = 0; /*Usati in in_pin_handler per riempire il buffer degli eventi*/
// BLE VARIABLE
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */
static ble_uuid_t adv_uuids[] = /**< Universally unique service identifiers. */ {
   {
      CUSTOM_SERVICE_UUID,
      BLE_UUID_TYPE_VENDOR_BEGIN
   }
};
/**@brief Struct that contains pointers to the encoded advertising data. */
// Definisco una struttura che contiene il buffer per contenere i dati di advertising e i dati di scan
static ble_gap_adv_data_t m_adv_data =
   // Cosa vuol dire questa scrittura?
{
   .adv_data = {
      .p_data = m_enc_advdata,
      .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
   },
   .scan_rsp_data = {
      .p_data = m_enc_scan_response_data,
      .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX

   }
};

// FLAGs
static bool is_notificated = false;
bool enable_uart = true;
bool enable_pin_out = true;
bool enable_pin_osc = false;

bool uart_busy = false;
bool last_packet = false;
///////////////////////////////// OTHER FUNCTIONS ///////////////////////////////////////////////
/**@brief Function for starting advertising.
 */
static void advertising_start(void) {
   ret_code_t err_code;

   // Start a fare advertising e accendo il led corrispondente
   err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
   APP_ERROR_CHECK(err_code);

   bsp_board_led_on(ADVERTISING_LED);
}
/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
   // Se ci sono delle operazioni da svolgere lo fa, altrimenti continua a stare in uno stato di idle (senza fare nulla), fino a che un nuovo evento è disponibile
   if (NRF_LOG_PROCESS() == false) {
      nrf_pwr_mgmt_run();
   }
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
}
///////////////////////////////// END OTHER FUNCTIONS /////////////////////////////////////////

/////////////////////////////////////// HANDLER //////////////////////////////////////////////
/*
  Handler for radio notification
*/
static void radio_evt_handler(bool radio_active)
{
   uint32_t now;
   ret_code_t err_code;
   now = rtc.p_reg->COUNTER;
   if (radio_active)
   {
      if (now - radio_on_pre > CONN_INT - 3 && now - radio_on_pre < CONN_INT + 3)
      {
         // I due dispositivi sono connessi: se le notifiche sono attive, allora possoono scambiarsi i dati

         //NRF_LOG_INFO("Count: %d", count);
         if (is_notificated)
         {
            if (!nrf_drv_timer_is_enabled(&TIMER_ACQUISITION))
            {
               TIMER_ACQUISITION.p_reg->TASKS_START = 1;
            }
            //NRF_LOG_INFO("Count: %d", count);
            //NRF_LOG_INFO("ACTIVE: %d", now );
            // Per processing dei dati
            packet[0] = count; // numero progressivo finestra
            packet[1] = 1; //channel id
            packet[2] = current_index; // numero eventi nella finestra
            memcpy(&packet[3], int_event, MAX_PACKET_SIZE - 3); //l'ultimo è il numero di byte da copiare (per ora 15)
            memset(int_event, 0, sizeof(int_event));
            current_index = 0;
            count++;
            if (enable_pin_out)
            {
               nrf_gpio_pin_toggle(PIN_OUT_PRE);
            }
            if (enable_pin_osc)
            {
               nrf_gpio_pin_toggle(PIN_OUT_OSC);
            }
            err_code = ble_cus_custom_value_update( & m_cus, packet);
            APP_ERROR_CHECK(err_code);
            //NRF_LOG_INFO("PUT IN QUEUE AT : %d", rtc.p_reg->COUNTER);
            if (enable_uart)
            {
               memcpy(tx_uart_buff, packet, 20);
               err_code = nrf_drv_uart_tx( & uart_inst, tx_uart_buff, (uint8_t) 20);
               APP_ERROR_CHECK(err_code);
               uart_busy = true;
            }
         }
         TIMER_ACQUISITION.p_reg->TASKS_CLEAR = 1;
      }
      radio_on_pre = now;
//    NRF_LOG_INFO("Radio on at: %d", now );
   } else
   {
      if (is_notificated)
      {
         //NRF_LOG_INFO("INACTIVE: %d", now );
      }
      /*
        TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
        now = TIMER_TIMESTAMP.p_reg->CC[0];
        NRF_LOG_INFO("Radio off at: %d", now)
        */
   }
}
/*
  Handler for custom service
*/
static void on_cus_evt(ble_cus_t * p_cus_service,
                       ble_cus_evt_t * p_evt) {
   ret_code_t err_code;
   // L'evento di connessione/disconessione non può essere gestito sia qua (indirettamente tramite ble_on_cus_evt) che in ble_evt_handler: bisogna gestirlo solo in una delle due parti
   if (p_evt -> evt_type != BLE_CUS_EVT_CONNECTED) {
      switch (p_evt -> evt_type) {
      // Evento quando vengono abilitate le notifiche
      case BLE_CUS_EVT_NOTIFICATION_ENABLED:
         /*In questa situazione accendo il LED3 e abilito la lettura sul pin di ingresso*/
         NRF_LOG_INFO("NOTIFICHE ON: %d", rtc.p_reg->COUNTER);
         is_notificated = true;
         nrf_drv_gpiote_in_event_enable(PIN_IN, true);
         bsp_board_led_on(NOTIFICATION_LED);
         break;
      // Evento quando vengono disabilitate le notifiche
      case BLE_CUS_EVT_NOTIFICATION_DISABLED:
         // Disabilito il sensing per il PIN di ingresso
         is_notificated = false;
         nrf_drv_gpiote_in_event_disable(PIN_IN);
         bsp_board_led_off(NOTIFICATION_LED);
         TIMER_ACQUISITION.p_reg->TASKS_STOP = 1;
         TIMER_ACQUISITION.p_reg->TASKS_CLEAR = 1;
         tx_uart_buff[0] = (int_counter & 0x00ff) >> 0;
         tx_uart_buff[1] = (int_counter & 0xff00) >> 8;
         err_code = nrf_drv_uart_tx(&uart_inst,tx_uart_buff, 20);
         APP_ERROR_CHECK(err_code);
         uart_busy = true;
         last_packet = true;
         break;
      // Evento quando viene scritto un valore sulle notifiche
      case BLE_CUS_EVT_WRITE_SUCCESS:
         nrf_gpio_pin_toggle(LED_4);
      // Quando viene scritta la caratteristica faccio partire un onda quadra (se sono state attivate le notifiche). Bisognerebbe prima aver settato il flag di notifica
      /*
      if (is_notificated)
      {

          // Toggle del led
      }
      */
      default:
         // No implementation needed.
         break;
      }
   }
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
   ret_code_t err_code;

   if (p_evt -> evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      APP_ERROR_CHECK(err_code);
   }
}
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
// Funzione che viene richiamata ad ogni BLE event. Vedo @BLE_GAP_EVTS per i vari eventi che possono essere scatenati.
static void ble_evt_handler(ble_evt_t
                            const * p_ble_evt, void * p_context) {
   uint32_t now;
   ret_code_t err_code;
   //NRF_LOG_INFO("BLE EVENT HEX : %x",p_ble_evt->header.evt_id);
   switch (p_ble_evt -> header.evt_id) {
   case BLE_L2CAP_EVT_CH_TX:
      NRF_LOG_INFO("BLE_L2CAP_TX");
   case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Connected");
      bsp_board_led_on(CONNECTED_LED);
      bsp_board_led_off(ADVERTISING_LED);
      m_conn_handle = p_ble_evt -> evt.gap_evt.conn_handle;
      // Assegna un handle a quella connessione, in modo che possiamo riferirirci ad essa
      err_code = nrf_ble_qwr_conn_handle_assign( & m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);
      /*PHY UPDATE ALLA CONNESSIONE ALLA 2MBPS*/
      ble_gap_phys_t
      const phys = {
         .rx_phys = BLE_GAP_PHY_2MBPS,
         .tx_phys = BLE_GAP_PHY_2MBPS,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt -> evt.gap_evt.conn_handle, & phys);
      APP_ERROR_CHECK(err_code);
      break;

   case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected");
      bsp_board_led_off(CONNECTED_LED);
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      //err_code = app_button_disable();
      //APP_ERROR_CHECK(err_code);
      // Disabilito timer
      count = 0;
      TIMER_ACQUISITION.p_reg->TASKS_STOP = 1;
      TIMER_ACQUISITION.p_reg->TASKS_CLEAR = 1;
      advertising_start();
      break;

   case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                             BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                             NULL,
                                             NULL);
      APP_ERROR_CHECK(err_code);
      break;

   case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t
      const phys = {
         .rx_phys = BLE_GAP_PHY_AUTO,
         .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt -> evt.gap_evt.conn_handle, & phys);
      APP_ERROR_CHECK(err_code);
   }
   break;

   case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      // No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

   case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      NRF_LOG_DEBUG("GATT Client Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt -> evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

   case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      NRF_LOG_DEBUG("GATT Server Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt -> evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;
   case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
      NRF_LOG_INFO("MAX TX OCTECT: %d", p_ble_evt -> evt.gap_evt.params.data_length_update.effective_params.max_tx_octets);
      NRF_LOG_INFO("MAX RX OCTECT: %d", p_ble_evt -> evt.gap_evt.params.data_length_update.effective_params.max_rx_octets);
      NRF_LOG_INFO("MAX TX TIME: %d", p_ble_evt -> evt.gap_evt.params.data_length_update.effective_params.max_tx_time_us);
      NRF_LOG_INFO("MAX RX TIME: %d", p_ble_evt -> evt.gap_evt.params.data_length_update.effective_params.max_rx_time_us);
   case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      count_sent++;
      /*
        if  (enable_pin_osc)
        {
        nrf_gpio_pin_clear(PIN_OUT_OSC);
        }
        */
      now = rtc.p_reg->COUNTER;
     /// NRF_LOG_INFO("BLE_GATTS_EVT_HVN_TX_COMPLETE: %d, Delay: %d", now, now - radio_on_pre);
      //tmp = now;
      //memset(packet, 0, sizeof(packet));
      if (enable_pin_out) {
         nrf_gpio_pin_toggle(PIN_OUT_POST);
      }
   default:
      // No implementation needed.
      break;
   }
}
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 * Ogni volta che viene premuto un tasto viene "richiamata" questa funzione
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action) {
   ret_code_t err_code;
   // Controllo il pin del tasto premuto
   switch (pin_no) {
   case BSP_BUTTON_0:
      if (button_action) {
         NRF_LOG_INFO("TIMER ABILITATO");
         bsp_board_led_on(TIMER_LED);
      }
      break;
   case BSP_BUTTON_1:
      if (button_action) {
         NRF_LOG_INFO("TIMER DISABILITATO");
         bsp_board_led_off(TIMER_LED);
      }
      break;
   default:
      APP_ERROR_HANDLER(pin_no);
      break;
   }
}
/*
    Handler for PIN_IN
*/
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
   uint8_t now;
   int8_t i;
   ret_code_t err_code;
   TIMER_ACQUISITION.p_reg->TASKS_CAPTURE[0] = 1;
   now = TIMER_ACQUISITION.p_reg->CC[0];
   if (is_notificated)
   {
      memset(&int_event[current_index], now, 1);
      current_index++;
   }
   NRF_LOG_INFO("INSIDE PIN IN");
   int_counter++;
   //nrf_gpio_pin_toggle(LED_4);
   /*
   if (is_notificated) {
     for (i = 0; i < MAX_PACKET_SIZE; i++) {
       if (int_event[i] == 0) {
         int_event[i] = now;
         break;
       }
     }
   }
   */
}
/*
  Handle for RTC
*/
static void rtc_handler(nrf_drv_rtc_int_type_t int_type) {
// NOTHING TO DO
}
/**
 * @brief Handler for timer acquisition
 */
void timer_acquisition_event_handler(nrf_timer_event_t event_type, void * p_context) {
// NOTHING TO DO
}
/**
 * @brief Handler for timer timestamp
 */
void timer_timestamp_event_handler(nrf_timer_event_t event_type, void * p_context) {
//NOTHING TO DO
}
/*
  Handler for UART event
*/
void uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context) {
   // Nothing to do
   memset(tx_uart_buff, 0xff, sizeof(tx_uart_buff));
   uart_busy = false;
   if (last_packet)
   {
    last_packet = false;
    nrf_drv_uart_tx(&uart_inst, tx_uart_buff, sizeof(tx_uart_buff));
   }
}
/////////////////////////////////////// END HANDLER //////////////////////////////////////////////

/////////////////////////////////////// ERROR HANDLER //////////////////////////////////////////////
/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
   APP_ERROR_HANDLER(nrf_error);
}
/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
   APP_ERROR_HANDLER(nrf_error);
}
/////////////////////////////////////// END ERROR HANDLER ///////////////////////////////////////

///////////////////////////////////////// UART ////////////////////////////////////////////////////
void init_uart() {
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
   err_code = nrf_drv_uart_init( & uart_inst, & uart_configuration, uart_event_handler);
   APP_ERROR_CHECK(err_code);
}

////////////////////////////////////////END UART ////////////////////////////////////////////////
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num,
                         const uint8_t * p_file_name) {
   app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//////////////////////////// TIMER/RTC INITIALIZATION ////////////////////////////////////////////////////////////////
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void) {
   uint32_t time_ticks;
   ret_code_t err_code;
   // Initialize timer module, making it use the scheduler
   err_code = app_timer_init();
   APP_ERROR_CHECK(err_code);

   // Init del timer per la finestra di acquisizione
   nrf_drv_timer_config_t timer_cfg;
   timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
   timer_cfg.mode = NRF_TIMER_MODE_TIMER;
   timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_8;
   timer_cfg.interrupt_priority = 6;
   err_code = nrf_drv_timer_init( & TIMER_ACQUISITION, & timer_cfg, timer_acquisition_event_handler);
   APP_ERROR_CHECK(err_code);
   /*Il timer viene abilitato a notifiche attivate!!!*/
   //Timer timestamp
   timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
   timer_cfg.mode = NRF_TIMER_MODE_TIMER;
   timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
   err_code = nrf_drv_timer_init(&TIMER_TIMESTAMP, &timer_cfg, timer_timestamp_event_handler);
   APP_ERROR_CHECK(err_code);
   nrf_drv_timer_enable(&TIMER_TIMESTAMP);
}
/*
  RTC configuration (f = 32768Hz) (il low frequency clock deve già essere abilitato---> viene fatto in ble_stack_init())
*/
static void rtc_init(void) {
   uint32_t err_code;

   //Initialize RTC instance
   nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
   config.prescaler = RTC_PRESCALER; /*f = 32768/(pr+1)*/
   config.interrupt_priority = 6; /*Con priorità = 3 bugga, = 4 FATAL ERROR!!*/
   err_code = nrf_drv_rtc_init( & rtc, & config, rtc_handler);
   APP_ERROR_CHECK(err_code);
   nrf_drv_rtc_enable(&rtc);

}

//////////////////////////////////// END TIMER/RTC INITIALIZATION //////////////////////////////////////////////////////

/////////////////////////////////// BLE INITIALIZATION ///////////////////////////////////////////
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
   ret_code_t err_code;
   ble_gap_conn_params_t gap_conn_params; //struct con alcuni parametri che definisco la connesione
   ble_gap_conn_sec_mode_t sec_mode; //struct contenente alcuni parametri per la sicurezza della connessione

   BLE_GAP_CONN_SEC_MODE_SET_OPEN( & sec_mode);

   //Setto il nome del GAP device
   err_code = sd_ble_gap_device_name_set( & sec_mode,
                                          (const uint8_t * ) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
   APP_ERROR_CHECK(err_code);

   //Inizializzo le configurazioni per questa connessione
   memset( & gap_conn_params, 0, sizeof(gap_conn_params));

   gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
   gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
   gap_conn_params.slave_latency = SLAVE_LATENCY;
   gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

   //Setto le configurazioni definite nelle righe sopra
   err_code = sd_ble_gap_ppcp_set( & gap_conn_params);
   APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
   //Inizializzo il modulo GATT (deve essere stato prima istanziato, vedi riga 100)
   ret_code_t err_code = nrf_ble_gatt_init( & m_gatt, NULL);
   APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void) {
   ret_code_t err_code;
   // ble_advdata_t: struct contenente tutti i parametri necessari per configurare e codificare i pacchetti di advertising
   // Guardo getting started with BLE per vedere i vari parametri come possono essere settati
   ble_advdata_t advdata; // Dati di advertising
   ble_advdata_t srdata; // Dati di scan response (ovviamente sono dello stesso tipo, essendo logicamente la stessa cosa)

   // Setto gli UUID usati nella mia applicazione (devo fornire sia il valore, sia il tipo del servizio).
   // In questo caso ho solo un servizio. Per il tipo vedo @ref BLE_UUID_TYPE_VENDOR_BEGIN

   // Build and set advertising data.
   memset( & advdata, 0, sizeof(advdata));

   advdata.name_type = BLE_ADVDATA_FULL_NAME;
   advdata.include_appearance = true;
   advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
   /* Settare dati propri da mandare tramite advertising
   ble_advdata_manuf_data_t                  manuf_data; //Variable to hold manufacturer specific data
   uint8_t data[]                            = "SomeData!"; //Our data to advertise
   manuf_data.company_identifier             =  0x0059; //Nordics company ID
   manuf_data.data.p_data                    = data;
   manuf_data.data.size                      = sizeof(data);
   init.advdata.p_manuf_specific_data = &manuf_data;
   */
   memset( & srdata, 0, sizeof(srdata));
   srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
   srdata.uuids_complete.p_uuids = adv_uuids;
   // Il secondo parametro è un out: ovvero i dati codificati andranno messi in quel campo di quella struttura
   err_code = ble_advdata_encode( & advdata, m_adv_data.adv_data.p_data, & m_adv_data.adv_data.len);
   APP_ERROR_CHECK(err_code);

   err_code = ble_advdata_encode( & srdata, m_adv_data.scan_rsp_data.p_data, & m_adv_data.scan_rsp_data.len);
   APP_ERROR_CHECK(err_code);

   // Questi parametri sono quelli che vengono settati per determinare il tipo di advertising!
   // Qua dentro si può decidere il tipo per il pacchetto di advrtising! (CONN, NON CONN, ecc).
   // Per la nostra applicazione potrebbe essere perfetto un tipo non connectable in modo che non abbia da switchare tra tx e rx
   ble_gap_adv_params_t adv_params;

   // Set advertising parameters.
   memset( & adv_params, 0, sizeof(adv_params));

   adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
   adv_params.duration = APP_ADV_DURATION;
   adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
   adv_params.p_peer_addr = NULL;
   adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
   adv_params.interval = APP_ADV_INTERVAL;

   //Setto la configurazione per poter poi iniziare a fare advertising (metto infatti m_adv_data che è il mio buffer che contiene i dati di adv e scan response)
   err_code = sd_ble_gap_adv_set_configure( & m_adv_handle, & m_adv_data, & adv_params);
   APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing services that will be used by the application.
 * In questa funzione dobbiamo mettere tutti i servizi che andremo ad usare nella nostra applicazione finale
 */
static void services_init(void) {
   ret_code_t err_code;
   // Per scrivere cose in coda (è una struttura con un campo per gestire gli errori)
   nrf_ble_qwr_init_t qwr_init = {
      0
   };

   // Initialize Queued Write Module.
   // Inizializzo il modulo per gestire gli errori (nrf_qwr_error_handler è una funzione!)
   qwr_init.error_handler = nrf_qwr_error_handler;

   err_code = nrf_ble_qwr_init( & m_qwr, & qwr_init);
   APP_ERROR_CHECK(err_code);

   // Init of custom service
   ble_cus_init_t cus_init;
   memset( & cus_init, 0, sizeof(cus_init));

   // Setto il permesso a TUTTI di scrivere l'attributo e il cccd e di leggere l'attributo
   BLE_GAP_CONN_SEC_MODE_SET_OPEN( & cus_init.custom_value_char_attr_md.cccd_write_perm);
   BLE_GAP_CONN_SEC_MODE_SET_OPEN( & cus_init.custom_value_char_attr_md.read_perm);
   BLE_GAP_CONN_SEC_MODE_SET_OPEN( & cus_init.custom_value_char_attr_md.write_perm);

   uint32_t tmp[MAX_PACKET_SIZE] = {
      0
   };
   cus_init.evt_handler = on_cus_evt;
   cus_init.initial_custom_value = tmp;
   err_code = ble_cus_init( & m_cus, & cus_init);
   APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the Connection Parameters module.
 */
// Inizializzzione dei parametri di connessione: verranno usati per gestire la connessione tra un master e uno slave
static void conn_params_init(void) {
   ret_code_t err_code;
   ble_conn_params_init_t cp_init;

   memset( & cp_init, 0, sizeof(cp_init));

   cp_init.p_conn_params = NULL; /*NULL perche settiamo i parametri in gap_params_init*/
   cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
   cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
   cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
   cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
   cp_init.disconnect_on_fail = false;
   cp_init.evt_handler = on_conn_params_evt; // Gestisco eventuali eventi (è una funzione che mi creo io) che succeddono in questo modulo
   cp_init.error_handler = conn_params_error_handler; //Gestisco eventuali errori che possono succedere

   err_code = ble_conn_params_init( & cp_init);
   APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
   ret_code_t err_code;
   //Enable SoftDevice e controllo che non ci siano errori
   err_code = nrf_sdh_enable_request();
   APP_ERROR_CHECK(err_code);

   // Enable radio notificatino event
   err_code = ble_radio_notification_init(6, NRF_RADIO_NOTIFICATION_DISTANCE_800US, radio_evt_handler);
   APP_ERROR_CHECK(err_code);
   // Configure the BLE stack using the default settings.
   // Fetch the start address of the application RAM.
   uint32_t ram_start = 0;
   //Configuro il bluetooth stack con le configurazioni date (vedo API)
   // Al suo interno viene già anche cambiato l'MTU size con quello massimo definito in sdk_config.h tramite NRF_SDH_BLE_GATT_MAX_MTU_SIZE
   err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, & ram_start);
   APP_ERROR_CHECK(err_code);
   // Cambio size di default della queue per l'invio del payload (ATTENZIONE: per fare cio devo allocare più memoria per la RAM è già stato cambiato!)
   // Per cambiare l'MTU devo settare il campo: ble_cfg.conn_cfg.gatt_conn_cfg
   ble_cfg_t ble_cfg;
   memset( & ble_cfg, 0, sizeof(ble_cfg));
   ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
   ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 50;
   err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, & ble_cfg, ram_start);
   APP_ERROR_CHECK(err_code);
   // Enable BLE stack.
   err_code = nrf_sdh_ble_enable( & ram_start);
   APP_ERROR_CHECK(err_code);

   // Register a handler for BLE events.
   // Senza questa macro, ogni evento che avviene sul SoC non verrà notificato! Quando avviene un evento, viene richiamato ble_evt_handler
   NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
////////////////////////////////////////END BLE INITIALIZATION //////////////////////////////////////

////////////////////////////////// OTHER INITIALIZATION ////////////////////////////////////////
static void log_init(void) {
   // ret_code_t: uint32_t. Viene usato per gestire gli errori
   ret_code_t err_code = NRF_LOG_INIT(NULL);
   // APP_ERROR_CHECK: usato per controllare che non ci siano errori. In caso di errori viene alzata un'eccezione
   APP_ERROR_CHECK(err_code);
   //Init backend per scrivere le informazioni di log del programma
   NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
   ret_code_t err_code;
   err_code = nrf_pwr_mgmt_init();
   APP_ERROR_CHECK(err_code);
}
////////////////////////////////// END OTHER INITIALIZATION ////////////////////////////////////////

////////////////////////////////////////// GPIO INITIALIZATION ////////////////////////////////
/*
      Initializing pin for interrupt
*/
void pins_init() {

   ret_code_t err_code;
   // INPUT PIN
   nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
   err_code = nrf_drv_gpiote_in_init(PIN_IN, & in_config, in_pin_handler);
   APP_ERROR_CHECK(err_code);
   // OUTPUT PIN DEBUG
   nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
   err_code = nrf_drv_gpiote_out_init(PIN_OUT_PRE, & out_config);
   APP_ERROR_CHECK(err_code);
   err_code = nrf_drv_gpiote_out_init(PIN_OUT_POST, & out_config);
   APP_ERROR_CHECK(err_code);
   // OUTPUT PIN PER OSCILLOSCOPIO
   err_code = nrf_drv_gpiote_out_init(PIN_OUT_OSC, & out_config);
   APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the button handler module.
 * Che differenza c'è tra inizializzarli così e con bsp_board_init??
 * Cosa cambia tra inizializzarli così e come nell'esempio ble_app_hrs??
 */
static void buttons_init(void) {
   /*Buttons init inizializza l'handler per quando viene premuto un pulsante.
    * In poche parole quando un pulsante viene premuto, verrà richiamata la funzione button_event_handler (che deve essere definita da noi)
    */
   ret_code_t err_code;

   //The array must be static because a pointer to it will be saved in the button handler module.
   //Definisco la configurazione del button (potrei anche inizializzare più tasti insieme.
   static app_button_cfg_t buttons[] = {
      {
         BSP_BUTTON_0,
         false,
         BUTTON_PULL,
         button_event_handler
      },
      {
         BSP_BUTTON_1,
         false,
         BUTTON_PULL,
         button_event_handler
      }
   };

   //Init vero e proprio dei bottoni
   err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                              BUTTON_DETECTION_DELAY);
   APP_ERROR_CHECK(err_code);
   err_code = app_button_enable();
   APP_ERROR_CHECK(err_code);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void) {
   // usato per inizializzare la board. Potrei passargli BSP_INIT_LEDS|BSP_INIT_BUTTONS per inizializzare anche i pulsanti
   bsp_board_init(BSP_INIT_LEDS);
}
//////////////////////////////////////////END GPIO INITIALIZATION ////////////////////////////////

//////////////////////////////////////////// MAIN ////////////////////////////////////////////////
/**@brief Function for application main entry.
 */
int main(void) {
   memset(packet, 0, sizeof(packet));
   // Initialize.
   log_init();
   timers_init();
   ble_stack_init();
   ble_options_set();
   rtc_init();
   leds_init();
   buttons_init();
   pins_init();
   //nrf_drv_gpiote_in_event_enable(PIN_IN, true);
   init_uart();
   
   power_management_init();
   gap_params_init();
   gatt_init();
   services_init();
   advertising_init();
   conn_params_init();
   //peer_manager_init(). Ha bisogno anche di un pm_evt_handler per gestire i vari eventi che possono scaturire. Vedo qualsiasi altro esempio (tipo hrs) per vedere come usare il modulo Peer Manager
   // Start execution.
   NRF_LOG_INFO("Custom example PERIPHERAL started");
   advertising_start();
   // Enter main loop.
   for (;;) {
      idle_state_handle();
   }
}
/////////////////////////////////// END MAIN //////////////////////////////////////////////////////