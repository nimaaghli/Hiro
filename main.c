

/** @file
 *
 * @defgroup Hiro main.c
 * @{
 * @brief Main project file for Hiro
 *
 * This file contains main source code for Hiro with NRF52832 nordic chips.
 * Please Refer to readme file in order ro compile and generate the output file 
 * @author  Nima Aghli
 * @version 1.0 07/18/18
 *   
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_gatts.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_bas.h"
#include "ble_tps.h"
#include "ble_ias.h"
#include "ble_ias_c.h"
#include "nrfx_saadc.h"
#include "ble_lls.h"
#include "ble_dis.h"
#include "uuid.h"
#include "uicr.h"
#include "ble_db_discovery.h"
#include "ble_dfu.h"
#include "low_power_pwm.h"  
#include "app_button.h"
#include "nrf_delay.h"
//#include "nrf_svci_async_function.h"
//#include "nrf_svci_async_handler.h"
//#include "nrf_bootloader_info.h"
//#include "nrf_drv_clock.h"
//#include "nrf_power.h"
/*Ticks before change duty cycle of each LED*/


static int tune[] = {30,55,30,55,35,54,55};
static int waits[] = {235,157,100,187,168,130,167};
static int counter = 0;

/* Valid Advertising States */
typedef enum {
    BLE_NO_ADV,
    BLE_FAST_ADV, /**< Fast advertising running. */
    BLE_SLOW_ADV, /**< Slow advertising running. */
    BLE_SLEEP,
} ble_advertising_mode_t;

#define IS_SRVC_CHANGED_CHARACT_PRESENT   1                                     /**<You must enable the Service Changed characteristic so that changes in the application are indicated to other devices, most importantly the DFU controller. */



#define DEVICE_NAME                    "HiroV2.1"                                   /**< Name of device. Will be included in the advertising data. */                                                            
#define MANUFACTURER                   "Super Hiro LLC"
#define VERSION                        "0.4r"

#define APP_ADV_INTERVAL                0x0020                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(20000)                  /**< Battery level measurement interval (ticks). This value corresponds to 20 seconds. */
#define BUTTON_LONG_PRESS_DELAY         APP_TIMER_TICKS(2000)

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024  

#define INITIAL_LLS_ALERT_LEVEL         BLE_CHAR_ALERT_LEVEL_NO_ALERT           /**< Initial value for the Alert Level characteristic in the Link Loss service. */

#define TX_POWER_LEVEL                  (0)                                      /**< TX Power Level value. This will be set both in the TX Power 
                                                                                service, in the advertising data, and also used to set the radio  
                                                                                transmit power. */
    
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                      /**< Delay from a GPIOTE event \
                                                                                 until a button is reported as  \
                                                                                 pushed (in number of timer     \
                                                                                 ticks). */

#define BUZZER_POWER_OFF_DURECTION      APP_TIMER_TICKS(900)
#define BUZZER_BUTTON_PUSHH_DURECTION   APP_TIMER_TICKS(400)
#define PUSH_BUTTON_PIN                 16                                      /**< Pin number for push button on Hiro */

        
#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


                                          /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;    
static nrf_saadc_value_t adc_buf[2];                                            /**< Handle of the current connection. */
static volatile bool m_is_high_alert_signalled;                                 /**< Variable to indicate whether or not high
                                                                                alert is signalled to the peer. */
static low_power_pwm_t low_power_pwm_0;
static low_power_pwm_t low_power_pwm_1;

static bool button_timer_running_p = false;
static uint8_t m_advertising_mode;                                             /**< Variable to keep track of when we are
                                                                               advertising. */
static volatile bool m_is_high_alert_signalled;                                /**< Variable to indicate whether a high alert has been signalled to the peer. */
static volatile bool m_is_ias_present = false;                                /**< Variable to indicate whether the immediate alert service has been discovered at the connected peer. */
static volatile bool m_is_poweroff_buzzed = false;   
static volatile bool m_is_button_push_buzzed = false; 
             
/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)


/*  Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);   
APP_TIMER_DEF(m_battery_timer_id); 
APP_TIMER_DEF(m_buzzer_timer_id);
APP_TIMER_DEF(m_button_timer_id);        
BLE_BAS_DEF(m_bas); 
BLE_LLS_DEF(m_lls);
BLE_TPS_DEF(m_tps);
BLE_IAS_C_DEF(m_ias_c);
BLE_IAS_DEF(m_ias, NRF_SDH_BLE_TOTAL_LINK_COUNT);  
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);                                       /**< Immediate Alert service instance. */

// Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
        {BLE_UUID_TX_POWER_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_IMMEDIATE_ALERT_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_LINK_LOSS_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};

static void advertising_start(bool erase_bonds);
static void on_lls_evt(ble_lls_t * p_lls, ble_lls_evt_t * p_evt);
static void on_ias_evt(ble_ias_t * p_ias, ble_ias_evt_t * p_evt);
static void on_ias_c_evt(ble_ias_c_t * p_lls, ble_ias_c_evt_t * p_evt);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void play_tune(uint8_t signal_id){
    ret_code_t err_code;
    if(signal_id == 0 ) // power off tune 
    {    m_is_poweroff_buzzed = true;
        err_code = low_power_pwm_start((&low_power_pwm_0), low_power_pwm_0.bit_mask);
        APP_ERROR_CHECK(err_code);
        err_code = low_power_pwm_start((&low_power_pwm_1), low_power_pwm_1.bit_mask);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(m_buzzer_timer_id, BUZZER_POWER_OFF_DURECTION, NULL);
        APP_ERROR_CHECK(err_code);
        
    }
    else if(signal_id == 1 ) // power off tune 
    {   m_is_button_push_buzzed = true;
        err_code = low_power_pwm_start((&low_power_pwm_0), low_power_pwm_0.bit_mask);
        APP_ERROR_CHECK(err_code);
        err_code = low_power_pwm_start((&low_power_pwm_1), low_power_pwm_1.bit_mask);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start(m_buzzer_timer_id, BUZZER_BUTTON_PUSHH_DURECTION, NULL);
        APP_ERROR_CHECK(err_code);
        
    }


}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}



/**@brief Function for the Signals alert event from Immediate Alert or Link Loss services.
 *
 * @param[in] alert_level  Requested alert level.
 */
static void alert_signal(uint8_t alert_level)
{
    ret_code_t err_code;

    switch (alert_level)
    {
        case BLE_CHAR_ALERT_LEVEL_NO_ALERT:
            NRF_LOG_INFO("No Alert.");
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
            APP_ERROR_CHECK(err_code);
            err_code = low_power_pwm_stop(&low_power_pwm_0);
            APP_ERROR_CHECK(err_code);
            err_code = low_power_pwm_stop(&low_power_pwm_1);
            APP_ERROR_CHECK(err_code);
            break; // BLE_CHAR_ALERT_LEVEL_NO_ALERT

        case BLE_CHAR_ALERT_LEVEL_MILD_ALERT:
            NRF_LOG_INFO("Mild Alert.");
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_CHAR_ALERT_LEVEL_MILD_ALERT

        case BLE_CHAR_ALERT_LEVEL_HIGH_ALERT:
            NRF_LOG_INFO("HIGH Alert.");
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_3);
            APP_ERROR_CHECK(err_code);
            err_code = low_power_pwm_start((&low_power_pwm_0), low_power_pwm_0.bit_mask);
            APP_ERROR_CHECK(err_code);
            err_code = low_power_pwm_start((&low_power_pwm_1), low_power_pwm_1.bit_mask);
            APP_ERROR_CHECK(err_code);
            break; // BLE_CHAR_ALERT_LEVEL_HIGH_ALERT

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for handling Link Loss events.
 *
 * @details This function will be called for all Link Loss events which are passed to the
 *          application.
 *
 * @param[in] p_lls  Link Loss structure.
 * @param[in] p_evt  Event received from the Link Loss service.
 */
static void on_lls_evt(ble_lls_t * p_lls, ble_lls_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_LLS_EVT_LINK_LOSS_ALERT:
            NRF_LOG_INFO("Link loss happend");
            alert_signal(p_evt->params.alert_level);
            break; // BLE_LLS_EVT_LINK_LOSS_ALERT

        default:
            // No implementation needed.
            break;
    }
}



// static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
// {
//     if (state == NRF_SDH_EVT_STATE_DISABLED)
//     {
//         // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
//         nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

//         //Go to system off.
//         nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
//     }
// }

// /* nrf_sdh state observer. */
// NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
// {
//     .handler = buttonless_dfu_sdh_state_observer,
// };

//  Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
// static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
// {
//     switch (event)
//     {
//         case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
//             NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
//             // YOUR_JOB: Disconnect all bonded devices that currently are connected.
//             //           This is required to receive a service changed indication
//             //           on bootup after a successful (or aborted) Device Firmware Update.
//             break;

//         case BLE_DFU_EVT_BOOTLOADER_ENTER:
//             // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
//             //           by delaying reset by reporting false in app_shutdown_handler
//             NRF_LOG_INFO("Device will enter bootloader mode.");
//             break;

//         case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
//             NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
//             // YOUR_JOB: Take corrective measures to resolve the issue
//             //           like calling APP_ERROR_CHECK to reset the device.
//             break;

//         case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
//             NRF_LOG_ERROR("Request to send a response to client failed.");
//             // YOUR_JOB: Take corrective measures to resolve the issue
//             //           like calling APP_ERROR_CHECK to reset the device.
//             APP_ERROR_CHECK(false);
//             break;

//         default:
//             NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
//             break;
//     }
// }



/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    //NRF_LOG_INFO("saadc_event_handler= ");
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);
        NRF_LOG_INFO("SAADC Event=  %i", percentage_batt_lvl);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_BUSY) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
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
    NRF_LOG_INFO("db_disc_handler!");
    ble_ias_c_on_db_disc_evt(&m_ias_c, p_evt);
}


static void adc_configure(void)
{
    nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG; 
    ret_code_t err_code = nrfx_saadc_init(&saadc_config, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
    NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err_code = nrfx_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}


// static void battery_level_update(void)
// {
//     // ret_code_t err_code;
//     // //uint8_t  battery_level;

//     // //battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

//     // err_code = ble_bas_battery_level_update(&m_bas, 99, BLE_CONN_HANDLE_ALL);
//     // if ((err_code != NRF_SUCCESS) &&
//     //     (err_code != NRF_ERROR_INVALID_STATE) &&
//     //     (err_code != NRF_ERROR_RESOURCES) &&
//     //     (err_code != NRF_ERROR_BUSY) &&
//     //     (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//     //    )
//     // {
//     //     APP_ERROR_HANDLER(err_code);
//     // }

// }


static void battery_level_meas_timeout_handler(void * p_context)
{
    // UNUSED_PARAMETER(p_context);
    // battery_level_update();
    //   UNUSED_PARAMETER(p_context);
    //NRF_LOG_INFO("SAADC battery_level_meas_timeout");
    ret_code_t err_code;
    err_code = nrfx_saadc_sample();
    APP_ERROR_CHECK(err_code);
}


static void button_timeout_handler(void *p_context) {
    NRF_LOG_INFO("Restarting");
    play_tune(0);
    //nrf_delay_ms(500);
    //NVIC_SystemReset();
    // Does not return
}

static void buzzer_timeout_handler(void *p_context) {
    ret_code_t err_code;
    err_code = low_power_pwm_stop(&low_power_pwm_0);
    APP_ERROR_CHECK(err_code);
    err_code = low_power_pwm_stop(&low_power_pwm_1);
    APP_ERROR_CHECK(err_code);
    if(m_is_poweroff_buzzed){ NVIC_SystemReset(); }
    m_is_button_push_buzzed = false;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
       // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);

    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_button_timer_id, APP_TIMER_MODE_SINGLE_SHOT,
                                button_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_buzzer_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                buzzer_timeout_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));

    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
    APP_ERROR_CHECK(err_code);

    /*  Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    //err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT,BLE_CONN_HANDLE_ALL,TX_POWER_LEVEL);
    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}




/**@brief Function for initializing the Link Loss Service.
 */
static void lls_init(void)
{
    ret_code_t     err_code;
    ble_lls_init_t lls_init_obj;

    // Initialize Link Loss Service
    memset(&lls_init_obj, 0, sizeof(lls_init_obj));

    lls_init_obj.evt_handler         = on_lls_evt;
    lls_init_obj.error_handler       = service_error_handler;
    lls_init_obj.initial_alert_level = INITIAL_LLS_ALERT_LEVEL;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lls_init_obj.lls_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lls_init_obj.lls_attr_md.write_perm);

    err_code = ble_lls_init(&m_lls, &lls_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    
    // Here the sec level for the Battery Service can be changed/increased.
    ble_bas_init_t     bas_init;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

static void dis_init(){
    ret_code_t         err_code;
    ble_dis_init_t     dis_init;
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, "00000000");
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, VERSION);
    //char buf[5] = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
    //sprintf(buf, "%" PRIu32, (NRF_FICR->DEVICEID[1]  & 0xff));
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, "57");

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

static void tps_init(void) {
    uint32_t err_code;
    ble_tps_init_t tps_init_obj;

    memset(&tps_init_obj, 0, sizeof(tps_init_obj));
    tps_init_obj.initial_tx_power_level = TX_POWER_LEVEL;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&tps_init_obj.tps_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&tps_init_obj.tps_attr_md.write_perm);

    err_code = ble_tps_init(&m_tps, &tps_init_obj);
    APP_ERROR_CHECK(err_code);
}

static void ias_init(void) {
    uint32_t err_code;
    ble_ias_init_t ias_init_obj;

    memset(&ias_init_obj, 0, sizeof(ias_init_obj));
    ias_init_obj.evt_handler = on_ias_evt;
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ias_init_obj.ias_attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&ias_init_obj.ias_attr_md.write_perm);
    err_code = ble_ias_init(&m_ias, &ias_init_obj);
    APP_ERROR_CHECK(err_code);
    
}


/**@brief Function for initializing the immediate alert service client.
 *
 * @details This will initialize the client side functionality of the Find Me profile.
 */
static void ias_client_init(void) {
    uint32_t err_code;
    ble_ias_c_init_t ias_c_init_obj;

    memset(&ias_c_init_obj, 0, sizeof(ias_c_init_obj));

     m_is_high_alert_signalled = false;

     ias_c_init_obj.evt_handler = on_ias_c_evt;
     ias_c_init_obj.error_handler = service_error_handler;

     err_code = ble_ias_c_init(&m_ias_c, &ias_c_init_obj);
     if(err_code == NRF_ERROR_INVALID_STATE){
        NRF_LOG_INFO("ias_init Error Code %i", err_code);

     }
     APP_ERROR_CHECK(err_code);
     
}


// static void dfu_init(void) {
//     // ble_dfu_init_t dfus_init;
//     // // Initialize the Device Firmware Update Service.
//     // memset(&dfus_init, 0, sizeof(dfus_init));
//     // dfus_init.evt_handler = dfu_app_on_dfu_evt;
//     // dfus_init.error_handler = NULL;
//     // uint32_t err_code = ble_dfu_init(&m_dfus, &dfus_init);
//     // APP_ERROR_CHECK(err_code);

//     // dfu_app_reset_prepare_set(reset_prepare);
//     uint32_t  err_code;
//     ble_dfu_buttonless_init_t dfus_init = {0};
//        // Initialize the async SVCI interface to bootloader.
//     err_code = ble_dfu_buttonless_async_svci_init();
//     APP_ERROR_CHECK(err_code);

//     dfus_init.evt_handler = ble_dfu_evt_handler;

//     err_code = ble_dfu_buttonless_init(&dfus_init);
//     APP_ERROR_CHECK(err_code);
// }


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{

    tps_init();
    ias_init();
    bas_init();
    lls_init();
    dis_init();
    ias_client_init();
  
    
    //dfu_init(); //fix the bootloader


   
    
     
}


/** @brief Database discovery module initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Immediate Alert events.
 *
 * @details This function will be called for all Immediate Alert events which are passed to the
 *          application.
 *
 * @param[in] p_ias  Immediate Alert structure.
 * @param[in] p_evt  Event received from the Immediate Alert service.
 */
static void on_ias_evt(ble_ias_t *p_ias, ble_ias_evt_t *p_evt) {
    NRF_LOG_INFO("BLE_IAS_EVT_ALERT_LEVEL_UPDATED");
    switch (p_evt->evt_type) {
    case BLE_IAS_EVT_ALERT_LEVEL_UPDATED:
        alert_signal(p_evt->p_link_ctx->alert_level);

        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for handling Link Loss events.
 *
 * @details This function will be called for all Link Loss events which are passed to the
 *          application.
 *
 * @param[in] p_lls  Link Loss structure.
 * @param[in] p_evt  Event received from the Link Loss service.
 */
static void on_ias_c_evt(ble_ias_c_t *p_ias_c, ble_ias_c_evt_t *p_evt) {
    
    ret_code_t err_code;
    switch (p_evt->evt_type) {
    case BLE_IAS_C_EVT_DISCOVERY_COMPLETE:
                        // IAS is found on peer. The Find Me Locator functionality of this app will work.
            err_code = ble_ias_c_handles_assign(&m_ias_c,
                                                p_evt->conn_handle,
                                                p_evt->alert_level.handle_value);
            NRF_LOG_INFO("BLE_IAS_C_EVT_DISCOVERY_COMPLETE!");
            APP_ERROR_CHECK(err_code);
            m_is_ias_present = true;
        break;

    case BLE_IAS_C_EVT_DISCOVERY_FAILED:
        // IAS is not found on peer. Do Nothing.
        break;

    case BLE_IAS_C_EVT_DISCONN_COMPLETE:
            m_is_ias_present = false;
            NRF_LOG_INFO("BLE_IAS_C_EVT_DISCONN_COMPLETE");
        break;

    default:
        // No implementation needed.
        break;
    }
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;

        default:
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
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            // Assign connection handle to the Queued Write module.
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            // Discover peer's services.
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    //** Pairing//
    sec_param.bond = false;
    sec_param.mitm = false;
    sec_param.lesc = 0;
    sec_param.keypress = 0;
    sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;
    sec_param.oob = false;
    sec_param.min_key_size = 7;
    sec_param.max_key_size = 16;
    sec_param.kdist_own.enc = 0;
    sec_param.kdist_own.id = 0;
    sec_param.kdist_peer.enc = 0;
    sec_param.kdist_peer.id = 0;
    /**Bonfing**///
    // sec_param.bond = true;
    // sec_param.mitm = false;
    // sec_param.lesc = 0;
    // sec_param.keypress = 0;
    // sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;
    // sec_param.oob = false;
    // sec_param.min_key_size = 7;
    // sec_param.max_key_size = 16;
    // sec_param.kdist_own.enc = 1;
    // sec_param.kdist_own.id = 1;
    // sec_param.kdist_peer.enc = 1;
    // sec_param.kdist_peer.id = 1;



    //err_code = pm_sec_params_set(&sec_param);
    //APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {     
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0
        case BSP_EVENT_KEY_3:
        {
            
            if (m_is_ias_present)
            {
                NRF_LOG_INFO("m_is_ias_present");
                if (!m_is_high_alert_signalled)
                {
                    err_code =
                        ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_HIGH_ALERT);
                        NRF_LOG_INFO("BLE_CHAR_ALERT_LEVEL_HIGH_ALERT");

                }
                else
                {
                    err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_NO_ALERT);
                    NRF_LOG_INFO("BLE_CHAR_ALERT_LEVEL_LOW_ALERT");
                }

                if (err_code == NRF_SUCCESS)
                {
                    m_is_high_alert_signalled = !m_is_high_alert_signalled;
                }
                else if (
                    (err_code != NRF_ERROR_RESOURCES)
                    &&
                    (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                    &&
                    (err_code != NRF_ERROR_NOT_FOUND)
                        )
                {
                    APP_ERROR_HANDLER(err_code);
                }
            }
        } break;    

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;
    int8_t tx_power_level = TX_POWER_LEVEL;
    m_advertising_mode = BLE_NO_ADV;
    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.advdata.p_tx_power_level        = &tx_power_level;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void button_event_handler(uint8_t pin_no, uint8_t button_action) {
    ret_code_t err_code;
    if (button_action == APP_BUTTON_PUSH) {
        app_timer_start(m_button_timer_id, BUTTON_LONG_PRESS_DELAY, NULL);
        button_timer_running_p = true;

    }
    else{
        if(button_timer_running_p){
            app_timer_stop(m_button_timer_id);
            NRF_LOG_INFO("THIS IS A SHORT Press");
            play_tune(1);
            if (m_is_ias_present)
            {
                NRF_LOG_INFO("m_is_ias_present");
                if (!m_is_high_alert_signalled)
                {
                    err_code =
                        ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_HIGH_ALERT);
                        NRF_LOG_INFO("BLE_CHAR_ALERT_LEVEL_HIGH_ALERT");

                }
                else
                {
                    err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_NO_ALERT);
                    NRF_LOG_INFO("BLE_CHAR_ALERT_LEVEL_LOW_ALERT");
                }

                if (err_code == NRF_SUCCESS)
                {
                    m_is_high_alert_signalled = !m_is_high_alert_signalled;
                }
                else if (
                    (err_code != NRF_ERROR_RESOURCES)
                    &&
                    (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                    &&
                    (err_code != NRF_ERROR_NOT_FOUND)
                        )
                {
                    APP_ERROR_HANDLER(err_code);
                }
            }
            //m_is_button_push_buzzed = false;
        }
    }
 
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    //bsp_event_t startup_event;
    static app_button_cfg_t buttons[] = {
    {PUSH_BUTTON_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP,
    button_event_handler}};
    app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                    BUTTON_DETECTION_DELAY);
     err_code = app_button_enable();
     APP_ERROR_CHECK(err_code);
    //err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    //APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    // APP_ERROR_CHECK(err_code);

    //*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
    //     //nrf_pwr_mgmt_run();
       uint32_t err_code = sd_app_evt_wait();
    /*
       This signals the softdevice handler that we want the CPU to
       sleep until an event/interrupt occurs. During this time the
       softdevice will do what it needs to do.
    */
    APP_ERROR_CHECK(err_code);
    }

}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @brief Function to be called in timer interrupt.
 *
 * @param[in] p_context     General purpose pointer (unused).
 */
static void pwm_handler(void * p_context)
{
    if(m_is_poweroff_buzzed){
        tune[0] = 30;
        waits[0] = BUZZER_POWER_OFF_DURECTION;
    }
    else if(m_is_button_push_buzzed){
        tune[0] = 20;
        waits[0] = BUZZER_BUTTON_PUSHH_DURECTION;
    }
    else{
        tune[0] = 30;
        waits[0] = 135;
    }
    uint8_t new_duty_cycle;
    static uint16_t led_0,led_1;
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);

    low_power_pwm_t * pwm_instance = (low_power_pwm_t*)p_context;

    if (pwm_instance->bit_mask == (1 <<  NRF_GPIO_PIN_MAP(0,14)))
    {
        led_0++;

        if (led_0 > waits[counter])
        {
             pwm_instance->period = tune[counter];
             new_duty_cycle = pwm_instance->period / 2;
             err_code = low_power_pwm_duty_set(pwm_instance, new_duty_cycle);
             APP_ERROR_CHECK(err_code);
            led_0 = 0;
            counter++;
            if (counter > 6)
            {
                counter = 0 ;
            }
      
            //APP_ERROR_CHECK(err_code);
        }
    }
     else if (pwm_instance->bit_mask == (1 <<  NRF_GPIO_PIN_MAP(0,15)))
    {
        led_1++;

        if (led_1 > waits[counter])
        {   
             pwm_instance->period = tune[counter];
             new_duty_cycle = pwm_instance->period / 2;
             err_code = low_power_pwm_duty_set(pwm_instance, new_duty_cycle);
             APP_ERROR_CHECK(err_code);
            led_1 = 0;
            //APP_ERROR_CHECK(err_code);
         
    
        }
    }
    else
    {
        /*empty else*/
    }
}


/**
 * @brief Function to initalize low_power_pwm instances.
 *
 */

static void pwm_init(void)
{
    uint32_t err_code;
    low_power_pwm_config_t low_power_pwm_config;

    APP_TIMER_DEF(lpp_timer_0);
    low_power_pwm_config.active_high    = true;
    low_power_pwm_config.period         = 30;
    low_power_pwm_config.bit_mask       = (1 <<  NRF_GPIO_PIN_MAP(0,14));
    low_power_pwm_config.p_timer_id     = &lpp_timer_0;
    low_power_pwm_config.p_port         = NRF_GPIO;

    err_code = low_power_pwm_init((&low_power_pwm_0), &low_power_pwm_config, pwm_handler);
    APP_ERROR_CHECK(err_code);
    err_code = low_power_pwm_duty_set(&low_power_pwm_0, 15);
    APP_ERROR_CHECK(err_code);

    APP_TIMER_DEF(lpp_timer_1);
    low_power_pwm_config.active_high    = true;
    low_power_pwm_config.period         = 30;
    low_power_pwm_config.bit_mask       = (1 <<  NRF_GPIO_PIN_MAP(0,15));
    low_power_pwm_config.p_timer_id     = &lpp_timer_1;
    low_power_pwm_config.p_port         = NRF_GPIO;

    err_code = low_power_pwm_init((&low_power_pwm_1), &low_power_pwm_config, pwm_handler);
    APP_ERROR_CHECK(err_code);
    err_code = low_power_pwm_duty_set(&low_power_pwm_1, 15);
    APP_ERROR_CHECK(err_code);


}



/**@brief Function for application main entry.
 */
int main(void)
{
    NRF_POWER->DCDCEN = 1;
    bool erase_bonds;
    get_uicr();
    // Initialize.
    log_init();
    timers_init();
    pwm_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    adc_configure();
    gap_params_init();
    gatt_init();
    advertising_init();
    db_discovery_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("Hiro started.");
    application_timers_start();

    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
