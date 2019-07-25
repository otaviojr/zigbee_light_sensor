/**
 * Copyright (c) 2019, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup zigbee_examples_multi_sensor_freertos main.c
 * @{
 * @ingroup zigbee_examples
 * @brief Zigbee Pressure and Temperature sensor using FreeRTOS
 */

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


#include "zboss_api.h"
#include "zb_mem_config_med.h"
#include "zb_error_handler.h"
#include "zigbee_helpers.h"
#include "app_timer.h"
#include "bsp.h"
#include "boards.h"
#include "sensorsim.h"

#include "nrf_assert.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "zb_multi_sensor.h"


#define IEEE_CHANNEL_MASK                  0x7FFF800		                /**< Scan only one, predefined channel to find the coordinator. */
#define ERASE_PERSISTENT_CONFIG            ZB_FALSE                              /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define ZIGBEE_NETWORK_STATE_LED           BSP_BOARD_LED_0                      /**< LED indicating that light switch successfully joind ZigBee network. */
#define BLINKY_LED                         BSP_BOARD_LED_0                      /**< Blinking led */

#define MIN_TEMPERATURE_VALUE              0                                    /**< Minimum temperature value as returned by the simulated measurement function. */
#define MAX_TEMPERATURE_VALUE              4000                                 /**< Maximum temperature value as returned by the simulated measurement function. */
#define TEMPERATURE_VALUE_INCREMENT        50                                   /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */
#define MIN_PRESSURE_VALUE                 700                                  /**< Minimum pressure value as returned by the simulated measurement function. */
#define MAX_PRESSURE_VALUE                 1100                                 /**< Maximum pressure value as returned by the simulated measurement function. */
#define PRESSURE_VALUE_INCREMENT           5                                    /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile End Device source code.
#endif


#define ZIGBEE_MAIN_TASK_PRIORITY               (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_zigbee_main_task_handle;
static SemaphoreHandle_t m_zigbee_main_task_mutex;

#define PRESSURE_MEASUREMENT_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE + 200U)
#define PRESSURE_MEASUREMENT_TASK_PRIORITY      (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_pressure_measurement_task_handle;

#define LED_TOGGLE_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE + 64U)
#define LED_TOGGLE_TASK_PRIORITY          (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_led_toggle_task_handle;


#if (NRF_LOG_ENABLED && NRF_LOG_DEFERRED)
#define LOG_TASK_STACK_SIZE               (1024U / sizeof(StackType_t))
#define LOG_TASK_PRIORITY                 (tskIDLE_PRIORITY + 1U)               /**< Must be lower than any task generating logs */
static TaskHandle_t m_logger_task_handle;
#endif


static sensor_device_ctx_t m_dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTR_LIST(identify_attr_list,
                                  m_dev_ctx.identify_attr);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
                                     &m_dev_ctx.basic_attr.zcl_version,
                                     &m_dev_ctx.basic_attr.app_version,
                                     &m_dev_ctx.basic_attr.stack_version,
                                     &m_dev_ctx.basic_attr.hw_version,
                                     m_dev_ctx.basic_attr.mf_name,
                                     m_dev_ctx.basic_attr.model_id,
                                     m_dev_ctx.basic_attr.date_code,
                                     &m_dev_ctx.basic_attr.power_source,
                                     m_dev_ctx.basic_attr.location_id,
                                     &m_dev_ctx.basic_attr.ph_env,
                                     m_dev_ctx.basic_attr.sw_ver);

ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(temperature_attr_list, 
                                            &m_dev_ctx.temp_attr.measure_value,
                                            &m_dev_ctx.temp_attr.min_measure_value, 
                                            &m_dev_ctx.temp_attr.max_measure_value, 
                                            &m_dev_ctx.temp_attr.tolerance);

ZB_ZCL_DECLARE_PRES_MEASUREMENT_ATTRIB_LIST(pressure_attr_list, 
                                            &m_dev_ctx.pres_attr.measure_value, 
                                            &m_dev_ctx.pres_attr.min_measure_value, 
                                            &m_dev_ctx.pres_attr.max_measure_value, 
                                            &m_dev_ctx.pres_attr.tolerance);

ZB_DECLARE_MULTI_SENSOR_CLUSTER_LIST(multi_sensor_clusters,
                                     basic_attr_list,
                                     identify_attr_list,
                                     temperature_attr_list,
                                     pressure_attr_list);

ZB_ZCL_DECLARE_MULTI_SENSOR_EP(multi_sensor_ep,
                               MULTI_SENSOR_ENDPOINT,
                               multi_sensor_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(multi_sensor_ctx, multi_sensor_ep);


static sensorsim_cfg_t   m_temperature_sim_cfg;                                 /**< Temperature sensor simulator configuration. */
static sensorsim_state_t m_temperature_sim_state;                               /**< Temperature sensor simulator state. */
static sensorsim_cfg_t   m_pressure_sim_cfg;                                    /**< Pressure sensor simulator configuration. */
static sensorsim_state_t m_pressure_sim_state;                                  /**< Pressure sensor simulator state. */

APP_TIMER_DEF(temperature_measurement_timer);

typedef struct
{
    zb_int16_t  measured_value;
} update_temperature_measurement_ctx_t;

/**@brief  Context used to convey data from @ref temperature_measurement_timer_handler to update_temperature_measurement_cb
 */
static update_temperature_measurement_ctx_t m_update_temperature_measurement_ctx;

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing all clusters attributes.
 */
static void multi_sensor_clusters_attr_init(void)
{
    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = SENSOR_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = SENSOR_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = SENSOR_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          SENSOR_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          SENSOR_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          SENSOR_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = SENSOR_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          SENSOR_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = SENSOR_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time        = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

    /* Temperature measurement cluster attributes data */
    m_dev_ctx.temp_attr.measure_value            = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.temp_attr.min_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.temp_attr.max_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.temp_attr.tolerance                = ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_MAX_VALUE;

    /* Pressure measurement cluster attributes data */
    m_dev_ctx.pres_attr.measure_value            = ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.pres_attr.min_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.pres_attr.max_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.pres_attr.tolerance                = ZB_ZCL_ATTR_PRES_MEASUREMENT_TOLERANCE_MAX_VALUE;
}

/**@brief Function for initializing LEDs.
 */
static void leds_init(void)
{
    ret_code_t error_code;

    /* Initialize LEDs - use BSP to control them. */
    error_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(error_code);

    bsp_board_leds_off();
}

/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_temperature_sim_cfg.min          = MIN_TEMPERATURE_VALUE;
    m_temperature_sim_cfg.max          = MAX_TEMPERATURE_VALUE;
    m_temperature_sim_cfg.incr         = TEMPERATURE_VALUE_INCREMENT;
    m_temperature_sim_cfg.start_at_max = false;

    sensorsim_init(&m_temperature_sim_state, &m_temperature_sim_cfg);

    m_pressure_sim_cfg.min          = MIN_PRESSURE_VALUE;
    m_pressure_sim_cfg.max          = MAX_PRESSURE_VALUE;
    m_pressure_sim_cfg.incr         = PRESSURE_VALUE_INCREMENT;
    m_pressure_sim_cfg.start_at_max = false;

    sensorsim_init(&m_pressure_sim_state, &m_pressure_sim_cfg);
}

/**@brief   Callback scheduled from @ref temperature_measurement_timer_handler
 *
 * @details This callback does the real job of updating zcl attribute value in ZBOSS stack. Data is passed
 *          through shared global @ref m_temperature_measurement_timer_handler_cb_ctx
 *
 * @param param[in]     Parameter passed t ZB_SCHEDULE_CALLBACK, unused in this example (required by API)
 */
static void update_temperature_measurement_cb(zb_uint8_t param)
{
    UNUSED_PARAMETER(param);

    /* Just to show that we are in zigbee_main_task context, and locking is not required */
    ASSERT(xTaskGetCurrentTaskHandle() == m_zigbee_main_task_handle);

    zb_int16_t new_temp_value;

    vTaskSuspendAll();
    new_temp_value = m_update_temperature_measurement_ctx.measured_value;
    UNUSED_RETURN_VALUE(xTaskResumeAll());

    zb_zcl_status_t zcl_status;

    /* Note: As we are in zigbee_main_task context it is perfectly correct to update without mutex locking */
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                     ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                     (zb_uint8_t *)&new_temp_value,
                                     ZB_FALSE);

    if (zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set temperature value fail. zcl_status: %d", zcl_status);
    }
}


/**@brief Function for handling nrf app timer
 *
 * @note nrf app timer when using FreeRTOS is implemented by using software timers from FreeRTOS.
 * This function is called from FreeRTOS software timers task, not zigbee_main_task
 * 
 * @param[in] context   void pointer to context, the function is called with (unused in this example, required by API)
 */
static void temperature_measurement_timer_handler(void * context)
{
    UNUSED_PARAMETER(context);

    //NRF_LOG_INFO("The pressure_measurement_timer_handler started.");

    /* Just to show that we are NOT in zigbee_main_task context */
    ASSERT(xTaskGetCurrentTaskHandle() != m_zigbee_main_task_handle);

    zb_int16_t new_temp_value;

    /* Get new temperature measured value */
    new_temp_value = (zb_int16_t)sensorsim_measure(&m_temperature_sim_state, &m_temperature_sim_cfg);

    /* Warning: According to https://freertos.org/RTOS-software-timer.html timer callback shall never attempt to block.
     * But call to zb_zcl_set_attr_val must be guarded by mutex OR must happen from zigbee_main_task context (no guarding required)
     * One method to solve the problem is to schedule a callback to zboss scheduler.
     * ZBOSS API callback types zb_callback_t and zb_callback2_t are unfortunately limited, in particular they don't allow
     * to pass opaque pointer to callback. That's why we pass data via global context here.
     */
    vTaskSuspendAll();
    m_update_temperature_measurement_ctx.measured_value = new_temp_value;
    UNUSED_RETURN_VALUE(xTaskResumeAll());

    zb_ret_t zb_ret;

    /* Note: ZB_SCHEDULE_CALLBACK is thread safe by exception, conversely to most ZBOSS API */
    zb_ret = ZB_SCHEDULE_CALLBACK(update_temperature_measurement_cb, 0U);
    if (zb_ret != RET_OK)
    {
        NRF_LOG_ERROR("Temperature sample lost.");
    }
}

/**@brief  Task performing pressure measurement.
 *
 * @note What this task does could be done from @ref temperature_measurement_timer_handler.
 * This is just to show different methods of updating attributes outgoing from zigbee stack.
 *
 * @param pvParam  Not used, required by freertos api
 */
static void pressure_measurement_task(void *pvParam)
{
    /* Just to show that we are NOT in zigbee_main_task context */
    ASSERT(xTaskGetCurrentTaskHandle() != m_zigbee_main_task_handle);

    NRF_LOG_INFO("The pressure_measurement_task started.");

    TickType_t  last_update_wake_timestamp;
    last_update_wake_timestamp = xTaskGetTickCount();

    while (true)
    {
        zb_zcl_status_t zcl_status;
        zb_int16_t      new_pres_value;

        /* Get new pressure measured value */
        new_pres_value = (zb_int16_t)sensorsim_measure(&m_pressure_sim_state, &m_pressure_sim_cfg);

        if (xSemaphoreTakeRecursive(m_zigbee_main_task_mutex, 1000U) == pdTRUE)
        {
            /* Set new pressure value as zcl attribute
             * NOTE this is not thread-safe and locking is required
             */
            zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                             ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                                             ZB_ZCL_CLUSTER_SERVER_ROLE,
                                             ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_ID,
                                             (zb_uint8_t *)&new_pres_value,
                                             ZB_FALSE);
            UNUSED_RETURN_VALUE(xSemaphoreGiveRecursive(m_zigbee_main_task_mutex));

            if (zcl_status != ZB_ZCL_STATUS_SUCCESS)
            {
                NRF_LOG_INFO("Set pressure value fail. zcl_status: %d", zcl_status);
            }
        }
        else
        {
            NRF_LOG_ERROR("Unable to take zigbee_task_mutex from pressure_measurement_task");
        }

        /* Let the task sleep for some time, consider it as pressure sample period  */
        vTaskDelayUntil(&last_update_wake_timestamp, pdMS_TO_TICKS(1000U));
    }
}


/**@brief ZigBee stack event handler.
 *
 * @param[in] param     Reference to ZigBee stack buffer used to pass arguments (signal).
 */
void zboss_signal_handler(zb_uint8_t param)
{
    /* Just to show that we are in zigbee_main_task context */
    ASSERT(xTaskGetCurrentTaskHandle() == m_zigbee_main_task_handle);

    zb_zdo_app_signal_hdr_t  * p_sg_p      = NULL;
    zb_zdo_app_signal_type_t   sig         = zb_get_app_signal(param, &p_sg_p);
    zb_ret_t                   status      = ZB_GET_APP_SIGNAL_STATUS(param);
    zb_bool_t                  comm_status;

    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == RET_OK)
            {
                NRF_LOG_INFO("Joined network successfully");
                bsp_board_led_on(ZIGBEE_NETWORK_STATE_LED);
                ret_code_t err_code = app_timer_start(temperature_measurement_timer, APP_TIMER_TICKS(1000), NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                NRF_LOG_ERROR("Failed to join network. Status: %d", status);
                bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);

                /* As we are in zboss_main_loop_iteration it is safe to call */
                comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
                ZB_COMM_STATUS_CHECK(comm_status);
            }
            break;

        case ZB_COMMON_SIGNAL_CAN_SLEEP:
            /* When freertos is used zb_sleep_now must not be used, due to
             * zigbee communication being not the only task to be performed by node.
             */
            break;

        case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            if (status != RET_OK)
            {
                NRF_LOG_WARNING("Production config is not present or invalid");
            }
            break;

        default:
            /* Unhandled signal. For more information see: zb_zdo_app_signal_type_e and zb_ret_e */
            NRF_LOG_INFO("Unhandled signal %d. Status: %d", sig, status);
            break;
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}


void zigbee_main_task(void *pvParameter)
{
    ret_code_t     err_code;
    UNUSED_PARAMETER(pvParameter);

    NRF_LOG_INFO("The zigbee_main_task started.");

   /* Start Zigbee Stack. */
    err_code = zboss_start();
    ZB_ERROR_CHECK(err_code);

    while (true)
    {
        if (xSemaphoreTakeRecursive(m_zigbee_main_task_mutex, 5) == pdTRUE)
        {
            zboss_main_loop_iteration();
            UNUSED_RETURN_VALUE(xSemaphoreGiveRecursive(m_zigbee_main_task_mutex));
        }
        vTaskDelay(1);
    }
}


/**@brief  Task function responsible for led blinking
 *
 * @note   This simple functionality could be achieved in other ways with less resources (e.g. by using timers).
 *         Here it shows just the device has some other activity to perform, not related to zigbee itself
 *
 * @param  pvParameter     FreeRTOS task parameter, unused here, required by FreeRTOS API.
 */
static void led_toggle_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    NRF_LOG_INFO("The led_toggle_task started.");

    TickType_t last_led_invert_timestamp;
    last_led_invert_timestamp = xTaskGetTickCount();

    while (true)
    {
        bsp_board_led_invert(BLINKY_LED);
        vTaskDelayUntil(&last_led_invert_timestamp, 200U);
    }
}


#if (NRF_LOG_ENABLED && NRF_LOG_DEFERRED)
/**@brief Task function responsible for deferred log processing.
 *
 * @note It must be running on lower priority than any task generating logs.
 *
 * @param pvParameter     FreeRTOS task parameter, unused here, required by FreeRTOS API.
 */
static void logger_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    NRF_LOG_INFO("Logger task");
    NRF_LOG_FINAL_FLUSH();

    while (true)
    {
        if (!(NRF_LOG_PROCESS()))
        {
            /* No more logs, let's sleep and wait for any */
            vTaskDelay(1);
        }
    }
}
#endif

/**@brief FreeRTOS hook function called from idle task */
void vApplicationIdleHook(void)
{
    /* No task is running, just idle on lowest priority, so we can lower power usage */
    __WFE();
}

/**@brief  FreeRTOS hook function called when stack overflow has been detected
 * @note   See FreeRTOS API
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    NRF_LOG_ERROR("vApplicationStackOverflowHook(%x,\"%s\")", xTask, pcTaskName);
    NRF_LOG_FINAL_FLUSH();
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
}

/**@brief Function which tries to sleep down the MCU
 *
 * @details This function is called from @ref zboss_main_loop_iteration, when there is no more
 * messages to process. Because @ref zigbee_main_task is not the only one, implementation found
 * in zb_nrf52840_common.c is not appropriate.
 *
 * @note    This function overrides implementation found in zb_nrf52840_common.c
 */
zb_void_t zb_osif_go_idle(zb_void_t)
{
    /* Intentionally empty implementation */
}

/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t     err_code;
    zb_ieee_addr_t ieee_addr;

    /* Initialize logging system and GPIOs. */
    log_init();
    NRF_LOG_INFO("Application start");
    NRF_LOG_FINAL_FLUSH();

    clock_init();
    timers_init();
    sensor_simulator_init();
    leds_init();

    /* Set ZigBee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize ZigBee stack. */
    ZB_INIT("multi_sensor");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set static long IEEE address. */
    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);

    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));
    zb_set_rx_on_when_idle(ZB_FALSE);

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register temperature sensor device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&multi_sensor_ctx);

    /* Initialize sensor device attibutes */
    multi_sensor_clusters_attr_init();

    BaseType_t rtos_result;
#if (NRF_LOG_ENABLED && NRF_LOG_DEFERRED)
    rtos_result = xTaskCreate(logger_task, "LOG", LOG_TASK_STACK_SIZE,
            NULL, LOG_TASK_PRIORITY, &m_logger_task_handle);
    if (rtos_result != pdTRUE)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    /* Create task for LED0 blinking with priority set to 2 */
    rtos_result = xTaskCreate(led_toggle_task, "LED", LED_TOGGLE_TASK_STACK_SIZE,
            NULL, LED_TOGGLE_TASK_PRIORITY, &m_led_toggle_task_handle);
    if (rtos_result != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    rtos_result = xTaskCreate(pressure_measurement_task, "PM", PRESSURE_MEASUREMENT_TASK_STACK_SIZE,
            NULL, PRESSURE_MEASUREMENT_TASK_PRIORITY, &m_pressure_measurement_task_handle);
    if (rtos_result != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    /* Create Timer for reporting attribute */
    err_code = app_timer_create(&temperature_measurement_timer, APP_TIMER_MODE_REPEATED,
            temperature_measurement_timer_handler);
    APP_ERROR_CHECK(err_code);

    m_zigbee_main_task_mutex = xSemaphoreCreateRecursiveMutex();
    if (m_zigbee_main_task_mutex == NULL)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    /* Create task where zboss_main_loop_iteration will be called from */
    rtos_result = xTaskCreate(zigbee_main_task, "ZB", ZIGBEE_MAIN_TASK_STACK_SIZE,
            NULL, ZIGBEE_MAIN_TASK_PRIORITY, &m_zigbee_main_task_handle);
    if (rtos_result != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    /* Start FreeRTOS scheduler. */
    NRF_LOG_INFO("Starting FreeRTOS scheduler");
    NRF_LOG_FLUSH();

    vTaskStartScheduler();

    while (true)
    {
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
    }
}


/**
 * @}
 */
