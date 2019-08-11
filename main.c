/**
 *     Changes:
 *
 *     2-8-2019: Otávio Ribeiro <otavio.ribeiro@gmail.com>
 *
 *     Addapting i2c digital light sensor to work as a zigbee illumination sensor
 *     using Nordic SDK
 */
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

#include "nrf_assert.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "zb_light_sensor.h"

#include "twi_config.h"
#include "nrf_drv_twi.h"
#include "tsl2561.h"

/**
 * TWI master instance.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

#define IEEE_CHANNEL_MASK                  0x7FFF800		                    /**< Scan only one, predefined channel to find the coordinator. */
#define ERASE_PERSISTENT_CONFIG            ZB_FALSE                             /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define ZIGBEE_NETWORK_STATE_LED           BSP_BOARD_LED_0                      /**< Joined, Joing, Failed Join */
#define ZIGBEE_NETWORK_COMMISSIONING_BTN   BSP_BUTTON_0                   		/**< Joined, Joing, Failed Join */

#define MIN_TEMPERATURE_VALUE              0                                    /**< Minimum temperature value as returned by the simulated measurement function. */
#define MAX_TEMPERATURE_VALUE              4000                                 /**< Maximum temperature value as returned by the simulated measurement function. */
#define TEMPERATURE_VALUE_INCREMENT        50                                   /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */
#define MIN_PRESSURE_VALUE                 700                                  /**< Minimum pressure value as returned by the simulated measurement function. */
#define MAX_PRESSURE_VALUE                 1100                                 /**< Maximum pressure value as returned by the simulated measurement function. */
#define PRESSURE_VALUE_INCREMENT           5                                    /**< Value by which the temperature value is incremented/decremented for each call to the simulated measurement function. */

#define MAX_ILLUMINANCE                    1500
#define MIN_ILLUMINANCE                    0

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile End Device source code.
#endif

#define ZIGBEE_MAIN_TASK_PRIORITY               (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_zigbee_main_task_handle;
static SemaphoreHandle_t m_zigbee_main_task_mutex;

#define ILLUMINANCE_MEASUREMENT_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE + 200U)
#define ILLUMINANCE_MEASUREMENT_TASK_PRIORITY      (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_illuminance_measurement_task_handle;

#define LED_TOGGLE_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE + 64U)
#define LED_TOGGLE_TASK_PRIORITY          (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_led_toggle_task_handle;

#if (NRF_LOG_ENABLED && NRF_LOG_DEFERRED)
#define LOG_TASK_STACK_SIZE               (1024U / sizeof(StackType_t))
#define LOG_TASK_PRIORITY                 (tskIDLE_PRIORITY + 1U)               /**< Must be lower than any task generating logs */
static TaskHandle_t m_logger_task_handle;
#endif

static sensor_device_ctx_t m_dev_ctx;

/* current light sensor sensitivity */
static tsl2561_sensitivity_t current_sensitivity = TSL2561_SENSITIVITY_HIGH;

/* keep track of how many button has been clicked */
static uint8_t button_count = 0;
static volatile bool ind_reset_network = false;

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

ZB_ZCL_DECLARE_ILLUMINANCE_MEASUREMENT_ATTRIB_LIST(illum_measure_attr_list,
                                            &m_dev_ctx.illum_attr.measure_value,
                                            &m_dev_ctx.illum_attr.min_measure_value,
                                            &m_dev_ctx.illum_attr.max_measure_value);

ZB_DECLARE_LIGHT_SENSOR_CLUSTER_LIST(light_sensor_clusters,
                                     basic_attr_list,
                                     identify_attr_list,
                                     illum_measure_attr_list);

ZB_ZCL_DECLARE_LIGHT_SENSOR_EP(light_sensor_ep,
                               LIGHT_SENSOR_ENDPOINT,
							   light_sensor_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(light_sensor_ctx, light_sensor_ep);

APP_TIMER_DEF(button_timer);

/**@brief   Callback scheduled from @ref button_timer_handler
 *
 * @details This callback does the real job executing the button action. That will depend on the number
 *          of times that the buttons has been clicked
 * 
 * @param param[in]     Parameter passed t ZB_SCHEDULE_CALLBACK, unused in this example (required by API)
 */
static void button_handle_cb(zb_uint8_t param)
{
    NRF_LOG_INFO("Processing button handle: %d clicks", param);
    if(param == 10){
        NRF_LOG_INFO("Factory Reset");
        ind_reset_network = true;
    } else if(param == 2){
        //TODO: save this change on flash to start the sensor right on reboot
        current_sensitivity = (current_sensitivity == TSL2561_SENSITIVITY_HIGH ? TSL2561_SENSITIVITY_LOW : TSL2561_SENSITIVITY_HIGH);
        NRF_LOG_INFO("Changing sensor sensitivity to %s", (current_sensitivity == TSL2561_SENSITIVITY_HIGH ? "high" : "low"));
        tsl2561_sensitivity(&m_twi_master, current_sensitivity);
    }
}

/**@brief Function for handling counting how many times the button has been pressed
 *
 * @param[in] context   void pointer to context
 */
static void button_timer_handler(void * context)
{
    ret_code_t error_code;
    zb_ret_t zb_ret;
    UNUSED_PARAMETER(context);

    /* Note: ZB_SCHEDULE_CALLBACK is thread safe by exception, conversely to most ZBOSS API */
    zb_ret = ZB_SCHEDULE_CALLBACK(button_handle_cb, button_count);
    if (zb_ret != RET_OK)
    {
        NRF_LOG_ERROR("Button Handle Lost.");
    }

    /* reset button count*/
    button_count = 0;

    /* stop timer */
    error_code = app_timer_stop(button_timer);
    APP_ERROR_CHECK(error_code);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    tsl2561_twi_handler(p_event, p_context);
}

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, twi_handler, NULL);

    if (NRF_SUCCESS == ret)
    {
        NRF_LOG_INFO("Twi interface started.");
        nrf_drv_twi_enable(&m_twi_master);
    } else {
        NRF_LOG_INFO("Error starting twi interface.");
    }

    return ret;
}

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
static void light_sensor_clusters_attr_init(void)
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

    /* Illuminance measurement cluster attributes data */
    m_dev_ctx.illum_attr.measure_value			 = ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_DEFAULT;
    m_dev_ctx.illum_attr.min_measure_value		 = MIN_ILLUMINANCE;
    m_dev_ctx.illum_attr.max_measure_value		 = MAX_ILLUMINANCE;
}

void bsp_evt_handler(bsp_event_t evt)
{
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            /* increase button count and start waiting */
            button_count ++;

            /* start our timer. This will fail after the second click, but its ok */
            app_timer_start(button_timer, APP_TIMER_TICKS(5000), NULL);
            break;

        default:
            return; // no implementation needed
    }
}


/**@brief Function for initializing LEDs and Buttons.
 */
static void board_init(void)
{
    ret_code_t error_code;

    /* Initialize LEDs - use BSP to control them. */
    error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(error_code);

    bsp_board_leds_off();
}

/**@brief  Task performing illuminance measurement.
 *
 * @param pvParam  Not used, required by freertos api
 */
static void illuminance_measurement_task(void *pvParam)
{
    NRF_LOG_INFO("The illuminance_measurement_task started.");

    TickType_t  last_update_wake_timestamp;
    last_update_wake_timestamp = xTaskGetTickCount();

    uint8_t signature = tsl2561_read_id(&m_twi_master);
    if(signature == 0x50){
        NRF_LOG_INFO("Digital Light Sensor TSL2561 found! Initializing...");
        tsl2561_init(&m_twi_master);
        tsl2561_sensitivity(&m_twi_master, current_sensitivity);
    } else  {
        NRF_LOG_ERROR("Digital Light Sensor TSL2561 not found!");
    }

    while (true)
    {
        zb_zcl_status_t zcl_status;
        zb_uint16_t new_illumination_value;

        /* Get new pressure measured value */
        if(tsl2561_read_lux(&m_twi_master, (uint16_t*)&new_illumination_value) != TSL2561_LUX_ERROR_NOERROR){
          new_illumination_value = MAX_ILLUMINANCE;
        }

        if(new_illumination_value > MAX_ILLUMINANCE){
            new_illumination_value = MAX_ILLUMINANCE;
        }

        if(new_illumination_value <= 1) new_illumination_value = 0;

        NRF_LOG_INFO("Illumination value is: %ld", new_illumination_value);

        if (xSemaphoreTakeRecursive(m_zigbee_main_task_mutex, 1000U) == pdTRUE)
        {
           /* Set new illuminance value as zcl attribute
            * NOTE this is not thread-safe and locking is required
            */
            zcl_status = zb_zcl_set_attr_val(LIGHT_SENSOR_ENDPOINT,
	                                     ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
                                             ZB_ZCL_CLUSTER_SERVER_ROLE,
	                                     ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
                                             (zb_uint8_t *)&new_illumination_value,
                                             ZB_FALSE);

            UNUSED_RETURN_VALUE(xSemaphoreGiveRecursive(m_zigbee_main_task_mutex));

            if (zcl_status != ZB_ZCL_STATUS_SUCCESS)
            {
                NRF_LOG_INFO("Set illumination value fail. zcl_status: %d", zcl_status);
            }
        }
        else
        {
            NRF_LOG_ERROR("Unable to take zigbee_task_mutex from illumination_measurement_task");
        }
        /* Let the task sleep for some time, consider it as illumination sample period  */
        vTaskDelayUntil(&last_update_wake_timestamp, pdMS_TO_TICKS(5000U));
    }
}


/**@brief ZigBee stack event handler.
 *
 * @param[in] param     Reference to ZigBee stack buffer used to pass arguments (signal).
 */
void zboss_signal_handler(zb_uint8_t param)
{
    zb_bool_t comm_status;
    zb_zdo_app_signal_hdr_t  * p_sg_p      = NULL;
    zb_zdo_app_signal_type_t   sig         = zb_get_app_signal(param, &p_sg_p);
    zb_ret_t                   status      = ZB_GET_APP_SIGNAL_STATUS(param);

    switch (sig)
    {
    	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
        case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
            if (!status == RET_OK) {
                NRF_LOG_ERROR("Failed to join network. Status: %d", status);
                comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
                ZB_COMM_STATUS_CHECK(comm_status);
            }
        	break;

        case ZB_ZDO_SIGNAL_LEAVE:
            NRF_LOG_INFO("Leaving Zigbee Network");
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

void leave_callback(zb_uint8_t param)
{
  zb_zdo_mgmt_leave_res_t *resp = (zb_zdo_mgmt_leave_res_t *)ZB_BUF_BEGIN(ZB_BUF_FROM_REF(param));
  NRF_LOG_INFO("LEAVE CALLBACK status %d", resp->status);
  zb_nvram_clear();
}

static void light_sensor_leave_and_join( zb_uint8_t param ){
    if (ZB_JOINED())
    {
		zb_buf_t *buf = ZB_BUF_FROM_REF(param);
		zb_zdo_mgmt_leave_param_t *req = ZB_GET_BUF_PARAM(buf, zb_zdo_mgmt_leave_param_t);

		UNUSED_RETURN_VALUE(ZB_BZERO(req, sizeof(zb_zdo_mgmt_leave_param_t)));
		req->dst_addr = ZB_PIBCACHE_NETWORK_ADDRESS();
		req->rejoin = ZB_FALSE;

		UNUSED_RETURN_VALUE(zdo_mgmt_leave_req(param, leave_callback));
    }
}

void zigbee_main_task(void *pvParameter)
{
	//zb_ret_t 		zb_err_code;
    ret_code_t     	err_code;
    UNUSED_PARAMETER(pvParameter);

    NRF_LOG_INFO("The zigbee_main_task started.");

   /* Start Zigbee Stack. */
    err_code = zboss_start();
    ZB_ERROR_CHECK(err_code);

    while (true)
    {
    	if(ind_reset_network){
    		ind_reset_network = false;
    		light_sensor_leave_and_join(1);
    	}

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
        bsp_board_led_invert(ZIGBEE_NETWORK_STATE_LED);
        vTaskDelayUntil(&last_led_invert_timestamp, (ZB_JOINED() ? 5000U : 200U));
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
    ret_code_t error_code;
    zb_ieee_addr_t ieee_addr;

    /* Initialize logging system and GPIOs. */
    log_init();
    NRF_LOG_INFO("Application start");
    NRF_LOG_FINAL_FLUSH();

    clock_init();
    timers_init();
    board_init();
    twi_master_init();

    /* Set ZigBee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize ZigBee stack. */
    ZB_INIT("zigbee_digital_light_sensor");

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
    ZB_AF_REGISTER_DEVICE_CTX(&light_sensor_ctx);

    /* Initialize sensor device attibutes */
    light_sensor_clusters_attr_init();

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

    rtos_result = xTaskCreate(illuminance_measurement_task, "LUX", ILLUMINANCE_MEASUREMENT_TASK_STACK_SIZE,
            NULL, ILLUMINANCE_MEASUREMENT_TASK_PRIORITY, &m_illuminance_measurement_task_handle);
    if (rtos_result != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

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

   /* Create Timer for executing button action */
    error_code = app_timer_create(&button_timer, APP_TIMER_MODE_REPEATED, button_timer_handler);
    APP_ERROR_CHECK(error_code);


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
