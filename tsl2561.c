/**
 * @file   tsl2561.c
 * @author Otavio Ribeiro
 * @date   02 Ago 2019
 * @brief  Communicating with tsl2561 digital light sensor through i2c
 *
 * Copyright (c) 2019 Otávio Ribeiro <otavio.ribeiro@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#include "nrf_log.h"

#include "twi_config.h"
#include "nrf_drv_twi.h"
#include "tsl2561.h"

#include  "tsl2561_lux.h"

/* Buffer for samples read from light sensor. */
volatile static char tsl2561_twi_processing = 0;
static uint8_t tsl2561_buffer[5];

/**
 * @brief TWI events handler.
 */
void tsl2561_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	if(tsl2561_twi_processing == 0) return;

    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
        	tsl2561_twi_processing = 0;
            break;

        default:
        	NRF_LOG_INFO("Unexpected TWI event received of type = 0x%x, xfer = 0x%x", p_event->type, p_event->xfer_desc.type);
        	tsl2561_twi_processing = 0;
        	break;
    }
}

/**
 * @brief Function for reading sensor id
 */
uint8_t tsl2561_read_id(const nrf_drv_twi_t* m_twi_master)
{
    tsl2561_twi_processing = 1;
    uint8_t byte_send[1] = {TSL2561_COMMAND | TSL2561_REG_ID};
    ret_code_t err_code = nrf_drv_twi_tx(m_twi_master, TSL2561_ADDR, byte_send, 1, true);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    tsl2561_twi_processing = 1;
    err_code = nrf_drv_twi_rx(m_twi_master, TSL2561_ADDR, tsl2561_buffer, 1);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    return tsl2561_buffer[0];
}

/**
 * @brief Function for adjust sensor timing and sensitivity
 */
void tsl2561_sensitivity(const nrf_drv_twi_t* m_twi_master, tsl2561_sensitivity_t sens)
{
    ret_code_t err_code;
    uint8_t byte_send[2];

    tsl2561_twi_processing = 1;
    byte_send[0] = TSL2561_COMMAND | TSL2561_REG_TIMMING;
    switch(sens){
        case TSL2561_SENSITIVITY_LOW:
            /**
             * GAIN => (0) 1x low gain
             * MANUAL => (0)
             * RESERV => (0)
             * INTEG => (10) 402ms
             */
            byte_send[1] = 0x2;
            break;

        case TSL2561_SENSITIVITY_MEDIUM:
            /**
             * GAIN => (1) 16x high gain
             * MANUAL => (0)
             * RESERV => (0)
             * INTEG => (01) 101ms
             */
            byte_send[1] = 0x11;
    	    break;

        case TSL2561_SENSITIVITY_HIGH:
        default:
            /**
             * GAIN => (1) 16x high gain
             * MANUAL => (0)
             * RESERV => (0)
             * INTEG => (10) 402ms
             */
            byte_send[1] = 0x12;
    	    break;
    }

    err_code = nrf_drv_twi_tx(m_twi_master, TSL2561_ADDR, byte_send, 2, true);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);
}

/**
 * @brief Function for starting conversion
 */
void tsl2561_init(const nrf_drv_twi_t* m_twi_master)
{
    ret_code_t err_code;
    uint8_t byte_send[2];

    tsl2561_twi_processing = 1;
    byte_send[0] = TSL2561_COMMAND | TSL2561_REG_CONTROL;
    byte_send[1] = 0x3;
    err_code = nrf_drv_twi_tx(m_twi_master, TSL2561_ADDR, byte_send, 2, true);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);
}

/**
 * @brief Function for reading the last conversion
 */
tsl2561_luxerror_t tsl2561_read_lux(const nrf_drv_twi_t* m_twi_master, uint16_t* lux)
{
    ret_code_t err_code;
    uint8_t byte_send[1];
    uint16_t data0;
    uint16_t data1;

    tsl2561_twi_processing = 1;
    byte_send[0] = TSL2561_COMMAND | TSL2561_REG_DATA0LOW;
    err_code = nrf_drv_twi_tx(m_twi_master, TSL2561_ADDR, byte_send, 1, true);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    tsl2561_twi_processing = 1;
    err_code = nrf_drv_twi_rx(m_twi_master, TSL2561_ADDR, tsl2561_buffer, 1);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    data0 = tsl2561_buffer[0];

    tsl2561_twi_processing = 1;
    byte_send[0] = TSL2561_COMMAND | TSL2561_REG_DATA0HIGH;
    err_code = nrf_drv_twi_tx(m_twi_master, TSL2561_ADDR, byte_send, 1, true);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    tsl2561_twi_processing = 1;
    err_code = nrf_drv_twi_rx(m_twi_master, TSL2561_ADDR, tsl2561_buffer, 1);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    data0 |= (tsl2561_buffer[0] << 8);

    tsl2561_twi_processing = 1;
    byte_send[0] = TSL2561_COMMAND | TSL2561_REG_DATA1LOW;
    err_code = nrf_drv_twi_tx(m_twi_master, TSL2561_ADDR, byte_send, 1, true);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    tsl2561_twi_processing = 1;
    err_code = nrf_drv_twi_rx(m_twi_master, TSL2561_ADDR, tsl2561_buffer, 1);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    data1 = tsl2561_buffer[0];

    tsl2561_twi_processing = 1;
    byte_send[0] = TSL2561_COMMAND | TSL2561_REG_DATA1HIGH;
    err_code = nrf_drv_twi_tx(m_twi_master, TSL2561_ADDR, byte_send, 1, true);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    tsl2561_twi_processing = 1;
    err_code = nrf_drv_twi_rx(m_twi_master, TSL2561_ADDR, tsl2561_buffer, 1);
    APP_ERROR_CHECK(err_code);
    while(tsl2561_twi_processing == 1);

    data1 |= (tsl2561_buffer[0] << 8);

    //NRF_LOG_INFO("Illumination value 0 is: 0x%x", data0);
    //NRF_LOG_INFO("Illumination value 1 is: 0x%x", data1);

    if(data0 == 0xFFFF) {
	    *lux = 0;
        return TSL2561_LUX_ERROR_HIGHLUM;
    }

    *lux = tsl2561_calculate_lux(data0, data1, 'T') & 0xFFFF;
    return TSL2561_LUX_ERROR_NOERROR;
}
