/**
 * @file   tsl2561.h
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

#ifndef __ZB_LIGHT_SENSOR_TSL2561_H__
#define __ZB_LIGHT_SENSOR_TSL2561_H__

/* TSL2561 */
#define TSL2561_ADDR					   0x29
#define TSL2561_COMMAND					   0x80

typedef enum _TSL2561_REGS {
    TSL2561_REG_CONTROL = 0x00,
    TSL2561_REG_TIMMING = 0x01,
    TSL2561_REG_THRESHLOWLOW = 0x02,
    TSL2561_REG_THRESHLOWHIGH  = 0x03,
    TSL2561_REG_THRESHHIGHLOW = 0x04,
    TSL2561_REG_THRESHHIGHHIGH= 0x05,
    TSL2561_REG_INTERRUPT = 0x06,
    TSL2561_REG_CRC = 0x07,
    TSL2561_REG_ID = 0xA,
    TSL2561_REG_DATA0LOW = 0x0C,
    TSL2561_REG_DATA0HIGH = 0x0D,
    TSL2561_REG_DATA1LOW = 0x0E,
    TSL2561_REG_DATA1HIGH = 0x0F
} TSL2561_REGS;

typedef enum _tsl2561_luxerror {
    TSL2561_LUX_ERROR_NOERROR = 0x00,
    TSL2561_LUX_ERROR_HIGHLUM = 0x01,
    TSL2561_LUX_ERROR_LOWLUM = 0x02
} tsl2561_luxerror_t;

typedef enum _tsl2561_sensitivity {
    TSL2561_SENSITIVITY_LOW = 0x01,
    TSL2561_SENSITIVITY_MEDIUM = 0x02,
    TSL2561_SENSITIVITY_HIGH = 0x03
} tsl2561_sensitivity_t;

void tsl2561_twi_handler(nrf_drv_twi_evt_t const *, void *);

void tsl2561_init(const nrf_drv_twi_t* m_twi_master);
void tsl2561_sensitivity(const nrf_drv_twi_t* m_twi_master, tsl2561_sensitivity_t sens);
uint8_t tsl2561_read_id(nrf_drv_twi_t const *);
tsl2561_luxerror_t tsl2561_read_lux(const nrf_drv_twi_t* m_twi_master, uint16_t* lux);

#endif /* __ZB_LIGHT_SENSOR_TSL2561_H__ */
