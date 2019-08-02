/**
 * @file   twi_config.h
 * @author Otavio Ribeiro
 * @date   02 Ago 2019
 * @brief  Nordic TWI/I2C configurations
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

#ifndef __ZB_LIGHT_SENSOR_TWI_CNFIG_H__
#define __ZB_LIGHT_SENSOR_TWI_CNFIG_H__

	/**
	 * TWI master configurations.
	 */
	#define MASTER_TWI_INST     0       //!< TWI interface used as a master accessing EEPROM memory.
	#define UART_TX_BUF_SIZE    1024    //!< UART TX buffer size.
	#define UART_RX_BUF_SIZE    32      //!< UART RX buffer size.
	#define TWI_SCL_M           11       //!< Master SCL pin.
	#define TWI_SDA_M           8       //!< Master SDA pin.
	#define IN_LINE_PRINT_CNT   (16u)   //!< Number of data bytes printed in a single line.

#endif /* __ZB_LIGHT_SENSOR_TWI_CNFIG_H__ */
