/**
 * @file   tsl2561_lux.c
 * @author Otavio Ribeiro
 * @date   02 Ago 2019
 * @brief  Calculating lux from both ADC sensors
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
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf_log.h"
#include "tsl2561_lux.h"

uint32_t tsl2561_calculate_lux(uint16_t ch0, uint16_t ch1, char iType)
{
	uint32_t lux = 0;
	float ratio = (float)ch1/(float)ch0;

	switch(iType){
		case 'T':
			if(ratio >= 0 && ratio <= 0.50) {
				lux = (0.0304*ch0) - (0.062*ch0*pow(ratio, 1.4));
			} else if(ratio > 0.50 && ratio <= 0.61) {
				lux = (0.0224*ch0) - (0.031*ch1);
			} else if(ratio > 0.61 && ratio <= 0.80) {
				lux = (0.0128*ch0) - (0.0153*ch1);
			} else if(ratio > 0.80 && ratio <= 1.30) {
				lux = (0.00146*ch0) - (0.00112*ch1);
			} else {
				lux = 0;
			}
			break;
		case 'C':
			if(ratio >= 0 && ratio <= 0.52) {
				lux = (0.0315*ch0) - (0.0593*ch0*pow(ratio, 1.4));
			} else if(ratio > 0.52 && ratio <= 0.65) {
				lux = (0.0229*ch0) - (0.0291*ch1);
			} else if(ratio > 0.65 && ratio <= 0.80) {
				lux = (0.0157*ch0) - (0.0180*ch1);
			} else if(ratio > 0.80 && ratio <= 1.30) {
				lux = (0.00338*ch0) - (0.00260*ch1);
			} else {
				lux = 0;
			}
			break;

		default:
			lux = 0;
			break;
	}
	return(lux);
}
