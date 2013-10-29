/*
 *  pins.h
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
 */

/*
    Original work Copyright (c) 2012 [Evaldis - RCG user name]
    Modified work Copyright 2012 Alan K. Adamson

    This file is part of EvvGC.

    EvvGC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EvvGC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EvvGC.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef PINS_H_
#define PINS_H_

#define LED1_PIN        GPIO_Pin_12
#define LED1_PORT       GPIOB

#define LED2_PIN        GPIO_Pin_3
#define LED2_PORT       GPIOA

#define I2C_SDA_PIN     GPIO_Pin_11
#define I2C_SDA_PORT    GPIOB

#define I2C_SCL_PIN     GPIO_Pin_10
#define I2C_SCL_PORT    GPIOB

void GPIO_Config(void);

#endif /* PINS_H_ */
