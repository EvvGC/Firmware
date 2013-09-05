/*
 *  pins.h
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
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
