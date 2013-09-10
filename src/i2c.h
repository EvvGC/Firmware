/*
 *  i2c.h
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis aka caat
 */

#ifndef I2C_H_
#define I2C_H_

//extern int I2Cerror = 0;      //need them to make local!;
//extern int I2Cerrorcount = 0; //need to make them local;
extern int I2Cerror;        //need them to make local!;
extern int I2Cerrorcount;   //need to make them local;

#define I2C_SDA_PIN     GPIO_Pin_11
#define I2C_SDA_PORT    GPIOB

#define I2C_SCL_PIN     GPIO_Pin_10
#define I2C_SCL_PORT    GPIOB

#define SDAH()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN,   Bit_SET)
#define SDAL()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN,   Bit_RESET)

#define SCLH()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN,   Bit_SET)
#define SCLL()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN,   Bit_RESET)

void I2C_delay(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Ack(void);
void I2C1_NoAck(void);
void I2C1_SendByte(unsigned char SendByte);
void I2C1_WaitAck(void);
//uint8_t I2C1_CheckAck(void);
uint8_t I2C1_ReceiveByte(void);

#endif /* I2C_H_*/
