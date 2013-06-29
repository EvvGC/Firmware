/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////
// I2C Initialize
///////////////////////////////////////////////////////////////////////////////

void i2cInit(I2C_TypeDef *I2C);

///////////////////////////////////////////////////////////////////////////////
// I2C Write Buffer
///////////////////////////////////////////////////////////////////////////////

bool i2cWriteBuffer(I2C_TypeDef *I2C, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);

///////////////////////////////////////////////////////////////////////////////
// I2C1Write
///////////////////////////////////////////////////////////////////////////////

bool i2cWrite(I2C_TypeDef *I2C, uint8_t addr_, uint8_t reg, uint8_t data);

///////////////////////////////////////////////////////////////////////////////
// I2C Read
///////////////////////////////////////////////////////////////////////////////

bool i2cRead(I2C_TypeDef *I2C, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);

///////////////////////////////////////////////////////////////////////////////
// Get I2C Error Count
///////////////////////////////////////////////////////////////////////////////

uint16_t i2cGetErrorCounter(I2C_TypeDef *I2C);

///////////////////////////////////////////////////////////////////////////////
