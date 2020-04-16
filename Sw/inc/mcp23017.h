/*
 * This file is part of the KAMBot project.
 * 
 *  Copyright (C) 2016 Frantisek Burian <bufran _at_ seznam.cz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef MCP23017_H
#define MCP23017_H

#include <stdbool.h>
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADDR_MCP23017_A(a)    (0x20 | ((a) & 7))

// bank 0 scheme - the power on reset value
// all registers little endian (A = lo, B= hi)
#define MCP23017_IODIR        0x00      // 16bit
#define MCP23017_IOPOL        0x02      // 16bit
#define MCP23017_GPINTEN      0x04      // 16bit
#define MCP23017_DEFVAL       0x06      // 16bit
#define MCP23017_INTCON       0x08      // 16bit
#define MCP23017_IOCON1       0x0A      // 8bit same as IOCON2
#define MCP23017_IOCON2       0x0B      // 8bit same as IOCON1
#define MCP23017_GPPU         0x0C      // 16bit
#define MCP23017_INTF         0x0E      // 16bit
#define MCP23017_INTCAP       0x10      // 16bit  
#define MCP23017_GPIO         0x12      // 16bit
#define MCP23017_OLAT         0x14      // 16bit

#define MCP23017_IOCON_BANK       (1 << 7)
#define MCP23017_IOCON_MIRROR     (1 << 6)
#define MCP23017_IOCON_SEQOP      (1 << 5)
#define MCP23017_IOCON_DISSLW     (1 << 4)
#define MCP23017_IOCON_HAEN       (1 << 3)
#define MCP23017_IOCON_ODR        (1 << 2)
#define MCP23017_IOCON_INTPOL     (1 << 1)
  
#define MCP23017_PB7              (1 << 15)
#define MCP23017_PB6              (1 << 14)
#define MCP23017_PB5              (1 << 13)
#define MCP23017_PB4              (1 << 12)
#define MCP23017_PB3              (1 << 11)
#define MCP23017_PB2              (1 << 10)
#define MCP23017_PB1              (1 << 9)
#define MCP23017_PB0              (1 << 8)
#define MCP23017_PA7              (1 << 7)
#define MCP23017_PA6              (1 << 6)
#define MCP23017_PA5              (1 << 5)
#define MCP23017_PA4              (1 << 4)
#define MCP23017_PA3              (1 << 3)
#define MCP23017_PA2              (1 << 2)
#define MCP23017_PA1              (1 << 1)
#define MCP23017_PA0              (1 << 0)  


static inline int mcp23017_input(int bus, int chip, uint16_t pins)
{
	return i2c_rmw_leuint16(bus, chip, MCP23017_IODIR, 0, pins, 0);
	return i2c_rmw_leuint16(bus, chip, MCP23017_GPPU, pins, 0, 0);
}

static inline int mcp23017_input_pullup(int bus, int chip, uint16_t pins)
{
	return i2c_rmw_leuint16(bus, chip, MCP23017_IODIR, 0, pins, 0);
	return i2c_rmw_leuint16(bus, chip, MCP23017_GPPU, 0, pins, 0);
}

static inline int mcp23017_output(int bus, int chip, uint16_t pins)
{
	return i2c_rmw_leuint16(bus, chip, MCP23017_IODIR, pins, 0, 0);
}

static inline int mcp23017_set(int bus, int chip, uint16_t pins, bool value)
{
	return i2c_rmw_leuint16(bus, chip, MCP23017_OLAT, value ? 0 : pins, value ? pins : 0, 0);
}

static inline int mcp23017_setlow(int bus, int chip, uint16_t pins)
{
  return i2c_rmw_leuint16(bus, chip, MCP23017_OLAT, pins, 0, 0);
}

static inline int mcp23017_sethigh(int bus, int chip, uint16_t pins)
{
  return i2c_rmw_leuint16(bus, chip, MCP23017_OLAT, 0, pins, 0);
}

static inline int mcp23017_toggle(int bus, int chip, uint16_t pins)
{
  return i2c_rmw_leuint16(bus, chip, MCP23017_OLAT, 0, 0, pins);
}
  
static inline int mcp23017_setpullup(int bus, int chip, uint16_t pins)
{
	return i2c_rmw_leuint16(bus, chip, MCP23017_GPPU, 0, pins, 0);
}

static inline int mcp23017_setgpio(int bus, int chip, uint16_t pins)
{
	return i2c_rmw_leuint16(bus, chip, MCP23017_GPIO, 0, pins, 0);
}
static inline int mcp23017_resetGPIO(int bus, int chip, uint16_t pins)
{
	return i2c_rmw_leuint16(bus, chip, MCP23017_GPIO, pins, 0, 0);
}

static inline bool mcp23017_get(int bus, int chip, uint16_t pins)
{
	uint16_t p = 0;
	// TODO return code !
	i2c_read_leuint16(bus, chip, MCP23017_GPIO, &p);
	return (pins & p) != 0;
}
  
#ifdef __cplusplus
}
#endif

#endif /* MCP23017_H */

