/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
#include "hm610.h"

struct hm610_board hm610_boards[] = {
	[HM610_BOARD] = {
		.name		= "HM610 DVB-S/S2 ",
		.adapters	= 8,
		.eeprom_i2c	= 0,
		.eeprom_addr	= 0x10,
		.adap_config	= {
			{
				.ts_in = 0,
				.i2c_bus_nr = 0,
				.gpio.demod_reset.lvl = HM610_GPIODEF_LOW,
				.gpio.demod_reset.nr  = 8,	
			}, 
			{
				.ts_in = 1,
				.i2c_bus_nr = 0,
			},
			{
				.ts_in = 2,
				.i2c_bus_nr = 0,
			},
			{
				.ts_in = 3,
				.i2c_bus_nr = 0,
			},
			{
				.ts_in = 4,
				.i2c_bus_nr = 0,
			},
			{
				.ts_in = 5,
				.i2c_bus_nr = 0,
			},
			{
				.ts_in = 6,
				.i2c_bus_nr = 0,
			},
			{
				.ts_in = 7,
				.i2c_bus_nr = 0,
			}
		}
	},
};
