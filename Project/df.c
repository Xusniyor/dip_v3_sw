/*
 * df.c
 *
 *  Created on: 1 февр. 2023 г.
 *      Author: Xusniyor
 */

#include "main.h"

#define DF_ST_TIME 100

typedef struct
{
	uint32_t last_time;
	uint16_t count;
	uint8_t ff;
	uint8_t st;
	uint8_t lock;
}__df_t;

__df_t _df[5] = {0};

uint8_t df_isSet(uint8_t inx, GPIO_PinState st)
{
	if (st == GPIO_PIN_SET)
	{
		if (_df[inx].ff == 1) {
			if (_df[inx].count > DF_ST_TIME) {
				_df[inx].st = 1;
			}
			else if (_df[inx].last_time - HAL_GetTick() > 1) {
				_df[inx].last_time = HAL_GetTick();
				_df[inx].count++;
			}
		}
		else
		{
			_df[inx].ff = 1;
			_df[inx].count = 0;
		}
	}
	else {
		if (_df[inx].ff == 0) {
			if (_df[inx].count > DF_ST_TIME) {
				_df[inx].st = 0;
			}
			else if (_df[inx].last_time - HAL_GetTick() > 1) {
				_df[inx].last_time = HAL_GetTick();
				_df[inx].count++;
			}
		}
		else
		{
			_df[inx].ff = 0;
			_df[inx].count = 0;
		}
	}
	return _df[inx].st;
}

