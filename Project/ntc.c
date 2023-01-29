/*
 * ntc.c
 *
 *  Created on: Jan 26, 2023
 *      Author: Xusniyor
 */

#include <math.h>
#include "ntc.h"

#define _NTC_R_SERIES         10000.0f
#define _NTC_R_NOMINAL        10000.0f
#define _NTC_TEMP_NOMINAL     25.0f
#define _NTC_ADC_MAX          4095.0f
#define _NTC_BETA             3950.0f

float ntc_convertToC(float adcValue)
{
  float rntc = (float)_NTC_R_SERIES / (((float)_NTC_ADC_MAX / (4095.0f - adcValue)) - 1.0f);
  float temp;
  temp = rntc / (float)_NTC_R_NOMINAL;
  temp = logf(temp);
  temp /= (float)_NTC_BETA;
  temp += 1.0f / ((float)_NTC_TEMP_NOMINAL + 273.15f);
  temp = 1.0f / temp;
  temp -= 273.15f;
  return temp;
}
