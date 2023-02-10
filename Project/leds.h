/*
 * leds.h
 *
 *  Created on: 1 февр. 2023 г.
 *      Author: Xusniyor
 */

#ifndef LEDS_H_
#define LEDS_H_

#define LED_WR(n)  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, n)  // Warning LED
#define LED_HV(n)  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, n)  // High voltage LED
#define LED_TMP(n) HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, n)  // Temperature indicator LED
#define LED_CV(n)  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, n)  // Control with voltage LED
#define LED_CC(n)  HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, n)  // Control with current LED

#endif /* LEDS_H_ */
