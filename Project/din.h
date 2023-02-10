/*
 * din.h
 *
 *  Created on: 1 февр. 2023 г.
 *      Author: Xusniyor
 */

#ifndef DIN_H_
#define DIN_H_

#include "main.h"

#define DIN_DON()  HAL_GPIO_ReadPin(DIN_1_GPIO_Port, DIN_1_Pin) // DIP    ON   Digital Input
#define DIN_DOFF() HAL_GPIO_ReadPin(DIN_2_GPIO_Port, DIN_2_Pin) // DIP    OFF  Digital Input
#define DIN_LON()  HAL_GPIO_ReadPin(DIN_3_GPIO_Port, DIN_3_Pin) // Light  ON   Digital Input
#define DIN_ERR()  HAL_GPIO_ReadPin(DIN_4_GPIO_Port, DIN_4_Pin) // ERROR       Digital Input
#define DIN_OEN()  HAL_GPIO_ReadPin(DIN_5_GPIO_Port, DIN_5_Pin) // OUTPUT EN   Digital Input
#define DIN_LEN()  HAL_GPIO_ReadPin(DIN_6_GPIO_Port, DIN_6_Pin) // Light  EN   Digital Input
#define DIN_DOO()  HAL_GPIO_ReadPin(DIN_7_GPIO_Port, DIN_7_Pin) // DOOR   OPEN Digital Input

#define DIN_USE_BUTTON()  !HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) // USE BUTTON Digital Input

#endif /* DIN_H_ */
