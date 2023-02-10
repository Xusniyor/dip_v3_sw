/*
 * relays.h
 *
 *  Created on: 1 февр. 2023 г.
 *      Author: Xusniyor
 */

#ifndef RELAYS_H_
#define RELAYS_H_

#define RELAY_NIG(n) HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, n)  // Light Relay
#define RELAY_CON(n) HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, n)  // Contactor Relay
#define RELAY_ERO(n) HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, n)  // Error Relay


#endif /* RELAYS_H_ */
