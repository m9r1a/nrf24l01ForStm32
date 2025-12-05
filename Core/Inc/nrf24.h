/*
 * nrf24.h
 *
 *  Created on: Nov 6, 2025
 *      Author: m.reza
 */

#ifndef INC_NRF24_H_
#define INC_NRF24_H_

#include "stdbool.h"

typedef void (*NRF_ReceiceCallback_t)(uint8_t *receiveData , uint8_t pipe);
/*Exported Function-------------------------*/
void NRF_Init(SPI_HandleTypeDef * hSpi,NRF_ReceiceCallback_t recCallback);
void NRF_FinishSpiTransaction();
void NRF_IrqInterruptRoutin(void);
void NRF_TransmitPacket(uint8_t *packet , uint8_t length,uint8_t pipe);
bool NRF_CheckTransmittingAvailability(void);
void NRF_Process();
#endif /* INC_NRF24_H_ */
