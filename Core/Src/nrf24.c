/*
 * nrf24.c
 *
 *  Created on: Nov 6, 2025
 *      Author: m.reza
 */
#include "main.h"
#include "stdbool.h"
#include <stdlib.h>
#include <string.h>
#include "nrf24.h"
/*definition ------------------------------------*/
/* RF Channel (0–125)
 * Final RF frequency = 2400 MHz + CHANNEL
 * Example: channel 40 → 2440 MHz
 */
#define NRF_CHANNEL                40      // RF channel number

/* RF_SETUP register value
 * Bit layout:
 *   Bit5 = RF_DR_LOW
 *   Bit3 = RF_DR_HIGH
 *   Bit2:1 = RF_PWR (00=-18dBm, 01=-12dBm, 10=-6dBm, 11=0dBm)
 *
 * 0x06 = 0000 0110:
 *     RF_DR_HIGH = 0 → 1Mbps
 *     RF_DR_LOW  = 0
 *     RF_PWR = 11 → 0dBm
 */
#define NRF_RF_SETUP_VALUE         0x26    // 1Mbps + 0dBm (stable)

/* Address width configuration (SETUP_AW register)
 *   0x01 = 3 bytes
 *   0x02 = 4 bytes
 *   0x03 = 5 bytes (recommended)
 */
#define NRF_ADDRESS_WIDTH          0x03    // 5-byte address width

/* Static payload length (when dynamic payload is disabled)
 * Valid range: 1–32 bytes
 */
#define NRF_PAYLOAD_SIZE           32      // fixed payload width

/* Default RX addresses for pipes 0 and 1 (full 5-byte addressing) */
static const uint8_t PIPE0_ADDR[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
static const uint8_t PIPE1_ADDR[5] = {0xC2,0xC2,0xC2,0xC2,0xC2};

/* Pipes 2–5 share the upper bytes of Pipe1.
 * Only the least significant byte (LSB) differs.
 */
#define PIPE2_ADDR_LSB             0xC3
#define PIPE3_ADDR_LSB             0xC4
#define PIPE4_ADDR_LSB             0xC5
#define PIPE5_ADDR_LSB             0xC6

#define CONF_REG 0x1b  //rx_dr enable, tx_ds enable, crc, power up set 1; crc 1 byte,max_rt set to 0 ; prx mode

/*Defined functions ------------------------*/
#define SPI_CSN_LOW()	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET)
#define SPI_CSN_HIGH()	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET)
#define NRF_CE_HIGH()	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)
#define NRF_CE_LOW()	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)
/* typedef ----------------------------------*/

typedef struct _NrfRegisterValues{
	/* ----------------------------------------------------
	 * 0x00 CONFIG REGISTER
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t PRIM_RX      :1; // bit 0
			uint8_t PWR_UP       :1;
			uint8_t CRCO         :1;
			uint8_t EN_CRC       :1;
			uint8_t MASK_MAX_RT  :1;
			uint8_t MASK_TX_DS   :1;
			uint8_t MASK_RX_DR   :1;
			uint8_t RESERVED     :1; // bit 7
		}bits;
		uint8_t value;
	}CONFIG;


	/* ----------------------------------------------------
	 * 0x01 EN_AA – ENABLE AUTO-ACKNOWLEDGMENT
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t ENAA_P0 :1;
			uint8_t ENAA_P1 :1;
			uint8_t ENAA_P2 :1;
			uint8_t ENAA_P3 :1;
			uint8_t ENAA_P4 :1;
			uint8_t ENAA_P5 :1;
			uint8_t RESERVED:2;
		}bits;
		uint8_t value;
	}EN_AA;


	/* ----------------------------------------------------
	 * 0x02 EN_RXADDR – ENABLE RX ADDRESSES
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t ERX_P0 :1;
			uint8_t ERX_P1 :1;
			uint8_t ERX_P2 :1;
			uint8_t ERX_P3 :1;
			uint8_t ERX_P4 :1;
			uint8_t ERX_P5 :1;
			uint8_t RESERVED:2;
		}bits;
		uint8_t value;
	}EN_RXADDR;


	/* ----------------------------------------------------
	 * 0x03 SETUP_AW – ADDRESS WIDTH
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t AW       :2; // 00 illegal, 01=3B,02=4B,03=5B
			uint8_t RESERVED :6;
		}bits;
		uint8_t value;
	}SETUP_AW;


	/* ----------------------------------------------------
	 * 0x04 SETUP_RETR – AUTO RETRANSMIT
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t ARC :4; // retry count 0..15
			uint8_t ARD :4; // delay (250us steps)
		}bits;
		uint8_t value;
	}SETUP_RETR;


	/* ----------------------------------------------------
	 * 0x05 RF_CH – RF CHANNEL
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t RF_CH    :7;
			uint8_t RESERVED :1;
		}bits;
		uint8_t value;
	}RF_CH;
	/* ----------------------------------------------------
	 * 0x06 RF_SETUP – RF POWER/SPEED
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t OBSOLETE  :1;
			uint8_t RF_PWR    :2; // 00=-18dBm ... 11=0dBm
			uint8_t RF_DR_HIGH:1; // 1=2Mbps
			uint8_t PLL_LOCK  :1;
			uint8_t RF_DR_LOW :1;
			uint8_t RESERVED  :1;
			uint8_t CONT_WAVE :1;
		}bits;
		uint8_t value;
	}RF_SETUP;
	/* ----------------------------------------------------
	 * 0x07 STATUS – IRQ STATUS
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t TX_FULL :1;
			uint8_t RX_P_NO :3;
			uint8_t MAX_RT  :1;
			uint8_t TX_DS   :1;
			uint8_t RX_DR   :1;
			uint8_t RESERVED:1;
		}bits;
		uint8_t value;
	}STATUS;
	/* ----------------------------------------------------
	 * 0x08 OBSERVE_TX – PACKET LOSS STATS
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t ARC_CNT  :4;
			uint8_t PLOS_CNT :4;
		}bits;
		uint8_t value;
	}OBSERVE_TX;
	/* ----------------------------------------------------
	 * 0x09 RPD – CARRIER DETECT
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t RPD       :1;
			uint8_t RESERVED :7;
		}bits;
		uint8_t value;
	}RPD;
	/* ----------------------------------------------------
	 * 0x0A..0x0F   RX_ADDR_P0..P5 (multi-byte)
	 * ---------------------------------------------------- */
	uint8_t RX_ADDR_P0[5];
	uint8_t RX_ADDR_P1[5];
	uint8_t RX_ADDR_P2;   // only LSB
	uint8_t RX_ADDR_P3;
	uint8_t RX_ADDR_P4;
	uint8_t RX_ADDR_P5;


	/* ----------------------------------------------------
	 * 0x10 TX_ADDR – 5 bytes
	 * ---------------------------------------------------- */
	uint8_t TX_ADDR[5];
	/* ----------------------------------------------------
	 * 0x11..0x16 RX_PW_P0..P5 – PAYLOAD WIDTH
	 * ---------------------------------------------------- */
	uint8_t RX_PW_P0;
	uint8_t RX_PW_P1;
	uint8_t RX_PW_P2;
	uint8_t RX_PW_P3;
	uint8_t RX_PW_P4;
	uint8_t RX_PW_P5;
	/* ----------------------------------------------------
	 * 0x17 FIFO_STATUS
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t RX_EMPTY :1;
			uint8_t RX_FULL  :1;
			uint8_t RESERVED1:2;
			uint8_t TX_EMPTY :1;
			uint8_t TX_FULL  :1;
			uint8_t TX_REUSE :1;
			uint8_t RESERVED2:1;
		}bits;
		uint8_t value;
	}FIFO_STATUS;
	/* ----------------------------------------------------
	 * 0x1C DYNPD – Dynamic Payload Enable
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t DPL_P0   :1; // bit 0
			uint8_t DPL_P1   :1;
			uint8_t DPL_P2   :1;
			uint8_t DPL_P3   :1;
			uint8_t DPL_P4   :1;
			uint8_t DPL_P5   :1;
			uint8_t RESERVED :2; // bits 6-7
		}bits;
		uint8_t value;
	}DYNPD;
	/* ----------------------------------------------------
	 * 0x1D FEATURE – Enable Advanced Features
	 * ---------------------------------------------------- */
	union{
		struct{
			uint8_t EN_DYN_ACK :1; // bit 0
			uint8_t EN_ACK_PAY :1; // bit 1
			uint8_t EN_DPL     :1; // bit 2
			uint8_t RESERVED   :5; // bits 3–7
		}bits;
		uint8_t value;
	}FEATURE;
}NrfRegisterValues_t;

/**
 * @brief nRF24L01+ SPI Commands (from datasheet)
 */
typedef enum {
    NRF_CMD_R_REGISTER        = 0x00,  // Read register (R_REGISTER + reg_addr)
    NRF_CMD_W_REGISTER        = 0x20,  // Write register (W_REGISTER + reg_addr)

    NRF_CMD_R_RX_PAYLOAD      = 0x61,  // Read RX payload
    NRF_CMD_W_TX_PAYLOAD      = 0xA0,  // Write TX payload

    NRF_CMD_FLUSH_TX          = 0xE1,  // Flush TX FIFO
    NRF_CMD_FLUSH_RX          = 0xE2,  // Flush RX FIFO

    NRF_CMD_REUSE_TX_PL       = 0xE3,  // Reuse last transmitted payload

    NRF_CMD_R_RX_PL_WID       = 0x60,  // Read RX payload width

    NRF_CMD_W_ACK_PAYLOAD     = 0xA8,  // Write ACK payload (A8 | pipe#)

    NRF_CMD_W_TX_PAYLOAD_NOACK = 0xB0, // Write TX payload without ACK

    NRF_CMD_NOP               = 0xFF   // No operation (returns STATUS)

} NRF_Command_t;

typedef enum {
	/* 0x00 */ NRF_Reg_CONFIG          = 0x00,
	/* 0x01 */ NRF_Reg_EN_AA           = 0x01,
	/* 0x02 */ NRF_Reg_EN_RXADDR       = 0x02,
	/* 0x03 */ NRF_Reg_SETUP_AW        = 0x03,
	/* 0x04 */ NRF_Reg_SETUP_RETR      = 0x04,
	/* 0x05 */ NRF_Reg_RF_CH           = 0x05,
	/* 0x06 */ NRF_Reg_RF_SETUP        = 0x06,
	/* 0x07 */ NRF_Reg_STATUS          = 0x07,
	/* 0x08 */ NRF_Reg_OBSERVE_TX      = 0x08,
	/* 0x09 */ NRF_Reg_RPD             = 0x09,

	/* 0x0A-0x0F (address registers) */
	/* 0x0A */ NRF_Reg_RX_ADDR_P0      = 0x0A,
	/* 0x0B */ NRF_Reg_RX_ADDR_P1      = 0x0B,
	/* 0x0C */ NRF_Reg_RX_ADDR_P2      = 0x0C,
	/* 0x0D */ NRF_Reg_RX_ADDR_P3      = 0x0D,
	/* 0x0E */ NRF_Reg_RX_ADDR_P4      = 0x0E,
	/* 0x0F */ NRF_Reg_RX_ADDR_P5      = 0x0F,

	/* 0x10 */ NRF_Reg_TX_ADDR         = 0x10,

	/* 0x11–0x16 payload width registers */
	/* 0x11 */ NRF_Reg_RX_PW_P0        = 0x11,
	/* 0x12 */ NRF_Reg_RX_PW_P1        = 0x12,
	/* 0x13 */ NRF_Reg_RX_PW_P2        = 0x13,
	/* 0x14 */ NRF_Reg_RX_PW_P3        = 0x14,
	/* 0x15 */ NRF_Reg_RX_PW_P4        = 0x15,
	/* 0x16 */ NRF_Reg_RX_PW_P5        = 0x16,

	/* 0x17 */ NRF_Reg_FIFO_STATUS     = 0x17,

	/* 0x1C */ NRF_Reg_DYNPD           = 0x1C,
	/* 0x1D */ NRF_Reg_FEATURE         = 0x1D

} NRF_Register_t;
typedef enum {
    NRF_CmdStateTest,                       // For testing or startup state

    NRF_CmdStateReadStatusWaiting,          // Waiting for STATUS register read

    NRF_CmdStateReadRegisterWaiting,        // Waiting for register read to complete
    NRF_CmdStateWriteRegisterWaiting,       // Waiting for register write to complete

    NRF_CmdStateReadRxPayloadWaiting,       // Waiting for RX payload read to complete
    NRF_CmdStateReadRxPayloadWidthWaiting,  // Waiting for RX payload width read

    NRF_CmdStateWriteTxPayloadWaiting,      // Waiting for TX payload write to complete
    NRF_CmdStateWriteAckPayloadWaiting,     // Waiting for ACK payload write to complete
    NRF_CmdStateWriteTxPayloadNoAckWaiting, // Waiting for TX payload (no ACK) write to complete

    NRF_CmdStateFlushTxWaiting,             // Waiting for TX FIFO flush to complete
    NRF_CmdStateFlushRxWaiting,             // Waiting for RX FIFO flush to complete

    NRF_CmdStateReuseTxPayloadWaiting,      // Waiting for REUSE_TX_PL command to complete

    Nrf_CmdStateIdle                        // Idle, ready for next command
} NRF_CmdStates_t;

typedef enum{
	NRF_ConfigState,
	NRF_TransmittingState,
	NRF_RecievingState,
	NRF_IrqState,
	NRF_ExistenceState,
	NRF_IdleState
}NrfMainStates_t;
typedef enum
{
    /* Disable Enhanced ShockBurst (auto-ack) */
    NRF_Config_DisableEnhancedShockBurstReq,
    NRF_Config_DisableEnhancedShockBurstWaitForReply,

	/* Enable pipes (EN_RXADDR) */
    NRF_Config_EnablePipesReq,
    NRF_Config_EnablePipesWaitForReply,

    /* Disable Auto-ReTransmit (SETUP_RETR=0) */
    NRF_Config_DisableAutoRetransmitReq,
    NRF_Config_DisableAutoRetransmitWaitForReply,

    /* Set Static Payload Size for all enabled pipes */
    NRF_Config_SetPayloadSizeReq,
    NRF_Config_SetPayloadSizeWaitForReply,

    /* Set RF Channel */
    NRF_Config_SetRFChannelReq,
    NRF_Config_SetRFChannelWaitForReply,

    /* Set RF Setup (data rate + power) */
    NRF_Config_SetRFSetupReq,
    NRF_Config_SetRFSetupWaitForReply,

    /* Set Address Width (default: 5 bytes) */
    NRF_Config_SetAddressWidthReq,
    NRF_Config_SetAddressWidthWaitForReply,

	/*set config register*/
    NRF_Config_SetConfigRegReq,
    NRF_Config_SetConfigRegWaitForReply,

    /* Set Pipe0 RX Address */
    NRF_Config_SetRxPipe0AddressReq,
    NRF_Config_SetRxPipe0AddressWaitForReply,

    /* Set Pipe1 RX Address */
    NRF_Config_SetRxPipe1AddressReq,
    NRF_Config_SetRxPipe1AddressWaitForReply,

    /* Set Pipe2 RX Address */
    NRF_Config_SetRxPipe2AddressReq,
    NRF_Config_SetRxPipe2WaitForReply,

    /* Set Pipe3 RX Address */
    NRF_Config_SetRxPipe3AddressReq,
    NRF_Config_SetRxPipe3WaitForReply,

    /* Set Pipe4 RX Address */
    NRF_Config_SetRxPipe4AddressReq,
    NRF_Config_SetRxPipe4WaitForReply,

    /* Set Pipe5 RX Address */
    NRF_Config_SetRxPipe5AddressReq,
    NRF_Config_SetRxPipe5WaitForReply,

	NRF_Config_ClearInterrupt,
	NRF_Config_ClearInterruptWaitForReply,

	NRF_Config_FlushRx,
	NRF_Config_FlushRxWaitForReply,

	NRF_Config_FlushTx,
	NRF_Config_FlushTxWaitForReply,

    NRF_Config_GetConfigReq,
    NRF_Config_GetConfigWaitForReply,

    /* End of configuration */
    NRF_Config_Done

} NrfConfigStates_t;

typedef enum {
	NRF_Irq_ReadStatus,
	NRF_Irq_ReadStatusWaitForReply,
	NRF_Irq_ClearInterruptSrcWaiting,
	NRF_Irq_ChangeFromPtxToPrx,
	NRF_Irq_ChangeFromPtxToPrxWaitForReply,
}NrfIrqStates_t;
typedef enum {
	NRF_Transmitter_SetConfigToTx,
	NRF_Transmitter_SetConfigToTxWaitForReply,
	NRF_Transmitter_SetTxAddress,
	NRF_Transmitter_SetTxAddressWaitForReply,
	NRF_Transmitter_LoadTxPayload,
	NRF_Transmitter_LoadTxPayloadWaitForReply,
	NRF_Transmitter_FinishTransmitting
}NrfTransmitterStates_t;

typedef enum {
	NRF_Receive_ReadPayload,
	NRF_Receive_ReadPayloadWaitForReply,

}NrfReceiveStates_t;

typedef enum {
	NRF_Existence_ReadRFChannelReq,
	NRF_Existence_ReadRFChannelWaitForReply,
	NRF_Existence_PowerDown,
	NRF_Existence_PowerDownWaitForReply,
	NRF_Existence_WaistTimeBeforeReset,

}NrfExistenceStates_t;


typedef struct _NrfControlFlag {
	bool dataRecievedFromSpi;
	bool nrfUpdated;
	uint8_t recievedPayloadLength;
	bool irqInterruptOccurred;
	uint32_t nrfTimeoutTimer;
	uint32_t nrfExistenceTimer;
	bool nrfForceResetFlag;
} NrfControlFlag_t;
/*Variables----------------------------------*/
SPI_HandleTypeDef *NrfSpi;
volatile NrfControlFlag_t NrfControlFlag;
/*variables used in command state machine */
NRF_CmdStates_t CmdState=0;
uint8_t spiRxBuf[35];
uint8_t spiTxBuf[35];
NrfRegisterValues_t RegisterValues ;
NRF_Register_t LastReadReg;
NrfMainStates_t NrfMainStates;
//variables used in config states
NrfConfigStates_t NrfConfigStates;
uint8_t NrfActiveChannel = NRF_CHANNEL;
//-
NrfIrqStates_t NrfIrqStates;
// transmit-
NrfTransmitterStates_t NrfTransmitterStates;
uint8_t NrfTransmitterPipe;
uint8_t NrfTransmitterAddr[5];
//receive-
NrfReceiveStates_t NrfReceiveStates;
NRF_ReceiceCallback_t NRF_ReceiceCallback;
//existence
NrfExistenceStates_t NrfExistenceStates;

/*prototypes -------------------------------*/
void NRF_CommandSubTask(void);
void NRF_NoOperation(void);
void NRF_ReadRegister(NRF_Register_t Register,uint8_t size);
void NRF_ReadRegParser(NRF_Register_t reg);
void NRF_GeneralParser(void);
void NRF_ConfigProcess(void);
void NRF_IrqProcess(void);
void NRF_TransmitterProcess();
void NRF_ReceiverProcess();
void NRF_ExistenceProcess();
/*functions---------------------------------*/
void NRF_ResetSoft(void){
	NrfControlFlag.nrfUpdated = false;
	NrfControlFlag.dataRecievedFromSpi = false;
	memset(&RegisterValues, 0, sizeof(RegisterValues));
	SPI_CSN_HIGH();
	NRF_CE_LOW();
	CmdState = Nrf_CmdStateIdle;
	NrfConfigStates=NRF_Config_DisableEnhancedShockBurstReq;
	NrfMainStates =NRF_ConfigState;
	NrfControlFlag.nrfTimeoutTimer=HAL_GetTick();
	NrfControlFlag.nrfExistenceTimer=HAL_GetTick();
	HAL_Delay(5);
}
void NRF_Init(SPI_HandleTypeDef * hSpi,NRF_ReceiceCallback_t recCallback ,uint8_t channel){
	NrfSpi =hSpi;
	NRF_ReceiceCallback =recCallback;
	if(channel<126){
		NrfActiveChannel=channel;
	}
	NRF_ResetSoft();
}
void NRF_SetChannel(uint8_t ch){
	NrfActiveChannel=ch;
	NRF_ResetSoft();
}
void NRF_TransmitReceive_DMA( uint8_t * pTxData, uint8_t * pRxData, uint16_t Size){
	SPI_CSN_LOW();
	__NOP();
	    HAL_SPI_TransmitReceive_DMA(NrfSpi, pTxData, pRxData, Size);



}
void NRF_Process(){
	NRF_CommandSubTask();
	switch(NrfMainStates){
	case NRF_ConfigState:
		NrfControlFlag.nrfTimeoutTimer=HAL_GetTick();
		NRF_ConfigProcess(); // at the end of this process go to receive state
		break;
	case NRF_TransmittingState:
		NRF_TransmitterProcess();
		break;
	case NRF_RecievingState:
		NRF_ReceiverProcess();
		break;
	case NRF_IrqState:
		 NRF_IrqProcess();
		break;
	case NRF_ExistenceState:
		NRF_ExistenceProcess();
		break;
	case NRF_IdleState:
		HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin,0);
		NrfControlFlag.nrfTimeoutTimer=HAL_GetTick();
	    if(NrfControlFlag.irqInterruptOccurred && CmdState == Nrf_CmdStateIdle){
	        NrfMainStates = NRF_IrqState;
	        NrfIrqStates = NRF_Irq_ReadStatus;
	        NrfControlFlag.irqInterruptOccurred = false;
	        break;
	    }
	    if(CmdState == Nrf_CmdStateIdle &&
	       !NrfControlFlag.dataRecievedFromSpi &&
	       (HAL_GetTick() - NrfControlFlag.nrfExistenceTimer) > 5500)
	    {
	        NrfControlFlag.nrfExistenceTimer = HAL_GetTick();
	        NrfMainStates = NRF_ExistenceState;
	        NrfExistenceStates = NRF_Existence_ReadRFChannelReq;
	    }
	    break;
	}
	if((HAL_GetTick()- NrfControlFlag.nrfTimeoutTimer)>5000){
		NRF_ResetSoft();
	}

}

void NRF_FinishSpiTransaction(){
	SPI_CSN_HIGH();

	NrfControlFlag.dataRecievedFromSpi=true;

}

//*********************   spi  communication  to module   *********************
void NRF_CommandSubTask(void){
	switch(CmdState){
	case NRF_CmdStateTest:
		//NRF_NoOperation();
		//NRF_ReadRegister(NRF_Reg_CONFIG,1);
		break;
	case NRF_CmdStateReadStatusWaiting:
		if(NrfControlFlag.dataRecievedFromSpi){
			NrfControlFlag.dataRecievedFromSpi=false;
			NrfControlFlag.nrfUpdated=true;
			CmdState = Nrf_CmdStateIdle;
		}
	break;
	case NRF_CmdStateReadRegisterWaiting:
		if(NrfControlFlag.dataRecievedFromSpi){
			NrfControlFlag.dataRecievedFromSpi=false;
			CmdState = Nrf_CmdStateIdle;
			NRF_ReadRegParser(LastReadReg);
			NrfControlFlag.nrfUpdated=true;
		}
		break;
	case NRF_CmdStateReadRxPayloadWidthWaiting:
		if(NrfControlFlag.dataRecievedFromSpi){
			NrfControlFlag.dataRecievedFromSpi=false;
			RegisterValues.STATUS.value = spiRxBuf[0];
			NrfControlFlag.recievedPayloadLength=spiRxBuf[1];
			NrfControlFlag.nrfUpdated=true;
			CmdState = Nrf_CmdStateIdle;
		}
		break;

	case Nrf_CmdStateIdle:
		//NRF_ReadRegister(NRF_Reg_CONFIG,1);
		//CmdState=NRF_CmdStateTest;
		break;
	default:
		NRF_GeneralParser();
		break;


	}
}
/*Parsers ---------------------------------------------*/
void NRF_GeneralParser(void){
	if(NrfControlFlag.dataRecievedFromSpi){
		NrfControlFlag.dataRecievedFromSpi=false;
		RegisterValues.STATUS.value = spiRxBuf[0];
		NrfControlFlag.nrfUpdated=true;
		CmdState = Nrf_CmdStateIdle;
	}
}
void NRF_ReadRegParser(NRF_Register_t reg)
{
	// (1) Always read STATUS from first byte
	RegisterValues.STATUS.value = spiRxBuf[0];

    switch (reg)
    {
    /* 1-byte registers */
    case NRF_Reg_CONFIG:
        RegisterValues.CONFIG.value = spiRxBuf[1];
        break;

    case NRF_Reg_EN_AA:
        RegisterValues.EN_AA.value = spiRxBuf[1];
        break;

    case NRF_Reg_EN_RXADDR:
        RegisterValues.EN_RXADDR.value = spiRxBuf[1];
        break;

    case NRF_Reg_SETUP_AW:
        RegisterValues.SETUP_AW.value = spiRxBuf[1];
        break;

    case NRF_Reg_SETUP_RETR:
        RegisterValues.SETUP_RETR.value = spiRxBuf[1];
        break;

    case NRF_Reg_RF_CH:
        RegisterValues.RF_CH.value = spiRxBuf[1];
        break;

    case NRF_Reg_RF_SETUP:
        RegisterValues.RF_SETUP.value = spiRxBuf[1];
        break;



    case NRF_Reg_OBSERVE_TX:
        RegisterValues.OBSERVE_TX.value = spiRxBuf[1];
        break;

    case NRF_Reg_RPD:
        RegisterValues.RPD.value = spiRxBuf[1];
        break;

    /* Multi-byte registers */

    case NRF_Reg_RX_ADDR_P0:
        memcpy(RegisterValues.RX_ADDR_P0, &spiRxBuf[1], 5);
        break;

    case NRF_Reg_RX_ADDR_P1:
        memcpy(RegisterValues.RX_ADDR_P1, &spiRxBuf[1], 5);
        break;

    case NRF_Reg_RX_ADDR_P2:
        RegisterValues.RX_ADDR_P2 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_ADDR_P3:
        RegisterValues.RX_ADDR_P3 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_ADDR_P4:
        RegisterValues.RX_ADDR_P4 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_ADDR_P5:
        RegisterValues.RX_ADDR_P5 = spiRxBuf[1];
        break;

    case NRF_Reg_TX_ADDR:
        memcpy(RegisterValues.TX_ADDR, &spiRxBuf[1], 5);
        break;

    case NRF_Reg_RX_PW_P0:
        RegisterValues.RX_PW_P0 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_PW_P1:
        RegisterValues.RX_PW_P1 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_PW_P2:
        RegisterValues.RX_PW_P2 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_PW_P3:
        RegisterValues.RX_PW_P3 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_PW_P4:
        RegisterValues.RX_PW_P4 = spiRxBuf[1];
        break;

    case NRF_Reg_RX_PW_P5:
        RegisterValues.RX_PW_P5 = spiRxBuf[1];
        break;

    case NRF_Reg_FIFO_STATUS:
        RegisterValues.FIFO_STATUS.value = spiRxBuf[1];
        break;

    case NRF_Reg_DYNPD:
        RegisterValues.DYNPD.value = spiRxBuf[1];
        break;

    case NRF_Reg_FEATURE:
        RegisterValues.FEATURE.value = spiRxBuf[1];
        break;

    default:
        // Unknown register (should not happen)
        break;
    }
}

/*API-------------------------------------------------*/
/* =========================
 *  READ status
 * ========================= */
void NRF_NoOperation(void){
	if (CmdState != Nrf_CmdStateIdle) return;
	static uint8_t cmd=NRF_CMD_NOP;
	CmdState= NRF_CmdStateReadStatusWaiting;
	NRF_TransmitReceive_DMA(&cmd,&RegisterValues.STATUS.value,1);

}
/* =========================
 *  READ RX PAYLOAD
 * ========================= */
void NRF_ReadRegister(NRF_Register_t reg, uint8_t size)
{
	if (CmdState != Nrf_CmdStateIdle) return;

    spiTxBuf[0] = (uint8_t)NRF_CMD_R_REGISTER | (uint8_t)reg;
    LastReadReg=reg;

    for(int i=1; i<=size; i++)
        spiTxBuf[i] = 0xFF;   // Dummy

    CmdState = NRF_CmdStateReadRegisterWaiting;

    NRF_TransmitReceive_DMA(spiTxBuf, spiRxBuf, size + 1);

}

/* =========================
 *  WRITE REGISTER
 * ========================= */
void NRF_WriteRegister(NRF_Register_t reg, uint8_t *data, uint8_t size)
{
    if (CmdState != Nrf_CmdStateIdle) return;

    spiTxBuf[0] = NRF_CMD_W_REGISTER | (uint8_t)reg;
    for (uint8_t i = 0; i < size; i++)
        spiTxBuf[i + 1] = data[i];

    CmdState = NRF_CmdStateWriteRegisterWaiting;

    NRF_TransmitReceive_DMA(spiTxBuf, spiRxBuf, size + 1);
}
/* =========================
 *  READ RX PAYLOAD
 * ========================= */
void NRF_ReadRxPayload(uint8_t *buffer, uint8_t size)
{
    if (CmdState != Nrf_CmdStateIdle) return;

    spiTxBuf[0] = NRF_CMD_R_RX_PAYLOAD;
    for (uint8_t i = 1; i <= size; i++)
        spiTxBuf[i] = 0xFF;

    CmdState = NRF_CmdStateReadRxPayloadWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, buffer, size + 1);
}

/* =========================
 *  WRITE TX PAYLOAD
 * ========================= */
void NRF_WriteTxPayload(uint8_t *data, uint8_t size)
{
    if (CmdState != Nrf_CmdStateIdle) return;

    spiTxBuf[0] = NRF_CMD_W_TX_PAYLOAD;
    for (uint8_t i = 0; i < size; i++)
        spiTxBuf[i + 1] = data[i];

    CmdState = NRF_CmdStateWriteTxPayloadWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, spiRxBuf, size + 1);
}

/* =========================
 *  FLUSH TX FIFO
 * ========================= */
void NRF_FlushTx(void)
{
    if (CmdState != Nrf_CmdStateIdle) return;

    spiTxBuf[0] =NRF_CMD_FLUSH_TX;
    CmdState = NRF_CmdStateFlushTxWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, &RegisterValues.STATUS.value, 1);
}

/* =========================
 *  FLUSH RX FIFO
 * ========================= */
void NRF_FlushRx(void)
{
    if (CmdState != Nrf_CmdStateIdle) return;

    spiTxBuf[0] = NRF_CMD_FLUSH_RX;
    CmdState = NRF_CmdStateFlushRxWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, &RegisterValues.STATUS.value, 1);
}

/* =========================
 *  REUSE TX PAYLOAD
 * ========================= */
void NRF_ReuseTxPayload(void)
{
    if (CmdState != Nrf_CmdStateIdle) return;

    spiTxBuf[0] = NRF_CMD_REUSE_TX_PL;
    CmdState = NRF_CmdStateReuseTxPayloadWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, &RegisterValues.STATUS.value, 1);
}

/* =========================
 *  READ RX PAYLOAD WIDTH (0x60)
 * ========================= */
void NRF_ReadRxPayloadWidth(void)
{
    if (CmdState != Nrf_CmdStateIdle)
        return;


    spiTxBuf[0] = NRF_CMD_R_RX_PL_WID;
    spiTxBuf[1] = 0xFF;   // Dummy byte for response

    CmdState = NRF_CmdStateReadRxPayloadWidthWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, spiRxBuf, 2);

}

/* =========================
 *  WRITE ACK PAYLOAD (0xA8 | pipe#)
 * ========================= */
void NRF_WriteAckPayload(uint8_t pipe, uint8_t *data, uint8_t size)
{
    if (CmdState != Nrf_CmdStateIdle)
        return;

    if (pipe > 5 || size > 32)
        return;

    spiTxBuf[0] = NRF_CMD_W_ACK_PAYLOAD | (pipe & 0x07);
    for (uint8_t i = 0; i < size; i++)
        spiTxBuf[i + 1] = data[i];

    CmdState = NRF_CmdStateWriteAckPayloadWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, spiRxBuf, size + 1);
}
/* =========================
 *  WRITE TX PAYLOAD WITHOUT ACK (0xB0)
 * ========================= */
void NRF_WriteTxPayloadWithNoAck(uint8_t *data, uint8_t size)
{
    if (CmdState != Nrf_CmdStateIdle)
        return;

    if (size > 32)
        return;

    spiTxBuf[0] = NRF_CMD_W_TX_PAYLOAD_NOACK;
    for (uint8_t i = 0; i < size; i++)
        spiTxBuf[i + 1] = data[i];

    CmdState = NRF_CmdStateWriteTxPayloadNoAckWaiting;
    NRF_TransmitReceive_DMA(spiTxBuf, spiRxBuf, size + 1);
}
//*********************   initial config logic   *********************
void NRF_ConfigProcess(void)
{
    static uint8_t data[5];
    static uint8_t pipe=0;
    switch(NrfConfigStates)
    {
    /* -------------------- Disable EN_AA -------------------- */
    case NRF_Config_DisableEnhancedShockBurstReq:
        data[0] = 0x00;
        NRF_WriteRegister(NRF_Reg_EN_AA, data, 1);
        NrfConfigStates = NRF_Config_DisableEnhancedShockBurstWaitForReply;
        break;

    case NRF_Config_DisableEnhancedShockBurstWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_EnablePipesReq;
        }
        break;

        /* -------------------- Disable EN_RXADDR -------------------- */
        case NRF_Config_EnablePipesReq:
            data[0] = 0xFF;
            NRF_WriteRegister(NRF_Reg_EN_RXADDR, data, 1);
            NrfConfigStates = NRF_Config_EnablePipesWaitForReply;
            break;

        case NRF_Config_EnablePipesWaitForReply:
            if (NrfControlFlag.nrfUpdated)
            {
                NrfControlFlag.nrfUpdated = false;
                NrfConfigStates = NRF_Config_DisableAutoRetransmitReq;
            }
            break;

    /* -------------------- Disable Auto Retransmit -------------------- */
    case NRF_Config_DisableAutoRetransmitReq:
        data[0] = 0x00;
        NRF_WriteRegister(NRF_Reg_SETUP_RETR, data, 1);
        NrfConfigStates = NRF_Config_DisableAutoRetransmitWaitForReply;
        break;

    case NRF_Config_DisableAutoRetransmitWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_SetPayloadSizeReq;
        }
        break;

    /* -------------------- Payload Size (32 bytes) -------------------- */
    case NRF_Config_SetPayloadSizeReq:
        data[0] = NRF_PAYLOAD_SIZE;
            NRF_WriteRegister(NRF_Reg_RX_PW_P0 + pipe, data, 1);
            NrfConfigStates = NRF_Config_SetPayloadSizeWaitForReply;
        break;

    case NRF_Config_SetPayloadSizeWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            pipe+=1;
            if(pipe>5){
            	pipe=0;
            	NrfConfigStates = NRF_Config_SetRFChannelReq;
            } else{
            	NrfConfigStates = NRF_Config_SetPayloadSizeReq;
            }

        }
        break;

    /* -------------------- RF Channel -------------------- */
    case NRF_Config_SetRFChannelReq:
        data[0] = NrfActiveChannel;
        NRF_WriteRegister(NRF_Reg_RF_CH, data, 1);
        NrfConfigStates = NRF_Config_SetRFChannelWaitForReply;
        break;

    case NRF_Config_SetRFChannelWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_SetRFSetupReq;
        }
        break;

    /* -------------------- RF Setup (Power + DataRate) -------------------- */
    case NRF_Config_SetRFSetupReq:
        data[0] = NRF_RF_SETUP_VALUE;
        NRF_WriteRegister(NRF_Reg_RF_SETUP, data, 1);
        NrfConfigStates = NRF_Config_SetRFSetupWaitForReply;
        break;

    case NRF_Config_SetRFSetupWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_SetAddressWidthReq;
        }
        break;

    /* -------------------- Address Width -------------------- */
    case NRF_Config_SetAddressWidthReq:
        data[0] = NRF_ADDRESS_WIDTH; // 5 bytes
        NRF_WriteRegister(NRF_Reg_SETUP_AW, data, 1);
        NrfConfigStates = NRF_Config_SetAddressWidthWaitForReply;
        break;

    case NRF_Config_SetAddressWidthWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_SetConfigRegReq;
        }
        break;
    /* --------------------- Config Register -----------------------*/
    case NRF_Config_SetConfigRegReq:
    	data[0]= CONF_REG;
    	NRF_WriteRegister(NRF_Reg_CONFIG, data, 1);
    	NrfConfigStates = NRF_Config_SetConfigRegWaitForReply;

    	break;
    case NRF_Config_SetConfigRegWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_SetRxPipe0AddressReq;
        }
    	break;
    /* -------------------- Pipe0 Address -------------------- */
    case NRF_Config_SetRxPipe0AddressReq:

		memcpy(data, PIPE0_ADDR, 5);
        NRF_WriteRegister(NRF_Reg_RX_ADDR_P0, data, 5);
        NrfConfigStates = NRF_Config_SetRxPipe0AddressWaitForReply;
        break;

    case NRF_Config_SetRxPipe0AddressWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_SetRxPipe1AddressReq;
        }
        break;

    /* -------------------- Pipe1 Address -------------------- */
    case NRF_Config_SetRxPipe1AddressReq:
    	memcpy(data, PIPE1_ADDR, 5);
        NRF_WriteRegister(NRF_Reg_RX_ADDR_P1, data, 5);
        NrfConfigStates = NRF_Config_SetRxPipe1AddressWaitForReply;
        break;

    case NRF_Config_SetRxPipe1AddressWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfConfigStates = NRF_Config_SetRxPipe2AddressReq;
        }
        break;

    /* -------------------- Pipe2–Pipe5 (only LSB byte) -------------------- */
    case NRF_Config_SetRxPipe2AddressReq:
    	data[0]=PIPE2_ADDR_LSB;
        NRF_WriteRegister(NRF_Reg_RX_ADDR_P2, &data[0], 1);
        NrfConfigStates = NRF_Config_SetRxPipe2WaitForReply;
        break;

    case NRF_Config_SetRxPipe2WaitForReply:
        if(NrfControlFlag.nrfUpdated) {
            NrfControlFlag.nrfUpdated=false;
            NrfConfigStates = NRF_Config_SetRxPipe3AddressReq;
        }
        break;

    case NRF_Config_SetRxPipe3AddressReq:
    	data[0]=PIPE3_ADDR_LSB;
        NRF_WriteRegister(NRF_Reg_RX_ADDR_P3, &data[0], 1);
        NrfConfigStates = NRF_Config_SetRxPipe3WaitForReply;
        break;

    case NRF_Config_SetRxPipe3WaitForReply:
        if(NrfControlFlag.nrfUpdated) {
            NrfControlFlag.nrfUpdated=false;
            NrfConfigStates = NRF_Config_SetRxPipe4AddressReq;
        }
        break;

    case NRF_Config_SetRxPipe4AddressReq:
    	data[0]=PIPE4_ADDR_LSB;
        NRF_WriteRegister(NRF_Reg_RX_ADDR_P4, &data[0], 1);
        NrfConfigStates = NRF_Config_SetRxPipe4WaitForReply;
        break;

    case NRF_Config_SetRxPipe4WaitForReply:
        if(NrfControlFlag.nrfUpdated) {
            NrfControlFlag.nrfUpdated=false;
            NrfConfigStates = NRF_Config_SetRxPipe5AddressReq;
        }
        break;

    case NRF_Config_SetRxPipe5AddressReq:
    	data[0]=PIPE5_ADDR_LSB;
        NRF_WriteRegister(NRF_Reg_RX_ADDR_P5, &data[0], 1);
        NrfConfigStates = NRF_Config_SetRxPipe5WaitForReply;
        break;

    case NRF_Config_SetRxPipe5WaitForReply:
        if(NrfControlFlag.nrfUpdated) {
            NrfControlFlag.nrfUpdated=false;
            NrfConfigStates = NRF_Config_ClearInterrupt;
        }
        break;

    case NRF_Config_ClearInterrupt:
        data[0]=0;
        data[0]=(RegisterValues.STATUS.bits.RX_DR << 6) |
                (RegisterValues.STATUS.bits.TX_DS << 5) |
				(RegisterValues.STATUS.bits.MAX_RT<< 4);
        NRF_WriteRegister(NRF_Reg_STATUS, &data[0], 1);
        NrfConfigStates = NRF_Config_ClearInterruptWaitForReply;
    	break;
    case NRF_Config_ClearInterruptWaitForReply:
        if(NrfControlFlag.nrfUpdated) {
        NrfControlFlag.nrfUpdated=false;
    	NrfConfigStates = NRF_Config_FlushRx;
        }
    	break;
    case NRF_Config_FlushRx:
    	NRF_FlushRx();
    	NrfConfigStates = NRF_Config_FlushRxWaitForReply;
    	break;
    case NRF_Config_FlushRxWaitForReply:
        if(NrfControlFlag.nrfUpdated) {
        NrfControlFlag.nrfUpdated=false;
    	NrfConfigStates = NRF_Config_GetConfigReq;
        }
    	break;

    case NRF_Config_FlushTx:
    	NRF_FlushTx();
    	NrfConfigStates = NRF_Config_FlushTxWaitForReply;
    	break;
    case NRF_Config_FlushTxWaitForReply:
        if(NrfControlFlag.nrfUpdated) {
        NrfControlFlag.nrfUpdated=false;
    	NrfConfigStates = NRF_Config_GetConfigReq;
        }
    	break;


    case NRF_Config_GetConfigReq:
    	NRF_ReadRegister( NRF_Reg_CONFIG, 1);// after reading the RegisterValues updated automatically
    	NrfConfigStates=NRF_Config_GetConfigWaitForReply;
    	break;
    case  NRF_Config_GetConfigWaitForReply :
        if(NrfControlFlag.nrfUpdated) {
            NrfControlFlag.nrfUpdated=false;
            if(RegisterValues.CONFIG.value==CONF_REG){
            	NrfConfigStates = NRF_Config_Done;
            	NRF_CE_HIGH(); // enter receive mode
            } else{
            	NrfConfigStates =NRF_Config_DisableEnhancedShockBurstReq; // module not exist, repeat again.

            }

        }
    	break;
    /* -------------------- DONE -------------------- */
    case NRF_Config_Done:
    	NrfMainStates = NRF_IdleState;
        break;

    }
}
//IRQ handler --------------
void NRF_IrqInterruptRoutin(void){
	// according to config only rx_dr activate this interrupt
	NrfControlFlag.irqInterruptOccurred = true;
}
void NRF_IrqProcess(void){
	static bool isTxInterrupt;
	static uint8_t clearMask;
	switch(NrfIrqStates){
	case NRF_Irq_ReadStatus:
		NRF_NoOperation();
		NRF_CE_LOW(); // for write operation in next states
		NrfIrqStates = NRF_Irq_ReadStatusWaitForReply;
		break;
	case NRF_Irq_ReadStatusWaitForReply:
        if(NrfControlFlag.nrfUpdated) {
            NrfControlFlag.nrfUpdated=false;
            clearMask=0;
            clearMask=(RegisterValues.STATUS.bits.RX_DR << 6) |
                    (RegisterValues.STATUS.bits.TX_DS << 5);
            if(RegisterValues.STATUS.bits.TX_DS){
            	isTxInterrupt=true;
            	RegisterValues.STATUS.bits.TX_DS=0;
            }else if (RegisterValues.STATUS.bits.RX_DR){
            	isTxInterrupt=false;
            	RegisterValues.STATUS.bits.RX_DR=0;
            }
            NRF_WriteRegister(NRF_Reg_STATUS, &clearMask, 1);
            NrfIrqStates = NRF_Irq_ClearInterruptSrcWaiting;
        }
		break;

	case NRF_Irq_ClearInterruptSrcWaiting:
        if(NrfControlFlag.nrfUpdated) {
            NrfControlFlag.nrfUpdated=false;
            if(isTxInterrupt){
            	//exit from tx mode // todo : set prim_rx =1

            	NrfIrqStates =NRF_Irq_ChangeFromPtxToPrx;

            }else{
            	// rx
            	NrfMainStates=NRF_RecievingState; //todo: set ce high when finish receive process
            	NrfReceiveStates = NRF_Receive_ReadPayload;
            }

        }
		break;
	case NRF_Irq_ChangeFromPtxToPrx:
		 RegisterValues.CONFIG.bits.PRIM_RX = 1;
		 NRF_WriteRegister(NRF_Reg_CONFIG, &RegisterValues.CONFIG.value, 1);
		 NrfIrqStates =NRF_Irq_ChangeFromPtxToPrxWaitForReply;
		break;
	case NRF_Irq_ChangeFromPtxToPrxWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfMainStates=NRF_IdleState;
            NRF_CE_HIGH();
        }
		break;
	}
}
// Transmit ---------------------------------

static void NRF_SelectTxPipeAddress(void)
{
    memcpy(NrfTransmitterAddr, PIPE1_ADDR, 5);
    switch (NrfTransmitterPipe)
    {
    case 0:
        memcpy(NrfTransmitterAddr, PIPE0_ADDR, 5);
        break;

    case 1:
        break;

    case 2:
    	NrfTransmitterAddr[0]=PIPE2_ADDR_LSB;
        break;

    case 3:
    	NrfTransmitterAddr[0]=PIPE3_ADDR_LSB;
        break;

    case 4:
    	NrfTransmitterAddr[0]=PIPE4_ADDR_LSB;
        break;

    case 5:
    	NrfTransmitterAddr[0]=PIPE5_ADDR_LSB;
        break;
    }
}
bool NRF_CheckTransmittingAvailability(void){
	if( NrfMainStates == NRF_IdleState &&
			 CmdState == Nrf_CmdStateIdle	&&
			 RegisterValues.STATUS.bits.TX_DS==0 &&
			 RegisterValues.STATUS.bits.RX_DR==0
	){
		return true;
	}
	return false;
}
uint8_t transmitterHolder[32];
void NRF_TransmitPacket(uint8_t *packet , uint8_t length,uint8_t pipe){
	NRF_CE_LOW();
	memset(transmitterHolder,0,sizeof(spiTxBuf));
	memcpy(transmitterHolder,packet,length);
	NrfTransmitterPipe=pipe;
	NRF_SelectTxPipeAddress();
	NrfTransmitterStates=NRF_Transmitter_SetConfigToTx;
	NrfMainStates=NRF_TransmittingState;

}

void NRF_TransmitterProcess()
{
    static uint32_t ceHighPulseTime;
    switch (NrfTransmitterStates)
    {
    /* ---------------------------------------------
     * 1) Set CONFIG Register → PRIM_RX = 0 (TX Mode)
     * --------------------------------------------- */
    case NRF_Transmitter_SetConfigToTx:
        RegisterValues.CONFIG.bits.PRIM_RX = 0;   // TX mode
        NRF_WriteRegister(NRF_Reg_CONFIG, &RegisterValues.CONFIG.value, 1);
        NrfTransmitterStates = NRF_Transmitter_SetConfigToTxWaitForReply;
        break;

    case NRF_Transmitter_SetConfigToTxWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfTransmitterStates = NRF_Transmitter_SetTxAddress;
        }
        break;

    /* ---------------------------------------------
     * 2) Set TX_ADDR
     * --------------------------------------------- */
    case NRF_Transmitter_SetTxAddress:
        NRF_WriteRegister(NRF_Reg_TX_ADDR,NrfTransmitterAddr, 5);
        NrfTransmitterStates = NRF_Transmitter_SetTxAddressWaitForReply;
        break;

    case NRF_Transmitter_SetTxAddressWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NrfTransmitterStates = NRF_Transmitter_LoadTxPayload;
        }
        break;

    /* ---------------------------------------------
     * 3) Load TX Payload (W_TX_PAYLOAD command)
     * --------------------------------------------- */
    case NRF_Transmitter_LoadTxPayload:
    	NRF_WriteTxPayload(transmitterHolder,NRF_PAYLOAD_SIZE);  // length fixed = 32
        NrfTransmitterStates = NRF_Transmitter_LoadTxPayloadWaitForReply;
        break;

    case NRF_Transmitter_LoadTxPayloadWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;

            /* Prepare for CE pulse */
            ceHighPulseTime = HAL_GetTick();
            NRF_CE_HIGH();

            NrfTransmitterStates = NRF_Transmitter_FinishTransmitting;
        }
        break;

    /* ---------------------------------------------
     * 4) Wait CE = High for >10µs then fall to idle
     * --------------------------------------------- */
    case NRF_Transmitter_FinishTransmitting:
        if ((HAL_GetTick() - ceHighPulseTime) >= 1)   // 1ms >> 10us
        {
            NRF_CE_LOW();
            // IRQ will complete the process (TX_DS interrupt)
            NrfMainStates = NRF_IdleState;
        }
        break;
    }
}
/*Receiver process -------------------------------------------*/

void NRF_ReceiverProcess(){
	// ce already low from IRQ process
	switch(NrfReceiveStates) {
	case NRF_Receive_ReadPayload:
		NRF_ReadRxPayload(spiRxBuf,32);
		NrfReceiveStates=NRF_Receive_ReadPayloadWaitForReply;
		break;
	case NRF_Receive_ReadPayloadWaitForReply:
        if (NrfControlFlag.nrfUpdated)
        {
            NrfControlFlag.nrfUpdated = false;
            NRF_ReceiceCallback(&spiRxBuf[1],RegisterValues.STATUS.bits.RX_P_NO);
            NrfMainStates=NRF_IdleState;
            NRF_CE_HIGH();
        }
		break;
	}
}
/*existence process ----------------------------------------*/
void NRF_ExistenceProcess(){
	static uint32_t timer;
    switch(NrfExistenceStates){

    case NRF_Existence_ReadRFChannelReq:

        NRF_ReadRegister(NRF_Reg_RF_CH, 1);
        NrfExistenceStates = NRF_Existence_ReadRFChannelWaitForReply;
        break;

    case NRF_Existence_ReadRFChannelWaitForReply:
        if(NrfControlFlag.nrfUpdated){
            NrfControlFlag.nrfUpdated = false;

            if(RegisterValues.RF_CH.value == NrfActiveChannel &&
            		RegisterValues.STATUS.bits.MAX_RT==0 &&
					RegisterValues.STATUS.bits.TX_FULL==0 &&
					!NrfControlFlag.nrfForceResetFlag

            ){
                NrfMainStates = NRF_IdleState;
            } else {
            	NrfExistenceStates = NRF_Existence_PowerDown;
            	NrfControlFlag.nrfForceResetFlag=false;

            }
        }
        break;
    case NRF_Existence_PowerDown:
    	RegisterValues.CONFIG.bits.PWR_UP=0;
    	NRF_WriteRegister(NRF_Reg_CONFIG, &RegisterValues.CONFIG.value, 1);
    	NrfExistenceStates = NRF_Existence_PowerDownWaitForReply;
    	break;
    case NRF_Existence_PowerDownWaitForReply:
    	if(NrfControlFlag.nrfUpdated){
    		NrfControlFlag.nrfUpdated=false;
    		timer=HAL_GetTick();
    		NrfExistenceStates = NRF_Existence_WaistTimeBeforeReset;
    	}
    	break;
    case NRF_Existence_WaistTimeBeforeReset:
    	if(HAL_GetTick()-timer>1000){
    		NRF_ResetSoft();
    	}
    	break;
    }
}

 void NRF_ResetModule(){
	 NrfControlFlag.nrfForceResetFlag=true;
 }
