/*
 * NRF24L01.c
 *
 *  Created on: Feb 11, 2023
 *      Author: Metisai
 */
#include "stm32f4xx_hal.h"
#include "NRF24L01.h"
extern SPI_HandleTypeDef hspi1;

#define NRF24_SPI		&hspi1
#define NRF24_CE_PORT   GPIOC
#define NRF24_CE_PIN    GPIO_PIN_7

#define NRF24_CSN_PORT   GPIOC
#define NRF24_CSN_PIN    GPIO_PIN_8
void EnableCS (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, RESET);
}
void DisableCS(void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN,SET);
}

void EnableCE (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, RESET);
}
void DisableCE(void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN,SET);
}
void NRF_SendToCmd(uint8_t cmd )
{
	EnableCS();
	HAL_SPI_Transmit(NRF24_SPI,&cmd, 1, 1000);
	DisableCS();

}
void NRF_WriteReg(uint8_t reg , uint8_t data)
{
	EnableCS();
	uint8_t buffer[2];
	buffer[0] = (reg|00100000u);
	buffer[1] = data;

	HAL_SPI_Transmit(NRF24_SPI, buffer, 2, 1000);

	DisableCS();
}
void NRF_WriteRegMulti(uint8_t reg , uint8_t *data, uint8_t size)
{
	EnableCS();
	uint8_t buffer[1];
	buffer[0] = (reg|00100000u);
	HAL_SPI_Transmit(NRF24_SPI, buffer, 1, 1000);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

	DisableCS();
}
void NRF_ReadReg(uint8_t reg, uint8_t *data)
{
	EnableCS();
	uint8_t buffer[2];
	buffer[0] = (reg|00000000u);
	HAL_SPI_Transmit(NRF24_SPI, buffer, 1, 1000);
	HAL_SPI_Receive(NRF24_SPI, data, 1, 1000);
	DisableCS();
}
void NRF_ReadRegMulti(uint8_t reg, uint8_t *data, uint8_t size)
{
	EnableCS();
	uint8_t buffer[2];
	buffer[0] = (reg|00000000u);
	HAL_SPI_Transmit(NRF24_SPI, buffer, 1, 1000);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);
	DisableCS();
}
void NRF_Reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		NRF_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		NRF_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	NRF_WriteReg(CONFIG, 0x08);
	NRF_WriteReg(EN_AA, 0x3F);
	NRF_WriteReg(EN_RXADDR, 0x03);
	NRF_WriteReg(SETUP_AW, 0x03);
	NRF_WriteReg(SETUP_RETR, 0x03);
	NRF_WriteReg(RF_CH, 0x02);
	NRF_WriteReg(RF_SETUP, 0x0E);
	NRF_WriteReg(STATUS, 0x00);
	NRF_WriteReg(OBSERVE_TX, 0x00);
	NRF_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	NRF_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	NRF_WriteReg(RX_ADDR_P2, 0xC3);
	NRF_WriteReg(RX_ADDR_P3, 0xC4);
	NRF_WriteReg(RX_ADDR_P4, 0xC5);
	NRF_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	NRF_WriteReg(RX_PW_P0, 0);
	NRF_WriteReg(RX_PW_P1, 0);
	NRF_WriteReg(RX_PW_P2, 0);
	NRF_WriteReg(RX_PW_P3, 0);
	NRF_WriteReg(RX_PW_P4, 0);
	NRF_WriteReg(RX_PW_P5, 0);
	NRF_WriteReg(FIFO_STATUS, 0x11);
	NRF_WriteReg(DYNPD, 0);
	NRF_WriteReg(FEATURE, 0);
	}
}

void NRF_Init(void)
{
	DisableCE();
	NRF_Reset(0);
	NRF_WriteReg(CONFIG, 0); // s?????? c??????u h????nh sau
	NRF_WriteReg(EN_AA, 0); //kh????ng c???? ack
	NRF_WriteReg(EN_RXADDR, 0); // kh????ng enable pipe t??????i ?????????y
	NRF_WriteReg(SETUP_AW, 0x03);// c??????u h????nh ???????????? d????i ????????????a ch??????? l???? 5 byte
	NRF_WriteReg(SETUP_RETR, 0); // kh????ng c??????n truy??????n l??????i d?????? li???????u
	/* RF_chanel l???? t??????n s??????? giao ti??????p gi??????a hai thi??????t b??????? [Frequency (in MHz) = 2400 + RF_CH]*/
	NRF_WriteReg(RF_CH, 0);// setup trong TX or RX
	NRF_WriteReg(RF_SETUP, 0x0E);// t???????c ???????????? truy??????n 2Mbps , v???? c????ng su??????t ???????????? l??????????i t???????i ?????a 0dm
	EnableCE();
}
void NRF_TxMode(uint8_t *address,uint8_t chanel)
{
	DisableCE();

	NRF_WriteReg(RF_CH, chanel);
	NRF_WriteRegMulti(TX_ADDR,address,5);

	uint8_t temp = 0;
	NRF_ReadReg(CONFIG, &temp);
//	temp |= (1<<1);
//	temp &= ~(1<<0);
	temp &= (0xF2);
	NRF_WriteReg(CONFIG, temp);
	EnableCE();
}
NRF_Status NFR_Transmit(uint8_t *data)
{

	uint8_t cmdtosend;
	EnableCS();
	cmdtosend = W_TX_PAYLOAD;
	NRF_SendToCmd(cmdtosend);
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);
	DisableCS();
	HAL_Delay(1);

	// ki??????m tra thanh tr??????ng th????i fifo
	uint8_t fifo_status;
	NRF_ReadReg(FIFO_STATUS, &fifo_status);
	if(fifo_status&(1<<4) && fifo_status&(1<<2))
	{
		cmdtosend = FLUSH_TX;
		NRF_SendToCmd(cmdtosend);
		NRF_Reset(FIFO_STATUS);
		return NRF_OK;
	}
	return NRF_ERROR;
}
void NRF_RxMode(uint8_t *address, uint8_t chanel)
{
	DisableCE();
	NRF_Reset(STATUS);

	NRF_WriteReg(RF_CH, chanel);

	uint8_t temp,temp1;
	NRF_ReadReg(EN_RXADDR,&temp);
	temp |= (1<<3);  //cho ph????p pipe 3
	NRF_WriteReg(EN_RXADDR,temp);

	/* m????nh c??????n ghi ????????????a ch??????? cho pipe 1 n??????u mu???????n s?????? d??????ng c????c pipe kh????c
	 * v???? kh????c pipe kh????c ch??????? c???? 1byte address c???? ngh????a l???? 4 byte tr???????????c
	 * s?????? gi???????ng pipe 1 (t?????? 2-5)
	 *
	 */
	NRF_WriteRegMulti(RX_ADDR_P1, address,5); // set cho pipe 1
	NRF_WriteReg(RX_ADDR_P3,0xEE);

	NRF_WriteReg(RX_PW_P3, 32);

	NRF_ReadReg(CONFIG,&temp1);
	temp1 |= (1<<0);
	temp1 |= (1<<1);
	NRF_WriteReg(CONFIG, temp1);
	EnableCE();
}
NRF_Status DataAvailable(uint8_t pipenum)
{
	uint8_t status;
	NRF_ReadReg(STATUS, &status);
	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{

		NRF_WriteReg(STATUS, (1<<6));

		return NRF_OK;
	}

	return NRF_ERROR;
}
void NRF_Receive (uint8_t *data)
{
	uint8_t cmdtosend = 0;
	EnableCS();

	//g??????i l???????nh cmd PAYLOAD
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	// Nh??????n data
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);
	DisableCS();

	HAL_Delay(1);
	// x????a h??????t FIFO
	cmdtosend = FLUSH_RX;
	NRF_SendToCmd(cmdtosend);
}

