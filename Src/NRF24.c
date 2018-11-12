#include "NRF24.h"
//------------------------------------------------
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
#define PWM4 TIM3->CCR2
//------------------------------------------------
#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 5
uint8_t TX_ADDRESS0[TX_ADR_WIDTH] = {0xb5,0xb5,0xa1};
uint8_t TX_ADDRESS1[TX_ADR_WIDTH] = {0xb7,0xb5,0xa1};
uint8_t RX_BUF[TX_PLOAD_WIDTH+1] = {0};
volatile uint8_t rx_flag = 0, tx_flag = 0;
uint32_t TIM1_Count=0;
uint16_t cnt1=0;
extern uint8_t buf1[20];

extern uint8_t SRV_C, SRV_Left, SRV_Right;
extern uint8_t MaxPwr;

extern uint16_t TimeLastRx;
extern uint8_t OptionFlag;
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 9;
  /* Wait till done */
  while (micros--) ;
}
//--------------------------------------------------
uint8_t NRF24_ReadReg(uint8_t addr)
{
  uint8_t dt=0, cmd;
  CS_ON;
  HAL_SPI_TransmitReceive(&hspi1,&addr,&dt,1,1000);
  if (addr!=STATUS)//anee aa?an ?aaai aa?an ?aaeno?a noaoon oi e aicaa?uaai aai ninoiyiea
  {
    cmd=0xFF;
    HAL_SPI_TransmitReceive(&hspi1,&cmd,&dt,1,1000);
  }
  CS_OFF;
  return dt;
}
//------------------------------------------------
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
  addr |= W_REGISTER;//aee??ei aeo caiene a aa?an
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//ioi?aaei aa?an a oeio
  HAL_SPI_Transmit(&hspi1,&dt,1,1000);//ioi?aaei aaiiua a oeio
  CS_OFF;
}
//------------------------------------------------
void NRF24_ToggleFeatures(void)
{
  uint8_t dt[1] = {ACTIVATE_NO_ACK};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  dt[0] = 0x73;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  CS_OFF;
}
//-----------------------------------------------
void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//ioi?aaei aa?an a oeio
  HAL_SPI_Receive(&hspi1,pBuf,bytes,1000);//ioi?aaei aaiiua a aooa?
  CS_OFF;
}
//------------------------------------------------
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  addr |= W_REGISTER;//aee??ei aeo caiene a aa?an
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//ioi?aaei aa?an a oeio
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//ioi?aaei aaiiua a aooa?
  CS_OFF;
}
//------------------------------------------------
void NRF24_FlushRX(void)
{
  uint8_t dt[1] = {FLUSH_RX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------
void NRF24_FlushTX(void)
{
  uint8_t dt[1] = {FLUSH_TX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------
void NRF24L01_RX_Mode(void)
{
  uint8_t regval=0x00;
  regval = NRF24_ReadReg(CONFIG);
  //?acaoaei iiaoeu e ia?aaaa?i aai a ?a?ei i?e?iieea, aee??ea aeou PWR_UP e PRIM_RX
  regval |= (1<<PWR_UP)|(1<<PRIM_RX);
  NRF24_WriteReg(CONFIG,regval);
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS1, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS1, TX_ADR_WIDTH);
  CE_SET;
  DelayMicro(150); //Caaa??ea ieieioi 130 ien
  // Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}
//------------------------------------------------
void NRF24L01_TX_Mode(uint8_t *pBuf)
{
  NRF24_Write_Buf(TX_ADDR, TX_ADDRESS0, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS0, TX_ADR_WIDTH);
  CE_RESET;
  // Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}
//------------------------------------------------
void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CE_RESET;
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//ioi?aaei aa?an a oeio
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//ioi?aaei aaiiua a aooa?
  CS_OFF;
  CE_SET;
}
//------------------------------------------------
uint8_t NRF24L01_Send(uint8_t *pBuf)
{
  uint8_t regval=0x00;
  NRF24L01_TX_Mode(pBuf);
  regval = NRF24_ReadReg(CONFIG);
  //anee iiaoeu ooae a niyuee ?a?ei, oi ?acaoaei aai, aee??ea aeo PWR_UP e auee??ea PRIM_RX
  regval |= (1<<PWR_UP);
  regval &= ~(1<<PRIM_RX);
  NRF24_WriteReg(CONFIG,regval);
  DelayMicro(150); //Caaa??ea ieieioi 130 ien
  //Ioi?aaei aaiiua a aicaoo
  NRF24_Transmit(WR_TX_PLOAD, pBuf, TX_PLOAD_WIDTH);
  CE_SET;
  DelayMicro(15); //minimum 10us high pulse (Page 21)
  CE_RESET;
  return 0;
}
//------------------------------------------------
void NRF24L01_Receive(void)
{
  if(rx_flag==1)
  {
		rx_flag = 0;
		TIM1_Count=0;
		if (RX_BUF[4] == 1)
			NRF24L01_Send(buf1);
		if (RX_BUF[4] < 2)
				OptionFlag = 0;
			else
				OptionFlag = 1;
			
		if (RX_BUF[4] == 2){
			buf1[0] = SRV_Left;
			buf1[4] = 2;
			NRF24L01_Send(buf1);
			if (RX_BUF[0]>30 && RX_BUF[0]<SRV_C)
				SRV_Left = RX_BUF[0];
			PWM4 = SRV_C - (SRV_C-SRV_Left);	
			HAL_Delay(10);
			TimeLastRx = 0;
		}
		if (RX_BUF[4] == 3){
			buf1[0] = SRV_Right;
			buf1[4] = 3;
			NRF24L01_Send(buf1);
			if (RX_BUF[0]>SRV_C && RX_BUF[0]<90)
				SRV_Right = RX_BUF[0];
			PWM4 = SRV_C + (SRV_Right-SRV_C);	
			HAL_Delay(10);	
			TimeLastRx = 0;
		}
		if (RX_BUF[4] == 5){
			buf1[0] = SRV_C;
			buf1[4] = 5;
			NRF24L01_Send(buf1);
			if (RX_BUF[0]>SRV_Left && RX_BUF[0]<SRV_Right)
				SRV_C = RX_BUF[0];
			PWM4 = SRV_C;	
			HAL_Delay(10);
			TimeLastRx = 0;			
		}
		if (RX_BUF[4] == 6){
			buf1[0] = MaxPwr;
			buf1[4] = 6;
			NRF24L01_Send(buf1);
			if (RX_BUF[0]<=8)
				MaxPwr = RX_BUF[0];
			HAL_Delay(10);
			TimeLastRx = 0;			
		}
  }
}
//------------------------------------------------
void NRF24_ini(void)
{
	CE_RESET;
  DelayMicro(5000);
	NRF24_WriteReg(CONFIG, 0x0a); // Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
  DelayMicro(5000);
	NRF24_WriteReg(EN_AA, 0x01); // Enable Pipe0
	NRF24_WriteReg(EN_RXADDR, 0x01); // Enable Pipe0
	NRF24_WriteReg(SETUP_AW, 0x01); // Setup address width=3 bytes
	NRF24_WriteReg(SETUP_RETR, 0x5F); // // 1500us, 15 retrans
	NRF24_ToggleFeatures();
	NRF24_WriteReg(FEATURE, 0);
	NRF24_WriteReg(DYNPD, 0);
	NRF24_WriteReg(STATUS, 0x70); //Reset flags for IRQ
	NRF24_WriteReg(RF_CH, 76); // ?anoioa 2476 MHz
	NRF24_WriteReg(RF_SETUP, 0x26); //0x26 250Mbps 0x06-TX_PWR:0dBm, Datarate:1Mbps
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS0, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS0, TX_ADR_WIDTH);
	NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH); //Number of bytes in RX payload in data pipe 0
 //iiea ooiaei a ?a?ei i?e?iieea
  NRF24L01_RX_Mode();
  LED_OFF;
}
//--------------------------------------------------
void IRQ_Callback(void)
{
  uint8_t status=0x01;
  uint8_t pipe;
  DelayMicro(10);
  status = NRF24_ReadReg(STATUS);
  if(status & 0x40)
  {
		TimeLastRx = 0;
    LED_TOG;
    pipe = (status>>1)&0x07;
    NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
    *(RX_BUF+5) = pipe;
    NRF24_WriteReg(STATUS, 0x40);
    rx_flag = 1;
  }
  if(status&TX_DS) //tx_ds == 0x20
  {
    LED_TOG;
    NRF24_WriteReg(STATUS, 0x20);
    NRF24L01_RX_Mode();
    tx_flag = 1;
  }
  else if(status&MAX_RT)
  {
    NRF24_WriteReg(STATUS, 0x10);
    NRF24_FlushTX();
    //Ooiaei a ?a?ei i?e?iieea
    NRF24L01_RX_Mode();
  }
}
//--------------------------------------------------
void TIM1_Callback(void)
{
//  if(TIM1_Count>=1)
//  {
//    
//    HAL_TIM_Base_Stop_IT(&htim2);
//  }
//  TIM1_Count++;
}
//--------------------------------------------------