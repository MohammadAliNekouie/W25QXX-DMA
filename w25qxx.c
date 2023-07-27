
#include "w25qxxConf.h"
#include "w25qxx.h"
#include <stdio.h>
#include <string.h>

#if (_W25QXX_DEBUG == 1)
#include <stdio.h>
#endif
const uint16_t w25q[] = {0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080, 0x0100, 0x0200, 0x0400};
const uint16_t PageSize = 0x0100; // 256
const uint16_t SectorSize = 0x1000; // 4096
const uint16_t Block32KSize = 0x8000; // 32768
const uint32_t BlockSize = 0x00010000; // 65536

#define W25QXX_DUMMY_BYTE 0xA5
//#define W25QXX_USE_DMA
extern uint8_t dataBuffer[SectorSize];
uint16_t cnt;
w25qxx_t w25qxx;
#if (_W25QXX_USE_FREERTOS == 1)
#define W25qxx_Delay(delay) osDelay(delay)
#include "cmsis_os.h"
#else
#define W25qxx_Delay(delay) for(cnt=0;cnt<delay;cnt++);//LL_mDelay(delay)
#endif
//###################################################################################################################
uint8_t W25qxx_Spi(uint8_t Data)
{
	uint8_t ret;
	//HAL_SPI_TransmitReceive(&_W25QXX_SPI, &Data, &ret, 1, 100);
	LL_SPI_TransmitData8(SPI_HANDLE, Data);
  while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE));
  
  while (LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE))
	{
		ret=LL_SPI_ReceiveData8(SPI_HANDLE);
	}
	return ret;
}

uint8_t Init_SPI_W25Qxx(void)
{
  uint8_t ret = 1;
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  // MOSI 
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  // MISO
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  // SCK
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  // NSS
  GPIO_InitStruct.Pin = SPI_CS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  LL_GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);  
  LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
  // SPI
  LL_SPI_InitTypeDef spi;
  spi.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi.Mode = LL_SPI_MODE_MASTER;
  spi.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi.ClockPolarity = LL_SPI_POLARITY_LOW;
  spi.ClockPhase = LL_SPI_PHASE_1EDGE;
  spi.NSS = LL_SPI_NSS_SOFT;
  spi.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  spi.BitOrder = LL_SPI_MSB_FIRST;
  spi.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	
	//Enable SPI2 Clock - Note for Other SPI interfaces you must change this.
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
	
  LL_SPI_Init(SPI_HANDLE, &spi);
  LL_SPI_Enable(SPI_HANDLE);

  // DMA Receive from SPI
  LL_DMA_InitTypeDef dma_spi_rx;
  dma_spi_rx.PeriphOrM2MSrcAddress = (uint32_t)&SPI_HANDLE->DR;
  dma_spi_rx.MemoryOrM2MDstAddress = (uint32_t)&dataBuffer;
  dma_spi_rx.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  dma_spi_rx.Mode = LL_DMA_MODE_NORMAL;
  dma_spi_rx.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_spi_rx.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_spi_rx.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_spi_rx.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_spi_rx.Priority = LL_DMA_PRIORITY_HIGH;
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_4, &dma_spi_rx); 

  // DMA Transmit to SPI
  LL_DMA_InitTypeDef dma_spi_tx;
  dma_spi_tx.PeriphOrM2MSrcAddress = (uint32_t)&SPI_HANDLE->DR;
  dma_spi_tx.MemoryOrM2MDstAddress = (uint32_t)&dataBuffer;
  dma_spi_tx.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_spi_tx.Mode = LL_DMA_MODE_NORMAL;
  dma_spi_tx.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_spi_tx.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_spi_tx.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_spi_tx.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_spi_tx.Priority = LL_DMA_PRIORITY_HIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_5, &dma_spi_tx);

  ret = 0;
  return ret;
}




// -------------------------------------------------------------  
/*
int W25Qxx_TransferDMASPI (uint32_t __bytes, DataDirectionTypeDef __dir, uint32_t __offset,uint8_t __command){
	
	static uint16_t timeout=0;
  LL_SPI_EnableDMAReq_TX(SPI_HANDLE);
 
  uint8_t pump = 0;
  if (__dir == WRITE)
  {
		
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, ((uint32_t)&dataBuffer) + (__offset * 256));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, __bytes);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
  }
  else
  {
		// Send a single dummy byte to start the SPI communication
		if(__command == Read_UniqueID || __command == Read_JedecID)
		{
			LL_SPI_TransmitData8(SPI_HANDLE, 0x00);
		}

		
    LL_SPI_EnableDMAReq_RX(SPI_HANDLE);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&pump);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, __bytes);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&dataBuffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, __bytes);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
  }

	timeout = 0;
  while (!LL_DMA_IsActiveFlag_TC5(DMA1))
	{
		timeout++;
		if(timeout>=SPI_TIMEOUT){return -1;}
	}
	
  LL_DMA_ClearFlag_TC5(DMA1);
  // Wait for DMA transfer to complete
	timeout = 0;
  while (!LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_5))
	{
		timeout++;
		if(timeout>=SPI_TIMEOUT){return -1;}
	}
	//we have to wait for DMA to stop so Disable SPI2 TX DMA request
	LL_SPI_DisableDMAReq_TX(SPI_HANDLE);
	// Wait for any ongoing SPI transmissions to complete
	timeout = 0;
  while (LL_SPI_IsActiveFlag_TXE(SPI_HANDLE) == RESET || LL_SPI_IsActiveFlag_BSY(SPI_HANDLE) == SET)
	{
		timeout++;
		if(timeout>=SPI_TIMEOUT){return -1;}
	}
	
  if (__dir == WRITE)
  {
		timeout = 0;
    while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE))
		{
			timeout++;
			if(timeout>=SPI_TIMEOUT){return -1;}
		}
  }
  else
  {
		timeout = 0;
    while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE))
		{
			timeout++;
			if(timeout>=SPI_TIMEOUT){return -1;}
		}			
		
		timeout = 0;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE))
		{
			timeout++;
			if(timeout>=SPI_TIMEOUT){return -1;}
		}
  }
	
	LL_SPI_DisableDMAReq_TX(SPI_HANDLE);
	LL_SPI_DisableDMAReq_RX(SPI_HANDLE);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	return 0;
}
*/
int W25Qxx_TransferDMASPI(uint8_t *buffer,uint32_t bytes, DataDirectionTypeDef dir, uint32_t offset, uint8_t command) {

    // Add your SPI handle definition here
    // SPI_HandleTypeDef spiHandle;
    uint16_t timeout = 0;
		uint8_t pump = 0;
    // Enable DMA request for TX
    LL_SPI_EnableDMAReq_TX(SPI_HANDLE);
  
    if (dir == WRITE) {
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, ((uint32_t)&buffer) + (offset * 256));
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, bytes);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    } else {
        // Send a single dummy byte to start the SPI communication
        if (command == Read_UniqueID || command == Read_JedecID) {
            LL_SPI_TransmitData8(SPI_HANDLE, 0x00);
        }

        // Enable DMA request for RX
        LL_SPI_EnableDMAReq_RX(SPI_HANDLE);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&pump);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, bytes);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&buffer);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, bytes);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
    }

    timeout = 0;
    while (!LL_DMA_IsActiveFlag_TC5(DMA1)) {
        timeout++;
        if (timeout >= SPI_TIMEOUT) {
            // Return an appropriate error code or handle the timeout error
            return -1;
        }
    }

    // Clear the DMA transfer complete flag
    LL_DMA_ClearFlag_TC5(DMA1);

    // Wait for DMA transfer to complete
    timeout = 0;
    while (!LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_5)) {
        timeout++;
        if (timeout >= SPI_TIMEOUT) {
            // Return an appropriate error code or handle the timeout error
            return -1;
        }
    }

    // Disable SPI TX DMA request
    LL_SPI_DisableDMAReq_TX(SPI_HANDLE);

    // Wait for any ongoing SPI transmissions to complete
    timeout = 0;
    while (LL_SPI_IsActiveFlag_TXE(SPI_HANDLE) == RESET || LL_SPI_IsActiveFlag_BSY(SPI_HANDLE) == SET) {
        timeout++;
        if (timeout >= SPI_TIMEOUT) {
            // Return an appropriate error code or handle the timeout error
            return -1;
        }
    }

    if (dir == WRITE) {
        timeout = 0;
        while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
            timeout++;
            if (timeout >= SPI_TIMEOUT) {
                // Return an appropriate error code or handle the timeout error
                return -1;
            }
        }
    } else {
        timeout = 0;
        while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
            timeout++;
            if (timeout >= SPI_TIMEOUT) {
                // Return an appropriate error code or handle the timeout error
                return -1;
            }
        }

        timeout = 0;
        while (!LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE)) {
            timeout++;
            if (timeout >= SPI_TIMEOUT) {
                // Return an appropriate error code or handle the timeout error
                return -1;
            }
        }
    }

    // Disable SPI TX and RX DMA requests
    LL_SPI_DisableDMAReq_TX(SPI_HANDLE);
    LL_SPI_DisableDMAReq_RX(SPI_HANDLE);

    // Disable DMA channels
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

    return 0;
}

///////////////Non DMA SPI MultiByte SPI Transfer//////////////////
int W25Qxx_MultiByteTransferSPI(uint8_t *buffer,uint32_t bytes, DataDirectionTypeDef dir, uint32_t offset, uint8_t command) {
    uint16_t timeout = 0;

    if (dir == WRITE) {
        // Send Write command if needed
        if (command == WRITE) {
            LL_SPI_TransmitData8(SPI_HANDLE, command);
            timeout = 0;
            while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
                timeout++;
                if (timeout >= SPI_TIMEOUT) {
                    // Return an appropriate error code or handle the timeout error
                    return -1;
                }
            }
        }

        // Send data bytes using SPI
        for (uint32_t i = 0; i < bytes; i++) {
            LL_SPI_TransmitData8(SPI_HANDLE, buffer[offset + i]);
            timeout = 0;
            while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
                timeout++;
                if (timeout >= SPI_TIMEOUT) {
                    // Return an appropriate error code or handle the timeout error
                    return -1;
                }
            }
        }
    } else {
        // Send Read command if needed
        if (command == Read_UniqueID || command == Read_JedecID) {
            LL_SPI_TransmitData8(SPI_HANDLE, command);
            timeout = 0;
            while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
                timeout++;
                if (timeout >= SPI_TIMEOUT) {
                    // Return an appropriate error code or handle the timeout error
                    return -1;
                }
            }
        }

        // Receive data bytes using SPI
        for (uint32_t i = 0; i < bytes; i++) {
            LL_SPI_TransmitData8(SPI_HANDLE, 0x00);
            timeout = 0;
            while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
                timeout++;
                if (timeout >= SPI_TIMEOUT) {
                    // Return an appropriate error code or handle the timeout error
                    return -1;
                }
            }

            timeout = 0;
            while (!LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE)) {
                timeout++;
                if (timeout >= SPI_TIMEOUT) {
                    // Return an appropriate error code or handle the timeout error
                    return -1;
                }
            }

            buffer[offset + i] = LL_SPI_ReceiveData8(SPI_HANDLE);
        }
    }

    // Wait for the SPI transmission to complete
    timeout = 0;
    while (LL_SPI_IsActiveFlag_BSY(SPI_HANDLE)) {
        timeout++;
        if (timeout >= SPI_TIMEOUT) {
            // Return an appropriate error code or handle the timeout error
            return -1;
        }
    }

    return 0;
}



int SPI_TRANSFER(uint8_t __command,uint8_t *__result)
{
	static uint16_t timeout=0;
	
	LL_SPI_TransmitData8(SPI_HANDLE, __command);
	while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE))
	{
		timeout++;
		if(timeout>=SPI_TIMEOUT){return -1;}
	}
	
	
	*__result = LL_SPI_ReceiveData8(SPI_HANDLE);
	timeout = 0;
  while (!LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE))
	{
		timeout++;
		if(timeout>=SPI_TIMEOUT){return -1;}
	}
	
	
	*__result = LL_SPI_ReceiveData8(SPI_HANDLE);
	//W25qxx_Delay(4); // a delay added to wait before finish the transmission and then chenge the CS state
	return 0;
}
/*
int W25Qxx_TransferSPI (uint8_t __command, int32_t __address, uint16_t __bytes, DataDirectionTypeDef __dir, uint32_t __offset)
{
	static int i , result = 0;
	uint8_t SPI_RESPONCE = 0;
	
  LL_GPIO_ResetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
	
	result = SPI_TRANSFER(__command,&SPI_RESPONCE);
	if(result == -1)
	{
		LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
		return -1;//Timeout Error
	}
	
  if (__address >= 0)
  {
    i = 4;
    while (--i)
    {
			result = SPI_TRANSFER( __address >> 8 * i,&SPI_RESPONCE);
			if(result == -1)
			{
				LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
				return -1;//Timeout Error
			}
    }
  }
  
  if (__command == FastRead)
  {
    i = 3;
    while (--i)
    {		
			result = SPI_TRANSFER(0,&SPI_RESPONCE);
			if(result == -1)
			{
				LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
				return -1;//Timeout Error
			}
		}
  }
  
  if (__bytes>1)
  {
    result = W25Qxx_TransferDMASPI(__bytes, __dir, __offset,__command);
		if(result == -1)
		{
			LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
			return -1;//Timeout Error
		}
  }
	else if(__bytes==1)
	{
		result = SPI_TRANSFER(W25QXX_DUMMY_BYTE,&SPI_RESPONCE);
		if(result == -1)
		{
			LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
			return -1;//Timeout Error
		}
	}
	
  LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
  return SPI_RESPONCE;
}
*/

int W25Qxx_TransferSPI(uint8_t command,uint8_t *buffer, int32_t address, uint16_t bytes, DataDirectionTypeDef dir, uint32_t offset)
{
    int result = 0;
    uint8_t spiResponse = 0;

    LL_GPIO_ResetOutputPin(SPI_CS_PORT, SPI_CS_PIN);

    result = SPI_TRANSFER(command, &spiResponse);
    if (result == -1)
    {
        LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
        return -1; // Timeout Error
    }

    if (address >= 0)
    {
        for (int i = 3; i > 0; i--)
        {
            result = SPI_TRANSFER((address >> (8 * i)) & 0xFF, &spiResponse);
            if (result == -1)
            {
                LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
                return -1; // Timeout Error
            }
        }
    }
		
    if (command == FastRead)
    {
        for (int i = 1; i > 0; i--)
        {
            result = SPI_TRANSFER(0, &spiResponse);
            if (result == -1)
            {
                LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
                return -1; // Timeout Error
            }
        }
    }
		
    if (bytes > 1)
    {
				#ifdef W25QXX_USE_DMA
        result = W25Qxx_TransferDMASPI(buffer, bytes, dir, offset, command);
				#else
				result = W25Qxx_MultiByteTransferSPI(buffer,bytes, dir, offset, command);
				#endif
			
        if (result == -1)
        {
            LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
            return -1; // Timeout Error
        }
    }
    else if (bytes == 1)
    {
        result = SPI_TRANSFER(buffer[0], &spiResponse);
        if (result == -1)
        {
            LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
            return -1; // Timeout Error
        }
    }

    LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
    return spiResponse;
}
//###################################################################################################################
uint32_t W25qxx_ReadID(void)
{
	static int result,retry;
	uint32_t Temp = 0xFFFFFFFF;
	while(retry<SPI_RETRY)
	{
		retry++;	
		result = W25Qxx_TransferSPI(Read_JedecID,dataBuffer,-1,4,READ,0);
		if(result != -1)
		{		
			Temp = (dataBuffer[0] << 16) | (dataBuffer[1] << 8) | dataBuffer[2];
			break;
		}
		W25qxx_Delay(1);
	}
	return Temp;
}
//###################################################################################################################
void W25qxx_ReadUniqID(void)
{
	W25Qxx_TransferSPI(Read_UniqueID,dataBuffer,-1,12,READ,0);
	memcpy(&w25qxx.UniqID[0],&dataBuffer[5],8);
}
//###################################################################################################################
void W25qxx_WriteEnable(void)
{
	W25Qxx_TransferSPI(WriteEnable,dataBuffer,-1,0,NEUTRAL,0);
	//W25qxx_Delay(1);
}
//###################################################################################################################
void W25qxx_WriteDisable(void)
{
	W25Qxx_TransferSPI(WtiteDisable,dataBuffer,-1,0,NEUTRAL,0);
	//W25qxx_Delay(1);
}
//###################################################################################################################
uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3)
{
	int result,retry;
	if (SelectStatusRegister_1_2_3 == 1)
	{
		retry = 0;
		while(retry<SPI_RETRY)
		{		
			retry++;
			result = W25Qxx_TransferSPI(Read_StatusRegister_1,dataBuffer,-1,1,READ,0);
			if(result!=-1){return result;}
		}
	}
	else if (SelectStatusRegister_1_2_3 == 2)
	{
		retry = 0;
		while(retry<SPI_RETRY)
		{		
			retry++;
			result = W25Qxx_TransferSPI(Read_StatusRegister_2,dataBuffer,-1,1,READ,0);
			if(result!=-1){return result;}
		}
	}
	else
	{
		retry = 0;
		while(retry<SPI_RETRY)
		{		
			retry++;
			result = W25Qxx_TransferSPI(Read_StatusRegister_3,dataBuffer,-1,1,READ,0);
			if(result!=-1){return result;}
		}
	}
	return 0xFF; //Error due to bad argument or timeout
}
//###################################################################################################################
void W25qxx_WriteStatusRegister(uint8_t SelectStatusRegister_1_2_3, uint8_t Data)
{
	dataBuffer[0]=Data;
	if (SelectStatusRegister_1_2_3 == 1)
	{
		W25Qxx_TransferSPI(Write_StatusRegister_1,dataBuffer,-1,1,WRITE,0);
		w25qxx.StatusRegister1 = Data;
	}
	else if (SelectStatusRegister_1_2_3 == 2)
	{
		W25Qxx_TransferSPI(Write_StatusRegister_2,dataBuffer,-1,1,WRITE,0);
		w25qxx.StatusRegister2 = Data;
	}
	else
	{
		W25Qxx_TransferSPI(Write_StatusRegister_3,dataBuffer,-1,1,WRITE,0);
		w25qxx.StatusRegister3 = Data;
	}
}
//###################################################################################################################
void W25qxx_WaitForWriteEnd(void)
{
	do
	{
		w25qxx.StatusRegister1 = W25qxx_ReadStatusRegister(1);
		//W25qxx_Delay(100);
	} while ((w25qxx.StatusRegister1 & 0x01) == 0x01);
}
//###################################################################################################################
void W25qxx_DisableWriteProtect(void)
{
	uint8_t statusReg;
	// Read Status Register
	statusReg = W25qxx_ReadStatusRegister(1);
	
	// Disable SR protection
	W25Qxx_TransferSPI(SR_Unlock,dataBuffer,-1,0,NEUTRAL,0);
	
	//Enable Write
	W25qxx_WriteEnable();
	
	// Disable Sector/Block Protection (SEC)
	statusReg &= ~(1 << 6);

	// Disable Block Protection (BP3 ,BP2, BP1)
	statusReg &= ~(0x1C);

	W25qxx_WriteStatusRegister(1,statusReg);
	W25qxx_Delay(1);
}
//###################################################################################################################
bool W25qxx_Init(void)
{
	w25qxx.Lock = 1;
	
	LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
	W25qxx_Delay(1);

#if (_W25QXX_DEBUG == 1)
	printf("w25qxx Init Begin...\r\n");
#endif
	
	
	w25qxx.ManID = W25qxx_ReadID();
	if(w25qxx.ManID == 0xFFFFFFFF) // W25Q64 No responded
	{		
			return false;
	}

	
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ID:0x%X\r\n", id);
#endif
	switch (w25qxx.ManID & 0x000000FF)
	{
	case 0x20: // 	w25q512
		w25qxx.ID = W25Q512;
		w25qxx.BlockCount = 1024;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q512\r\n");
#endif
		break;
	case 0x19: // 	w25q256
		w25qxx.ID = W25Q256;
		w25qxx.BlockCount = 512;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q256\r\n");
#endif
		break;
	case 0x18: // 	w25q128
		w25qxx.ID = W25Q128;
		w25qxx.BlockCount = 256;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q128\r\n");
#endif
		break;
	case 0x17: //	w25q64
		w25qxx.ID = W25Q64;
		w25qxx.BlockCount = 128;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q64\r\n");
#endif
		break;
	case 0x16: //	w25q32
		w25qxx.ID = W25Q32;
		w25qxx.BlockCount = 64;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q32\r\n");
#endif
		break;
	case 0x15: //	w25q16
		w25qxx.ID = W25Q16;
		w25qxx.BlockCount = 32;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q16\r\n");
#endif
		break;
	case 0x14: //	w25q80
		w25qxx.ID = W25Q80;
		w25qxx.BlockCount = 16;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q80\r\n");
#endif
		break;
	case 0x13: //	w25q40
		w25qxx.ID = W25Q40;
		w25qxx.BlockCount = 8;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q40\r\n");
#endif
		break;
	case 0x12: //	w25q20
		w25qxx.ID = W25Q20;
		w25qxx.BlockCount = 4;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q20\r\n");
#endif
		break;
	case 0x11: //	w25q10
		w25qxx.ID = W25Q10;
		w25qxx.BlockCount = 2;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q10\r\n");
#endif
		break;
	default:
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Unknown ID\r\n");
#endif
		w25qxx.Lock = 0;
		return false;
	}
	w25qxx.PageSize = 256;
	w25qxx.SectorSize = 0x1000;
	w25qxx.SectorCount = w25qxx.BlockCount * 16;
	w25qxx.PageCount = (w25qxx.SectorCount * w25qxx.SectorSize) / w25qxx.PageSize;
	w25qxx.BlockSize = w25qxx.SectorSize * 16;
	w25qxx.Capacity = (w25qxx.SectorCount * w25qxx.SectorSize) ;
	w25qxx.CapacityInKiloByte = w25qxx.Capacity / 1024;
	W25qxx_ReadUniqID();
	W25qxx_ReadStatusRegister(1);
	W25qxx_ReadStatusRegister(2);
	//W25qxx_ReadStatusRegister(3);
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx Page Size: %d Bytes\r\n", w25qxx.PageSize);
	printf("w25qxx Page Count: %d\r\n", w25qxx.PageCount);
	printf("w25qxx Sector Size: %d Bytes\r\n", w25qxx.SectorSize);
	printf("w25qxx Sector Count: %d\r\n", w25qxx.SectorCount);
	printf("w25qxx Block Size: %d Bytes\r\n", w25qxx.BlockSize);
	printf("w25qxx Block Count: %d\r\n", w25qxx.BlockCount);
	printf("w25qxx Capacity: %d KiloBytes\r\n", w25qxx.CapacityInKiloByte);
	printf("w25qxx Init Done\r\n");
#endif
	w25qxx.Lock = 0;
	return true;
}
//###################################################################################################################
void W25qxx_EraseChip(void)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx EraseChip Begin...\r\n");
#endif
	W25qxx_WriteEnable();
	W25Qxx_TransferSPI(Erase_Chip,dataBuffer,-1,0,NEUTRAL,0);
	W25qxx_Delay(100);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock done after %d ms!\r\n", HAL_GetTick() - StartTime);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_EraseSector(uint32_t SectorAddr)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx EraseSector %d Begin...\r\n", SectorAddr);
#endif
	
	W25qxx_WaitForWriteEnd();
	SectorAddr = SectorAddr * w25qxx.SectorSize;
	W25qxx_WriteEnable();

	if (w25qxx.ID >= W25Q256)
	{
		W25Qxx_TransferSPI(Erase_Sector_256,dataBuffer,((SectorAddr & 0xFF000000) >> 24),0,WRITE,0);
	}
	else
	{
		W25Qxx_TransferSPI(Erase_Sector,dataBuffer,SectorAddr,0,WRITE,0);
	}
	W25qxx_Delay(20);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseSector done after %d ms\r\n", HAL_GetTick() - StartTime);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_EraseBlock(uint32_t BlockAddr)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock %d Begin...\r\n", BlockAddr);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	W25qxx_WaitForWriteEnd();
	BlockAddr = BlockAddr * w25qxx.SectorSize * 16;
	W25qxx_WriteEnable();
	if (w25qxx.ID >= W25Q256)
	{
		W25Qxx_TransferSPI(Erase_Block_64_256,dataBuffer,((BlockAddr & 0xFF000000) >> 24),0,WRITE,0);		
	}
	else
	{
		W25Qxx_TransferSPI(Erase_Block_64,dataBuffer,BlockAddr,0,WRITE,0);
	}
	W25qxx_Delay(80);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock done after %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
uint32_t W25qxx_PageToSector(uint32_t PageAddress)
{
	return ((PageAddress * w25qxx.PageSize) / w25qxx.SectorSize);
}
//###################################################################################################################
uint32_t W25qxx_PageToBlock(uint32_t PageAddress)
{
	return ((PageAddress * w25qxx.PageSize) / w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress)
{
	return ((SectorAddress * w25qxx.SectorSize) / w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress)
{
	return (SectorAddress * w25qxx.SectorSize) / w25qxx.PageSize;
}
//###################################################################################################################
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress)
{
	return (BlockAddress * w25qxx.BlockSize) / w25qxx.PageSize;
}
//###################################################################################################################
bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if (((NumByteToCheck_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || (NumByteToCheck_up_to_PageSize == 0))
		NumByteToCheck_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage:%d, Offset:%d, Bytes:%d begin...\r\n", Page_Address, OffsetInByte, NumByteToCheck_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.PageSize; i += sizeof(pBuffer))
	{
		WorkAddress = (i + Page_Address * w25qxx.PageSize);
		if (w25qxx.ID >= W25Q256)
		{
			W25Qxx_TransferSPI(FastRead_256,pBuffer,((WorkAddress & 0xFF000000) >> 24),sizeof(pBuffer),READ,0);					
		}
		else
		{
			W25Qxx_TransferSPI(FastRead,pBuffer,WorkAddress,sizeof(pBuffer),READ,0);	
		}
		memcpy(pBuffer,dataBuffer,sizeof(pBuffer));

		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.PageSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.PageSize; i++)
		{

			WorkAddress = (i + Page_Address * w25qxx.PageSize);
			if (w25qxx.ID >= W25Q256)
			{
				W25Qxx_TransferSPI(FastRead_256,pBuffer,((WorkAddress & 0xFF000000) >> 24),sizeof(pBuffer),READ,0);		
			}
			else
			{
				W25Qxx_TransferSPI(FastRead,pBuffer,WorkAddress,sizeof(pBuffer),READ,0);	
			}						
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage is Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage is Not Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return false;
}
//###################################################################################################################
bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if ((NumByteToCheck_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToCheck_up_to_SectorSize == 0))
		NumByteToCheck_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector:%d, Offset:%d, Bytes:%d begin...\r\n", Sector_Address, OffsetInByte, NumByteToCheck_up_to_SectorSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.SectorSize; i += sizeof(pBuffer))
	{
		WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
		if (w25qxx.ID >= W25Q256)
		{
			W25Qxx_TransferSPI(FastRead_256,pBuffer,((WorkAddress & 0xFF000000) >> 24),sizeof(pBuffer),READ,0);		
		}
		else
		{
			W25Qxx_TransferSPI(FastRead,pBuffer,WorkAddress,sizeof(pBuffer),READ,0);	
		}
		memcpy(pBuffer,dataBuffer,sizeof(pBuffer));		
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.SectorSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.SectorSize; i++)
		{
			LL_GPIO_ResetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
			WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
			if (w25qxx.ID >= W25Q256)
			{
				W25Qxx_TransferSPI(FastRead_256,pBuffer,((WorkAddress & 0xFF000000) >> 24),sizeof(pBuffer),READ,0);		
			}
			else
			{
				W25Qxx_TransferSPI(FastRead,pBuffer,WorkAddress,sizeof(pBuffer),READ,0);	
			}				
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector is Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector is Not Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return false;
}
//###################################################################################################################
bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if ((NumByteToCheck_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToCheck_up_to_BlockSize == 0))
		NumByteToCheck_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock:%d, Offset:%d, Bytes:%d begin...\r\n", Block_Address, OffsetInByte, NumByteToCheck_up_to_BlockSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.BlockSize; i += sizeof(pBuffer))
	{
		WorkAddress = (i + Block_Address * w25qxx.BlockSize);

		if (w25qxx.ID >= W25Q256)
		{
			W25Qxx_TransferSPI(FastRead_256,pBuffer,((WorkAddress & 0xFF000000) >> 24),sizeof(pBuffer),READ,0);		
		}
		else
		{
			W25Qxx_TransferSPI(FastRead,pBuffer,WorkAddress,sizeof(pBuffer),READ,0);	
		}
		memcpy(pBuffer,dataBuffer,sizeof(pBuffer));	
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.BlockSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.BlockSize; i++)
		{
			WorkAddress = (i + Block_Address * w25qxx.BlockSize);

			if (w25qxx.ID >= W25Q256)
			{
				W25Qxx_TransferSPI(FastRead_256,pBuffer,((WorkAddress & 0xFF000000) >> 24),1,READ,0);		
			}
			else
			{
				W25Qxx_TransferSPI(FastRead,pBuffer,WorkAddress,1,READ,0);	
			}			
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock is Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock is Not Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return false;
}
//###################################################################################################################
void W25qxx_WriteByte(uint8_t *pBuffer, uint32_t WriteAddr_inBytes)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx WriteByte 0x%02X at address %d begin...", pBuffer, WriteAddr_inBytes);
#endif
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();

	if (w25qxx.ID >= W25Q256)
	{
		W25Qxx_TransferSPI(PageProgram_256,pBuffer,((WriteAddr_inBytes & 0xFF000000) >> 24),sizeof(pBuffer),WRITE,0);	
	}
	else
	{
		W25Qxx_TransferSPI(PageProgram,pBuffer,WriteAddr_inBytes,sizeof(pBuffer),WRITE,0);	
	}
	W25qxx_Delay(1);
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx WriteByte done after %d ms\r\n", HAL_GetTick() - StartTime);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if (((NumByteToWrite_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || (NumByteToWrite_up_to_PageSize == 0))
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
	if ((OffsetInByte + NumByteToWrite_up_to_PageSize) > w25qxx.PageSize)
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx WritePage:%d, Offset:%d ,Writes %d Bytes, begin...\r\n", Page_Address, OffsetInByte, NumByteToWrite_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	W25qxx_WaitForWriteEnd();
	Page_Address = (Page_Address * w25qxx.PageSize) + OffsetInByte;
	if (w25qxx.ID >= W25Q256)
	{
		W25Qxx_TransferSPI(PageProgram_256,pBuffer,((Page_Address & 0xFF000000) >> 24),NumByteToWrite_up_to_PageSize,WRITE,0);	
	}
	else
	{
		W25Qxx_TransferSPI(PageProgram,pBuffer,Page_Address,NumByteToWrite_up_to_PageSize,WRITE,0);	
	}
	
	W25qxx_Delay(1);	
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	StartTime = HAL_GetTick() - StartTime;
	for (uint32_t i = 0; i < NumByteToWrite_up_to_PageSize; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx WritePage done after %d ms\r\n", StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize)
{
	if ((NumByteToWrite_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToWrite_up_to_SectorSize == 0))
		NumByteToWrite_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx WriteSector:%d, Offset:%d ,Write %d Bytes, begin...\r\n", Sector_Address, OffsetInByte, NumByteToWrite_up_to_SectorSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.SectorSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx WriteSector Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToWrite = w25qxx.SectorSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_SectorSize;
	StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, w25qxx.PageSize);//BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx WriteSector Done\r\n");
	W25qxx_Delay(100);
#endif
}
//###################################################################################################################
void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize)
{
	if ((NumByteToWrite_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToWrite_up_to_BlockSize == 0))
		NumByteToWrite_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx WriteBlock:%d, Offset:%d ,Write %d Bytes, begin...\r\n", Block_Address, OffsetInByte, NumByteToWrite_up_to_BlockSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.BlockSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx WriteBlock Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToWrite = w25qxx.BlockSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_BlockSize;
	StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx WriteBlock Done\r\n");
	W25qxx_Delay(100);
#endif
}
//###################################################################################################################
void W25qxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx ReadByte at address %d begin...\r\n", Bytes_Address);
#endif

	W25qxx_WaitForWriteEnd();
	if (w25qxx.ID >= W25Q256)
	{
		W25Qxx_TransferSPI(FastRead_256,dataBuffer,((Bytes_Address & 0xFF000000) >> 24),1,READ,0);
	}
	else
	{
		W25Qxx_TransferSPI(FastRead,dataBuffer,Bytes_Address,1,READ,0);	
	}

#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ReadByte 0x%02X done after %d ms\r\n", *pBuffer, HAL_GetTick() - StartTime);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx ReadBytes at Address:%d, %d Bytes  begin...\r\n", ReadAddr, NumByteToRead);
#endif
	W25qxx_WaitForWriteEnd();
	if (w25qxx.ID >= W25Q256)
	{
		W25Qxx_TransferSPI(FastRead_256,pBuffer,((ReadAddr & 0xFF000000) >> 24),NumByteToRead,READ,0);
	}
	else
	{
		W25Qxx_TransferSPI(FastRead,pBuffer,ReadAddr,NumByteToRead,READ,0);
	}
	
#if (_W25QXX_DEBUG == 1)
	StartTime = HAL_GetTick() - StartTime;
	for (uint32_t i = 0; i < NumByteToRead; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx ReadBytes done after %d ms\r\n", StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
	W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if ((NumByteToRead_up_to_PageSize > w25qxx.PageSize) || (NumByteToRead_up_to_PageSize == 0))
		NumByteToRead_up_to_PageSize = w25qxx.PageSize;
	if ((OffsetInByte + NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
		NumByteToRead_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ReadPage:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Page_Address, OffsetInByte, NumByteToRead_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	W25qxx_WaitForWriteEnd();
	Page_Address = Page_Address * w25qxx.PageSize + OffsetInByte;
	if (w25qxx.ID >= W25Q256)
	{
		W25Qxx_TransferSPI(FastRead_256,pBuffer,((Page_Address & 0xFF000000) >> 24),NumByteToRead_up_to_PageSize,READ,0);
	}
	else
	{
		W25Qxx_TransferSPI(FastRead,pBuffer,Page_Address,NumByteToRead_up_to_PageSize,READ,0);
	}

#if (_W25QXX_DEBUG == 1)
	StartTime = HAL_GetTick() - StartTime;
	for (uint32_t i = 0; i < NumByteToRead_up_to_PageSize; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx ReadPage done after %d ms\r\n", StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
	if ((NumByteToRead_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToRead_up_to_SectorSize == 0))
		NumByteToRead_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx ReadSector:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Sector_Address, OffsetInByte, NumByteToRead_up_to_SectorSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.SectorSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx ReadSector Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToRead = w25qxx.SectorSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;
	StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, w25qxx.PageSize);//BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx ReadSector Done\r\n");
	W25qxx_Delay(100);
#endif
}
//###################################################################################################################
void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize)
{
	if ((NumByteToRead_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToRead_up_to_BlockSize == 0))
		NumByteToRead_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx ReadBlock:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Block_Address, OffsetInByte, NumByteToRead_up_to_BlockSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.BlockSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx ReadBlock Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToRead = w25qxx.BlockSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_BlockSize;
	StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx ReadBlock Done\r\n");
	W25qxx_Delay(100);
#endif
}
//###################################################################################################################
