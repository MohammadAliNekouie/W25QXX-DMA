#ifndef _W25QXXCONFIG_H
#define _W25QXXCONFIG_H

#define WriteEnable             0x06
#define WtiteDisable            0x04
#define Read_JedecID            0x9f
#define Read_UniqueID           0x4b
#define Read_ManufatureID       0x90
#define Read_DeviceID           0xab
#define Read_StatusRegister_1   0x05
#define Read_StatusRegister_2   0x35
#define Read_StatusRegister_3   0x15
#define Write_StatusRegister    0x01
#define Write_StatusRegister_1  0x01
#define Write_StatusRegister_2  0x31
#define Write_StatusRegister_3  0x11
#define PageProgram             0x02
#define PageProgram_256         0x12
#define ReadData                0x03
#define FastRead                0x0b
#define FastRead_256            0x0c
#define Erase_Chip              0xc7
#define Erase_Block_64_256      0xdc
#define Erase_Block_64          0xd8
#define Erase_Block_32          0x52
#define Erase_Sector            0x20
#define Erase_Sector_256        0x21
#define Erase_Suspend           0x75
#define Erase_Resume            0x7h
#define PowerDown               0xb9
#define PowerDown_Release       0xab
#define Reset_Enable						0x66
#define Reset_Device						0x99
#define Global_Unlock						0x98
#define SR_Unlock								0x50

#define _BUSY_                  0x01 // Erase or write in progress
#define _WEL_                   0x02 // Write enable Latch
#define _BP0_                   0x04 // Block protect bit 0 (non-volatile)
#define _BP1_                   0x08 // Block protect bit 1 (non-volatile)
#define _BP2_                   0x10 // Block protect bit 2 (non-volatile)
#define _TB_                    0x20 // Top/Bottom write protect (non-volatile)
#define _SEC_                   0x40 // Sector protect (non-volatile)
#define _SRP0_                  0x80 // Status register protect bit 0 (non-volatile)

#define _SRP1_                  0x01 // Status register protect bit 1 (non-volatile)
#define _SREQ_                  0x02 // Quad enable

#define _W25QXX_SPI                   hspi2
#define  SPI_CS_PIN  									LL_GPIO_PIN_12
#define  SPI_CS_PORT 									GPIOB
#define  SPI_HANDLE 									SPI2
#define _W25QXX_USE_FREERTOS          1
#define _W25QXX_DEBUG                 0

typedef enum
{
  NEUTRAL = 2,
  READ = 1,
  WRITE = 0
} DataDirectionTypeDef;



#endif

