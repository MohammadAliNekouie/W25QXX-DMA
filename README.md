## DMA based W25QXX SPI FLASH Library for STM32 Low Layer Libs

This project based on two other below projects:

https://github.com/zoosmand/Winbond-W25Qxx-EEPROM-SPI-with-DMA-on-STM32

https://github.com/nimaltd/w25qxx

* Enable SPI and a Gpio as output(CS pin).Connect WP and HOLD to VCC.
* Select software CS pin.
* Config `w25qxxConf.h`.
* Call `W25qxx_Init()`. 
* After init, you can watch `w25qxx` struct.(Chip ID,page size,sector size and ...)
* In Read/Write Function, you can put 0 to `NumByteToRead/NumByteToWrite` parameter to maximum.
* Dont forget to erase page/sector/block before write.

## Test Code:
```
  Init_SPI_W25Qxx();
  LL_SPI_Enable(SPI2);
  W25qxx_Init();
  W25qxx_EraseSector(1);
  for(int i=0;i<256;i++)
  {
    dataBuffer[i]=127;
  }	
  W25qxx_WriteSector(dataBuffer,1,0,256);
  
  W25qxx_ReadSector(dataBuffer,1,0,256);
```




Thanks to @nimaltd and @zoosmand
