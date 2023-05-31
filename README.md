## DMA based W25QXX SPI FLASH Library for STM32 Low Layer Libs

This project based on two other below projects:

https://github.com/zoosmand/Winbond-W25Qxx-EEPROM-SPI-with-DMA-on-STM32

https://github.com/nimaltd/w25qxx

### Before Start:
* Be Sure you Enabled SPI and a Gpio as output(CS pin).Connect WP and HOLD to VCC.
* Select software CS pin and declear it at `w25qxxConf.h`.
* Config SPI handel at `w25qxxConf.h`.
* Call `W25qxx_Init()`. 
* After init, you can watch `w25qxx` struct.(Chip ID,page size,sector size and ...)
* In Read/Write Function, you can put 0 to `NumByteToRead/NumByteToWrite` parameter to maximum.
* Dont forget to erase page/sector/block before write.

## Test Code:
```
  //Init
  Init_SPI_W25Qxx();
  LL_SPI_Enable(SPI2);
  W25qxx_Init();
  
  //Read Chip info
  sprintf(string,"MEMORY INFORMATION:\r\n"
	 " MEM:\r\n  W25Q%d\r\n"
	 " JID %d\r\n "
	 " UID %d\r\n "
 	 " BLC %d\r\n "
	 " CAP %d\r\n",
 	 (0x01<<(w25qxx.ID-1)),
	 (int)w25qxx.ManID,
	 (int)w25qxx.UniqID,
	 w25qxx.BlockCount,
	 w25qxx.Capacity);
  printf(string);
  
  //Erase
  W25qxx_EraseSector(1);
  
  //Write 
  for(int i=0;i<256;i++)
  {
    dataBuffer[i]=127;
  }	
  W25qxx_WriteSector(dataBuffer,1,0,256);
  
  //Read
  W25qxx_ReadSector(dataBuffer,1,0,256);
```




Thanks to @nimaltd and @zoosmand
