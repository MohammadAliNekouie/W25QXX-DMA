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

