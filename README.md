## SPI2USB

Receives data as a SPI slave and outputs it through a usb serial connection (cdcacm).

Tested up to 2Mhz SPI speed with 72Mhz core clock speed.

Writen for stm32f103, but it should work on other stm32s with slight modifications. 

~~This really only exists because I wanted to read logs from a nrf52 in real time, but its uart doesn't have dma support and its spi does.~~

Upon closer inspection, the uart peripheral does support dma ... \*facepalm\*

Update: nrf51 devices doesn't have UARTE (uart with dma), so this program might be useful with those devices

### Pins

A3: reset: pulled low, pulse to reset spi reciever  
A5: CLK  
A7: MOSI  

### SPI Settings

SPI mode 0: CPOL=0, CPHA=0  
LSB first  
