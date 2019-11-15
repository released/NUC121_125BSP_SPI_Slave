# NUC121_125BSP_SPI_Slave
 NUC121_125BSP_SPI_Slave

update @ 2019/11/15

USCI SPI slave hardware setting : 

- NUC125 EVM , CS (PC.1) , CLK (PC.0), MOSI (PC.3), GND

CS pin connect to master CS pin (suggest) , or GND

user can use ENABLE_SPI_NORMAL , or ENABLE_SPI_PDMA to check SPI RX behavior 

If master transmit data prettry fast (ex : 5ms) , slave side (NUC12) suggest use PDMA to RX receive and CS connect to master
