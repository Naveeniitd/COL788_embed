


a3:

    Copy the driver files FATFS/Target/user_diskio_spi.c and FATFS/Target/user_diskio_spi.h into your CubeIDE project which has been configured to use FatFS.
    In FATFS/Target/user_diskio.c add #include "user_diskio_spi.h"
    In FATFS/Target/user_diskio.c , 


    In main.h ensure that you have #defines for SD_SPI_HANDLE (e.g. hspi2), SD_CS_GPIO_Port, and SD_CS_Pin.
    Double check what would be suitable high/low speeds for your SPI driver and the prescalars you will need to use for the SPI port. Configure these at in user_diskio_spi.c:
