/**************************************************
 * @file    main.c
 * @brief   SPI_MasterSlave Demo
 * @details Shows Master loopback demo for QSPI0
 *          Read the printf() for instructions
**************************************************/

/***** Includes *****/ // Held over from SPI_MasterSlave
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "gpio.h"
#include "led.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "pb.h"
#include "spi.h"
#include "spimss.h"
#include "uart.h"

// Other includes
#include <stdlib.h>
#include "driver_w25qxx.h"
#include "driver_w25qxx_basic.h"
#include "driver_w25qxx_advance.h"

void testing(void);


void testing(void)
{
    printf("\n******************** Renesas Chip SPI w/ Winbond SDK Attempt ********************\n");
    printf("This is an attempt to communicate with the Winbond flash chip using the MAX32660.\n");
    printf("SPI0 is configured as the master (hopefully).\n\n");

    /***** Initialize the chip *****/
    printf("testing()...\n");
    uint8_t retVal = w25qxx_basic_init(W25Q16, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);
    printf("Initialization completed with value %d!\n", retVal);
}


int main(void)
{
    printf("\n************************ SPI MAX to Flash attempt ************************\n");
    printf("This is an attempt to communicate with the Winbond flash chip using the MAX32660.\n");
    printf("SPI0 is configured as the master (hopefully).\n\n");

    // testing();
    // return;

    /***** Initialize the chip *****/
    printf("test \n");
    printf("\n--\nInitializing the chip..\n");
    uint8_t retVal = w25qxx_basic_init(W25Q16, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);
    printf("Initialization completed with value %d!\n", retVal);


    /***** Try reading the manufacturer and device ID information *****/
    // Both are just one byte large.
    uint8_t man_id = 0;
    uint8_t dev_id = 0;
    printf("\n--\nGetting manufacturer and device ID..\n");
    int result = w25qxx_basic_get_id(&man_id, &dev_id);
    // Manufacturer ID should be EFh for Winbond.
    if (man_id != 0xEF) {
        printf("Expected 0xEF for manufacterer ID, but got %X instead!\n", man_id);
    }
    else {
        printf("Yay~~ Correct manufacturer ID of 0x%02X!\n", man_id);
    }
    // Device ID should be 16h for W25Q64JV, 14h for W25Q16JV.
    if (dev_id != 0x14) {
        printf("Expected 0x16 for device ID, but got %X instead!\n", dev_id);
    }
    else {
        printf("Yay~~ Correct device ID of 0x%02X!\n", dev_id);
    }


    /***** Try erasing the chip *****/
    printf("\n--\nConducting a chip erase. This may take a few seconds...\n");
    if (w25qxx_basic_chip_erase() > 0) {
        printf("Agh, basic chip erase threw and error :(\n");
    }
    else {
        printf("Chip erase finished without an error!\n");
    }


    /***** Try reading an empty block *****/
    // Pick a number of bytes to read out.
    uint32_t read_size = 256;
    uint32_t read_addr = 0x0;
    printf("\n--\nReading %d bytes from address 0x%06X.\n", read_size, read_addr);
    // Malloc some space to hold the read data.
    uint8_t* data_boi = (uint8_t*) malloc(read_size*sizeof(uint8_t));
    // Check if the malloc worked.
    if (data_boi == NULL) {
        printf("Failed to allocate memory for data_boi.\n");
        exit(0);
    }
    // Do the read.
    if (w25qxx_basic_read(read_addr, data_boi, read_size) > 0) {
        printf("Agh, basic read threw and error :(\n");
    } 
    else {
        // Print out the read bytes as a sanity check.
        printf("Read %d bytes! They are as follows (in hex):", read_size);
        for (int i = 0; i < read_size; i++) {
            // Print a -- between each 256 byte page.
            if (i % 256 == 0) {
                printf("\n\nPage %d:", (read_addr + i)/256);
            }
            // Print out the address at the start of every 16 bytes.
            if (i % 16 == 0) {
                printf("\n0x%06X |   ", read_addr + i);
            }
            else if (i % 8 == 0) {
                // Add an extra space between every 8 bytes.
                printf("  ");
            }
            // Print out our byte.
            printf("%02X ", data_boi[i]);
        }
        printf("\n");
    }

    // printf("before delay\n");
    // // Add time delay before starting write
    // for (int i = 0; i < 10000000; i++) {
    //     asm("nop");
    // }
    // printf("After delay\n");


    /***** Try writing to a page *****/
    uint32_t write_size = 200;
    printf("\n--\nWriting %d bytes to address 0x%06x.\n", write_size, read_addr);
    // Fill our data_boi buffer with some recognizable values.
    for (int i = 0; i < write_size; i++) {
        switch(i % 4) {
            case 0:
                data_boi[i] = 0xde;
                break;
            case 1:
                data_boi[i] = 0xad;
                break;
            case 2:
                data_boi[i] = 0xbe;
                break;
            case 3:
                data_boi[i] = 0xef;
                break;
        }
    }

    // Do the write. 
    if (w25qxx_basic_write(read_addr, data_boi, write_size) > 0) {
        printf("Agh, basic write threw an error :(\n");
    }
    else {
        printf("Write finished without an error!\n");
    }


    /***** Try reading Status Register 1 to get the Busy flag *****/

    /***** Try reading the page you just wrote *****/
    printf("\n--\nReading %d bytes from address 0x%06x.\n", read_size, read_addr);
    // Do the read.
    if (w25qxx_basic_read(read_addr, data_boi, read_size) > 0) {
        printf("Agh, basic read threw and error :(\n");
    } else {
        // Print out the read bytes as a sanity check.
        printf("Read %d bytes! They are as follows (in hex):", read_size);
        for (int i = 0; i < read_size; i++) {
            // Print a -- between each 256 byte page.
            if (i % 256 == 0) {
                printf("\n\nPage %d:", (read_addr + i)/256);
            }
            // Print out the address at the start of every 16 bytes.
            if (i % 16 == 0) {
                printf("\n0x%06x |   ", read_addr + i);
            } else if (i % 8 == 0) {
                // Add an extra space between every 8 bytes.
                printf("  ");
            }
            // Print out our byte.
            printf("%02x ", data_boi[i]);
        }
        printf("\n");
    }

    /***** Try clearing the block *****/

    /***** Try reading Status Register 1 to get the Busy flag again *****/

    /***** Try reading the cleared block *****/

    /***** De-initialize the chip? *****/
    w25qxx_basic_deinit();

    LED_On(0); // indicates SUCCESS
    printf("\nExample Finished\n");
    return E_NO_ERROR;
}
