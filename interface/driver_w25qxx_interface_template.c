/*********************************************************************
 * @file      driver_w25qxx_interface_template.c
 * @brief     driver w25qxx interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-07-15
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
*********************************************************************/

/***** Includes *****/ // Copied from SPI_MasterSlave
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

// Other includes.
#include <stdlib.h>
#include "driver_w25qxx_interface.h"
#include "mxc_delay.h"

/***** Definitions *****/
// Taken from SPI_MasterSlave example.
//#define DATA_LEN 1024 // Words. Used for transaction size, but this code provides arguments for it I think.
#define DATA_SIZE 8
//#define VALUE 0xFF // Unused
#define SPI_SPEED 100000 // Bit Rate (Max.: 1,850,000)

#define SPI_MASTER MXC_SPI0 //MXC_SPIMSS // Set SPI0 to be the master
#define SPI_MASTER_SSIDX 0


/***************************************
 * @brief  interface spi qspi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi qspi init failed
 * @note   none
***************************************/
uint8_t w25qxx_interface_spi_qspi_init(void)
{
    /***** Configure master (SPIMSS) *****/
    if (MXC_SPI_Init(SPI_MASTER, 1, 0, 1, 0, SPI_SPEED) != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return 1;
    }
    if (MXC_SPI_SetDataSize(SPI_MASTER, DATA_SIZE) != E_NO_ERROR) {
        printf("\nSPI SET DATASIZE ERROR\n");
        return 1;
    }
    if (MXC_SPI_SetWidth(SPI_MASTER, SPI_WIDTH_STANDARD) != E_NO_ERROR) {
        printf("\nSPI SET WIDTH ERROR\n");
        return 1;
    }
    return 0;
}


/*****************************************
 * @brief  interface spi qspi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi qspi deinit failed
 * @note   none
*****************************************/
uint8_t w25qxx_interface_spi_qspi_deinit(void)
{
    if (MXC_SPI_Shutdown(SPI_MASTER) != E_NO_ERROR) {
        printf("\nSPI SHUTDOWN ERROR\n");
        return 1;
    }
    return 0;
}


/**************************************************************
 * @brief      interface spi qspi bus write read
 * @param[in]  instruction is the sent instruction
 * @param[in]  instruction_line is the instruction phy lines
 * @param[in]  address is the register address
 * @param[in]  address_line is the address phy lines
 * @param[in]  address_len is the address length
 * @param[in]  alternate is the register address
 * @param[in]  alternate_line is the alternate phy lines
 * @param[in]  alternate_len is the alternate length
 * @param[in]  dummy is the dummy cycle
 * @param[in]  *in_buf points to a input buffer
 * @param[in]  in_len is the input length
 * @param[out] *out_buf points to a output buffer
 * @param[in]  out_len is the output length
 * @param[in]  data_line is the data phy lines
 * @return     status code
 *             - 0 success
 *             - 1 write read failed
 * @note       none
**************************************************************/
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                             uint32_t address, uint8_t address_line, uint8_t address_len,
                                             uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                             uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                             uint8_t *out_buf, uint32_t out_len, uint8_t data_line)
{
    // This was originally in `int main(void)`. Am I allowed to put it here?
    // Or maybe we cast the out_buf to this thing?
    mxc_spi_req_t master_req;

    // Ok, actually, it looks like I will have to.. construct the tx_data myself?
    // Aight, so.. I'm not entirely sure what "phy lines" means, but.. looks like the basic procedure is:
    // Drive the CS low,
    // instruction code (generally 8 bits?)
    // address (generally 24-bits if it exists, otherwise alternate if that exists)
    // dummy number of clock cycles (values in this dummy doesn't matter)
    // in data (if it exists) -- originally I thought this should've been out data, but it isn't.

    // So we just count all of these up then?
    // I think.. all the len values are given in terms of bytes (i.e. word sizes) while the dummy is given in bits?
    // I'm hoping dummy is always 8 (or some multiple of 8) cuz idk how to do this otherwise..
    // Also, seems like one usage is for the instruction to be all 0s, in which case we do not want to include it >.>
    uint32_t tx_size = (instruction > 0) ? 1 : 0;
    tx_size += address_len + alternate_len + (dummy/DATA_SIZE) + in_len + out_len;

    // I assume it's kosher to malloc buffers for both tx and rx? 
    // As opposed to just giving master_req pointers from the argument?
    // I wonder if we're able to use the same pointer for both TX and RX in order to save some space?
    uint8_t * master_tx = (uint8_t*) malloc(tx_size*sizeof(uint8_t));
    uint8_t * master_rx = (uint8_t*) malloc(tx_size*sizeof(uint8_t)); //(out_len > 0) ? (uint8_t*) malloc((out_len)*sizeof(uint8_t)) : NULL;

    // Check if the mallocs worked.
    if (master_tx == NULL) {
        printf("Failed to allocate memory for master_tx.\n");
        exit(0);
    }
    if (master_rx == NULL) { //((out_len > 0) && (master_rx == NULL)) {
        printf("Failed to allocate memory for master_rx.\n");
        exit(0);
    }

    // If memory was successfully allocated, we can fill in master_tx as applicable.
    uint32_t cur_ind = 0;
    // Instruction code
    if (instruction > 0) {
        master_tx[cur_ind] = instruction;
        cur_ind++;
    }
    // Address
    if (address_len > 0) {
        memcpy(&(master_tx[cur_ind]), &address, address_len);
        cur_ind += address_len;
    }
    // Alternate
    if (alternate_len > 0) {
        memcpy(&(master_tx[cur_ind]), &alternate, alternate_len);
        cur_ind += alternate_len;
    }
    // Dummy
    cur_ind += dummy/DATA_SIZE;
    // In data
    if (in_len > 0) {
        memcpy(&(master_tx[cur_ind]), in_buf, in_len);
        cur_ind += in_len;
    }
    // Out data in order to get a clock signal for the RX.
    // Not sure if this is needed though..
    cur_ind += out_len;

    // Confirm we did our math correct lmao.
    if (cur_ind != tx_size) {
        printf("Tried to construct a transaction of %d bytes but made one with %d bytes instead..\n", tx_size, cur_ind);
        return 1;
    }

    /***** Initialize Transaction Parameters *****/
    master_req.spi = SPI_MASTER;
    master_req.txData = (uint8_t *) master_tx;
    master_req.rxData = (uint8_t *) master_rx;
    master_req.txLen = tx_size; // Words. Number of bytes to be sent from txData.
    master_req.rxLen = tx_size; //out_len; // Words. Number of bytes to be stored in rxData.
    master_req.ssIdx = 0;
    master_req.ssDeassert = 1;
    master_req.txCnt = 0; // Number of bytes actually transmitted from txData.
    master_req.rxCnt = 0; // Number of bytes stored in rxData.
    master_req.completeCB = NULL; // pointer to function called when transaction is complete.

    /***** Perform Transaction *****/
    MXC_SPI_MasterTransaction(&master_req);

    // Confirm that we sent the number of bytes we wanted to.
    if (master_req.txCnt != tx_size) {
        printf("Tried to send %d bytes but sent %d bytes instead.\n", tx_size, master_req.txCnt);
        return 1;
    }

    // Confirm that we received the number of bytes we wanted to.
    if (master_req.rxCnt != tx_size) { //out_len) {
        printf("Tried to receive %d bytes but received %d bytes instead.\n", out_len, master_req.rxCnt);
        return 1;
    }

    // Copy over the received data if it's successful.
    memcpy(out_buf, &(master_rx[tx_size-out_len]), out_len);

    // Free the mallocs when we're done?
    free(master_tx);
    free(master_rx);

    return 0;
}


/**********************************
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
**********************************/
void w25qxx_interface_delay_ms(uint32_t ms)
{
    MXC_Delay(ms*1000);
}


/**
 * @brief     interface delay us
 * @param[in] us
 * @note      none
 */
void w25qxx_interface_delay_us(uint32_t us)
{
    MXC_Delay(us);
}


/******************************************
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
******************************************/
void w25qxx_interface_debug_print(const char *const fmt, ...)
{
    printf(fmt);
}
