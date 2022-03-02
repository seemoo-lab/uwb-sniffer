/*! ----------------------------------------------------------------------------
 *  @file    uwb_sniffer.h
 *  @brief   A UWB frame sniffer that receives frames and sends the content and timestamp to the host over UART. Code based on a DWM3000 example by Decawave / Qorvo
 *
 * @attention
 *
 * Copyright 2021 - 2022 TU Darmstadt, ETH Zurich
 *
 *
 * @author Alexander Heinrich
 */

#include <string.h>

void handle_receive();
void handle_error(uint32_t status_reg);
void receive();
void configure();
int sniff_uwb(void);

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2e^6s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 63898
#define NS_TO_DWT_TIME 63.898
#define DWTU_MAX ((0xffffffffff / 63898))

/******************************************************************************
* @brief Bit definitions for register RX_FINFO
*
* RX_FINFO contains frame information. It is constructed from the PHR
**/
#define RX_FINFO_ID                          0x4c                  // REG ID
#define RXFLEN_MASK    0x0000007FUL    /* Receive Frame Length (0 to 127) */

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read 32-bit value from the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 32 bit register value
 */
uint32_t dwt_read32bitoffsetreg(int regFileID, int regOffset);
#define dwt_read32bitreg(addr)     dwt_read32bitoffsetreg(addr,0)
