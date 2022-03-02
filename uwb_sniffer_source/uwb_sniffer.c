/*! ----------------------------------------------------------------------------
 *  @file    uwb_sniffer.c
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
#include <float.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <config_options.h>
#include <main.h>
#include <shared_functions.h>
#include "sensniff.h"
#include "uart_send.h"
#include "uwb_sniffer.h"
#include "deca_probe_interface.h"


extern void test_run_info(unsigned char *data);
extern void sensniff_rx_frame(uint8_t* rx_buffer, int frame_len);
struct Sensniff_command* sensniff_receive();
extern void sensniff_reply(unsigned char *data, uint16_t data_length,  uint8_t cmd);


extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;

/* Example application name */
#define APP_NAME "UWB Sniffer v1.0"

// Nearby Interaction config
#define PCODE 3 // Preamble code
#define CHANNEL 9
#define STSMODE DWT_STS_MODE_1
#define STSLEN DWT_STS_LEN_64 // Length defined in 802.15.4z
#define SFD_TIMEOUT 256 + 64 // (maximum expected preamble length)
#define SFD_MODE 3 /*Should be 0 or 3 */



static dwt_config_t config = {
	    9,                  /* Channel number. */
	    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
	    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
	    12,                  /* TX preamble code. Used in TX only. */
	    12,                  /* RX preamble code. Used in RX only. */
	    0,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
		DWT_BR_6M8,         /* Data rate. */
	    DWT_PHRMODE_STD,    /* PHY header mode. */
	    DWT_PHRRATE_STD,    /* PHY header rate. */
		64 + 64 + 1,  		/* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	    DWT_STS_MODE_1,     /* Mode 1 STS enabled */
	    DWT_STS_LEN_64,    /* (STS length  in blocks of 8) - 1*/
	    DWT_PDOA_M0         /* PDOA mode off */
	};



/* Index to the start of the payload data in the TX frame */
#define FRAME_PAYLOAD_IDX 9

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
/*Delay between the response frame and final frame. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS (100 + CPU_COMP)

#define REPORT_TIMESTAMP 1


/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;



extern int use_sensniff;

#if REPORT_TIMESTAMP
uint64_t rx_last = UINT32_MAX;
uint64_t rx_total = 0;
#endif

/**
 * Application entry point.
 */
int sniff_uwb(void)
{

	configure();

	while (1) {
		receive();
	}

	return DWT_SUCCESS;
}


void configure() {
    /* Display application name on UART. */
	log_uart(APP_NAME);

    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    uint8_t idle = dwt_checkidlerc();
    while (!idle) /* Need to make sure DW IC is in IDLE_RC before proceeding */
       {
    	idle = dwt_checkidlerc();
       };

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1)
        { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
        * Note, in real low power applications the LEDs should not be used. */
    dwt_setleds(DWT_LEDS_ENABLE) ;

    /* Configure DW IC */
    if(dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    if(config.chan == 5)
    {
        dwt_configuretxrf(&txconfig_options);
    }
    else
    {
        dwt_configuretxrf(&txconfig_options_ch9);
    }

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help diagnostics, and also TX/RX LEDs */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void receive() {

	/* Activate reception immediately. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
	while (!((status_reg = dwt_readsysstatuslo()) & (DWT_INT_RXFR_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	    	    { };

    /*
     * Check for a good frame and STS count.
     */
    if (!(status_reg & DWT_INT_RXFCG_BIT_MASK))
    {
    	handle_error(status_reg);

    }

    handle_receive();
    status_reg = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

#if REPORT_TIMESTAMP
/**
 * Reports a timestamp in uus when the last frame has been received. The timestamp starts at 0 for the first frame.
 */
uint64_t get_timestamp() {
	uint64_t rx_now_dtu = get_rx_timestamp_u64();
	uint64_t rx_now_uus = (rx_now_dtu / UUS_TO_DWT_TIME);


	if (rx_last == UINT32_MAX) {
		rx_last = rx_now_uus;
	}else {
		uint64_t rx_diff = 0;
		if (rx_now_uus >= rx_last) {
			 rx_diff = rx_now_uus - rx_last;
		}else {
			rx_diff = DWTU_MAX - rx_last + rx_now_uus;
		}

		rx_total += rx_diff;
		rx_last = rx_now_uus;
	}

	return rx_total;
}
#endif

void handle_receive() {

    uint16_t frame_len = dwt_getframelength();
	char textBuffer[100];
	snprintf(textBuffer, 100, "Frame length is: %d", frame_len);
	log_uart(textBuffer);



    if (frame_len > 0 && frame_len <= FRAME_LEN_MAX) {
    	//Try to read the rx buffer
    	 //if (USING_CRC_LEN) frame_len-=FCS_LEN; /* No need to read the CRC. This example uses CRC */
		dwt_readrxdata(rx_buffer, frame_len, 0);


#if REPORT_TIMESTAMP
		uint64_t timestamp = get_timestamp();
		sensniff_rx_frame_timestamp(rx_buffer, frame_len, timestamp);
#else
		sensniff_rx_frame(rx_buffer, frame_len);
#endif

    }else if (frame_len == 0 && config.stsMode == DWT_STS_MODE_ND) {
    	// Send a 0 frame when in STS Mode 3
    	uint8_t empty_buffer[1] = {0};

#if REPORT_TIMESTAMP
		uint64_t timestamp = get_timestamp();
		sensniff_rx_frame_timestamp(empty_buffer, 1, timestamp);
#else
		sensniff_rx_frame(empty_buffer, 1);
#endif
    }
}


void handle_error(uint32_t status_reg) {
	 /* Clear RX error events in the DW IC status register. */
	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);

	// A reception error occurred   SYS_STATUS_ALL_RX_ERR
	if (status_reg & SYS_STATUS_ALL_RX_ERR) {
		log_uart(" SYS_STATUS_ALL_RX_ERR ");
	}

	if (status_reg & DWT_INT_RXPHE_BIT_MASK) {
		/* Mask receiver PHY header error event
		 *  */
		log_uart(" SYS_STATUS_RXPHE_BIT_MASK "); //This error is received when: preamble length to more than 256, pac = 3 and STS mode = 1
	}

	if (status_reg & DWT_INT_RXFCE_BIT_MASK) {
		/* Receiver FCS (Frame Check Sequence) Error */
		log_uart(" SYS_STATUS_RXFCE_BIT_MASK "); //This error is received when setting the expected preamble length to more than 256 and pac = 3 and STS = mode 3
	}

	if (status_reg & DWT_INT_RXFSL_BIT_MASK) {
		/* Mask receiver Reed Solomon Frame Sync Loss event */
		log_uart(" SYS_STATUS_RXFSL_BIT_MASK ");
	}

	if (status_reg & DWT_INT_RXSTO_BIT_MASK) {
		/* Mask Receive SFD timeout event.
		 * The timeout value is set too low   */
		log_uart(" SYS_STATUS_RXSTO_BIT_MASK "); // This error occurs. We don't know what this error actually  means
	}

	if (status_reg & DWT_INT_ARFE_BIT_MASK) {
		log_uart(" SYS_STATUS_ARFE_BIT_MASK ");
	}

	if (status_reg & DWT_INT_CIAERR_BIT_MASK) {
		log_uart(" SYS_STATUS_CIAERR_BIT_MASK ");
	}

}

