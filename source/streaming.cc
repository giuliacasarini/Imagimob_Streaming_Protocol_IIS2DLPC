/******************************************************************************
* File Name:   streaming.c
*
* Description: This file contains functions for streaming data over a serial
*              interface. It supports using either USB CDC (default) or UART
*              over the debug port. See comment below for how to switch to the
*              debug port UART.
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
******************************************************************************/

#include "streaming.h"


#include "cybsp.h"
#include "cyhal_uart.h"
#include "cy_retarget_io.h"



/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_BAUD_RATE   (1000000u)
/* NOTE: The debug UART may not support standard baud rates like 921600 due to
 * its clock configuration, so consider this before changing. It should work
 * well with 500000 and 1000000. I'm not sure why, but possibly because it's
 * based on the 32 MHz MPU clock, so there's no integer divider for e.g.

/* Should be enough for a complete protocol command. */

/*******************************************************************************
* Local Variables
*******************************************************************************/




/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: streaming_uart_event_handler
********************************************************************************
* Summary:
*  UART TX and RX completion handler.
*
*******************************************************************************/


/*******************************************************************************
* Function Name: streaming_init
********************************************************************************
* Summary:
*  Initializes the streaming interface. Call this once before using any other
*  function in this file.
*
*******************************************************************************/

//#define CYBSP_DEBUG_UART_TX P5_1
//#define CYBSP_DEBUG_UART_RX P5_0

void streaming_init()
{

	cy_rslt_t result;
	result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX,
	                                    CYBSP_DEBUG_UART_RX,
	                                    CYBSP_DEBUG_UART_CTS,
	                                    CYBSP_DEBUG_UART_RTS,
										UART_BAUD_RATE );

}

/*******************************************************************************
* Function Name: streaming_receive
********************************************************************************
* Summary:
*  Reads bytes from the streaming interface into the given buffer if available.
*  This function may block until a preceding UART operation is complete.
**
* Parameters:
*  data: pointer to buffer where data will be stored
*  size: buffer size
*
* Return:
*  The number of bytes received; 0 if no bytes were available.
*
*******************************************************************************/
size_t streaming_receive(void* data, size_t size)
{


    /* Do read */

	uint32_t numval;
    numval = cyhal_uart_readable(&cy_retarget_io_uart_obj);
    for (uint32_t i=0; i<numval;i++)
    	cyhal_uart_getc(&cy_retarget_io_uart_obj,(uint8_t*)data + i, 0);



     return numval;
}

/*******************************************************************************
* Function Name: streaming_send
********************************************************************************
* Summary:
*  Sends the given bytes to the streaming interface. This function may block
*  until a preceding UART operation is complete.
*
* Parameters:
*  data: pointer to data to send
*  size: number of bytes to send
*
*******************************************************************************/
void streaming_send(const void* data, size_t size)
{	size_t remaining;
	remaining = size;
	/* Ensure UART available */
	while (remaining)
		{
		size_t sizeToSend;
		sizeToSend = remaining;
		cyhal_uart_write(&cy_retarget_io_uart_obj, (void*) data + (size-remaining) , &sizeToSend);
		remaining -= sizeToSend;
		}

}

