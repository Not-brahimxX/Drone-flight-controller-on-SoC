/*
 * ibus.c
 *
 *  Created on: Feb 4, 2021
 *      Author: mokhwasomssi
 */

#include "xuartlite.h"
#include "xparameters.h"
#include "ibus.h"
#include "xil_printf.h"

// Global Variables
static XUartLite UartLite;
static uint8_t uart_rx_buffer[IBUS_LENGTH] = {0};
static uint8_t fail_safe_flag = 0;

// Main Functions
int ibus_init()
{
    int Status;

    // Initialize the UART Lite driver
    Status = XUartLite_Initialize(&UartLite, XPAR_UARTLITE_0_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        // Handle error
        xil_printf("UART Lite initialization failed.\r\n");
        return XST_FAILURE;
    }
    xil_printf("UART Lite initialized successfully.\r\n");
    return XST_SUCCESS;
}

int ibus_read(uint16_t* ibus_data)
{
    int ReceivedCount = 0;

    // Poll until the required number of bytes are received
    while (ReceivedCount < IBUS_LENGTH) {
        ReceivedCount += XUartLite_Recv(&UartLite, &uart_rx_buffer[ReceivedCount], IBUS_LENGTH - ReceivedCount);
    }

    xil_printf("Received data: ");
    for (int i = 0; i < IBUS_LENGTH; i++) {
        xil_printf("%02X ", uart_rx_buffer[i]);
    }
    xil_printf("\r\n");

    if (!ibus_is_valid()) {
        xil_printf("Invalid iBus data.\r\n");
        return 0;
    }

    if (!ibus_checksum()) {
        xil_printf("Checksum failed.\r\n");
        return 0;
    }

    for (int i = 0; i < IBUS_USER_CHANNELS; i++) {
        ibus_data[i] = uart_rx_buffer[2 + i * 2] | (uart_rx_buffer[3 + i * 2] << 8);
    }

    // Print the channel values
    for (int i = 0; i < IBUS_USER_CHANNELS; i++) {
        xil_printf("Channel %d: %d\r\n", i + 1, ibus_data[i]);
    }

    return 1;
}

// Sub Functions
int ibus_is_valid()
{
    // Check if the received data is iBus data
    return (uart_rx_buffer[0] == IBUS_LENGTH && uart_rx_buffer[1] == IBUS_COMMAND40);
}

int ibus_checksum()
{
    uint16_t checksum_cal = 0xffff;
    uint16_t checksum_ibus;

    for (int i = 0; i < 30; i++) {
        checksum_cal -= uart_rx_buffer[i];
    }

    checksum_ibus = uart_rx_buffer[31] << 8 | uart_rx_buffer[30]; // Extract checksum value from iBus data
    return (checksum_ibus == checksum_cal);
}

/**
 * @note FS-A8S doesn't have a fail-safe feature, so we implement a software fail-safe.
 */
void ibus_soft_failsafe(uint16_t* ibus_data, uint8_t fail_safe_max)
{
    fail_safe_flag++;

    if (fail_safe_flag < fail_safe_max)
        return;

    // Clear iBus data
    for (int i = 0; i < IBUS_USER_CHANNELS; i++)
        ibus_data[i] = 0;

    // Clear iBus buffer
    for (int j = 0; j < IBUS_LENGTH; j++)
        uart_rx_buffer[j] = 0;

    fail_safe_flag = 0;
}

/**
 * @note This function should be called in the UART receive complete callback.
 */
void ibus_reset_failsafe()
{
    fail_safe_flag = 0; // Reset fail-safe flag
}
