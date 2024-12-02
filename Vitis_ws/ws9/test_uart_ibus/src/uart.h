#ifndef UART_H_
#define UART_H_

#include "xuartlite.h"

#define UARTLITE_DEVICE_ID  XPAR_UARTLITE_0_DEVICE_ID
#define RECV_BUFFER_SIZE    32
#define IBUS_FRAME_SIZE     32
#define IBUS_CHANNEL_COUNT  4

extern XUartLite UartLite;
extern u8 RecvBuffer[RECV_BUFFER_SIZE];
extern u16 IBusChannels[IBUS_CHANNEL_COUNT];

int InitializeUartLite(u16 DeviceId);
void RecvHandler(void *CallBackRef, unsigned int EventData);
void DecodeIBusFrame(u8 *frame);

#endif /* UART_H_ */
