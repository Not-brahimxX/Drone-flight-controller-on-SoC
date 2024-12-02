#include "uart.h"
#include "xparameters.h"
#include "xil_exception.h"
#include "xil_printf.h"

XUartLite UartLite;
u8 RecvBuffer[RECV_BUFFER_SIZE];
u16 IBusChannels[IBUS_CHANNEL_COUNT];

int InitializeUartLite(u16 DeviceId)
{
    int Status;

    Status = XUartLite_Initialize(&UartLite, DeviceId);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XUartLite_SetRecvHandler(&UartLite, RecvHandler, &UartLite);
    XUartLite_EnableInterrupt(&UartLite);

    return XST_SUCCESS;
}

void DecodeReceivedData(u8 *data)
{
    // Fonction pour d�coder les donn�es re�ues depuis Arduino
    // Vous devez adapter cette fonction pour votre protocole sp�cifique
    // Dans cet exemple, nous supposons que les donn�es sont envoy�es sous forme de cha�ne
    // et s�par�es par des virgules
    char *token;
    token = strtok((char *)data, ",");
    int i = 0;
    while (token != NULL && i < IBUS_CHANNEL_COUNT) {
        IBusChannels[i++] = atoi(token);
        token = strtok(NULL, ",");
    }
}

void RecvHandler(void *CallBackRef, unsigned int EventData)
{
    // Gestionnaire d'interruption de r�ception UART
    XUartLite_Recv(&UartLite, RecvBuffer, RECV_BUFFER_SIZE);

    // D�coder les donn�es re�ues
    DecodeReceivedData(RecvBuffer);

    // Afficher les valeurs re�ues pour le d�bogage
    for (int i = 0; i < IBUS_CHANNEL_COUNT; i++) {
        xil_printf("Value %d: %d\n", i+1, IBusChannels[i]);
    }
}
