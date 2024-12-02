#include "xparameters.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "uart.h"

int main(void)
{
    int Status;

    // Initialiser l'UART Lite
    Status = InitializeUartLite(UARTLITE_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        xil_printf("Erreur lors de l'initialisation de l'UART Lite.\n");
        return XST_FAILURE;
    }

    while (1) {
        // Attendre la réception de données (géré dans l'interruption)
        // Vous pouvez ajouter d'autres traitements ou logique ici

        // Pour cet exemple, nous n'avons pas besoin de faire quelque chose ici
        // parce que la gestion des données reçues est gérée dans le gestionnaire d'interruption
        // et les valeurs sont déjà imprimées dans RecvHandler
    }

    return XST_SUCCESS;
}
