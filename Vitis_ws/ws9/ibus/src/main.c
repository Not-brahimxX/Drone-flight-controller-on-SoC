#include "ibus.h"
#include "xil_printf.h"

int main() {
    uint16_t ibus_data[IBUS_USER_CHANNELS];
    int status;

    // Initialize the iBus
    status = ibus_init();
    if (status != XST_SUCCESS) {
        xil_printf("Failed to initialize iBus.\r\n");
        return XST_FAILURE;
    }

    while (1) {
        // Read iBus data
        xil_printf("Reading iBus data...\r\n");
        if (ibus_read(ibus_data)) {
            xil_printf("Successfully read iBus data.\r\n");
        } else {
            xil_printf("Failed to read iBus data.\r\n");
        }

        // Add a delay or implement additional logic if needed
    }

    return 0;
}
