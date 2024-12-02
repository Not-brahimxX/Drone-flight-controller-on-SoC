#ifndef __IBUS_H__
#define __IBUS_H__

#include "xuartlite.h"
#include <stdbool.h>
#include <stdint.h>

/* User configuration */
#define IBUS_UART                XPAR_UARTLITE_0_DEVICE_ID
#define IBUS_USER_CHANNELS       6    // Use 6 channels

/* Defines */
#define IBUS_LENGTH              0x20 // 32 bytes
#define IBUS_COMMAND40           0x40 // Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNELS        14

/* Main Functions */
int ibus_init();
int ibus_read(uint16_t* ibus_data);

/* Sub Functions */
int ibus_is_valid();
int ibus_checksum();
void ibus_update(uint16_t* ibus_data);
void ibus_soft_failsafe(uint16_t* ibus_data, uint8_t fail_safe_max);
void ibus_reset_failsafe();

#endif /* __IBUS_H__ */
