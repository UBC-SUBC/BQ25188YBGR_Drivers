#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include "stdbool.h"
#include "stdint.h"

#ifndef INC_BQ25188_H
#define INC_BQ25188_H

#define BQ25188_STAT0       0x00 // Charger Status
#define BQ25188_STAT1       0x01 // Charger Status and Faults
#define BQ25188_FLAG0       0x02 // Charger Flags Register
#define BQ25188_VBAT_CTRL   0x03 // Battery Voltage Control
#define BQ25188_ICHG_CTRL   0x04 // Fast Charge Current Control
#define BQ25188_CHARGECTRL0 0x05 // Charger Control 0
#define BQ25188_CHARGECTRL1 0x06 // Charger Control 1
#define BQ25188_IC_CTRL     0x07 // IC Control
#define BQ25188_TMR_ILIM    0x08 // Timer and Input Current Limit Control
#define BQ25188_SHIP_RST    0x09 // Shipmode, Reset and Pushbutton Control
#define BQ25188_SYS_REG     0x0A // SYS Regulation Voltage Control
#define BQ25188_TS_CONTROL  0x0B // TS Control
#define BQ25188_MASK_ID     0x0C // MASK and Device ID

#endif /* INC_BQ25188_H */