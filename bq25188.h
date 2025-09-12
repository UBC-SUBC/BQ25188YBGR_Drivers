#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include "stdbool.h"
#include "stdint.h"

#ifndef INC_BQ25188_H
#define INC_BQ25188_H

#define BQ25188_STAT0       0x00 // Charger Status
#define BQ25188_STAT1       0x01 // Charger Status and Faults
#define BQ25188_FLAG0       0x02 // Charger Flags Register
#define BQ25188_VBAT_CTRL   0x03 //  Battery Voltage Control
#define BQ25188_ICHG_CTRL   0x04 // Fast Charge Current Control
#define BQ25188_CHARGECTRL0 0x05 // Charger Control 0
#define BQ25188_CHARGECTRL1 0x06 // Charger Control 1
#define BQ25188_IC_CTRL     0x07 // IC Control
#define BQ25188_TMR_ILIM    0x08 // Timer and Input Current Limit Control
#define BQ25188_SHIP_RST    0x09 // Shipmode, Reset and Pushbutton Control
#define BQ25188_SYS_REG     0x0A // SYS Regulation Voltage Control
#define BQ25188_TS_CONTROL  0x0B // TS Control
#define BQ25188_MASK_ID     0x0C // MASK and Device ID

enum bq25188_chg_stat {
    BQ25188_CHG_NOT_CHARGING     = 0b00,
    BQ25188_CHG_CONSTANT_CURRENT = 0b01,
    BQ25188_CHG_CONSTANT_VOLTAGE = 0b10,
    BQ25188_CHG_CHARGE_DONE      = 0b11,
};

struct bq25188_stat0 {
    uint8_t ts_open_stat;
    enum bq25188_chg_stat chg_stat;
    uint8_t ilim_active_stat;
    uint8_t vdppm_active_stat;
    uint8_t vindpm_active_stat;
    uint8_t thermreg_active_stat;
    uint8_t vin_pgood_stat;
};

enum bq25188_ts_stat {
    BQ25188_TS_NORMAL                   = 0b00      
    BQ25188_TS_CHARGING_SUSPENDED       = 0b01,
    BQ25188_TS_CHARGING_CURRENT_REDUCED = 0b10,
    BQ25188_TS_CHARGING_VOLTAGE_REDUCED = 0b11,
};

struct bq25188_stat1 {
    uint8_t vin_ovp_stat;
    uint8_t buvlo_stat;
    enum bq25188_ts_stat ts_stat;
    uint8_t safety_tmr_fault_flag;
    uint8_t wake1_flag;
    uint8_t wake2_flag;
};

struct bq25188_flag0 {
    bool ts_fault;
    bool ilim_active_flag;
    bool vdppm_active_flag;
    bool vindpm_active_flag;
    bool thermeg_active_flag;
    bool vin_ovp_fault_flag;
    bool buvlo_fault_flag;
    bool bat_ocp_fault;
}

// struct for vbat_ctrl

// struct for ichg_ctrl

// enum for iprecgh

// enum for iterm

// enum vindpm

// struct chargectrl0

// enum ibat_ocp

// enum buvlo

// struct chargectrl1

// enum safety timer

// enum watchdog sel

// struct ic ctrl

// enum mr_lpress

// enum autowake

// enum ilim

// struct tmr_ilim

// enum en_rst_ship

// enum pb lpress action

// struct ship_rst

// enum sys_reg_ctrl

// enum sys_mode

// struct sys_reg

// enum ts_hot

// enum  ts_cold

// struct ts_control

// struct  mask id

// struct watchdog setup

// struct shipmode setuo

// struct data


// global function delcarations




#endif /* INC_BQ25188_H */