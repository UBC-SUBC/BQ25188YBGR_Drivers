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
    BQ25188_CHG_NOT_CHARGING     = 0x00,
    BQ25188_CHG_CONSTANT_CURRENT = 0x01,
    BQ25188_CHG_CONSTANT_VOLTAGE = 0x02,
    BQ25188_CHG_CHARGE_DONE      = 0x03,
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
    BQ25188_TS_NORMAL                   = 0x00      
    BQ25188_TS_CHARGING_SUSPENDED       = 0x01,
    BQ25188_TS_CHARGING_CURRENT_REDUCED = 0x02,
    BQ25188_TS_CHARGING_VOLTAGE_REDUCED = 0x03,
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

struct bq25188_vbat_ctrl {
    uint8_t vbatreg;
};

struct bq25188_ichg_ctrl {
    uint8_t chg_dis;
    uint8_t ichg
};

enum bq25188_iprechg {
    BQ25188_IPRECHG_2x = 0x00,
    BQ25188_IPRECHG_1x = 0x01,
};

enum bq25188_iterm {
    BQ25188_ITERM_DISABLE = 0x00, 
    BQ25188_ITERM_5       = 0x01,
    BQ25188_ITERM_10      = 0x02,
    BQ25188_ITERM_20      = 0x03,
};

enum bq25188_vindpm {
    BQ25188_VINDPM_4_2     = 0x00,
    BQ25188_VINDPM_4_5     = 0x01,
    BQ25188_VINDPM_4_7     = 0x02,
    BQ25188_VINDPM_DISABLE = 0x03,
};

struct bq25188_chargectrl0 {
    enum bq25188_iprechg iprechg;
    enum bq25188_iterm iterm;
    enum bq25188_vindpm vindpm;
    uint8_t therm_reg;
}

enum bq25188_ibat_ocp {
    BQ25188_IBAT_OCP_500mA  = 0x00,
    BQ25188_IBAT_OCP_1000mA = 0x01,
    BQ25188_IBAT_OCP_1500mA = 0x02,
    BQ25188_IBAT_OCP_3000mA = 0x03,
};

enum bq25188_buvlo {
    BQ25188_BUVLO_3_0V_A = 0x00,
    BQ25188_BUVLO_3_0V_B = 0x01,
    BQ25188_BUVLO_3_0V   = 0x02,
    BQ25188_BUVLO_2_8V   = 0x03,
    BQ25188_BUVLO_2_6V   = 0x04,
    BQ25188_BUVLO_2_4V   = 0x05,
    BQ25188_BUVLO_2_2V   = 0x06,
    BQ25188_BUVLO_2_0V   = 0x07,
};


struct bq25188_chargectrl1 {
    enum bq25188_ibat_ocp ibat_ocp;
    enum bq25188_buvlo buvlo;
    uint8_t chg_status_int_mask;
    uint8_t ilim_int_mask;
    uint8_t vindpm_int_mask;
};

enum bq25188_safety_timer {
    BQ25188_SAFETY_TIMER_3_HR    = 0x00,
    BQ25188_SAFETY_TIMER_6_HR    = 0x01,
    BQ25188_SAFETY_TIMER_12_HR   = 0x02,
    BQ25188_SAFETY_TIMER_DISABLE = 0x03,
};

enum bq25188_watchdog_sel {
    BQ25188_WD_SEL_160s_DEFAULT  = 0x00,
    BQ25188_WD_SEL_160s_HW_RESET = 0x01,
    BQ25188_WD_SEL_40S_HW_RESET  = 0x02,
    BQ25188_WD_SEL_DISABLE       = 0x03,
};

struct bq25188_ic_ctrl {
    uint8_t ts_en;
    uint8_t vlowv_sel;
    uint8_t vrch_0;
    uint8_t tmr_en_2x;
    enum bq25188_safety_timer safety_timer;
    enum bq25188_watchdog_sel watchdog_sel;
}

enum bq25188_mr_lpress {
    BQ25188_MR_LPRESS_5s  = 0X00,
    BQ25188_MR_LPRESS_10s = 0x01,
    BQ25188_MR_LPRESS_15s = 0x02,
    BQ25188_MR_LPRESS_20s = 0x03,
};

enum bq25188_autowake {
    BQ25188_AUTOWAKE_0_5s = 0x00,
    BQ25188_AUTOWAKE_1s   = 0x01,
    BQ25188_AUTOWAKE_2s   = 0x02,
    BQ25188_AUT0WAKE_4s   = 0x03,
};

enum bq25188_ilim {
    BQ25188_ILIM_50mA   = 0x00,
    BQ25188_ILIM_100mA  = 0x01,
    BQ25188_ILIM_200mA  = 0x02,
    BQ25188_ILIM_300mA  = 0x03,
    BQ25188_ILIM_400mA  = 0x04,
    BQ25188_ILIM_500mA  = 0x05,
    BQ25188_ILIM_665mA  = 0x06,
    BQ25188_ILIM_1050mA = 0X07,
}; 

struct bq25188_tmr_ilim {
    enum bq25188_mr_lpress mr_lpress;
    uint8_t mr_reset_vin;
    enum bq25188_autowake autowake;
    enum bq25188_ilim ilim;
}

enum bq25188_en_rst_ship {
    BQ25188_EN_RST_SHIP_DO_NTHNG       = 0x00,
    BQ25188_EN_RST_SHIP_EN_SHTDWN_MODE = 0x01,
    BQ25188_EN_RST_SHIP_EN_SHIP_MODE   = 0X02,
    BQ25188_EN_RST_SHIP_HW_RST         = 0x03,
};

enum bq25188_pb_lpress_action {
    BQ25188_PB_LPRESS_DO_NTHNG       = 0x00,
    BQ25188_PB_LPRESS_HW_RST         = 0x01,
    BQ25188_PB_LPRESS_EN_SHIP_MODE   = 0x02,
    BQ25188_PB_LPRESS_EN_SHTDWN_MODE = 0x03,
};


struct bq25188_ship_rst {
    uint8_t reg_rst;
    enum bq25188_en_rst_ship en_rst_ship;
    enum bq25188_pb_lpress_action pb_lpress_action;
    uint8_t wake1_tmr;
    uint8_t wake2_tmr;
    uint8_t en_push;
}

enum bq25188_sys_reg_ctrl {
    BQ25188_SYS_REG_CTRL_BATTERY_TRACKING = 0x00,
    BQ25188_SYS_REG_CTRL_4_4V             = 0x01,
    BQ25188_SYS_REG_CTRL_4_5V             = 0x02,
    BQ25188_SYS_REG_CTRL_4_6V             = 0x03,
    BQ25188_SYS_REG_CTRL_4_7V             = 0x04,
    BQ25188_SYS_REG_CTRL_4_8V             = 0x05,
    BQ25188_SYS_REG_CTRL_4_9V             = 0x06,
    BQ25188_SYS_REG_CTRL_PASS_THROUGH     = 0x07,
};

enum bq25188_sys_mode {
    BQ25188_SYS_MODE_VIN_VBAT     = 0x00, 
    BQ25188_SYS_MODE_VBAT         = 0x01,
    BQ25188_SYS_MODE_DIS_FLOAT    = 0x02,
    BQ25188_SYS_MODE_DIS_PULLDOWN = 0x03,
};

struct bq25188_sys_reg {
    enum bq25188_sys_reg_ctrl sys_reg_ctrl;
    uint8_t pg_gpo;
    enum bq25188_sys_mode sys_mode;
    uint8_t watchdog_15s_enable;
    uint8_t vdppm_dis;
};

enum bq25188_ts_hot {
    BQ25188_TS_HOT_60C = 0x00,
    BQ25188_TS_HOT_65C = 0x01,
    BQ25188_TS_HOT_50C = 0x02,
    BQ25188_TS_HOT_45C = 0x03,
};

enum bq25188_ts_cold {
    BQ25188_TS_COLD_0C  = 0x00,
    BQ25188_TS_COLD_3C  = 0x01,
    BQ25188_TS_COLD_5C  = 0x02,
    BQ25188_TS_COLD_N3C = 0x03,
};


struct bq25188_ts_control {
    enum bq25188_ts_hot ts_hot;
    enum bq25188_ts_cold ts_cold;
    uint8_t ts_warm;
    uint8_t ts_cool;
    uint8_t ts_ichg;
    uint8_t ts_vrcg;
};

struct bq25188_mask_id {
    uint8_t ts_int_mask;
    uint8_t treg_int_mask;
    uint8_t bat_int_mask;
    uint8_t pg_int_mask;
    uint8_t device_id;
};

/* ----- End of Internal Regs ----- */



// global function delcarations




#endif /* INC_BQ25188_H */