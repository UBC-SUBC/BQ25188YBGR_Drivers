/* 
* BQ25188YBGR Battery Charger Driver
* 
* Author: Thomas McGrath
* Edited: 2025.09.10
* Reference PCB: Battery_Pack_1A
* PCB Author: Rosemary Rosemary
*/

#include "bq25188.h"

/*
* @brief read a byte from a register over I2C
*
* @param dev Pointer to the I2C device tree info
* @param reg Register address to read from
* @param data Pointer to store the read byte
*
* @return 0 on success, negative error code on failure
* @return negative on fail
*/
static int bq25188_read(const struct i2c_dt_spec *dev, const uint8_t reg, uint8_t *data) {
    return i2c_reg_read_byte_dt(dev, reg, data);
}

/*
* @brief write a byte to a register over I2C
*
* @param dev Pointer to the I2C device tree info
* @param reg Register address to write to
* @param data Byte to write
*
* @return 0 on success
* @return negative on fail
*/
static int bq25188_write(const struct i2c_dt_spec *dev, const uint8_t reg, const uint8_t data) {
    return i2c_reg_write_byte_dt(dev, reg, data);
}

static inline int bq25188_update_bits(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    return i2c_reg_update_byte_dt(dev, reg, mask, value, mask);
}

/*
* @brief Reads and parses bits from STAT0 reg
*
* @param dev pointer to I2C device tree info
* @param stat1 pointer to bq25188 stat0 structure
*
* @return 0 on success
* @return negative on fail
*/
static int bq25188_fetch_stat0(const struct i2c_dt_spec *dev, struct bq25180_stat0 *stat0) {
    int error;
    uint8_t stat0_bits;

    error = bq25188_read(dev, BQ25188_STAT0, &stat0_bits);
    if (error < 0 ) {
        return error;
    }

    stat0->ts_open_stat             = (stat0_bits & BIT(7)) >> 7;
    stat0->chg_stat                 = (stat0_bits & (BIT(6) | BIT(5))) >> 5;
    stat0->ilim_active_stat         = (stat0_bits & BIT(4)) >> 4;
    stat0->vdppm_active_stat        = (stat0_bits & BIT(3)) >> 3;
    stat0->vindpm_active_stat      = (stat0_bits & BIT(2)) >> 2;
    stat0->thermreg_active_stat     = (stat0_bits & BIT(1)) >> 1;
    stat0->vin_pgood_stat           = (stat0_bits & BIT(0));

    return 0;
}

/*
* @brief Reads and parses bits from the STAT1 reg
*
* @param dev pointer to I2C device tree info
* @param stat1 pointer to bq25188 stat1 structure
*
* @return 0 on success
* @return negative on fail
*/
static int bq25188_fetch_stat0(const struct i2c_dt_spec *dev, struct bq25188_stat1 *stat1) {
    int error;
    uint8_t stat1_bits;

    error = bq25188_read(dev, BQ25188_stat0, &stat0_bits);
    if (error < 0 ) {
        return error;
    }

    stat1->vin_ovp_stat          = (stat1_bits & BIT(7)) >> 7;
    stat1->buvlo_stat            = (stat1_bits & BIT(6)) >> 6;
    stat1->ts_stat               = (stat1_bits & (BIT(4) & BIT(3))) >> 3;
    stat1->safety_tmr_fault_flag = (stat1_bits & BIT(2)) >> 2;
    stat1->wake1_flag            = (stat1_bits & BIT(1)) >> 1;
    stat1->wake2_flag            = (stat1_bits & BIT(0));

    return 0;    
}

/*
* @brief Reads and parses bits from FLAG0 register
* 
* @param dev pointer to I2C device tree info
* @param stat1 pointer to bq25188 stat0 structure
* 
* @return 0 on success
* @return negative on fail
*/
static int bq25188_fetch_flag0(const struct i2c_dt_spec *dev, struct bq25188_flag0 *flag0) {
    int error;
    uint8_t flag0_bits;

    error = bq25180_read(dev, BQ25180_FLAG0, &flag0_bits)
    if (error <0) {
        return error;
    }

    flag0->ts_fault             = flag0_bits & BIT(7) ? true : false;
    flag0->ilim_active_flag     = flag0_bits & BIT(6) ? true : false;
    flag0->vdppm_active_flag    = flag0_bits & BIT(5) ? true : false;
    flag0->vindpm_active_flag   = flag0_bits & BIT(4) ? true : false;
    flag0->thermreg_active_flag = flag0_bits & BIT(3) ? true : false;
    flag0->vin_ovp_fault_flag   = flag0_bits & BIT(2) ? true : false;
    flag0->buvlo_fault_flag     = flag0_bits & BIT(1) ? true : false;
    flag0->bat_ocp_fault        = flag0_bits & BIT(0) ? true : false;

    return 0;
}

/*
* @brief Reads and parses bits from VBAT_REG reg
*
* @param dev pointer to I2C device tree info
* @param vbat_reg pointer to bq25188 vbat_reg structure
*
* @return 0 on success
* @return negative on fail
*/

static int bq25188_fetch_vbat_ctrl(const struct i2c_dt_spec *dev, struct bq25188_vbat_ctrl *vbat_ctrl) {
    int error;
    uint8_t vbat_ctrl_bits;

    error = bq25188_read(dev, BQ25188_VBAT_CTRL, &vbat_ctrl_vits);
    if (error < 0) {
        return error;
    }

    // pg_mode (bit 7) is reserved - IGNORE -
    vbat_ctrl->vbatreg = (vbat_ctrl_bits & (~BIT(7)));
    return 0;
}

static int bq25188_set_vbat_ctrl(const struct i2c_dt_spec *dev, struct bq25188_vbat_ctrl *vbat_ctrl) {
    int error;
    uint8_t vbat_ctrl_bits = 0;

    vbat_ctrl_bits |= vbat_ctrl->vbatreg;
    return bq25188_update_bits(dev, BQ25188_VBAT_CTRL, ~BIT(7), vbat_ctrl_bits);
}

/*
// Back up function if update bits doesn't work
static int bq25188_set_vbat_ctrl(const struct i2c_dt_spec *dev, struct bq25188_vbat_ctrl *vbat_ctrl) {
    return bq25188_write(dev, BQ25180_VBAT_CTRL, vbat_ctrl->vbatreg);
}
*/

static int bq25188_fetch_ichg_ctrl(const struct i2c_dt_spec *dev, struct bq25188_ichg_ctrl *ichg_ctrl) {
    int error;
    uint8_t ichg_ctrl_bits;
    error = bq25188_read(dev, BQ25188_ICHG_CTRL, &ichg_ctrl_bits);
    if (error < 0) {
        return error;
    }
    
    ichg_ctrl->chg_dis = (ichg_ctrl_bits & BIT(7)) >> 7;
    ichg_ctrl->ichg = (ichg_ctrl_bits & (~BIT(7)));

    return 0;
}

static int bq25188_set_ichg_ctrl(const struct i2c_dt_spec *dev, struct bq25188_ichg *ichg_ctrl) {
    int error;
    uint8_t ichg_ctrl_bits = 0;

    ichg_ctrl_bits |= (ichg_ctrl->chg_dis << 6;)
    ichg_ctrl_bits |= ichg_ctrl->ichg;

    return bq25188_write(dev, BQ25188_ICHG_CTRL, ichg_ctrl_bits);
}

static int bq25188_fetch_chargectrl0(const truct i2c_dt_spec *dev, struct bq25188_chargectrl0 *chargectrl0) {
    int error;
    uint8_t chargerctrl0_bits;
    
    error = bq25188_read(dev, BQ25188_CHARGECTRL0, &chargerctrl0_bits);
    if (error < 0) {
        return error;
    }
    
    chargectrl0->iprechg   = (chargerctrl0_bits & BIT(6)) >> 6;
    chargectrl0->iterm     = (chargerctrl0_bits & (BIT(5) | BIT(4))) >> 4;
    chargectrl0->vindpm    = (chargerctrl0_bits & (BIT(3) | BIT(2))) >> 2;
    chargectrl0->therm_reg = (chargerctrl0_bits & (BIT(1) | BIT(0)));

    return 0;
}

static int bq25188_set_chargectrl0(const struct i2c_dt_spec *dev, struct bq25188_chargectrl0 *chargectrl0) {
    int error;
    uint8_t chargectrl0_bits = 0;

    chargectrl0_bits |= (chargectrl0->iprechg << 6);
    chargectrl0_bits |= (chargectrl0->iterm << 4);
    chargectrl0_bits |= (chargectrl0->vindpm << 2);
    chargectrl0_bits |= chargectrl0->therm_reg;

    return bq25188_write(dev, BQ25188_CHARGECTRL0, chargectrl0_bits);
}

static int bq25188_fetch_chargectrl1(const struct i2c_dt_spec *dev, struct bq25188_chargectrl1 *chargectrl1) {
    int error;
    uint8_t chargectrl1_bits;

    error = bq25188_read(dev, BQ25188_CHARGECTRL1, &chargectrl1_bits);
    if (error < 0) {
        return error;
    }

    chargectrl1->chargectrl1->ibat_ocp = (chargectrl1_bits & (BIT(7) | BIT(6))) >> 6;
    chargectrl1->buvlo                 = (chargectrl1_bits & (BIT(5) | BIT(4) | BIT(3))) >> 3;
    chargectrl1->chg_status_int_mask   = (chargectrl1_bits & (BIT(2))) >> 2;
    chargectrl1->ilim_int_mask         = (chargectrl1_bits & (BIT(1))) >> 1;
    chargectrl1->vindpm_int_mask       = (chargectrl1_bits & (BIT(0)));

    return 0;
}

static int bq25188_set_chargectrl1(const struct i2c_dt_spec *dev, struct bq25188_chargectrl1 *chargectrl1) {
    int error;
    uint8_t chargectrl1_bits = 0;

    chargectrl1_bits |= (chargectrl1->ibat_ocp << 6);
    chargectrl1_bits |= (chargectrl1->buvlo << 3);
    chargectrl1_bits |= (chargectrl1->chg_status_int_mask << 2);
    chargectrl1_bits |= (chargectrl1->ilim_int_mask << 1);
    chargectrl1_bits |= chargectrl1->vindpm_int_mask;

    return bq25188_write(dev, BQ25188_CHARGECTRL1, chargectrl1_bits);
}

static int bq25188_set_ic_ctrl(const struct i2c_dt_spec *dev, struct bq25188_ic_ctrl  *ic_ctrl) {
    int error;
    uint8_t chargectrl1_bits = 0;

    chargectrl1_bits |= (chargectrl1->ts_en

    return bq25188_write(dev, BQ25188_CHARGECTRL1, chargectrl1_bits);
}