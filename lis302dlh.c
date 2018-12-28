#include "lis302dlh.h"
#include "i2cmaster.h"
#include "CException.h"

/* I2C device slave adddress of LIS302DLH: */
static const uint8_t lis302dlh_addr = 0x30u;

static const uint8_t reg_who_am_i        = 0x0fu;
static const uint8_t reg_ctrl_reg1       = 0x20u;
static const uint8_t reg_ctrl_reg2       = 0x21u;
static const uint8_t reg_ctrl_reg3       = 0x22u;
static const uint8_t reg_ctrl_reg4       = 0x23u;
static const uint8_t reg_ctrl_reg5       = 0x24u;
static const uint8_t reg_hp_filter_reset = 0x25u;
static const uint8_t reg_reference       = 0x26u;
static const uint8_t reg_status_reg      = 0x27u;
static const uint8_t reg_out_x_l         = 0x28u;
static const uint8_t reg_out_y_l         = 0x2au;
static const uint8_t reg_out_z_l         = 0x2cu;
static const uint8_t reg_int1_cfg        = 0x30u;
static const uint8_t reg_int1_source     = 0x31u;
static const uint8_t reg_int1_ths        = 0x32u;
static const uint8_t reg_int1_duration   = 0x33u;
static const uint8_t reg_int2_cfg        = 0x34u;
static const uint8_t reg_int2_source     = 0x35u;
static const uint8_t reg_int2_ths        = 0x36u;
static const uint8_t reg_int2_duration   = 0x37u;

typedef struct bitmask
{
    const uint8_t mask;
    const uint8_t shift;
} bitmask_t;

static const bitmask_t bitmask_0    = {0b00000001u, 0u};
static const bitmask_t bitmask_1    = {0b00000010u, 1u};
static const bitmask_t bitmask_2    = {0b00000100u, 2u};
static const bitmask_t bitmask_3    = {0b00001000u, 3u};
static const bitmask_t bitmask_4    = {0b00010000u, 4u};
static const bitmask_t bitmask_5    = {0b00100000u, 5u};
static const bitmask_t bitmask_6    = {0b01000000u, 6u};
static const bitmask_t bitmask_7    = {0b10000000u, 7u};

static const bitmask_t bitmask_765  = {0b11100000u, 5u};
static const bitmask_t bitmask_76   = {0b11000000u, 6u};
static const bitmask_t bitmask_65   = {0b01100000u, 5u};
static const bitmask_t bitmask_54   = {0b00110000u, 4u};
static const bitmask_t bitmask_43   = {0b00011000u, 3u};
static const bitmask_t bitmask_10   = {0b00000011u, 0u};
static const bitmask_t bitmask_full = {0b11111111u, 0u};


/* This function merges a two's complements high byte with a low byte
 * and returns a signed 16 bit integer */
static int16_t
merge_signed(const uint8_t high,
             const uint8_t low)
{
    return ((int16_t) ((high << 8) | (low & 0xff)));
}

static void
lis302dlh_read_bytes(uint8_t bytes_to_read,
                     const uint8_t reg,
                     uint8_t *res)
{
    if (bytes_to_read > 0)
    {
        i2c_start_wait(lis302dlh_addr + I2C_WRITE);

        // In order to read multiple bytes, MSB of reg must be 1
        uint8_t reg_multi_bytes_read = (reg | (1 << 7));
        if (i2c_write(reg_multi_bytes_read))
        {
           Throw(E_LIS_302_DLH_I2C_WRITE);
        }

        if (i2c_rep_start(lis302dlh_addr + I2C_READ))
        {
            Throw(E_LIS_302_DLH_I2C_REP_START);
        }
        --bytes_to_read;
        for (uint8_t pos = 0; pos < bytes_to_read; pos++)
        {
            res[pos] = i2c_readAck();
        }
        res[bytes_to_read] = i2c_readNak();
        i2c_stop();
    }
}

static uint8_t
lis302dlh_read_byte(const uint8_t reg)
{
    i2c_start_wait(lis302dlh_addr + I2C_WRITE);
    if (i2c_write(reg))
    {
        Throw(E_LIS_302_DLH_I2C_WRITE);
    }
    if (i2c_rep_start(lis302dlh_addr + I2C_READ))
    {
        Throw(E_LIS_302_DLH_I2C_REP_START);
    }
    uint8_t res = i2c_readNak();
    i2c_stop();
    return res;
}

static void
lis302dlh_write_byte(const uint8_t reg,
                const uint8_t val)
{
    i2c_start_wait(lis302dlh_addr + I2C_WRITE);
    if (i2c_write(reg) || i2c_write(val))
    {
        Throw(E_LIS_302_DLH_I2C_WRITE);
    }
    i2c_stop();
}

void
lis302dlh_init(void)
{
    i2c_init();
    lis302dlh_set_power_mode_to_normal_mode();
}

static uint8_t
lis302dlh_query(const uint8_t reg,
                const bitmask_t bm)
{
    uint8_t data = lis302dlh_read_byte(reg);
    data = ((data & bm.mask) >> bm.shift);
    return data;
}

static void
lis302dlh_set(const uint8_t reg,
              const bitmask_t bm,
              uint8_t val)
{
    uint8_t data = lis302dlh_read_byte(reg);
    data = data & ((uint8_t) ~bm.mask);
    val = (val << bm.shift) + data;

    lis302dlh_write_byte(reg, val);
}

/* Query-functions for all readable registers: */

/* This function returns the measured accelerometer data for the x, y, z axes.*/
lis302dlh_data_t
lis302dlh_query_accel_data(void)
{
    lis302dlh_data_t data;
    uint8_t res[6] = {0};

    lis302dlh_read_bytes(6, reg_out_x_l , res);

    data.x = merge_signed(res[1], res[0]);
    data.y = merge_signed(res[3], res[2]);
    data.z = merge_signed(res[5], res[4]);

    return data;
}


int16_t lis302dlh_query_accel_data_z(void)
{
    int16_t res = 0;
    uint8_t data[2] = {0};

    lis302dlh_read_bytes(2, reg_out_z_l, data);
    res = merge_signed(data[1], data[0]);

    return res;
}

int16_t lis302dlh_query_accel_data_y(void)
{
    int16_t res = 0;
    uint8_t data[2] = {0};

    lis302dlh_read_bytes(2, reg_out_y_l, data);
    res = merge_signed(data[1], data[0]);

    return res;
}

int16_t lis302dlh_query_accel_data_x(void)
{
    int16_t res = 0;
    uint8_t data[2] = {0};

    lis302dlh_read_bytes(2, reg_out_x_l, data);
    res = merge_signed(data[1], data[0]);

    return res;
}

uint8_t
lis302dlh_query_device_id(void)
{
    return lis302dlh_query(reg_who_am_i, bitmask_full);
}

uint8_t
lis302dlh_query_power_mode(void)
{
    return lis302dlh_query(reg_ctrl_reg1, bitmask_765);
}

uint8_t
lis302dlh_query_data_rate(void)
{
    return lis302dlh_query(reg_ctrl_reg1, bitmask_43);
}

uint8_t
lis302dlh_query_z_axis_enabled(void)
{
    return lis302dlh_query(reg_ctrl_reg1, bitmask_2);
}

uint8_t
lis302dlh_query_y_axis_enabled(void)
{
    return lis302dlh_query(reg_ctrl_reg1, bitmask_1);
}

uint8_t
lis302dlh_query_x_axis_enabled(void)
{
    return lis302dlh_query(reg_ctrl_reg1, bitmask_0);
}

uint8_t
lis302dlh_query_reboot_memory_bit(void)
{
    return lis302dlh_query(reg_ctrl_reg2, bitmask_7);
}

uint8_t
lis302dlh_query_high_pass_filter_mode_selection(void)
{
    return lis302dlh_query(reg_ctrl_reg2, bitmask_65);
}

uint8_t
lis302dlh_query_filtered_data_selection(void)
{
    return lis302dlh_query(reg_ctrl_reg2, bitmask_4);
}

uint8_t
lis302dlh_query_high_pass_filter_enabled_for_int2_source(void)
{
    return lis302dlh_query(reg_ctrl_reg2, bitmask_3);
}

uint8_t
lis302dlh_query_high_pass_filter_enabled_for_int1_source(void)
{
    return lis302dlh_query(reg_ctrl_reg2, bitmask_2);
}

uint8_t
lis302dlh_query_high_pass_filter_cut_off_frequency_configuration(void)
{
    return lis302dlh_query(reg_ctrl_reg2, bitmask_10);
}

uint8_t
lis302dlh_query_interrupt_active(void)
{
    return lis302dlh_query(reg_ctrl_reg3, bitmask_7);
}

uint8_t
lis302dlh_query_push_pull_open_drain_selection(void)
{
    return lis302dlh_query(reg_ctrl_reg3, bitmask_6);
}

uint8_t
lis302dlh_query_latch_interrupt_request_on_int2_src_register(void)
{
    return lis302dlh_query(reg_ctrl_reg3, bitmask_5);
}

uint8_t
lis302dlh_query_data_signal_on_int2_pad_control_bits(void)
{
    return lis302dlh_query(reg_ctrl_reg3, bitmask_43);
}

uint8_t
lis302dlh_query_latch_interrupt_request_on_int1_src_register(void)
{
    return lis302dlh_query(reg_ctrl_reg3, bitmask_2);
}

uint8_t
lis302dlh_query_data_signal_on_int1_pad_control_bits(void)
{
    return lis302dlh_query(reg_ctrl_reg3, bitmask_10);
}

uint8_t
lis302dlh_query_block_data_update(void)
{
    return lis302dlh_query(reg_ctrl_reg4, bitmask_7);
}

uint8_t
lis302dlh_query_big_little_endian_data_selection(void)
{
    return lis302dlh_query(reg_ctrl_reg4, bitmask_6);
}

uint8_t
lis302dlh_query_full_scale_selection(void)
{
    return lis302dlh_query(reg_ctrl_reg4, bitmask_54);
}

uint8_t
lis302dlh_query_self_test_sign(void)
{
    return lis302dlh_query(reg_ctrl_reg4, bitmask_3);
}

uint8_t
lis302dlh_query_self_test_enable(void)
{
    return lis302dlh_query(reg_ctrl_reg4, bitmask_1);
}

uint8_t
lis302dlh_query_spi_serial_interface_mode_selection(void)
{
    return lis302dlh_query(reg_ctrl_reg4, bitmask_0);
}

uint8_t
lis302dlh_query_sleep_to_wake_function_status(void)
{
    return lis302dlh_query(reg_ctrl_reg5, bitmask_10);
}

/* By reading the specified register, the function
 * lis302dlh_reset_high_pass_filter() zeroes instantaneously
 * the content of the internal high pass-filter. */
void
lis302dlh_reset_high_pass_filter(void)
{
    volatile uint8_t data = lis302dlh_read_byte(reg_hp_filter_reset);
    (void) data;
}

uint8_t
lis302dlh_query_reference_value_for_high_pass_filter(void)
{
    return lis302dlh_query(reg_reference, bitmask_full);
}

uint8_t
lis302dlh_query_xyz_axis_data_overrun(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_7);
}

uint8_t
lis302dlh_query_z_axis_data_overrun(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_6);
}

uint8_t
lis302dlh_query_y_axis_data_overrun(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_5);
}

uint8_t
lis302dlh_query_x_axis_data_overrun(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_4);
}

uint8_t
lis302dlh_query_xyz_axis_new_data_available(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_3);
}

uint8_t
lis302dlh_query_z_axis_new_data_available(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_2);
}

uint8_t
lis302dlh_query_y_axis_new_data_available(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_1);
}

uint8_t
lis302dlh_query_x_axis_new_data_available(void)
{
    return lis302dlh_query(reg_status_reg, bitmask_0);
}

uint8_t
lis302dlh_query_int1_and_or_combination_of_interrupt_events(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_7);
}

uint8_t
lis302dlh_query_int1_6_direction_detection_function_enable(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_6);
}

uint8_t
lis302dlh_query_int1_interrupt_generation_on_z_high_event(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_5);
}

uint8_t
lis302dlh_query_int1_interrupt_generation_on_z_low_event(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_4);
}

uint8_t
lis302dlh_query_int1_interrupt_generation_on_y_high_event(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_3);
}

uint8_t
lis302dlh_query_int1_interrupt_generation_on_y_low_event(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_2);
}

uint8_t
lis302dlh_query_int1_interrupt_generation_on_x_high_event(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_1);
}

uint8_t
lis302dlh_query_int1_interrupt_generation_on_x_low_event(void)
{
    return lis302dlh_query(reg_int1_cfg, bitmask_0);
}

uint8_t
lis302dlh_query_int1_interrupt_active(void)
{
    return lis302dlh_query(reg_int1_source, bitmask_6);
}

uint8_t
lis302dlh_query_int1_z_high_event_has_occured(void)
{
    return lis302dlh_query(reg_int1_source, bitmask_5);
}

uint8_t
lis302dlh_query_int1_z_low_event_has_occured(void)
{
    return lis302dlh_query(reg_int1_source, bitmask_4);
}

uint8_t
lis302dlh_query_int1_y_high_event_has_occured(void)
{
    return lis302dlh_query(reg_int1_source, bitmask_3);
}

uint8_t
lis302dlh_query_int1_y_low_event_has_occured(void)
{
    return lis302dlh_query(reg_int1_source, bitmask_2);
}

uint8_t
lis302dlh_query_int1_x_high_event_has_occured(void)
{
    return lis302dlh_query(reg_int1_source, bitmask_1);
}

uint8_t
lis302dlh_query_int1_x_low_event_has_occured(void)
{
    return lis302dlh_query(reg_int1_source, bitmask_0);
}

uint8_t
lis302dlh_query_int1_threshold(void)
{
    return lis302dlh_query(reg_int1_ths, bitmask_full);
}

uint8_t
lis302dlh_query_int1_duration(void)
{
    return lis302dlh_query(reg_int1_duration, bitmask_full);
}

uint8_t
lis302dlh_query_int2_and_or_combination_of_interrupt_events(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_7);
}

uint8_t
lis302dlh_query_int2_6_direction_detection_function(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_6);
}

uint8_t
lis302dlh_query_int2_interrupt_generation_on_z_high_event(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_5);
}

uint8_t
lis302dlh_query_int2_interrupt_generation_on_z_low_event(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_4);
}

uint8_t
lis302dlh_query_int2_interrupt_generation_on_y_high_event(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_3);
}

uint8_t
lis302dlh_query_int2_interrupt_generation_on_y_low_event(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_2);
}

uint8_t
lis302dlh_query_int2_interrupt_generation_on_x_high_event(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_1);
}

uint8_t
lis302dlh_query_int2_interrupt_generation_on_x_low_event(void)
{
    return lis302dlh_query(reg_int2_cfg, bitmask_0);
}

uint8_t
lis302dlh_query_int2_interrupt_active(void)
{
    return lis302dlh_query(reg_int2_source, bitmask_6);
}

uint8_t
lis302dlh_query_int2_z_high_event_has_occured(void)
{
    return lis302dlh_query(reg_int2_source, bitmask_5);
}

uint8_t
lis302dlh_query_int2_z_low_event_has_occured(void)
{
    return lis302dlh_query(reg_int2_source, bitmask_4);
}

uint8_t
lis302dlh_query_int2_y_high_event_has_occured(void)
{
    return lis302dlh_query(reg_int2_source, bitmask_3);
}

uint8_t
lis302dlh_query_int2_y_low_event_has_occured(void)
{
    return lis302dlh_query(reg_int2_source, bitmask_2);
}

uint8_t
lis302dlh_query_int2_x_high_event_has_occured(void)
{
    return lis302dlh_query(reg_int2_source, bitmask_1);
}

uint8_t
lis302dlh_query_int2_x_low_event_has_occured(void)
{
    return lis302dlh_query(reg_int2_source, bitmask_0);
}

uint8_t
lis302dlh_query_int2_threshold(void)
{
    return lis302dlh_query(reg_int2_ths, bitmask_full);
}

uint8_t
lis302dlh_query_int2_duration(void)
{
    return lis302dlh_query(reg_int2_duration, bitmask_full);
}

/* Set-functions for all writable registers: */

void
lis302dlh_set_power_mode_to_power_down_mode(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_765, 0b000);
}

void
lis302dlh_set_power_mode_to_normal_mode(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_765, 0b001);
}

void
lis302dlh_set_power_mode_to_low_power_mode1(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_765, 0b010);
}

void
lis302dlh_set_power_mode_to_low_power_mode2(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_765, 0b011);
}

void
lis302dlh_set_power_mode_to_low_power_mode3(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_765, 0b100);
}

void
lis302dlh_set_power_mode_to_low_power_mode4(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_765, 0b101);
}

void
lis302dlh_set_power_mode_to_low_power_mode5(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_765, 0b110);
}

void
lis302dlh_set_data_rate_to_50hz(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_43, 0b00);
}

void
lis302dlh_set_data_rate_to_100hz(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_43, 0b01);
}

void
lis302dlh_set_data_rate_to_400hz(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_43, 0b10);
}

void
lis302dlh_set_data_rate_to_1000hz(void)
{
	lis302dlh_set(reg_ctrl_reg1, bitmask_43, 0b11);
}

void
lis302dlh_enable_z_axis(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_2, 1);
}

void
lis302dlh_disable_z_axis(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_2, 0);
}

void
lis302dlh_enable_y_axis(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_1, 1);
}

void
lis302dlh_disable_y_axis(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_1, 0);
}

void
lis302dlh_enable_x_axis(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_0, 1);
}

void
lis302dlh_disable_x_axis(void)
{
    lis302dlh_set(reg_ctrl_reg1, bitmask_0, 0);
}


void
lis302dlh_reboot_memory_content(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_7, 1);
}

void
lis302dlh_set_high_pass_filter_normal_mode(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_65, 0b00);
}

void
lis302dlh_set_high_pass_filter_reference_signal(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_65, 0b01);
}

// FDS register: Filtered data selection: internal filter bypassed
void
lis302dlh_disable_internal_filter(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_4, 0);
}

/* FDS register: Filtered data selection: data from internal filter sent to
 * output register */
void
lis302dlh_enable_internal_filter(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_4, 1);
}

// HPen2 bit: High pass filter enabled for interrupt 2 source
void
lis302dlh_enable_high_pass_filter_for_int2_source(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_3, 1);
}

// HPen2 bit: High pass filter disabled for interrupt 2 source
void
lis302dlh_disable_high_pass_filter_for_int2_source(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_3, 0);
}

// HPen1 bit: High pass filter enabled for interrupt 1 source. Default value: 0
void
lis302dlh_enable_high_pass_filter_for_int_1_source(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_2, 1);
}

// HPen1 bit: High pass filter disabled for interrupt 1 source. Default value: 0
void
lis302dlh_disable_high_pass_filter_for_int_1_source(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_2, 0);
}

/* High pass filter cut-off frequency configuration. Default value: 00
 * The calculation of the cut-off frequency depends on the HP-coefficient
 * and on the data rate. By defining the coefficient and the data rate, certain
 * cut-off frequencies can be set, see datasheet page 26 for further details */
void
lis302dlh_set_high_pass_coefficient_to_8(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_10, 0b00);
}

void
lis302dlh_set_high_pass_coefficient_to_16(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_10, 0b01);
}

void
lis302dlh_set_high_pass_coefficient_to_32(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_10, 0b10);
}

void
lis302dlh_set_high_pass_coefficient_to_64(void)
{
    lis302dlh_set(reg_ctrl_reg2, bitmask_10, 0b11);
}

void
lis302dlh_set_interrupt_active_high(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_7, 0);
}

void
lis302dlh_set_interrupt_active_low(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_7, 1);
}


void
lis302dlh_set_push_pull_on_int1(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_6, 0);
}

void
lis302dlh_set_open_drain_on_int1(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_6, 1);
}

void
lis302dlh_set_interrupt_request_not_latched_on_int2_src_reg(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_5, 0);
}

void
lis302dlh_set_interrupt_request_latched_on_int2_src_reg(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_5, 1);
}


void
lis302dlh_set_data_signal_on_int_2_pad_to_int2_source(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_43, 0b00);
}

void
lis302dlh_set_data_signal_on_int_2_pad_to_int1_source_or_int2_source(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_43, 0b01);
}

void
lis302dlh_set_data_signal_on_int_2_pad_to_data_ready(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_43, 0b10);
}

void
lis302dlh_set_data_signal_on_int_2_pad_to_boot_running(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_43, 0b11);
}


void
lis302dlh_set_interrupt_request_not_latched_on_int1_src_reg(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_2, 0);
}

void
lis302dlh_set_interrupt_request_latched_on_int1_src_reg(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_2, 1);
}

void
lis302dlh_set_data_signal_on_int_1_pad_to_int1_source(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_10, 0b00);
}

void
lis302dlh_set_data_signal_on_int_1_pad_to_int1_source_or_int2_source(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_10, 0b01);
}

void
lis302dlh_set_data_signal_on_int_1_pad_to_data_ready(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_10, 0b10);
}

void
lis302dlh_set_data_signal_on_int_1_pad_to_boot_running(void)
{
    lis302dlh_set(reg_ctrl_reg3, bitmask_10, 0b11);
}

void
lis302dlh_enable_block_data_updates_between_msb_and_lsb_reading(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_7, 0);
}

void
lis302dlh_disable_block_data_updates_between_msb_and_lsb_reading(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_7, 1);
}

void
lis302dlh_set_big_endian_data(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_6, 1);
}

void
lis302dlh_set_little_endian_data(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_6, 0);
}

void
lis302dlh_set_full_scale_to_2g(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_54, 0b00);
}

void
lis302dlh_set_full_scale_to_4g(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_54, 0b01);
}

void
lis302dlh_set_full_scale_to_8g(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_54, 0b11);
}

void
lis302dlh_set_self_test_sign_to_plus(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_3, 0);
}

void
lis302dlh_set_self_test_sign_to_minus(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_3, 1);
}

void
lis302dlh_set_enable_self_test(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_1, 1);
}

void
lis302dlh_set_disable_self_test(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_1, 0);
}

void
lis302dlh_set_spi_serial_interface_mode_to_4_wire(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_0, 0);
}

void
lis302dlh_set_spi_serial_interface_mode_to_3_wire(void)
{
    lis302dlh_set(reg_ctrl_reg4, bitmask_0, 0);
}


void
lis302dlh_enable_sleep_to_wake_function(void)
{
    lis302dlh_set(reg_ctrl_reg5, bitmask_10, 0b11);
}

void
lis302dlh_disable_sleep_to_wake_function(void)
{
    lis302dlh_set(reg_ctrl_reg5, bitmask_10, 0b00);
}

/* Reference value for high-pass filter. Default value: 0x00 */
void
lis302dlh_set_reference(uint8_t val)
{
    lis302dlh_set(reg_reference, bitmask_full, val);
}

void
lis302dlh_set_int1_or_combination_of_interrupt_events(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_76, 0b00);
}

void
lis302dlh_set_int1_6_direction_movement_recognition(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_76, 0b01);
}

void
lis302dlh_set_int1_and_combination_of_interrupt_events(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_76, 0b10);
}

void
lis302dlh_set_int1_6_direction_position_recognition(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_76, 0b11);
}


/* Enable interrupt request on measured accel. value higher than preset
 * threshold */
void
lis302dlh_enable_int1_interrupt_generation_on_z_high_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_5, 1);
}

void
lis302dlh_disable_int1_interrupt_generation_on_z_high_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_5, 0);
}

/* Enable interrupt generation on Z low event. Default value: 0 */
void
lis302dlh_enable_int1_interrupt_generation_on_z_low_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_4, 1);
}

void
lis302dlh_disable_int1_interrupt_generation_on_z_low_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_4, 0);
}

/* Enable interrupt generation on Y high event. Default value: 0 */
void
lis302dlh_enable_int1_interrupt_generation_on_y_high_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_3, 1);
}

void
lis302dlh_disable_int1_interrupt_generation_on_y_high_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_3, 0);
}

/* Enable interrupt generation on Y low event. Default value: 0 */
void
lis302dlh_enable_int1_interrupt_generation_on_y_low_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_2, 1);
}

void
lis302dlh_disable_int1_interrupt_generation_on_y_low_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_2, 0);
}

void
lis302dlh_enable_int1_interrupt_generation_on_x_high_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_1, 1);
}

void
lis302dlh_disable_int1_interrupt_generation_on_x_high_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_1, 0);
}

void
lis302dlh_enable_int1_interrupt_generation_on_x_low_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_0, 1);
}

void
lis302dlh_disable_int1_interrupt_generation_on_x_low_event(void)
{
    lis302dlh_set(reg_int1_cfg, bitmask_0, 0);
}


void
lis302dlh_set_interrupt_1_threshold(uint8_t val)
{
    val = val & (bitmask_7.mask);
    lis302dlh_set(reg_int1_ths, bitmask_full, val);
}

void
lis302dlh_set_interrupt_1_duration(uint8_t val)
{
    val = val & (bitmask_7.mask);
    lis302dlh_set(reg_int1_duration, bitmask_full, val);
}


void
lis302dlh_set_int2_or_combination_of_interrupt_events(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_76, 0b00);
}

void
lis302dlh_set_int2_6_direction_movement_recognition(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_76, 0b01);
}

void
lis302dlh_set_int2_and_combination_of_interrupt_events(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_76, 0b10);
}

void
lis302dlh_set_int2_6_direction_position_recognition(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_76, 0b11);
}

void
lis302dlh_enable_int2_interrupt_generation_on_z_high_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_5, 1);
}

void
lis302dlh_disable_int2_interrupt_generation_on_z_high_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_5, 0);
}

/* Enable interrupt generation on Z low event. Default value: 0 */
void
lis302dlh_enable_int2_interrupt_generation_on_z_low_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_4, 1);
}

void
lis302dlh_disable_int2_interrupt_generation_on_z_low_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_4, 0);
}

/* Enable interrupt generation on Y high event. Default value: 0 */
void
lis302dlh_enable_int2_interrupt_generation_on_y_high_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_3, 1);
}

void
lis302dlh_disable_int2_interrupt_generation_on_y_high_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_3, 0);
}

/* Enable interrupt generation on Y low event. Default value: 0 */
void
lis302dlh_enable_int2_interrupt_generation_on_y_low_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_2, 1);
}

void
lis302dlh_disable_int2_interrupt_generation_on_y_low_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_2, 0);
}

void
lis302dlh_enable_int2_interrupt_generation_on_x_high_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_1, 1);
}

void
lis302dlh_disable_int2_interrupt_generation_on_x_high_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_1, 0);
}

void
lis302dlh_enable_int2_interrupt_generation_on_x_low_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_0, 1);
}

void
lis302dlh_disable_int2_interrupt_generation_on_x_low_event(void)
{
    lis302dlh_set(reg_int2_cfg, bitmask_0, 0);
}

void
lis302dlh_set_int2_threshold(uint8_t val)
{
    val = val & (bitmask_7.mask);
    lis302dlh_set(reg_int2_ths, bitmask_full, val);
}

void
lis302dlh_set_int2_duration(uint8_t val)
{
    val = val & (bitmask_7.mask);
    lis302dlh_set(reg_int2_duration, bitmask_full, val);
}
