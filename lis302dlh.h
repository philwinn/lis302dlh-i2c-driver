#pragma once

#include <stdint.h>

static const uint8_t E_LIS_302_DLH_I2C_START     = 1U;
static const uint8_t E_LIS_302_DLH_I2C_WRITE     = 2U;
static const uint8_t E_LIS_302_DLH_I2C_REP_START = 3U;

typedef struct lis302dlh_data
{
    int16_t x;
    int16_t y;
    int16_t z;
} lis302dlh_data_t;

/* The function lis302dlh_init initializes the I2C communication and sets
 * the power mode to normal mode, thus, this function must be called first
 * before calling any other function. */
void lis302dlh_init(void);

/* The function lis302dlh_query_accel_data(void)
 * returns accelerometer data for all three axes. */
lis302dlh_data_t lis302dlh_query_accel_data(void);

/* Query-functions for all readable registers: */

// WHO_AM_I (0x0F)
uint8_t lis302dlh_query_device_id(void);

// CTRL_REG1 (0x20)
uint8_t lis302dlh_query_power_mode(void);
uint8_t lis302dlh_query_data_rate(void);
uint8_t lis302dlh_query_z_axis_enabled(void);
uint8_t lis302dlh_query_y_axis_enabled(void);
uint8_t lis302dlh_query_x_axis_enabled(void);

// CTRL_REG2 (0x21)
uint8_t lis302dlh_query_reboot_memory_bit(void);
uint8_t lis302dlh_query_high_pass_filter_mode_selection(void);
uint8_t lis302dlh_query_filtered_data_selection(void);
uint8_t lis302dlh_query_high_pass_filter_enabled_for_int2_source(void);
uint8_t lis302dlh_query_high_pass_filter_enabled_for_int1_source(void);
uint8_t lis302dlh_query_high_pass_filter_cut_off_frequency_configuration(void);

// CTRL_REG3 [Interrupt CTRL register] (0x22)
uint8_t lis302dlh_query_interrupt_active(void);
uint8_t lis302dlh_query_push_pull_open_drain_selection(void);
uint8_t lis302dlh_query_latch_interrupt_request_on_int2_src_register(void);
uint8_t lis302dlh_query_data_signal_on_int2_pad_control_bits(void);
uint8_t lis302dlh_query_latch_interrupt_request_on_int1_src_register(void);
uint8_t lis302dlh_query_data_signal_on_int1_pad_control_bits(void);

// CTRL_REG4 (0x23)
uint8_t lis302dlh_query_block_data_update(void);
uint8_t lis302dlh_query_big_little_endian_data_selection(void);
uint8_t lis302dlh_query_full_scale_selection(void);
uint8_t lis302dlh_query_self_test_sign(void);
uint8_t lis302dlh_query_self_test_enable(void);
uint8_t lis302dlh_query_spi_serial_interface_mode_selection(void);

// CTRL_REG5 (0x24)
uint8_t lis302dlh_query_sleep_to_wake_function_status(void);

// HP_FILTER_RESET (0x25)
void lis302dlh_reset_high_pass_filter(void);

// REFERENCE (0x26)
uint8_t lis302dlh_query_reference_value_for_high_pass_filter(void);

// STATUS_REG (0x27)
uint8_t lis302dlh_query_xyz_axis_data_overrun(void);
uint8_t lis302dlh_query_z_axis_data_overrun(void);
uint8_t lis302dlh_query_y_axis_data_overrun(void);
uint8_t lis302dlh_query_x_axis_data_overrun(void);
uint8_t lis302dlh_query_xyz_axis_new_data_available(void);
uint8_t lis302dlh_query_z_axis_new_data_available(void);
uint8_t lis302dlh_query_y_axis_new_data_available(void);
uint8_t lis302dlh_query_x_axis_new_data_available(void);

// OUT_* Register (0x28 - 0x2d)
int16_t lis302dlh_query_accel_data_z(void);
int16_t lis302dlh_query_accel_data_y(void);
int16_t lis302dlh_query_accel_data_x(void);

// INT1_CFG (0x30)
uint8_t lis302dlh_query_int1_and_or_combination_of_interrupt_events(void);
uint8_t lis302dlh_query_int1_6_direction_detection_function(void);
uint8_t lis302dlh_query_int1_interrupt_generation_on_z_high_event(void);
uint8_t lis302dlh_query_int1_interrupt_generation_on_z_low_event(void);
uint8_t lis302dlh_query_int1_interrupt_generation_on_y_high_event(void);
uint8_t lis302dlh_query_int1_interrupt_generation_on_z_low_event(void);
uint8_t lis302dlh_query_int1_interrupt_generation_on_x_high_event(void);
uint8_t lis302dlh_query_int1_interrupt_generation_on_z_low_event(void);

// INT1_SRC (0x31)
uint8_t lis302dlh_query_int1_interrupt_active(void);
uint8_t lis302dlh_query_int1_z_high_event_has_occured(void);
uint8_t lis302dlh_query_int1_z_low_event_has_occured(void);
uint8_t lis302dlh_query_int1_y_high_event_has_occured(void);
uint8_t lis302dlh_query_int1_y_low_event_has_occured(void);
uint8_t lis302dlh_query_int1_x_high_event_has_occured(void);
uint8_t lis302dlh_query_int1_x_low_event_has_occured(void);

// INT1_THS (0x32)
uint8_t lis302dlh_query_int1_threshold(void);

// INT1_DURATION (0x33)
uint8_t lis302dlh_query_int1_duration(void);

// INT2_CFG (0x34)
uint8_t lis302dlh_query_int2_and_or_combination_of_interrupt_events(void);
uint8_t lis302dlh_query_int2_6_direction_detection_function(void);
uint8_t lis302dlh_query_int2_interrupt_generation_on_z_high_event(void);
uint8_t lis302dlh_query_int2_interrupt_generation_on_z_low_event(void);
uint8_t lis302dlh_query_int2_interrupt_generation_on_y_high_event(void);
uint8_t lis302dlh_query_int2_interrupt_generation_on_y_low_event(void);
uint8_t lis302dlh_query_int2_interrupt_generation_on_x_high_event(void);
uint8_t lis302dlh_query_int2_interrupt_generation_on_x_high_event(void);

// INT2_SRC (35x)
uint8_t lis302dlh_query_int2_interrupt_active(void);
uint8_t lis302dlh_query_int2_z_high_event_has_occured(void);
uint8_t lis302dlh_query_int2_z_low_event_has_occured(void);
uint8_t lis302dlh_query_int2_y_high_event_has_occured(void);
uint8_t lis302dlh_query_int2_y_low_event_has_occured(void);
uint8_t lis302dlh_query_int2_x_high_event_has_occured(void);
uint8_t lis302dlh_query_int2_x_low_event_has_occured(void);

// INT2_THS (0x36)
uint8_t lis302dlh_query_int2_threshold(void);

// INT2_DURATION (0x37)
uint8_t lis302dlh_query_int2_duration(void);


/* Set-functions for all writable registers: */

// CTRL_REG1 (0x20):
// Power mode selection. Default value: 000 (Power-down)
void lis302dlh_set_power_mode_to_power_down_mode(void);
void lis302dlh_set_power_mode_to_normal_mode(void);
void lis302dlh_set_power_mode_to_low_power_mode1(void);
void lis302dlh_set_power_mode_to_low_power_mode2(void);
void lis302dlh_set_power_mode_to_low_power_mode3(void);
void lis302dlh_set_power_mode_to_low_power_mode4(void);
void lis302dlh_set_power_mode_to_low_power_mode5(void);

// Data rate selection. Default value: 00 (50Hz)
void lis302dlh_set_data_rate_to_50hz(void);
void lis302dlh_set_data_rate_to_100hz(void);
void lis302dlh_set_data_rate_to_400hz(void);
void lis302dlh_set_data_rate_to_1000hz(void);

// Z axis enable. Default value: 1 (enabled)
void lis302dlh_enable_z_axis(void);
void lis302dlh_disable_z_axis(void);

// Y axis enable. Default value: 1 (enabled)
void lis302dlh_enable_y_axis(void);
void lis302dlh_disable_y_axis(void);

// X axis enable. Default value: 1 (enabled)
void lis302dlh_enable_x_axis(void);
void lis302dlh_disable_x_axis(void);


// CTRL_REG2 (0x21):

// Reboot memory content. Default value: 0 (normal mode)
void lis302dlh_reboot_memory_content(void);

// High pass filter mode selection. Default value: 00 (normal mode)
void lis302dlh_set_high_pass_filter_normal_mode(void);
void lis302dlh_set_high_pass_filter_reference_signal(void);

// Filtered data selection. Default value: 0 (internal filter bypassed)
void lis302dlh_disable_internal_filter(void);
void lis302dlh_enable_internal_filter(void);

// High pass filter enabled for interrupt 2 source. Default value: 0 (filter bypassed, i.e. disabled)
void lis302dlh_enable_high_pass_filter_for_int2_source(void);
void lis302dlh_disable_high_pass_filter_for_int2_source(void);

// High pass filter enabled for interrupt 1 source. Default value: 0 (filter bypassed, i.e. disabled)
void lis302dlh_enable_high_pass_filter_for_int1_source(void);
void lis302dlh_disable_high_pass_filter_for_int1_source(void);

// High pass filter cut-off frequency configuration. Default value: 00 (HPc=8)
void lis302dlh_set_high_pass_coefficient_to_8(void);
void lis302dlh_set_high_pass_coefficient_to_16(void);
void lis302dlh_set_high_pass_coefficient_to_32(void);
void lis302dlh_set_high_pass_coefficient_to_64(void);


// CTRL_REG3 [Interrupt CTRL register] (0x22):

// Interrupt active high, low. Default value: 0 (active high)
void lis302dlh_set_interrupt_active_high(void);
void lis302dlh_set_interrupt_active_low(void);

// Push-pull/Open drain selection on interrupt pad. Default value 0 (push-pull)
void lis302dlh_set_push_pull_on_int1(void);
void lis302dlh_set_open_drain_on_int1(void);

/* Latch interrupt request on INT2_SRC register, with INT2_SRC register cleared by
 * reading INT2_SRC itself. Default value: 0 (interrupt request not latched) */
void lis302dlh_set_interrupt_request_not_latched_on_int2_src_reg(void);
void lis302dlh_set_interrupt_request_latched_on_int2_src_reg(void);

// Data signal on INT 2 pad control bits. Default value: 00 (Interrupt 2 source)
void lis302dlh_set_data_signal_on_int_2_pad_to_int2_source(void);
void lis302dlh_set_data_signal_on_int_2_pad_to_int1_source_or_int2_source(void);
void lis302dlh_set_data_signal_on_int_2_pad_to_data_ready(void);
void lis302dlh_set_data_signal_on_int_2_pad_to_boot_running(void);

/* Latch interrupt request on INT1_SRC register, with INT1_SRC register cleared by
 * reading INT1_SRC register. Default value: 0 (interrupt request not latched) */
void lis302dlh_set_interrupt_request_not_latched_on_int1_src_reg(void);
void lis302dlh_set_interrupt_request_latched_on_int1_src_reg(void);

// Data signal on INT 1 pad control bits. Default value: 00 (Interrupt 1 source)
void lis302dlh_set_data_signal_on_int_1_pad_to_int1_source(void);
void lis302dlh_set_data_signal_on_int_1_pad_to_int1_source_or_int2_source(void);
void lis302dlh_set_data_signal_on_int_1_pad_to_data_ready(void);
void lis302dlh_set_data_signal_on_int_1_pad_to_boot_running(void);


// CTRL_REG4 (0x23):

// Block data update. Default value: 0 (continuos update)
void lis302dlh_enable_block_data_updates_between_msb_and_lsb_reading(void);
void lis302dlh_disable_block_data_updates_between_msb_and_lsb_reading(void);

// Big/little endian data selection. Default value 0 (data LSB @ lower address)
void lis302dlh_set_big_endian_data(void);
void lis302dlh_set_little_endian_data(void);

// Full-scale selection. Default value: 00 (+-2g)
void lis302dlh_set_full_scale_to_2g(void);
void lis302dlh_set_full_scale_to_4g(void);
void lis302dlh_set_full_scale_to_8g(void);

// Self-test sign. Default value: 00 (self-test plus)
void lis302dlh_set_self_test_sign_to_plus(void);
void lis302dlh_set_self_test_sign_to_minus(void);

// Self-test enable. Default value: 0 (self-test disabled)
void lis302dlh_set_enable_self_test(void);
void lis302dlh_set_disable_self_test(void);

// SPI serial interface mode selection. Default value: 0 (4-wire interface)
void lis302dlh_set_spi_serial_interface_mode_to_4_wire(void);
void lis302dlh_set_spi_serial_interface_mode_to_3_wire(void);


// CTRL_REG5 (0x24):

// Turn-on mode selection for sleep to wake function. Default value: 00 (Sleep to wake function is disabled)
void lis302dlh_enable_sleep_to_wake_function(void);
void lis302dlh_disable_sleep_to_wake_function(void);


// REFERENCE (0x26)

// This register sets the acceleration value taken as a reference for the high-pass filter output.
void lis302dlh_set_reference(uint8_t val);


// INT1_CFG (0x30):

// Interrupt 1 source configurations. Default value: 00 (OR combination of interrupt events)
void lis302dlh_set_int1_or_combination_of_interrupt_events(void);
void lis302dlh_set_int1_6_direction_movement_recognition(void);
void lis302dlh_set_int1_and_combination_of_interrupt_events(void);
void lis302dlh_set_int1_6_direction_position_recognition(void);

// Enable or disable interrupt generation on Z high event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int1_interrupt_generation_on_z_high_event(void);
void lis302dlh_disable_int1_interrupt_generation_on_z_high_event(void);

// Enable or disable interrupt generation on Z low event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int1_interrupt_generation_on_z_low_event(void);
void lis302dlh_disable_int1_interrupt_generation_on_z_low_event(void);

// Enable or disable interrupt generation on Y high event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int1_interrupt_generation_on_y_high_event(void);
void lis302dlh_disable_int1_interrupt_generation_on_y_high_event(void);

// Enable or disable interrupt generation on Y low event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int1_interrupt_generation_on_y_low_event(void);
void lis302dlh_disable_int1_interrupt_generation_on_y_low_event(void);

// Enable or disable interrupt generation on X high event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int1_interrupt_generation_on_x_high_event(void);
void lis302dlh_disable_int1_interrupt_generation_on_x_high_event(void);

// Enable or disable interrupt generation on X low event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int1_interrupt_generation_on_x_low_event(void);
void lis302dlh_disable_int1_interrupt_generation_on_x_low_event(void);


// INT1_THS (0x32)

// Interrupt 1 threshold. Default value: 0b0000000
void lis302dlh_set_int1_threshold(uint8_t val);


// INT1_DURATION (0x33)

// Interrupt 1 duration value. Default value: 0b0000000
void lis302dlh_set_int1_duration(uint8_t val);


// INT2_CFG (0x34)

// Interrupt 2 source configurations. Default value: 00 (OR combination of interrupt events)
void lis302dlh_set_int2_or_combination_of_interrupt_events(void);
void lis302dlh_set_int2_6_direction_movement_recognition(void);
void lis302dlh_set_int2_and_combination_of_interrupt_events(void);
void lis302dlh_set_int2_6_direction_position_recognition(void);

// Enable or disable interrupt generation on Z high event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int2_interrupt_generation_on_z_high_event(void);
void lis302dlh_disable_int2_interrupt_generation_on_z_high_event(void);

// Enable or disable interrupt generation on Z low event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int2_interrupt_generation_on_z_low_event(void);
void lis302dlh_disable_int2_interrupt_generation_on_z_low_event(void);

// Enable or disable interrupt generation on Y high event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int2_interrupt_generation_on_y_high_event(void);
void lis302dlh_disable_int2_interrupt_generation_on_y_high_event(void);

// Enable or disable interrupt generation on Y low event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int2_interrupt_generation_on_y_low_event(void);
void lis302dlh_disable_int2_interrupt_generation_on_y_low_event(void);

// Enable or disable interrupt generation on X high event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int2_interrupt_generation_on_x_high_event(void);
void lis302dlh_disable_int2_interrupt_generation_on_x_high_event(void);

// Enable or disable interrupt generation on X low event. Default value: 0 (disable interrupt request)
void lis302dlh_enable_int2_interrupt_generation_on_x_low_event(void);
void lis302dlh_disable_int2_interrupt_generation_on_x_low_event(void);


// INT2_THS (0x36)

// Interrupt 2 threshold. Default value: 0b0000000
void lis302dlh_set_int2_threshold(uint8_t val);


// INT2_DURATION (0x37)

// Interrupt 2 duration value. Default value: 0b0000000
void lis302dlh_set_int2_duration(uint8_t val);
