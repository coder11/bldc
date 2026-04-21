/*
	Copyright 2020 Mitch Lustig

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "lsm6ds3.h"
#include "terminal.h"
#include "i2c_bb.h"
#include "commands.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

#define LSM6DS3_RATE_HZ_MAX				6660
#define LSM6DS3_RATE_HZ_MAX_STANDARD_GYRO		1660

#ifndef LSM6DS3_SMA_FILTER_LEN
#define LSM6DS3_SMA_FILTER_LEN				4
#endif

#if LSM6DS3_SMA_FILTER_LEN < 1
#error "LSM6DS3_SMA_FILTER_LEN must be at least 1"
#endif

static thread_t *lsm6ds3_thread_ref = NULL;
static i2c_bb_state *m_i2c_bb;
static volatile uint16_t lsm6ds3_addr;
static int rate_hz = LSM6DS3_RATE_HZ_MAX;
static IMU_FILTER filter;
static float sma_samples[6][LSM6DS3_SMA_FILTER_LEN];
static float sma_sum[6];
static int sma_pos;
static int sma_sample_num;

static void terminal_read_reg(int argc, const char **argv);
static uint8_t read_single_reg(uint8_t reg);
static void reset_sma(void);
static void filter_sma(float *accel, float *gyro);
static THD_FUNCTION(lsm6ds3_thread, arg);

// Function pointers
static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;


void lsm6ds3_set_rate_hz(int hz) {
	(void)hz;
}

void lsm6ds3_set_filter(IMU_FILTER f) {
	filter = f;
}

void lsm6ds3_init(i2c_bb_state *i2c_state,
		stkalign_t *work_area, size_t work_area_size) {

	read_callback = 0;
	reset_sma();

	m_i2c_bb = i2c_state;

	uint8_t txb[2];
	uint8_t rxb[2];

	txb[0] = LSM6DS3_ACC_GYRO_WHO_AM_I_REG;
	lsm6ds3_addr = LSM6DS3_ACC_GYRO_ADDR_A;
	bool res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 1, rxb, 1);
	if (!res || (rxb[0] != 0x69 && rxb[0] != 0x6A && rxb[0] != 0x6C)) {
		commands_printf("LSM6DS3 Address A failed, trying B (rx: %d)", rxb[0]);
		lsm6ds3_addr = LSM6DS3_ACC_GYRO_ADDR_B;
		res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 1, rxb, 1);
		if (!res || (rxb[0] != 0x69 && rxb[0] != 0x6A && rxb[0] != 0x6C)) {
			commands_printf("LSM6DS3 Address B failed (rx: %d)", rxb[0]);
			return;
		}
	}

	bool is_trc = false;
	if (rxb[0] == 0x6A){
		is_trc = true;
	}

	// TRC variant supports configurable hardware filters
	// oversampling is achieved by configuring higher bandwidth + stronger filtering
	#define LSM6DS3TRC_BW0_XL 0x1
	#define LSM6DS3TRC_LPF1_BW_SEL 0x2
	rate_hz = is_trc ? LSM6DS3_RATE_HZ_MAX : LSM6DS3_RATE_HZ_MAX_STANDARD_GYRO;

	// Configure imu
	// Set accel to the maximum available output data rate.
	txb[0] = LSM6DS3_ACC_GYRO_CTRL1_XL;
	txb[1] = LSM6DS3_ACC_GYRO_BW_XL_400Hz |
			LSM6DS3_ACC_GYRO_FS_XL_16g |
			LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
	if (is_trc && (filter >= IMU_FILTER_MEDIUM)) {
		txb[1] |= LSM6DS3TRC_LPF1_BW_SEL;
		if (filter == IMU_FILTER_HIGH) {
			txb[1] |= LSM6DS3TRC_BW0_XL;
		}
	}
	res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 2, rxb, 1);
	if (!res){
		commands_printf("LSM6DS3 Accel Config FAILED");
		return;
	}

	// Set gyro to the maximum available output data rate for this variant.
	txb[0] = LSM6DS3_ACC_GYRO_CTRL2_G;
	txb[1] = LSM6DS3_ACC_GYRO_FS_G_2000dps;
	if (is_trc) {
		txb[1] |= LSM6DS3TRC_ACC_GYRO_ODR_G_6660Hz;
	} else {
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
	}
	res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 2, rxb, 1);
	if (!res){
		commands_printf("LSM6DS3 Gyro Config FAILED");
		return;
	}

	// Filtering
	txb[0] = LSM6DS3_ACC_GYRO_CTRL4_C;
	// TRC Variant CTRL4 register is very different from other variants
	if (is_trc) {
		if (filter >= IMU_FILTER_MEDIUM) {
			// Enable gyroscope digital low-pass filter LPF1
			txb[1] = LSM6DS3_ACC_GYRO_LPF1_SEL_G_ENABLED;
		} else {
			txb[1] = 0;
		}
	} else {
		// Standard LSM6DS3 only: Set XL anti-aliasing filter to be manually configured
		txb[1] = LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}
	res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 2, rxb, 1);
	if (!res){
		commands_printf("LSM6DS3 ODR Config FAILED");
		return;
	}

	if (is_trc && (filter == IMU_FILTER_HIGH)) {
		// Low-pass filter with ODR/9 data rate
		#define LSM6DS3TRC_LPF2_XL_EN 0x80
		#define LSM6DS3TRC_HPCF_XL_ODR9 0x40
		txb[0] = LSM6DS3_ACC_GYRO_CTRL8_XL;
		txb[1] = LSM6DS3TRC_LPF2_XL_EN | LSM6DS3TRC_HPCF_XL_ODR9;
		res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 2, rxb, 1);
		if (!res) {
			commands_printf("LSM6DS3 Accel Low Pass Config FAILED");
			return;
		}
	}

	// Disable IMU writing to output registers
	txb[0] = LSM6DS3_ACC_GYRO_CTRL3_C;
	txb[1] = LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE | LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
	i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 2, rxb, 1);

	terminal_register_command_callback(
			"lsm_read_reg",
			"Read register of the LSM6DS3",
			"[reg]",
			terminal_read_reg);

	lsm6ds3_thread_ref = chThdCreateStatic(work_area, work_area_size, NORMALPRIO, lsm6ds3_thread, NULL);
}

void lsm6ds3_stop(void) {
	if (lsm6ds3_thread_ref != NULL){
		chThdTerminate(lsm6ds3_thread_ref);
		chThdWait(lsm6ds3_thread_ref);
	}
	lsm6ds3_thread_ref = NULL;
	terminal_unregister_callback(terminal_read_reg);
}

void lsm6ds3_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

static uint8_t read_single_reg(uint8_t reg) {
	uint8_t txb[2];
	uint8_t rxb[2];

	txb[0] = reg;
	bool res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 1, rxb, 2);

	if (res) {
		return rxb[0];
	} else {
		return 0;
	}
}

static void terminal_read_reg(int argc, const char **argv) {
	if (argc == 2) {
		int reg = -1;
		sscanf(argv[1], "%d", &reg);

		if (reg >= 0) {
			unsigned int res = read_single_reg(reg);

			char bl[9];
			utils_byte_to_binary(res & 0xFF, bl);

			commands_printf("Reg 0x%02x: %s (0x%02x)\n", reg, bl, res);
		} else {
			commands_printf("Invalid argument(s).\n");
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static void reset_sma(void) {
	memset(sma_samples, 0, sizeof(sma_samples));
	memset(sma_sum, 0, sizeof(sma_sum));
	sma_pos = 0;
	sma_sample_num = 0;
}

static void filter_sma(float *accel, float *gyro) {
#if LSM6DS3_SMA_FILTER_LEN > 1
	float sample[6] = {
			gyro[0], gyro[1], gyro[2],
			accel[0], accel[1], accel[2]
	};
	const int sample_num = sma_sample_num < LSM6DS3_SMA_FILTER_LEN ?
			sma_sample_num + 1 : LSM6DS3_SMA_FILTER_LEN;

	for (int i = 0; i < 6; i++) {
		sma_sum[i] += sample[i] - sma_samples[i][sma_pos];
		sma_samples[i][sma_pos] = sample[i];
		sample[i] = sma_sum[i] / (float)sample_num;
	}

	sma_pos++;
	if (sma_pos >= LSM6DS3_SMA_FILTER_LEN) {
		sma_pos = 0;
	}

	if (sma_sample_num < LSM6DS3_SMA_FILTER_LEN) {
		sma_sample_num++;
	}

	gyro[0] = sample[0];
	gyro[1] = sample[1];
	gyro[2] = sample[2];
	accel[0] = sample[3];
	accel[1] = sample[4];
	accel[2] = sample[5];
#else
	(void)accel;
	(void)gyro;
#endif
}

static THD_FUNCTION(lsm6ds3_thread, arg) {
	(void)arg;
	chRegSetThreadName("LSM6SD3");

	systime_t iteration_timer = chVTGetSystemTimeX();
	const systime_t desired_interval = US2ST(1000000 / rate_hz);

	while (!chThdShouldTerminateX()) {
		uint8_t txb[2];
		uint8_t rxb[12];

		// Read IMU output registers
		txb[0] = LSM6DS3_ACC_GYRO_OUTX_L_G;
		bool res = i2c_bb_tx_rx(m_i2c_bb, lsm6ds3_addr, txb, 1, rxb, 12);

		// Parse 6 axis values
		float gx = (float)((int16_t)((uint16_t)rxb[1] << 8) + rxb[0]) * 4.375 * (2000 / 125) / 1000;
		float gy = (float)((int16_t)((uint16_t)rxb[3] << 8) + rxb[2]) * 4.375 * (2000 / 125) / 1000;
		float gz = (float)((int16_t)((uint16_t)rxb[5] << 8) + rxb[4]) * 4.375 * (2000 / 125) / 1000;
		float ax = (float)((int16_t)((uint16_t)rxb[7] << 8) + rxb[6]) * 0.061 * (16 >> 1) / 1000;
		float ay = (float)((int16_t)((uint16_t)rxb[9] << 8) + rxb[8]) * 0.061 * (16 >> 1) / 1000;
		float az = (float)((int16_t)((uint16_t)rxb[11] << 8) + rxb[10]) * 0.061 * (16 >> 1) / 1000;

		if (res && read_callback) {
			float tmp_accel[3] = {ax,ay,az}, tmp_gyro[3] = {gx,gy,gz}, tmp_mag[3] = {1,2,3};
			filter_sma(tmp_accel, tmp_gyro);
			read_callback(tmp_accel, tmp_gyro, tmp_mag);
		}

		// Delay between loops
		iteration_timer += desired_interval;
		systime_t current_time = chVTGetSystemTimeX();
		systime_t remainin_sleep_time = iteration_timer - current_time;
		if (remainin_sleep_time > 0 && remainin_sleep_time < desired_interval) {
			// Sleep the remaining time.
			chThdSleep(remainin_sleep_time);
		} else {
			// Read was too slow or CPU was too buzy, reset the schedule.
			iteration_timer = current_time;
			chThdSleep(desired_interval);
		}
	}
}
