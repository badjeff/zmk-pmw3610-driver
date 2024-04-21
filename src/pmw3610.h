#pragma once

#include <zephyr/drivers/sensor.h>
#include "pixart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timings (in us) used in SPI communication. Since MCU should not do other tasks during wait,
 * k_busy_wait is used instead of k_sleep */
// - sub-us time is rounded to us, due to the limitation of k_busy_wait, see :
// https://github.com/zephyrproject-rtos/zephyr/issues/6498
#define T_NCS_SCLK 1     /* 120 ns (rounded to 1us) */
#define T_SCLK_NCS_WR 10 /* 10 us */
#define T_SRAD 4         /* 4 us */
#define T_SRAD_MOTBR 4   /* same as T_SRAD */
#define T_SRX 1          /* 250 ns (rounded to 1 us) */
#define T_SWX 30         /* SWW: 30 us, SWR: 20 us */
#define T_BEXIT 1        /* 250 ns (rounded to 1us)*/

/* Sensor registers (addresses) */
#define PMW3610_REG_PRODUCT_ID 0x00
#define PMW3610_REG_REVISION_ID 0x01
#define PMW3610_REG_MOTION 0x02
#define PMW3610_REG_DELTA_X_L 0x03
#define PMW3610_REG_DELTA_Y_L 0x04
#define PMW3610_REG_DELTA_XY_H 0x05
#define PMW3610_REG_SQUAL 0x06
#define PMW3610_REG_SHUTTER_HIGHER 0x07
#define PMW3610_REG_SHUTTER_LOWER 0x08
#define PMW3610_REG_PIX_MAX 0x09
#define PMW3610_REG_PIX_AVG 0x0A
#define PMW3610_REG_PIX_MIN 0x0B

#define PMW3610_REG_CRC0 0x0C
#define PMW3610_REG_CRC1 0x0D
#define PMW3610_REG_CRC2 0x0E
#define PMW3610_REG_CRC3 0x0F
#define PMW3610_REG_SELF_TEST 0x10

#define PMW3610_REG_PERFORMANCE 0x11
#define PMW3610_REG_MOTION_BURST 0x12

#define PMW3610_REG_RUN_DOWNSHIFT 0x1B
#define PMW3610_REG_REST1_RATE 0x1C
#define PMW3610_REG_REST1_DOWNSHIFT 0x1D
#define PMW3610_REG_REST2_RATE 0x1E
#define PMW3610_REG_REST2_DOWNSHIFT 0x1F
#define PMW3610_REG_REST3_RATE 0x20
#define PMW3610_REG_OBSERVATION 0x2D

#define PMW3610_REG_PIXEL_GRAB 0x35
#define PMW3610_REG_FRAME_GRAB 0x36

#define PMW3610_REG_POWER_UP_RESET 0x3A
#define PMW3610_REG_SHUTDOWN 0x3B

#define PMW3610_REG_SPI_CLK_ON_REQ 0x41
#define PMW3610_REG_RES_STEP 0x85

#define PMW3610_REG_NOT_REV_ID 0x3E
#define PMW3610_REG_NOT_PROD_ID 0x3F

#define PMW3610_REG_PRBS_TEST_CTL 0x47
#define PMW3610_REG_SPI_PAGE0 0x7F
#define PMW3610_REG_VCSEL_CTL 0x9E
#define PMW3610_REG_LSR_CONTROL 0x9F
#define PMW3610_REG_SPI_PAGE1 0xFF

/* Sensor identification values */
#define PMW3610_PRODUCT_ID 0x3E

/* Power-up register commands */
#define PMW3610_POWERUP_CMD_RESET 0x5A
#define PMW3610_POWERUP_CMD_WAKEUP 0x96

/* spi clock enable/disable commands */
#define PMW3610_SPI_CLOCK_CMD_ENABLE 0xBA
#define PMW3610_SPI_CLOCK_CMD_DISABLE 0xB5

/* Max register count readable in a single motion burst */
#define PMW3610_MAX_BURST_SIZE 10

/* Register count used for reading a single motion burst */
#define PMW3610_BURST_SIZE 7

/* Position in the motion registers */
#define PMW3610_X_L_POS 1
#define PMW3610_Y_L_POS 2
#define PMW3610_XY_H_POS 3
#define PMW3610_SHUTTER_H_POS 5
#define PMW3610_SHUTTER_L_POS 6

/* cpi/resolution range */
#define PMW3610_MAX_CPI 3200
#define PMW3610_MIN_CPI 200

/* write command bit position */
#define SPI_WRITE_BIT BIT(7)

/* Helper macros used to convert sensor values. */
#define PMW3610_SVALUE_TO_CPI(svalue) ((uint32_t)(svalue).val1)
#define PMW3610_SVALUE_TO_TIME(svalue) ((uint32_t)(svalue).val1)

/** @brief Sensor specific attributes of PMW3610. */
enum pmw3610_attribute {

	/** Sensor CPI for both X and Y axes. */
	PMW3610_ATTR_CPI,

	/** Entering time from Run mode to REST1 mode [ms]. */
	PMW3610_ATTR_RUN_DOWNSHIFT_TIME,

	/** Entering time from REST1 mode to REST2 mode [ms]. */
	PMW3610_ATTR_REST1_DOWNSHIFT_TIME,

	/** Entering time from REST2 mode to REST3 mode [ms]. */
	PMW3610_ATTR_REST2_DOWNSHIFT_TIME,

	/** Sampling frequency time during REST1 mode [ms]. */
	PMW3610_ATTR_REST1_SAMPLE_TIME,

	/** Sampling frequency time during REST2 mode [ms]. */
	PMW3610_ATTR_REST2_SAMPLE_TIME,

	/** Sampling frequency time during REST3 mode [ms]. */
	PMW3610_ATTR_REST3_SAMPLE_TIME,

};

#ifdef __cplusplus
}
#endif
