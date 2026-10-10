#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_PMW3610_ALT_FRAME_CAPTURE)
#define PMW3610_FRAME_WIDTH 22
#define PMW3610_FRAME_HEIGHT 22
#define PMW3610_FRAME_PIXELS (PMW3610_FRAME_WIDTH * PMW3610_FRAME_HEIGHT)

/* Chunked vendor input report: the 484-byte frame plus PMW3610_FRAME_TAILER
 * bytes of zero pad (504 bytes total) are sent as 9 chunks of 56 bytes, so the
 * full report is 1 + 5 + 56 = 62 bytes and fits a single 64-byte full speed
 * interrupt packet. The 9th chunk carries pixels 448..483 then the pad.
 *
 * The pad rounds the frame up to an exact multiple of the chunk payload, so
 * there is never a short trailing chunk.
 */
#define PMW3610_FRAME_CHUNK_PAYLOAD 56
#define PMW3610_FRAME_TAILER                                                    \
    ((PMW3610_FRAME_CHUNK_PAYLOAD -                                             \
      (PMW3610_FRAME_PIXELS % PMW3610_FRAME_CHUNK_PAYLOAD)) %                   \
     PMW3610_FRAME_CHUNK_PAYLOAD)
#define PMW3610_FRAME_TOTAL (PMW3610_FRAME_PIXELS + PMW3610_FRAME_TAILER)
#define PMW3610_FRAME_CHUNKS (PMW3610_FRAME_TOTAL / PMW3610_FRAME_CHUNK_PAYLOAD)
#endif

/* device data structure */
struct pixart_data {
    const struct device          *dev;
    int64_t                      dx;
    int64_t                      dy;
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    int64_t                      last_smp_time;
    int64_t                      last_rpt_time;
#endif
    bool                         sw_smart_flag; // for pmw3610 smart algorithm

#if IS_ENABLED(CONFIG_PMW3610_ALT_FRAME_CAPTURE)
    uint8_t                      frame_buf[2][PMW3610_FRAME_PIXELS];
    uint8_t                      frame_rd;
    uint8_t                      frame_wr;
    uint32_t                     frame_seq;
    bool                         frame_valid;
    atomic_t                     capture_busy;
    uint32_t                     capture_last_request;
    atomic_t                     capture_running;
    struct k_mutex               frame_lock;

    struct k_work_delayable      capture_work;
#endif

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;

    bool                         ready; // whether init is finished successfully
    int                          err; // error code during async init
};

// device config data structure
struct pixart_config {
	struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
    bool swap_xy;
    bool inv_x;
    bool inv_y;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool force_awake;
    bool force_awake_4ms_mode;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
