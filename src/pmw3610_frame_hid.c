/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zmk/usb.h>
#include <zmk/hid.h>

#include "pmw3610.h"
#include "pmw3610_frame_hid.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610_frame_hid, CONFIG_PMW3610_ALT_LOG_LEVEL);

#if IS_ENABLED(CONFIG_PMW3610_ALT_FRAME_CAPTURE_HID_AUX)

#define PMW3610_FRAME_REPORT_ID 0x01
#define PMW3610_FRAME_REPORT_MAGIC 0xA5

/* preemptible, well below the system workqueue (-1) so capturing/input
 * never waits on USB
 */
#define PMW3610_FRAME_HID_THREAD_PRIO 7

struct pmw3610_frame_report {
    uint8_t report_id;
    uint8_t magic;  // PMW3610_FRAME_REPORT_MAGIC, lets the host find the
                    // payload offset regardless of report id handling
    uint8_t seq;    // frame sequence, low 8 bits (gaps => dropped frames)
    uint8_t chunk;  // 0-based chunk index
    uint8_t total;  // chunks per frame
    uint8_t len;    // valid payload bytes in this chunk
    uint8_t payload[PMW3610_FRAME_CHUNK_PAYLOAD];
} __packed;

BUILD_ASSERT(sizeof(struct pmw3610_frame_report) <= 64,
             "frame report must fit a 64-byte full speed packet");
BUILD_ASSERT(PMW3610_FRAME_CHUNKS * PMW3610_FRAME_CHUNK_PAYLOAD == PMW3610_FRAME_TOTAL,
             "chunks must tile the padded frame exactly, no short last chunk");

#ifndef HID_USAGE_PAGE16
#define HID_USAGE_PAGE16(page, page2)                                                              \
    HID_ITEM(HID_ITEM_TAG_USAGE_PAGE, HID_ITEM_TYPE_GLOBAL, 2), page, page2
#endif

static const uint8_t pmw3610_frame_report_desc[] = {
    HID_USAGE_PAGE16(0x0C, 0xFF), // vendor-defined usage page 0xFF0C
    HID_USAGE(0x01),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    HID_REPORT_ID(PMW3610_FRAME_REPORT_ID),
    HID_USAGE(0x02),
    HID_LOGICAL_MIN8(0x00),
    HID_LOGICAL_MAX16(0xFF, 0x00), // 0..255
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(sizeof(struct pmw3610_frame_report) - 1), // 61
    HID_INPUT(ZMK_HID_MAIN_VAL_DATA | ZMK_HID_MAIN_VAL_VAR | ZMK_HID_MAIN_VAL_ABS),
    HID_END_COLLECTION,
};

static const struct device *hid_dev;
static const struct device *frame_dev = DEVICE_DT_GET_ONE(pixart_pmw3610_alt);
static const struct device *volatile pending_dev;

static K_SEM_DEFINE(frame_ready_sem, 0, 1);
static K_SEM_DEFINE(hid_in_sem, 1, 1);

static struct pmw3610_frame_report last_sent_report; // for GET_REPORT
static uint8_t frame_copy[PMW3610_FRAME_PIXELS + PMW3610_FRAME_TAILER];

void pmw3610_frame_hid_signal(const struct device *dev) {
    pending_dev = dev;
    k_sem_give(&frame_ready_sem);
}

static void in_ready_cb(const struct device *dev) { k_sem_give(&hid_in_sem); }

#define HID_GET_REPORT_TYPE_MASK 0xff00
#define HID_GET_REPORT_ID_MASK 0x00ff
#define HID_REPORT_TYPE_INPUT 0x100
#define HID_REPORT_TYPE_FEATURE 0x300

static int get_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {
    if ((setup->wValue & HID_GET_REPORT_TYPE_MASK) != HID_REPORT_TYPE_INPUT &&
        (setup->wValue & HID_GET_REPORT_TYPE_MASK) != HID_REPORT_TYPE_FEATURE) {
        return -ENOTSUP;
    }

    switch (setup->wValue & HID_GET_REPORT_ID_MASK) {
    case PMW3610_FRAME_REPORT_ID:
        *data = (uint8_t *)&last_sent_report;
        *len = sizeof(last_sent_report);
        pmw3610_capture_request(frame_dev);
        break;
    default:
        LOG_ERR("Get: invalid report ID %d", setup->wValue & HID_GET_REPORT_ID_MASK);
        return -EINVAL;
    }

    return 0;
}

static const struct hid_ops ops = {
    .int_in_ready = in_ready_cb,
    .get_report = get_report_cb,
};

static int pmw3610_usb_send_report(const uint8_t *report, size_t len) {
    switch (zmk_usb_get_status()) {
    case USB_DC_SUSPEND:
        return usb_wakeup_request();
    case USB_DC_ERROR:
    case USB_DC_RESET:
    case USB_DC_DISCONNECTED:
    case USB_DC_UNKNOWN:
        return -ENODEV;
    default:
        if (k_sem_take(&hid_in_sem, K_MSEC(10)) != 0) {
            return -EAGAIN;
        }
        int err = hid_int_ep_write(hid_dev, report, len, NULL);
        if (err) {
            k_sem_give(&hid_in_sem);
        }
        return err;
    }
}

static void pmw3610_frame_hid_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        k_sem_take(&frame_ready_sem, K_FOREVER);

        const struct device *dev = pending_dev;
        if (dev == NULL) {
            continue;
        }

        uint32_t seq = 0;
        if (pmw3610_get_frame(dev, frame_copy, sizeof(frame_copy), &seq) != 0) {
            continue;
        }

        // LOG_DBG("frame chunks");

        for (uint8_t chunk = 0; chunk < PMW3610_FRAME_CHUNKS; chunk++) {
            struct pmw3610_frame_report report = {
                .report_id = PMW3610_FRAME_REPORT_ID,
                .magic = PMW3610_FRAME_REPORT_MAGIC,
                .seq = (uint8_t)seq,
                .chunk = chunk,
                .total = PMW3610_FRAME_CHUNKS,
            };
            size_t offset = chunk * PMW3610_FRAME_CHUNK_PAYLOAD;

            report.len = MIN(PMW3610_FRAME_CHUNK_PAYLOAD, PMW3610_FRAME_TOTAL - offset);
            memcpy(report.payload, &frame_copy[offset], report.len);
            last_sent_report = report;

            // LOG_HEXDUMP_DBG(&report, sizeof(report), "chunk");

            if (pmw3610_usb_send_report((const uint8_t *)&report, sizeof(report)) != 0) {
                break; // host gone / tx timeout, drop the rest of this frame
            }
        }
    }
}

K_THREAD_DEFINE(pmw3610_frame_hid_tid, 2048, pmw3610_frame_hid_thread, NULL, NULL, NULL,
                PMW3610_FRAME_HID_THREAD_PRIO, 0, 0);

static int pmw3610_frame_hid_init(void) {
    hid_dev = device_get_binding("HID_" STRINGIFY(CONFIG_PMW3610_ALT_FRAME_CAPTURE_HID_BINDING_NUM));
    if (hid_dev == NULL) {
        LOG_ERR("Unable to locate HID device, check CONFIG_USB_HID_DEVICE_COUNT "
                "and CONFIG_PMW3610_ALT_FRAME_CAPTURE_HID_BINDING_NUM");
        return -EINVAL;
    }

    usb_hid_register_device(hid_dev, pmw3610_frame_report_desc,
                            sizeof(pmw3610_frame_report_desc), &ops);

    usb_hid_init(hid_dev);

    LOG_INF("PMW3610 frame HID input report ready on HID_%d (%u bytes, %u chunks)",
            CONFIG_PMW3610_ALT_FRAME_CAPTURE_HID_BINDING_NUM,
            (unsigned int)sizeof(struct pmw3610_frame_report),
            (unsigned int)PMW3610_FRAME_CHUNKS);

    return 0;
}

SYS_INIT(pmw3610_frame_hid_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif // CONFIG_PMW3610_ALT_FRAME_CAPTURE_HID_AUX
