#pragma once

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_PMW3610_ALT_FRAME_CAPTURE_HID_AUX)

void pmw3610_frame_hid_signal(const struct device *dev);

#endif // CONFIG_PMW3610_ALT_FRAME_CAPTURE_HID_AUX
