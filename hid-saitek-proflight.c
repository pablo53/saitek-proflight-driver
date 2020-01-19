// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HID driver for Saitek Pro Flight series devices.
 * 
 * Copyright (c) 2020 Paweł A. Ryszawa
 */

/*
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/kernel.h>

#ifndef USB_VENDOR_ID_SAITEK
#  define USB_VENDOR_ID_SAITEK 0x06a3
#endif

#define USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL 0x0d05
#define USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL 0x0d06

typedef struct {
    int initialized;
} proflight;

static int saitek_proflight_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
    proflight *driver_data;
    int ret;

    printk("Saitek ProFlight driver probe...");

    driver_data = devm_kzalloc(&hdev->dev, sizeof(proflight), GFP_KERNEL);
    if (!driver_data) {
        hid_err(hdev, "Cannot allocate memory for Saitek ProFlight driver data.\n");
        return -ENOMEM;
    }
    hid_set_drvdata(hdev, driver_data);
    
    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "Initial HID parse failed for Saitek ProFlight driver.\n");
        return ret;
    }

    ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
    if (ret) {
        hid_err(hdev, "Initial HID hw start failed for Saitek ProFlight driver.\n");
        return ret;
    }

    driver_data->initialized = 1;

    return 0;
}

void saitek_proflight_remove(struct hid_device *dev)
{
    proflight *driver_data;
    
    printk("Saitek ProFlight driver remove...");

    driver_data = hid_get_drvdata(dev);
    if (driver_data->initialized)
        hid_hw_stop(dev);
}

static __u8* saitek_proflight_report_fixup(struct hid_device *hdev, __u8 *buf, unsigned int *size)
{
    printk("Saitek ProFlight driver report fixup...");
    return buf; // no fix at the moment
}

static int saitek_proflight_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
    printk("Saitek ProFlight driver raw event...");

    return 0; // -1 = error, 0 = continue processing, 1 = no further processing
}

static int saitek_proflight_event(struct hid_device *hdev, struct hid_field *field, struct hid_usage *usage, __s32 value)
{
    printk("Saitek ProFlight driver event...");

    return 1; // -1 = error, 0 = continue processing, 1 = no further processing
}

static const struct hid_device_id saitek_proflight_devices[] = {
    { HID_USB_DEVICE(USB_VENDOR_ID_SAITEK, USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL) },
    { HID_USB_DEVICE(USB_VENDOR_ID_SAITEK, USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL) },
    { }
};

MODULE_DEVICE_TABLE(hid, saitek_proflight_devices);

static struct hid_driver saitek_proflight_driver = {
	.name = "saitek_proflight",
	.id_table = saitek_proflight_devices,
	.probe = saitek_proflight_probe,
    .remove = saitek_proflight_remove,
	.report_fixup = saitek_proflight_report_fixup,
	.raw_event = saitek_proflight_raw_event,
	.event = saitek_proflight_event,
};
module_hid_driver(saitek_proflight_driver);


MODULE_LICENSE("GPL");
