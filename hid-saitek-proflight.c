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

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#ifndef USB_VENDOR_ID_SAITEK
#  define USB_VENDOR_ID_SAITEK 0x06a3
#endif

#ifndef USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL
#  define USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL 0x0d05
#endif
#ifndef USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL
#  define USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL 0x0d06
#endif

#define MAX_BUFFER 8

#define RADIOPANEL_MODE_COM1 1
#define RADIOPANEL_MODE_COM2 2
#define RADIOPANEL_MODE_NAV1 3
#define RADIOPANEL_MODE_NAV2 4
#define RADIOPANEL_MODE_ADF  5
#define RADIOPANEL_MODE_DME  6
#define RADIOPANEL_MODE_XPDR 7

#define MULTIPANEL_MODE_ALT 1
#define MULTIPANEL_MODE_VS  2
#define MULTIPANEL_MODE_IAS 3
#define MULTIPANEL_MODE_HDG 4
#define MULTIPANEL_MODE_CRS 5

struct proflight_radiopanel {
        unsigned int act_stby1 : 1;
        unsigned int act_stby2 : 1;
        int mode1;
        int mode2;
        int knob1;
        int knob2;
};

struct proflight_multipanel {
        unsigned int hdg : 1;
        unsigned int nav : 1;
        unsigned int ias : 1;
        unsigned int alt : 1;
        unsigned int vs : 1;
        unsigned int apr : 1;
        unsigned int rev : 1;
        unsigned int ap : 1;
        unsigned int flaps_up : 1;
        unsigned int flaps_down : 1;
        unsigned int auto_throttle : 1;
        unsigned int pitch_trim_up : 1;
        unsigned int pitch_trim_down : 1;
        unsigned int knob_right : 1;
        unsigned int knob_left : 1;
        int mode;
        int flaps; // accumulated movement (DN - decreases, UP - increases)
        int pitch_trim; // accumulated movement (DN - decreases, UP - increases)
        int knob; // accumulated movement (left - decreases, right - increases)
};

union proflight_panel_data
{
        struct proflight_radiopanel *radiopanel;
        struct proflight_multipanel *multipanel;
};

struct proflight {
        int initialized;
        __u32 product_id;
        const char *proc_name;
        struct proc_dir_entry *proflight_dir_entry;
        union proflight_panel_data data;
        char buffer[MAX_BUFFER];
};

static ssize_t saitek_proc_read(struct file *f, char __user *buf, size_t count, loff_t *offset)
{
        struct proflight *data;

        if (*offset >= MAX_BUFFER)
                return 0;
        data = PDE_DATA(file_inode(f));
        if (!data) {
                printk(KERN_ERR "Cannot find Saitek ProFlight device writer data.\n");
                return -EIO;
        }
        if (*offset + count >= MAX_BUFFER)
                count = MAX_BUFFER - *offset;
        copy_to_user(buf, data->buffer, count);
        *offset += count;

        return count;
}

static ssize_t saitek_proc_write(struct file *f, const char __user *buf, size_t count, loff_t *offset)
{
        struct proflight *data;

        if (*offset >= MAX_BUFFER)
                return EFBIG;
        data = PDE_DATA(file_inode(f));
        if (!data) {
                printk(KERN_ERR "Cannot find Saitek ProFlight device reader data.\n");
                return -EIO;
        }
        if (*offset + count >= MAX_BUFFER)
                count = MAX_BUFFER - *offset;
        copy_from_user(data->buffer, buf, count);
        *offset += count;

        return count;
}

static const struct file_operations saitek_proc_fops = {
        .read = saitek_proc_read,
        .write = saitek_proc_write,
};

static inline const char* proc_name(__u32 product_id)
{
        switch (product_id) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                return "proflight-radiopanel";
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                return "proflight-multipanel";
        default:
                return NULL;
        }
}

static int saitek_proflight_alloc_panel_data(
                struct hid_device *hdev, const struct hid_device_id *id,
                struct proflight **driver_data_p)
{
        union proflight_panel_data panel_data;

        switch (id->product) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                panel_data.radiopanel = devm_kzalloc(&hdev->dev, sizeof(struct proflight_radiopanel), GFP_KERNEL);
                if (!panel_data.radiopanel) {
                        hid_err(hdev, "No memory for Saitek ProFlight Radio Panel data.\n");
                        return -ENOMEM;
                }
                break;
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                panel_data.multipanel = devm_kzalloc(&hdev->dev, sizeof(struct proflight_multipanel), GFP_KERNEL);
                if (!panel_data.multipanel) {
                        hid_err(hdev, "No memory for Saitek ProFlight Multi Panel data.\n");
                        return -ENOMEM;
                }
                break;
        }
        *driver_data_p = devm_kzalloc(&hdev->dev, sizeof(struct proflight), GFP_KERNEL);
        if (!(*driver_data_p)) {
                hid_err(hdev, "No memory for Saitek ProFlight driver data.\n");
                return -ENOMEM;
        }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        (*driver_data_p)->data = panel_data;
#pragma GCC diagnostic pop

        return 0;
}

static int saitek_proflight_hid_start(struct hid_device *hdev)
{
        int res;

        res = hid_parse(hdev);
        if (res) {
                hid_err(hdev, "Initial HID parse failed for Saitek ProFlight driver.\n");
                return res;
        }

        res = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
        if (res) {
                hid_err(hdev, "Initial HID hw start failed for Saitek ProFlight driver.\n");
                return res;
        }

        return 0;
}

static int saitek_proflight_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
        struct proflight *driver_data;
        int res = 0;

        hid_info(hdev, "Saitek ProFlight driver probe.\n");

        res = saitek_proflight_alloc_panel_data(hdev, id, &driver_data);
        if (res) {
                hid_err(hdev, "Failed to initialize panel data.\n");
                goto fail;
        }
        hid_set_drvdata(hdev, driver_data);
        driver_data->product_id = id->product;
        driver_data->proc_name = proc_name(id->product);

        driver_data->proflight_dir_entry = proc_create_data(driver_data->proc_name,
                        0666, NULL, &saitek_proc_fops, driver_data);
        if (!driver_data->proflight_dir_entry) {
                hid_err(hdev, "Cannot create '/proc/%s' entry.\n", driver_data->proc_name);
                res = -EIO;
                goto fail_proc;
        }

        res = saitek_proflight_hid_start(hdev);
        if (res)
                goto fail_proc;

        driver_data->initialized = 1;

        goto exit;

fail_proc:
        remove_proc_entry(driver_data->proc_name, NULL);
fail:
exit:
        return res;
}

void saitek_proflight_remove(struct hid_device *hdev)
{
        struct proflight *driver_data;
    
        printk("Saitek ProFlight driver remove...\n");

        driver_data = hid_get_drvdata(hdev);
        if (!driver_data) {
                hid_err(hdev, "Saitek ProFlight driver data not found.\n");
                return;
        }
        if (driver_data->initialized) {
                hid_hw_stop(hdev);
                remove_proc_entry(driver_data->proc_name, NULL);
                driver_data->initialized = 0;
        }
}

static int saitek_proflight_multipanel_raw_event(
                struct proflight_multipanel *multipanel,
                struct hid_device *hdev, struct hid_report *report, u8 *data,
                int size)
{
        if (report->id != 0 || report->type != 0)
                return 0; // Uknown report, let it be processed the default way.
        if (size < 3)
                return -1; // We expect 3 bytes
        
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverflow"
        multipanel->hdg             = data[1] & 0x01;
        multipanel->nav             = data[1] & 0x02;
        multipanel->ias             = data[1] & 0x04;
        multipanel->alt             = data[1] & 0x08;
        multipanel->vs              = data[1] & 0x10;
        multipanel->apr             = data[1] & 0x20;
        multipanel->hdg             = data[1] & 0x40;
        multipanel->ap              = data[0] & 0x80; // unnecessary GCC warning on overflow in conversion here
        multipanel->flaps_up        = data[2] & 0x01;
        multipanel->flaps_down      = data[2] & 0x02;
        multipanel->auto_throttle   = data[1] & 0x80; // unnecessary GCC warning on overflow in conversion here
        multipanel->pitch_trim_up   = data[2] & 0x08;
        multipanel->pitch_trim_down = data[2] & 0x04;
        multipanel->knob_right      = data[0] & 0x02;
        multipanel->knob_left       = data[0] & 0x04;
#pragma GCC diagnostic pop
        if (data[0] & 0x01)
                multipanel->mode = MULTIPANEL_MODE_ALT;
        else if (data[0] & 0x02)
                multipanel->mode = MULTIPANEL_MODE_VS;
        else if (data[0] & 0x04)
                multipanel->mode = MULTIPANEL_MODE_IAS;
        else if (data[0] & 0x08)
                multipanel->mode = MULTIPANEL_MODE_HDG;
        else if (data[0] & 0x10)
                multipanel->mode = MULTIPANEL_MODE_CRS;
        else
                multipanel->mode = 0; // should never occur
        
        // TODO: some kind of a software debouncer
        if (multipanel->flaps_up)
                multipanel->flaps++;
        else if (multipanel->flaps_down)
                multipanel->flaps--;
        if (multipanel->pitch_trim_up)
                multipanel->pitch_trim++;
        else if (multipanel->pitch_trim_down)
                multipanel->pitch_trim--;
        if (multipanel->knob_right)
                multipanel->knob++;
        else if (multipanel->knob_left)
                multipanel->knob--;

        return 1;
}

static int saitek_proflight_radiopanel_raw_event(
                struct proflight_radiopanel *radiopanel,
                struct hid_device *hdev, struct hid_report *report, u8 *data,
                int size)
{

        return 1;
}

static int saitek_proflight_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
        struct proflight *driver_data;

        driver_data = hid_get_drvdata(hdev);
        if (!driver_data)
                return -1;
        switch (driver_data->product_id) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                return saitek_proflight_multipanel_raw_event(driver_data->data.multipanel, hdev, report, data, size);
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                return saitek_proflight_radiopanel_raw_event(driver_data->data.radiopanel, hdev, report, data, size);
        default:
                return -1;
        }
}

static int saitek_proflight_event(struct hid_device *hdev, struct hid_field *field, struct hid_usage *usage, __s32 value)
{
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
        .raw_event = saitek_proflight_raw_event,
        .event = saitek_proflight_event,
};

static int __init saitek_proflight_init(void)
{
        int ret;
        
        ret = hid_register_driver(&saitek_proflight_driver);
        if (ret)
                printk(KERN_ERR "Cannot register Saitek Pro Flight driver (err %i).\n", ret);
        
        return ret;
}

static void __exit saitek_proflight_exit(void)
{
        hid_unregister_driver(&saitek_proflight_driver);
}

module_init(saitek_proflight_init);
module_exit(saitek_proflight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paweł A. Ryszawa <pablo53@poczta.onet.eu>");
MODULE_DESCRIPTION("A driver for Saitek ProFlight series of HIDs.");
