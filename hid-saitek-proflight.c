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

struct proflight {
        int initialized;
        __u32 product_id;
        const char *proc_name;
        struct proc_dir_entry *proflight_dir_entry;
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
                return 0;
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

static int saitek_proflight_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
        struct proflight *driver_data;
        int res = 0;

        printk("Saitek ProFlight driver probe...");

        driver_data = devm_kzalloc(&hdev->dev, sizeof(struct proflight), GFP_KERNEL);
        if (!driver_data) {
                hid_err(hdev, "Cannot allocate memory for Saitek ProFlight driver data.\n");
                res = -ENOMEM;
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

        res = hid_parse(hdev);
        if (res) {
                hid_err(hdev, "Initial HID parse failed for Saitek ProFlight driver.\n");
                goto fail_proc;
        }

        res = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
        if (res) {
                hid_err(hdev, "Initial HID hw start failed for Saitek ProFlight driver.\n");
                goto fail_proc;
        }

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
    
        printk("Saitek ProFlight driver remove...");

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

//module_hid_driver(saitek_proflight_driver);

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
