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
#include <linux/ctype.h>


#define SWITCH(x) ((x) ? "ON " : "OFF")

#define CHECK(x) ((x) ? 1 : 0)

#define MULTIPANEL_MODE(x) ( \
                ((x) == MULTIPANEL_MODE_ALT) ? "ALT" : \
                ((x) == MULTIPANEL_MODE_VS)  ? "VS " : \
                ((x) == MULTIPANEL_MODE_IAS) ? "IAS" : \
                ((x) == MULTIPANEL_MODE_HDG) ? "HDG" : \
                ((x) == MULTIPANEL_MODE_CRS) ? "CRS" : \
                "   ")

#define RADIOPANEL_MODE(x) ( \
                ((x) == RADIOPANEL_MODE_COM1) ? "COM1" : \
                ((x) == RADIOPANEL_MODE_COM2) ? "COM2" : \
                ((x) == RADIOPANEL_MODE_NAV1) ? "NAV1" : \
                ((x) == RADIOPANEL_MODE_NAV2) ? "NAV2" : \
                ((x) == RADIOPANEL_MODE_ADF) ? "ADF " : \
                ((x) == RADIOPANEL_MODE_DME) ? "DME " : \
                ((x) == RADIOPANEL_MODE_XPDR) ? "XPDR" : \
                "   ")

#define PANEL_DIGIT_MINUS 0xe0
#define PANEL_DIGIT_DOT   0xd0
#define PANEL_DIGIT_NULL  0xff

#define MULTIPANEL_LIGHT_AP  0x01
#define MULTIPANEL_LIGHT_HDG 0x02
#define MULTIPANEL_LIGHT_NAV 0x04
#define MULTIPANEL_LIGHT_IAS 0x08
#define MULTIPANEL_LIGHT_ALT 0x10
#define MULTIPANEL_LIGHT_VS  0x20
#define MULTIPANEL_LIGHT_APR 0x40
#define MULTIPANEL_LIGHT_REV 0x80

#ifndef USB_VENDOR_ID_SAITEK
#  define USB_VENDOR_ID_SAITEK 0x06a3
#endif

#ifndef USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL
#  define USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL 0x0d05
#endif
#ifndef USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL
#  define USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL 0x0d06
#endif

#define MAX_BUFFER 256
#define SAITEK_HID_BUF_LEN 13

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
        struct proflight *parent;
        unsigned int act_stby1 : 1;
        unsigned int act_stby2 : 1;
        int mode1;
        int mode2;
        int knob1;
        int knob2;
};

struct proflight_multipanel {
        struct proflight *parent;
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
        char display0[5];
        char display1[5];
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
        struct hid_device *hdev;
        __u8 *dmabuf;
        char buffer[MAX_BUFFER];
};

static int saitek_parse_multipanel_display(char *display, const char *msg, size_t size)
{
        int dig = 0;
        int i = 0;
        char ch;

        while (dig < 5 && i < size) {
                ch = msg[i++];
                if (ch == ' ')
                        display[dig++] = PANEL_DIGIT_NULL;
                else if (ch == '-')
                        display[dig++] = PANEL_DIGIT_MINUS;
                else if (ch == '.')
                        display[dig++] = PANEL_DIGIT_DOT;
                else if (isdigit(ch))
                        display[dig++] = ch - '0';
                else
                        display[dig++] = PANEL_DIGIT_NULL;
        }

        return i;
}

static ssize_t saitek_proc_read_radiopanel(struct proflight_radiopanel *radiopanel,
                struct file *f, char __user *buf, size_t count, loff_t *offset)
{
        return 0; // TODO
}

static ssize_t saitek_proc_read_multipanel(struct proflight_multipanel *multipanel,
                struct file *f, char __user *buf, size_t count, loff_t *offset)
{
        char sbuf[MAX_BUFFER];
        int res;
        int outlen;

        res = snprintf(sbuf, MAX_BUFFER,
                        "MODE:%s HDG:%s NAV:%s IAS:%s ALT:%s VS:%s APR:%s REV:%s AP:%s "
                        "AUTO-THROTTLE:%s FLAPS:%3d PITCH-TRIM:%3d KNOB:%3d",
                        MULTIPANEL_MODE(multipanel->mode), SWITCH(multipanel->hdg),
                        SWITCH(multipanel->nav), SWITCH(multipanel->ias),
                        SWITCH(multipanel->alt), SWITCH(multipanel->vs),
                        SWITCH(multipanel->apr), SWITCH(multipanel->rev),
                        SWITCH(multipanel->ap), SWITCH(multipanel->auto_throttle),
                        multipanel->flaps, multipanel->pitch_trim, multipanel->knob);
        outlen = strlen(sbuf);

        if (*offset >= outlen)
                return 0;
        if (*offset + count >= outlen)
                count = outlen - *offset;
        copy_to_user(buf, &sbuf[*offset], count);
        *offset += count;

        return count;
}

static ssize_t saitek_proc_read(struct file *f, char __user *buf, size_t count, loff_t *offset)
{
        struct proflight *driver_data;

        driver_data = PDE_DATA(file_inode(f));
        if (!driver_data) {
                printk(KERN_ERR "Cannot find Saitek ProFlight device data.\n");
                return -EIO;
        }

        switch(driver_data->product_id) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                return saitek_proc_read_radiopanel(driver_data->data.radiopanel,
                                f, buf, count, offset);
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                return saitek_proc_read_multipanel(driver_data->data.multipanel,
                                f, buf, count, offset);
        }

        return -ENXIO; // default
}

static ssize_t saitek_proc_write_radiopanel(struct proflight_radiopanel *radiopanel,
                struct file *f, const char __user *buf, size_t count, loff_t *offset)
{
        if (*offset + count >= MAX_BUFFER)
                count = MAX_BUFFER - *offset;
        copy_from_user(&(radiopanel->parent->buffer[*offset]), buf, count);
        *offset += count;

        return count;
}

static ssize_t saitek_proc_write_multipanel(struct proflight_multipanel *multipanel,
                struct file *f, const char __user *buf, size_t count, loff_t *offset)
{
        int res;

        if (*offset + count >= MAX_BUFFER)
                count = MAX_BUFFER - *offset;
        copy_from_user(&(multipanel->parent->buffer[*offset]), buf, count);
        *offset += count;

        multipanel->parent->dmabuf[0] = 0; // also: report id
        saitek_parse_multipanel_display(&(multipanel->parent->dmabuf[1]), &(multipanel->parent->buffer[0]), 5);
        saitek_parse_multipanel_display(&(multipanel->parent->dmabuf[6]), &(multipanel->parent->buffer[6]), 5);
        multipanel->parent->dmabuf[11] = 0; // TODO: set as per MULTIPANEL_LIGHT_* masks
        multipanel->parent->dmabuf[12] = 0;

        res = hid_hw_raw_request(multipanel->parent->hdev,
                        multipanel->parent->dmabuf[0], multipanel->parent->dmabuf,
                        13, HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
        if (res < 0)
                printk(KERN_ERR "Error sending report: %d.\n", res);

        return count;
}

static ssize_t saitek_proc_write(struct file *f, const char __user *buf, size_t count, loff_t *offset)
{
        struct proflight *driver_data;

        if (*offset >= MAX_BUFFER)
                return EFBIG;
        driver_data = PDE_DATA(file_inode(f));
        if (!driver_data) {
                printk(KERN_ERR "Cannot find Saitek ProFlight device data.\n");
                return -EIO;
        }
        
        switch(driver_data->product_id) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                return saitek_proc_write_radiopanel(driver_data->data.radiopanel,
                                f, buf, count, offset);
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                return saitek_proc_write_multipanel(driver_data->data.multipanel,
                                f, buf, count, offset);
        }

        return -ENXIO;
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
        switch (id->product) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                panel_data.radiopanel->parent = *driver_data_p;
                break;
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                panel_data.multipanel->parent = *driver_data_p;
                break;
        }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        (*driver_data_p)->data = panel_data;
#pragma GCC diagnostic pop

        (*driver_data_p)->dmabuf = devm_kzalloc(&hdev->dev, SAITEK_HID_BUF_LEN, GFP_KERNEL);
        if (!((*driver_data_p)->dmabuf)) {
                hid_err(hdev, "No memory for Saitek ProFlight HID buffer.\n");
                return -ENOMEM;
        }

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
        driver_data->hdev = hdev;

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
        multipanel->hdg             = CHECK(data[1] & 0x01);
        multipanel->nav             = CHECK(data[1] & 0x02);
        multipanel->ias             = CHECK(data[1] & 0x04);
        multipanel->alt             = CHECK(data[1] & 0x08);
        multipanel->vs              = CHECK(data[1] & 0x10);
        multipanel->apr             = CHECK(data[1] & 0x20);
        multipanel->rev             = CHECK(data[1] & 0x40);
        multipanel->ap              = CHECK(data[0] & 0x80); // unnecessary GCC warning on overflow in conversion here
        multipanel->flaps_up        = CHECK(data[2] & 0x01);
        multipanel->flaps_down      = CHECK(data[2] & 0x02);
        multipanel->auto_throttle   = CHECK(data[1] & 0x80); // unnecessary GCC warning on overflow in conversion here
        multipanel->pitch_trim_up   = CHECK(data[2] & 0x08);
        multipanel->pitch_trim_down = CHECK(data[2] & 0x04);
        multipanel->knob_right      = CHECK(data[0] & 0x20);
        multipanel->knob_left       = CHECK(data[0] & 0x40);
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
