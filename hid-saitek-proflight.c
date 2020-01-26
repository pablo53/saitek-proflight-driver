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
#include <linux/gfp.h>

#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/rwsem.h>


#define SAITEK_LOCK_TYPE          struct rw_semaphore *
#define SAITEK_LOCK_SIZE          sizeof(struct rw_semaphore)
#define SAITEK_INIT_LOCK(lock)    init_rwsem(lock)
#define SAITEK_LOCK_READ(lock)    down_read(lock)
#define SAITEK_UNLOCK_READ(lock)  up_read(lock)
#define SAITEK_LOCK_WRITE(lock)   down_write(lock)
#define SAITEK_UNLOCK_WRITE(lock) up_write(lock)

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

#define PANEL_DIGIT_MINUS ((char)0x0e)
#define PANEL_DIGIT_DOT   ((char)0xd0)
#define PANEL_DIGIT_NULL  ((char)0x0f)

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

#define MAX_BUFFER PAGE_SIZE
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

#define SAITEK_MAX_BTN 9

#define SAITEK_MAX_FLAPS       99
#define SAITEK_MIN_FLAPS      -99
#define SAITEK_MAX_PITCH_TRIM  99
#define SAITEK_MIN_PITCH_TRIM -99
#define SAITEK_MAX_KNOB        99
#define SAITEK_MIN_KNOB       -99

struct proflight_radiopanel {
        struct proflight *parent;
        unsigned int actstby0 : 1;
        unsigned int actstby1 : 1;
        unsigned int innerknob0_right : 1;
        unsigned int innerknob0_left : 1;
        unsigned int outerknob0_right : 1;
        unsigned int outerknob0_left : 1;
        unsigned int innerknob1_right : 1;
        unsigned int innerknob1_left : 1;
        unsigned int outerknob1_right : 1;
        unsigned int outerknob1_left : 1;
        int mode0;
        int mode1;
        int aactstby0; // accumulated
        int aactstby1; // accumulated
        int innerknob0; // accumulated movement (left - decreases, right - increases)
        int outerknob0; // accumulated movement (left - decreases, right - increases)
        int innerknob1; // accumulated movement (left - decreases, right - increases)
        int outerknob1; // accumulated movement (left - decreases, right - increases)
        char display0l[5];
        char display0r[5];
        char display1l[5];
        char display1r[5];
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
        int ahdg; // accumulated hdg
        int anav; // accumulated nav
        int aias; // accumulated ias
        int aalt; // accumulated alt
        int avs;  // accumualted vs
        int aapr; // accumulated apr
        int arev; // accumulated rev
        int aap;  // accumulated ap
        int flaps; // accumulated movement (DN - decreases, UP - increases)
        int pitch_trim; // accumulated movement (DN - decreases, UP - increases)
        int knob; // accumulated movement (left - decreases, right - increases)
        char display0[5];
        char display1[5];
        unsigned int led_hdg : 1;
        unsigned int led_nav : 1;
        unsigned int led_ias : 1;
        unsigned int led_alt : 1;
        unsigned int led_vs : 1;
        unsigned int led_apr : 1;
        unsigned int led_rev : 1;
        unsigned int led_ap : 1;
};

union proflight_panel_data
{
        struct proflight_radiopanel *radiopanel;
        struct proflight_multipanel *multipanel;
};

struct proflight {
        int initialized;
        SAITEK_LOCK_TYPE lock;
        char mode; // 'R' - reset Flaps, Pith Trim, Knob after every read; 'N' - normal
        __u32 product_id;
        union proflight_panel_data data;
        struct hid_device *hdev;
        __u8 *dmabuf;
};

static void saitek_parse_radiopanel_display(char *display, const char *buf, size_t bufsize)
{
        int digno = 0;
        int i = 0;
        char ch;

        while (digno < 5) {
                ch = (i < bufsize) ? buf[i++] : ' ';
                if (ch == ' ') {
                        display[digno++] = PANEL_DIGIT_NULL;
                } else if (isdigit(ch)) {
                        display[digno++] = ch - '0';
                } else if (ch == '.') {
                        if (digno == 0)
                                display[digno++] = PANEL_DIGIT_NULL;
                        display[digno - 1] |= PANEL_DIGIT_DOT;
                } else {
                        display[digno++] = PANEL_DIGIT_NULL;
                }
        }
        if (i < bufsize && digno > 0)
                if (buf[i] == '.')
                        display[digno - 1] |= PANEL_DIGIT_DOT;
}

static void saitek_parse_multipanel_display(char *display, const char *buf, size_t bufsize)
{
        int digno = 0;
        int i = 0;
        char ch;

        while (digno < 5 && i < bufsize) {
                ch = buf[i++];
                if (ch == ' ')
                        display[digno++] = PANEL_DIGIT_NULL;
                else if (ch == '-')
                        display[digno++] = PANEL_DIGIT_MINUS;
                else if (isdigit(ch))
                        display[digno++] = ch - '0';
                else
                        display[digno++] = PANEL_DIGIT_NULL;
        }
}

static int saitek_format_radiopanel_display(char *buf, const char *display, size_t bufsize)
{
        int chno = 0;
        int i = 0;
        char dig, digflag;

        while (chno < bufsize && i < 5) {
                dig = display[i] & (char)0x0f;
                digflag = display[i] & (char)0xf0;
                i++;
                if (dig == PANEL_DIGIT_NULL)
                        buf[chno++] = ' ';
                else if (0 <= dig && dig < 10)
                        buf[chno++] = '0' + dig;
                else
                        buf[chno++] = '?';
                if (digflag == PANEL_DIGIT_DOT)
                        buf[chno++] = '.';
        }

        return chno;
}

static void saitek_format_multipanel_display(char *buf, const char *display, size_t bufsize)
{
        int chno = 0;
        int i = 0;
        char dig;

        while (chno < bufsize && i < 5) {
                dig = display[i++];
                if (dig == PANEL_DIGIT_NULL)
                        buf[chno++] = ' ';
                else if (dig == PANEL_DIGIT_MINUS)
                        buf[chno++] = '-';
                else if (0 <= dig && dig < 10)
                        buf[chno++] = '0' + dig;
                else
                        buf[chno++] = ' ';
        }
}

static int saitek_buf_format_radiopanel(char *buf, struct proflight_radiopanel *radiopanel)
{
        int len;
        char hrdisp0l[11];
        char hrdisp0r[11];
        char hrdisp1l[11];
        char hrdisp1r[11];
        int len0l, len0r, len1l, len1r;

        len0l = saitek_format_radiopanel_display(hrdisp0l, radiopanel->display0l, 10);
        len0r = saitek_format_radiopanel_display(hrdisp0r, radiopanel->display0r, 10);
        len1l = saitek_format_radiopanel_display(hrdisp1l, radiopanel->display1l, 10);
        len1r = saitek_format_radiopanel_display(hrdisp1r, radiopanel->display1r, 10);
        hrdisp0l[len0l] = 0;
        hrdisp0r[len0r] = 0;
        hrdisp1l[len1l] = 0;
        hrdisp1r[len1r] = 0;
        len = snprintf(buf, MAX_BUFFER, "[RP] %-10.10s %-10.10s %-10.10s %-10.10s %c "
                        "%c %1.1d %3.2d %3.2d %4.4s "
                        "%c %1.1d %3.2d %3.2d %4.4s",
                        hrdisp0l, hrdisp0r, hrdisp1l, hrdisp1r, radiopanel->parent->mode,
                        radiopanel->actstby0 ? '1' : '0', radiopanel->aactstby0,
                        radiopanel->innerknob0, radiopanel->outerknob0,
                        RADIOPANEL_MODE(radiopanel->mode0),
                        radiopanel->actstby1 ? '1' : '0', radiopanel->aactstby1,
                        radiopanel->innerknob1, radiopanel->outerknob1,
                        RADIOPANEL_MODE(radiopanel->mode1)
        );
        switch (radiopanel->parent->mode) {
                case 'R':
                        radiopanel->aactstby0 = 0;
                        radiopanel->aactstby1 = 0;
                        radiopanel->innerknob0 = 0;
                        radiopanel->outerknob0 = 0;
                        radiopanel->innerknob1 = 0;
                        radiopanel->outerknob1 = 0;
                        break;
        }

        return len;
}

static int saitek_buf_format_multipanel(char *buf, struct proflight_multipanel *multipanel)
{
        int len;
        char hrdisp0[6];
        char hrdisp1[6];
        char leds[9];
        char btns[9];

        saitek_format_multipanel_display(hrdisp0, multipanel->display0, 5);
        saitek_format_multipanel_display(hrdisp1, multipanel->display1, 5);
        hrdisp0[5] = 0;
        hrdisp1[5] = 0;
        leds[0] = multipanel->led_hdg ? '1' : '0';
        leds[1] = multipanel->led_nav ? '1' : '0';
        leds[2] = multipanel->led_ias ? '1' : '0';
        leds[3] = multipanel->led_alt ? '1' : '0';
        leds[4] = multipanel->led_vs  ? '1' : '0';
        leds[5] = multipanel->led_apr ? '1' : '0';
        leds[6] = multipanel->led_rev ? '1' : '0';
        leds[7] = multipanel->led_ap  ? '1' : '0';
        leds[8] = 0;
        btns[0] = multipanel->hdg ? '1' : '0';
        btns[1] = multipanel->nav ? '1' : '0';
        btns[2] = multipanel->ias ? '1' : '0';
        btns[3] = multipanel->alt ? '1' : '0';
        btns[4] = multipanel->vs  ? '1' : '0';
        btns[5] = multipanel->apr ? '1' : '0';
        btns[6] = multipanel->rev ? '1' : '0';
        btns[7] = multipanel->ap  ? '1' : '0';
        btns[8] = 0;
        len = snprintf(buf, MAX_BUFFER,
                        "[MP] %5.5s %5.5s %8.8s %c %8.8s %c %+3.2d %+3.2d %+3.2d "
                        "%1.1d %1.1d %1.1d %1.1d %1.1d %1.1d %1.1d %1.1d %s\n"
                        "DEVICE TYPE: MULTI PANEL\n"
                        "MODE:%s\nHDG:%s (%1.1d)\nNAV:%s (%1.1d)\n"
                        "IAS:%s (%1.1d)\nALT:%s (%1.1d)\nVS:%s (%1.1d)"
                        "\nAPR:%s (%1.1d)\nREV:%s (%1.1d)\nAP:%s (%1.1d)\n"
                        "AUTO-THROTTLE:%s\nFLAPS:%3d\nPITCH-TRIM:%3d\nKNOB:%3d",
                        hrdisp0, hrdisp1, leds, multipanel->parent->mode, btns,
                        multipanel->auto_throttle ? '1' : '0',
                        multipanel->flaps, multipanel->pitch_trim, multipanel->knob,
                        multipanel->ahdg, multipanel->anav, multipanel->aias,
                        multipanel->aalt, multipanel->avs, multipanel->aapr,
                        multipanel->arev, multipanel->aap,
                        MULTIPANEL_MODE(multipanel->mode),
                        MULTIPANEL_MODE(multipanel->mode),
                        SWITCH(multipanel->hdg), multipanel->ahdg,
                        SWITCH(multipanel->nav), multipanel->anav,
                        SWITCH(multipanel->ias), multipanel->aias,
                        SWITCH(multipanel->alt), multipanel->aalt,
                        SWITCH(multipanel->vs), multipanel->avs,
                        SWITCH(multipanel->apr), multipanel->aapr,
                        SWITCH(multipanel->rev), multipanel->arev,
                        SWITCH(multipanel->ap), multipanel->aap,
                        SWITCH(multipanel->auto_throttle),
                        multipanel->flaps, multipanel->pitch_trim, multipanel->knob);
        switch (multipanel->parent->mode) {
                case 'R':
                        multipanel->flaps = 0;
                        multipanel->pitch_trim = 0;
                        multipanel->knob = 0;
                        multipanel->ahdg = 0;
                        multipanel->anav = 0;
                        multipanel->aias = 0;
                        multipanel->aalt = 0;
                        multipanel->avs  = 0;
                        multipanel->aapr = 0;
                        multipanel->arev = 0;
                        multipanel->aap  = 0;
                        break;
        }
        
        return len;
}

static int saitek_set_radiopanel(struct proflight_radiopanel *radiopanel)
{
        int res = 0;

        radiopanel->parent->dmabuf[0] = 0; // also: report id
        memcpy(&(radiopanel->parent->dmabuf[1]), radiopanel->display0l, 5);
        memcpy(&(radiopanel->parent->dmabuf[6]), radiopanel->display0r, 5);
        memcpy(&(radiopanel->parent->dmabuf[11]), radiopanel->display1l, 5);
        memcpy(&(radiopanel->parent->dmabuf[16]), radiopanel->display1r, 5);
        radiopanel->parent->dmabuf[21] = 0;
        radiopanel->parent->dmabuf[22] = 0;

        res = hid_hw_raw_request(radiopanel->parent->hdev,
                        radiopanel->parent->dmabuf[0], radiopanel->parent->dmabuf,
                        23, HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
        
        return res;
}

static void saitek_buf_parse_radiopanel(struct proflight_radiopanel *radiopanel,
                const char *buf, size_t count)
{
        if (count < 45) {
                hid_err(radiopanel->parent->hdev,
                                "Saitek ProFlight Radio Panel state ('%.*s') too short (%li).\n",
                                (int)count, buf, count);
                return;
        } else if (count > 45) {
                hid_warn(radiopanel->parent->hdev,
                                "Saitek ProFlight Radio Panel state ('%.*s') too long (%li).\n",
                                (int)count, buf, count);
        }
        saitek_parse_radiopanel_display(radiopanel->display0l, &buf[0], 10);
        saitek_parse_radiopanel_display(radiopanel->display0r, &buf[11], 10);
        saitek_parse_radiopanel_display(radiopanel->display1l, &buf[22], 10);
        saitek_parse_radiopanel_display(radiopanel->display1r, &buf[33], 10);

        switch (buf[44]) {
                case 'N':
                case 'R':
                        radiopanel->parent->mode = buf[44];
        }
}

static int saitek_set_multipanel(struct proflight_multipanel *multipanel)
{
        int res = 0;

        multipanel->parent->dmabuf[0] = 0; // also: report id
        memcpy(&(multipanel->parent->dmabuf[1]), multipanel->display0, 5);
        memcpy(&(multipanel->parent->dmabuf[6]), multipanel->display1, 5);
        multipanel->parent->dmabuf[11] =
                          (multipanel->led_hdg ? MULTIPANEL_LIGHT_HDG : 0)
                        + (multipanel->led_nav ? MULTIPANEL_LIGHT_NAV : 0)
                        + (multipanel->led_ias ? MULTIPANEL_LIGHT_IAS : 0)
                        + (multipanel->led_alt ? MULTIPANEL_LIGHT_ALT : 0)
                        + (multipanel->led_vs  ? MULTIPANEL_LIGHT_VS  : 0)
                        + (multipanel->led_apr ? MULTIPANEL_LIGHT_APR : 0)
                        + (multipanel->led_rev ? MULTIPANEL_LIGHT_REV : 0)
                        + (multipanel->led_ap  ? MULTIPANEL_LIGHT_AP  : 0);

        multipanel->parent->dmabuf[12] = 0;

        res = hid_hw_raw_request(multipanel->parent->hdev,
                        multipanel->parent->dmabuf[0], multipanel->parent->dmabuf,
                        13, HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
        
        return res;
}

static void saitek_buf_parse_multipanel(struct proflight_multipanel *multipanel,
                const char *buf, size_t count)
{
        if (count < 22) {
                hid_err(multipanel->parent->hdev,
                                "Saitek ProFlight Multi Panel state ('%.*s') too short (%li).\n",
                                (int)count, buf, count);
                return;
        } else if (count > 22) {
                hid_warn(multipanel->parent->hdev,
                                "Saitek ProFlight Multi Panel state ('%.*s') too long (%li).\n",
                                (int)count, buf, count);
        }
        saitek_parse_multipanel_display(multipanel->display0, &buf[0], 5);
        saitek_parse_multipanel_display(multipanel->display1, &buf[6], 5);
#define SET_IF_CHAR_BIN(variable,value) variable = value == '1' ? 1 : value == '0' ? 0 : variable
        SET_IF_CHAR_BIN(multipanel->led_hdg,buf[12]);
        SET_IF_CHAR_BIN(multipanel->led_nav,buf[13]);
        SET_IF_CHAR_BIN(multipanel->led_ias,buf[14]);
        SET_IF_CHAR_BIN(multipanel->led_alt,buf[15]);
        SET_IF_CHAR_BIN(multipanel->led_vs,buf[16]);
        SET_IF_CHAR_BIN(multipanel->led_apr,buf[17]);
        SET_IF_CHAR_BIN(multipanel->led_rev,buf[18]);
        SET_IF_CHAR_BIN(multipanel->led_ap,buf[19]);
#undef SET_IF_CHAR_BIN
        switch (buf[21]) {
                case 'N':
                case 'R':
                        multipanel->parent->mode = buf[21];
        }
}

static ssize_t saitek_proflight_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct proflight *driver_data;
        ssize_t ret_val;

        driver_data = dev_get_drvdata(dev);
        if (!driver_data) {
                printk(KERN_ERR "Cannot find Saitek ProFlight device data.\n");
                return -EIO;
        }

        SAITEK_LOCK_READ(driver_data->lock);
        switch(driver_data->product_id) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                ret_val = saitek_buf_format_radiopanel(buf,
                                driver_data->data.radiopanel);
                break;
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                ret_val = saitek_buf_format_multipanel(buf,
                                driver_data->data.multipanel);
                break;
        default:
                ret_val = -ENXIO;
        }
        SAITEK_UNLOCK_READ(driver_data->lock);

        return ret_val;
}

static ssize_t saitek_proflight_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct proflight *driver_data;
        ssize_t ret_val;
        size_t true_count;

        driver_data = dev_get_drvdata(dev);
        if (!driver_data) {
                printk(KERN_ERR "Cannot find Saitek ProFlight device data.\n");
                return -EIO;
        }

        true_count = count;
        if (count > 0 && buf[count- 1] == '\n')
                true_count--; // ignore trailing new line character
        SAITEK_LOCK_WRITE(driver_data->lock);
        switch(driver_data->product_id) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                saitek_buf_parse_radiopanel(driver_data->data.radiopanel, buf, true_count);
                ret_val = saitek_set_radiopanel(driver_data->data.radiopanel);
                if (ret_val < 0)
                        printk(KERN_ERR "Error setting Saitek ProFlight Radio Panel: %ld.\n", ret_val);
                else
                        ret_val = count; // informing the caller that we consumed the whole buffer
                break;
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                saitek_buf_parse_multipanel(driver_data->data.multipanel, buf, true_count);
                ret_val = saitek_set_multipanel(driver_data->data.multipanel);
                if (ret_val < 0)
                        printk(KERN_ERR "Error setting Saitek ProFlight Multi Panel: %ld.\n", ret_val);
                else
                        ret_val = count; // informing the caller that we consumed the whole buffer
                break;
        default:
                ret_val = -ENXIO;
        }
        SAITEK_UNLOCK_WRITE(driver_data->lock);

        return ret_val;
}

static DEVICE_ATTR(proflight,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
                saitek_proflight_show, saitek_proflight_store);

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

        (*driver_data_p)->dmabuf = devm_kzalloc(&hdev->dev, SAITEK_HID_BUF_LEN, GFP_DMA | GFP_KERNEL);
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
                goto exit;
        }
        driver_data->lock = devm_kzalloc(&hdev->dev, sizeof(SAITEK_LOCK_SIZE), GFP_KERNEL);
        if (!driver_data->lock) {
                hid_err(hdev, "Failed to initialize semaphore.\n");
                goto exit;
        }
        SAITEK_INIT_LOCK(driver_data->lock);
        SAITEK_LOCK_WRITE(driver_data->lock);
        res = device_create_file(&hdev->dev, &dev_attr_proflight);
        if (res) {
                hid_err(hdev, "Failed to initialize device attribuutes.\n");
                goto fail_dev;
        }
        driver_data->product_id = id->product;
        driver_data->hdev = hdev;
        driver_data->mode = 'R';
        hid_set_drvdata(hdev, driver_data);

        res = saitek_proflight_hid_start(hdev);
        if (res)
                goto fail_dev;

        driver_data->initialized = 1;

        goto exit_sem;

fail_dev:
        device_remove_file(&hdev->dev, &dev_attr_proflight);
exit_sem:
        SAITEK_UNLOCK_WRITE(driver_data->lock);
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
        SAITEK_LOCK_WRITE(driver_data->lock);
        if (driver_data->initialized) {
                hid_hw_stop(hdev);
                device_remove_file(&hdev->dev, &dev_attr_proflight);
                driver_data->initialized = 0;
        }
        SAITEK_UNLOCK_WRITE(driver_data->lock);
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
        
#define SAITEK_ADJUST_COUNTED_BTN(btn,byteno,mask) \
        if (CHECK(data[byteno] & mask)) { \
                if (!multipanel->btn) { \
                        if (multipanel->a ## btn < SAITEK_MAX_BTN) \
                                multipanel->a ## btn++; \
                        multipanel->btn = 1; \
                } \
        } else { \
                multipanel->btn = 0; \
        }

#define SAITEK_ADJUST_COUNTED_ENCDR(btn,up,bytenoup,maskup,valmax,down,bytenodown,maskdown,valmin) \
        if (CHECK(data[bytenoup] & maskup)) { \
                if (!multipanel->btn ## _ ## up) { \
                        if (multipanel->btn < (valmax)) \
                                multipanel->btn++; \
                        multipanel->btn ## _ ## up = 1; \
                } \
        } else { \
                multipanel->btn ## _ ## up = 0; \
        } \
        if (CHECK(data[bytenodown] & maskdown)) { \
                if (!multipanel->btn ## _ ## down) \
                        if (multipanel->btn > (valmin)) \
                                multipanel->btn--; \
                        multipanel->btn ## _ ## down = 1; \
        } else { \
                multipanel->btn ## _ ## down = 0; \
        }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverflow"
        // TODO: some kind of a software debouncer
        SAITEK_ADJUST_COUNTED_BTN(hdg, 1, 0x01);
        SAITEK_ADJUST_COUNTED_BTN(nav, 1, 0x02);
        SAITEK_ADJUST_COUNTED_BTN(ias, 1, 0x04);
        SAITEK_ADJUST_COUNTED_BTN(alt, 1, 0x08);
        SAITEK_ADJUST_COUNTED_BTN(vs,  1, 0x10);
        SAITEK_ADJUST_COUNTED_BTN(apr, 1, 0x20);
        SAITEK_ADJUST_COUNTED_BTN(rev, 1, 0x40);
        SAITEK_ADJUST_COUNTED_BTN(ap,  0, 0x80); // unnecessary GCC warning on overflow in conversion here
        SAITEK_ADJUST_COUNTED_ENCDR(flaps,up,2,0x01,SAITEK_MAX_FLAPS,down,2,0x02,SAITEK_MIN_FLAPS);
        multipanel->auto_throttle = CHECK(data[1] & 0x80); // unnecessary GCC warning on overflow in conversion here
        SAITEK_ADJUST_COUNTED_ENCDR(pitch_trim,up,2,0x08,SAITEK_MAX_PITCH_TRIM,down,2,0x04,SAITEK_MIN_PITCH_TRIM);
        SAITEK_ADJUST_COUNTED_ENCDR(knob,right,0,0x20,SAITEK_MAX_KNOB,left,0,0x40,SAITEK_MIN_KNOB);
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
#pragma GCC diagnostic pop
        
#undef SAITEK_ADJUST_COUNTED_ENCDR
#undef SAITEK_ADJUST_COUNTED_BTN

        return 1;
}

static int saitek_proflight_radiopanel_raw_event(
                struct proflight_radiopanel *radiopanel,
                struct hid_device *hdev, struct hid_report *report, u8 *data,
                int size)
{
        if (report->id != 0 || report->type != 0) {
                hid_warn(hdev, "Unknown Saitek Pro Flight Radio Panel HID report"
                                " (ID=%i TYPE=%i).", report->id, report->type);
                return 0; // process the default way
        }
        if (size < 3)
                return -1; // we expect 3 bytes

#define SAITEK_ADJUST_COUNTED_BTN(btn,byteno,mask) \
        if (CHECK(data[byteno] & mask)) { \
                if (!radiopanel->btn) { \
                        if (radiopanel->a ## btn < SAITEK_MAX_BTN) \
                                radiopanel->a ## btn++; \
                        radiopanel->btn = 1; \
                } \
        } else { \
                radiopanel->btn = 0; \
        }

#define SAITEK_ADJUST_COUNTED_ENCDR(btn,up,bytenoup,maskup,valmax,down,bytenodown,maskdown,valmin) \
        if (CHECK(data[bytenoup] & maskup)) { \
                if (!radiopanel->btn ## _ ## up) { \
                        if (radiopanel->btn < (valmax)) \
                                radiopanel->btn++; \
                        radiopanel->btn ## _ ## up = 1; \
                } \
        } else { \
                radiopanel->btn ## _ ## up = 0; \
        } \
        if (CHECK(data[bytenodown] & maskdown)) { \
                if (!radiopanel->btn ## _ ## down) \
                        if (radiopanel->btn > (valmin)) \
                                radiopanel->btn--; \
                        radiopanel->btn ## _ ## down = 1; \
        } else { \
                radiopanel->btn ## _ ## down = 0; \
        }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverflow"

        SAITEK_ADJUST_COUNTED_BTN(actstby0, 1, 0x40);
        SAITEK_ADJUST_COUNTED_BTN(actstby1, 1, 0x80);
        SAITEK_ADJUST_COUNTED_ENCDR(innerknob0,right,2,0x01,SAITEK_MAX_KNOB,left,2,0x02,SAITEK_MIN_KNOB);
        SAITEK_ADJUST_COUNTED_ENCDR(outerknob0,right,2,0x04,SAITEK_MAX_KNOB,left,2,0x08,SAITEK_MIN_KNOB);
        SAITEK_ADJUST_COUNTED_ENCDR(innerknob1,right,2,0x10,SAITEK_MAX_KNOB,left,2,0x20,SAITEK_MIN_KNOB);
        SAITEK_ADJUST_COUNTED_ENCDR(outerknob1,right,2,0x40,SAITEK_MAX_KNOB,left,2,0x80,SAITEK_MIN_KNOB);
        if (data[0] & 0x01)
                radiopanel->mode0 = RADIOPANEL_MODE_COM1;
        else if (data[0] & 0x02)
                radiopanel->mode0 = RADIOPANEL_MODE_COM2;
        else if (data[0] & 0x04)
                radiopanel->mode0 = RADIOPANEL_MODE_NAV1;
        else if (data[0] & 0x08)
                radiopanel->mode0 = RADIOPANEL_MODE_NAV2;
        else if (data[0] & 0x10)
                radiopanel->mode0 = RADIOPANEL_MODE_ADF;
        else if (data[0] & 0x20)
                radiopanel->mode0 = RADIOPANEL_MODE_DME;
        else if (data[0] & 0x40)
                radiopanel->mode0 = RADIOPANEL_MODE_XPDR;
        else
                radiopanel->mode0 = 0; // should never occur
        if (data[0] & 0x80)
                radiopanel->mode1 = RADIOPANEL_MODE_COM1;
        else if (data[1] & 0x01)
                radiopanel->mode1 = RADIOPANEL_MODE_COM2;
        else if (data[1] & 0x02)
                radiopanel->mode1 = RADIOPANEL_MODE_NAV1;
        else if (data[1] & 0x04)
                radiopanel->mode1 = RADIOPANEL_MODE_NAV2;
        else if (data[1] & 0x08)
                radiopanel->mode1 = RADIOPANEL_MODE_ADF;
        else if (data[1] & 0x10)
                radiopanel->mode1 = RADIOPANEL_MODE_DME;
        else if (data[1] & 0x20)
                radiopanel->mode1 = RADIOPANEL_MODE_XPDR;
        else
                radiopanel->mode1 = 0; // should never occur

#pragma GCC diagnostic pop

#undef SAITEK_ADJUST_COUNTED_ENCDR
#undef SAITEK_ADJUST_COUNTED_BTN

        return 1;
}

static int saitek_proflight_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
        struct proflight *driver_data;
        int ret_val = -1;

        driver_data = hid_get_drvdata(hdev);
        if (!driver_data)
                return -1;
        
        SAITEK_LOCK_WRITE(driver_data->lock);
        switch (driver_data->product_id) {
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_MULTIPANEL:
                ret_val = saitek_proflight_multipanel_raw_event(driver_data->data.multipanel, hdev, report, data, size);
                break;
        case USB_DEVICE_ID_SAITEK_PROFLIGHT_RADIOPANEL:
                ret_val = saitek_proflight_radiopanel_raw_event(driver_data->data.radiopanel, hdev, report, data, size);
                break;
        }
        SAITEK_UNLOCK_WRITE(driver_data->lock);

        return ret_val;
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
