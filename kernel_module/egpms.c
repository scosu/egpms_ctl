/*
 * USB Gembird Energenie PMS driver
 *
 * Copyright (C) 2012 Markus Pargmann (mpargmann@allfex.org)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * Based on the protocol reverse engineered by the sispmctl project.
 * (http://sispmctl.sourceforge.net/)
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/usb.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/time.h>


#define GEMBIRD_ID 0x04B4
#define PRODUCT_ID_SISPM 0xFD11
#define PRODUCT_ID_SISPM_FLASH_NEW 0xFD13
#define PRODUCT_ID_MSISPM_OLD 0xFD10
#define PRODUCT_ID_MSISPM_FLASH 0xFD12

#define EGPMS_URB_BASE		0x0300
#define EGPMS_URB_SERIAL	0x0001
#define EGPMS_URB_OUTLET(outlet) (3 * ((outlet) + 1))
#define EGPMS_URB_OUTLET_SCHED(outlet) \
		(EGPMS_URB_OUTLET(outlet) + 1)

#define EGPMS_OUTLET_ENABLE	0x03
#define EGPMS_OUTLET_DISABLE	0x00

#define EGPMS_TIMEOUT 2000

/* Time extension, if set, this event is a extension time cell */
#define EGPMS_SCHED_FLAG_TIME_EXT BIT(14)
#define EGPMS_SCHED_FLAG_ENABLE BIT(15)
#define EGPMS_SCHED_TIME_MASK 0x3fff
#define EGPMS_SCHED_MAX_TIME 0x3ffe
#define EGPMS_SCHED_EMPTY 0x3fff

enum egpms_type {
	SISPM,
	SISPM_FLASH_NEW,
	MSISPM_OLD,
	MSISPM_FLASH
};

static const struct usb_device_id supported_devices[] = {
	{ USB_DEVICE(GEMBIRD_ID, PRODUCT_ID_SISPM),
		.driver_info = SISPM },
	{ USB_DEVICE(GEMBIRD_ID, PRODUCT_ID_MSISPM_OLD),
		.driver_info = MSISPM_OLD },
	{ USB_DEVICE(GEMBIRD_ID, PRODUCT_ID_MSISPM_FLASH),
		.driver_info = MSISPM_FLASH },
	{ USB_DEVICE(GEMBIRD_ID, PRODUCT_ID_SISPM_FLASH_NEW),
		.driver_info = SISPM_FLASH_NEW },
	{ },
};
MODULE_DEVICE_TABLE(usb, supported_devices);

struct egpms_data {
	struct usb_device	*udev;
	enum egpms_type	type;
};

struct egpms_ext_attr {
	struct device_attribute attr;
	unsigned int outlet_id;
};

struct egpms_sched {
	u8 outlet;
	__be32 timestamp;
	__be16 events[0x11];
} __packed;

/*
 * Get current outlet state, 0 = off, 1 = on
 */
static ssize_t egpms_outlet_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct egpms_data *egpms = usb_get_intfdata(intf);
	struct egpms_ext_attr *ext_attr =
			container_of(attr, struct egpms_ext_attr, attr);
	unsigned int outlet_id = ext_attr->outlet_id;
	int ret;
	u8 cmd[2];

	cmd[0] = EGPMS_URB_OUTLET(outlet_id);
	cmd[1] = 0;
	ret = usb_control_msg(egpms->udev, usb_rcvctrlpipe(egpms->udev, 0),
			0x01,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN,
			EGPMS_URB_BASE + EGPMS_URB_OUTLET(outlet_id),
			0, cmd, sizeof(cmd), EGPMS_TIMEOUT);
	if (ret != sizeof(cmd)) {
		dev_err(&egpms->udev->dev,
				"Failed to get outlet %u state (%d)\n",
				outlet_id, ret);
		return -1;
	}
	if (cmd[1] & 1)
		return sprintf(buf, "enabled\n");
	else
		return sprintf(buf, "disabled\n");
}

/*
 * Parse buf and set outlet_id accordingly
 */
static ssize_t egpms_outlet_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct egpms_data *egpms = usb_get_intfdata(intf);
	struct egpms_ext_attr *ext_attr =
			container_of(attr, struct egpms_ext_attr, attr);
	unsigned int outlet_id = ext_attr->outlet_id;
	bool enable;
	int ret;
	unsigned char cmd[2];

	if (!strncmp("enable", buf, strlen("enable")) ||
			!strncmp("on", buf, strlen("on"))) {
		enable = 1;
	} else if (!strncmp("disable", buf, strlen("disable"))
			|| !strncmp("off", buf, strlen("off"))) {
		enable = 0;
	} else {
		int val;

		ret = kstrtouint(buf, 10, &val);
		if (ret < 0 || (val != 0 && val != 1))
			goto error_out;

		enable = val;
	}

	if (enable)
		cmd[1] = EGPMS_OUTLET_ENABLE;
	else
		cmd[1] = EGPMS_OUTLET_DISABLE;
	cmd[0] = EGPMS_URB_OUTLET(outlet_id);

	ret = usb_control_msg(egpms->udev, usb_sndctrlpipe(egpms->udev, 0),
			0x09,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
			EGPMS_URB_BASE + EGPMS_URB_OUTLET(outlet_id),
			0, cmd, sizeof(cmd), EGPMS_TIMEOUT);
	if (ret != sizeof(cmd))
		dev_err(&egpms->udev->dev, "Failed setting outlet, retval: %d\n",
				ret);


	return count;

error_out:
	dev_err(&egpms->udev->dev, "Invalid state, only '0' and '1' allowed");
	return count;
}

static ssize_t egpms_serial_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct usb_interface *intf = to_usb_interface(dev);
	struct egpms_data *egpms = usb_get_intfdata(intf);
	unsigned char data[5] = {0, 0, 0, 0, 0};

	ret = usb_control_msg(egpms->udev, usb_rcvctrlpipe(egpms->udev, 0),
			0x01,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN,
			EGPMS_URB_BASE + EGPMS_URB_SERIAL,
			0, data, sizeof(data), EGPMS_TIMEOUT);
	if (ret != sizeof(data))
		goto error;

	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x\n",
			data[0], data[1], data[2], data[3], data[4]);

error:
	dev_err(&egpms->udev->dev, "Failed getting serial id (%d)\n", ret);
	return sprintf(buf, "unknown\n");
}
DEVICE_ATTR(serial_id, S_IRUGO, egpms_serial_id_show, NULL);


static u32 egpms_calc_next_event_time(struct egpms_sched *sched, int *pos)
{
	int i;
	int esc = 0;
	u32 time = 0;
	u16 last_time;
	u16 evt;

	for (i = *pos, last_time = EGPMS_SCHED_MAX_TIME;
			last_time >= EGPMS_SCHED_MAX_TIME && ++esc < 100;
			i = (i + 1) % ARRAY_SIZE(sched->events)) {
		evt = sched->events[i];
		if (i != *pos && !(evt & EGPMS_SCHED_FLAG_TIME_EXT)) {
			*pos = i;
			return time;
		}

		last_time = evt & EGPMS_SCHED_TIME_MASK;

		time += last_time;
	}
	*pos = i;
	return time;
}

static int egpms_write_next_event_time(struct egpms_sched *sched, int *pos,
		u32 time)
{
	int i;

	if (!time) {
		*pos = (*pos + 1) % ARRAY_SIZE(sched->events);
		return 0;
	}

	for (i = *pos; time; i = (i + 1) % ARRAY_SIZE(sched->events)) {
		if (time > EGPMS_SCHED_MAX_TIME) {
			sched->events[i] |= EGPMS_SCHED_MAX_TIME;
			time -= EGPMS_SCHED_MAX_TIME;
		} else {
			sched->events[i] |= time;
			time = 0;
		}

		if (i != *pos)
			sched->events[i] |= EGPMS_SCHED_FLAG_TIME_EXT;
	}

	*pos = i;
	return 0;
}

static ssize_t egpms_outlet_sched_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct egpms_data *egpms = usb_get_intfdata(intf);
	struct egpms_ext_attr *ext_attr =
			container_of(attr, struct egpms_ext_attr, attr);
	unsigned int outlet_id = ext_attr->outlet_id;
	int ret;
	struct egpms_sched sched;
	u32 time = 0; /* minutes */
	int i = ARRAY_SIZE(sched.events) - 1;
	ssize_t count;
	int esc = 0;

	ret = usb_control_msg(egpms->udev, usb_rcvctrlpipe(egpms->udev, 0),
			0x01,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN,
			EGPMS_URB_BASE + EGPMS_URB_OUTLET_SCHED(outlet_id),
			0, &sched, sizeof(sched), EGPMS_TIMEOUT);
	if (ret != sizeof(sched)) {
		dev_err(&egpms->udev->dev, "Failed getting schedule, read %d/%u\n",
				ret, (unsigned int)sizeof(sched));
		return sprintf(buf, "Unknown schedule\n");
	}

	count = sprintf(buf, "%u:", sched.timestamp);

	do {
		if ((sched.events[i] & EGPMS_SCHED_TIME_MASK) ==
				EGPMS_SCHED_EMPTY) {
			break;
		} else {
			time = egpms_calc_next_event_time(&sched, &i);

			if ((sched.events[i] & EGPMS_SCHED_TIME_MASK) ==
					EGPMS_SCHED_EMPTY)
				break;

			count += sprintf(buf + count, "%u,%u;", time,
					!!(sched.events[i] &
						EGPMS_SCHED_FLAG_ENABLE));
		}
	} while (i != ARRAY_SIZE(sched.events) - 1 && ++esc < 100);

	if (time)
		count += sprintf(buf + count, "%u\n", time);
	else
		count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t egpms_outlet_sched_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct egpms_data *egpms = usb_get_intfdata(intf);
	struct egpms_ext_attr *ext_attr =
			container_of(attr, struct egpms_ext_attr, attr);
	int ret;
	const char *line_split;
	const char *line_start;
	u32 event_time_delta; /* Minutes */
	struct egpms_sched sched;
	int i;
	int outlet_id = ext_attr->outlet_id;

	memset(&sched, 0, sizeof(sched));

	sched.outlet = EGPMS_URB_OUTLET_SCHED(outlet_id);
	sched.timestamp = current_kernel_time().tv_sec;

	for (line_start = buf,
	     line_split = strnstr(line_start, ";", buf - line_start + count),
	     i = ARRAY_SIZE(sched.events) - 1;
	     line_split < buf + count && line_split - line_start > 1;
	     line_start = line_split + 1,
	     line_split = strnstr(line_start, ";", buf - line_start + count)) {

		char line_buf[64];
		char *space_split;

		if (line_split == NULL)
			line_split = buf + count;
		if (*(line_split - 1) == '\n') {
			--line_split;
			if (line_split <= line_start)
				break;
		}

		if (line_split - line_start > 64) {
			dev_err(&egpms->udev->dev, "Failed parsing, line unreasonably long\n");
			return count;
		}

		memcpy(line_buf, line_start, line_split - line_start);
		line_buf[line_split - line_start] = '\0';

		space_split = strnstr(line_buf, ",", line_split - line_start);
		if (space_split) {
			*space_split = '\0';
			++space_split;
		}

		ret = kstrtou32(line_buf, 10, &event_time_delta);
		if (ret)
			goto user_error;

		ret = egpms_write_next_event_time(&sched, &i, event_time_delta);
		if (ret)
			goto user_error;

		if (space_split) {
			if (!strcmp(space_split, "1") ||
					!strcmp(space_split, "on") ||
					!strcmp(space_split, "enable")) {
				sched.events[i] |= EGPMS_SCHED_FLAG_ENABLE;
			} else if (!strcmp(space_split, "0") ||
					!strcmp(space_split, "off") ||
					!strcmp(space_split, "disable")) {
				sched.events[i] &= ~EGPMS_SCHED_FLAG_ENABLE;
			} else {
				dev_err(&egpms->udev->dev, "Failed to parse state value '%s'\n",
						space_split);
				return count;
			}
		} else {
			--i;
			break;
		}
	}

	++i;
	if (i == ARRAY_SIZE(sched.events)) {
		sched.events[i - 1] |= EGPMS_SCHED_EMPTY;
		i = 0;
	}

	for (; i < ARRAY_SIZE(sched.events) - 1; ++ i)
		sched.events[i] |= EGPMS_SCHED_EMPTY;

	ret = usb_control_msg(egpms->udev, usb_sndctrlpipe(egpms->udev, 0),
			0x09,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
			EGPMS_URB_BASE + EGPMS_URB_OUTLET_SCHED(outlet_id),
			0, &sched, sizeof(sched), EGPMS_TIMEOUT);
	if (ret != sizeof(sched))
		goto error;

	return count;
user_error:
	dev_err(&egpms->udev->dev, "Failed parsing schedule\n");
	return count;
error:
	dev_err(&egpms->udev->dev, "Failed setting schedule (%d)\n", ret);
	return count;
}

#define EGPMS_MAX_SLOTS 4

static char egpms_attr_names[EGPMS_MAX_SLOTS * 2][32];
static struct egpms_ext_attr egpms_attrs[EGPMS_MAX_SLOTS * 2];

static int egpms_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct egpms_data *egpms = NULL;
	int ret = -ENOMEM;
	int slots = 0;
	int i = 0;

	egpms = devm_kzalloc(&udev->dev, sizeof(*egpms), GFP_KERNEL);
	if (egpms == NULL)
		return -ENOMEM;

	egpms->udev = usb_get_dev(udev);
	egpms->type = id->driver_info;

	usb_set_intfdata(intf, egpms);

	ret = device_create_file(&intf->dev, &dev_attr_serial_id);
	if (ret) {
		dev_err(&intf->dev, "Failed creating serial_id file\n");
		goto err;
	}

	switch (egpms->type) {
	case SISPM:
	case SISPM_FLASH_NEW:
		slots = 4;
		break;
	case MSISPM_OLD:
	case MSISPM_FLASH:
		slots = 1;
		break;
	default:
		dev_err(&egpms->udev->dev, "unknown device type %d\n", egpms->type);
		goto err;
	}

	slots *= 2;

	for (i = 0; i != slots; ++i) {
		ret = device_create_file(&intf->dev, &egpms_attrs[i].attr);
		if (ret)
			goto err;
	}

	dev_info(&intf->dev, "USB Gembird Energenie PMS connected\n");
	return 0;
err:
	if (i) {
		while (i--)
			device_remove_file(&intf->dev, &egpms_attrs[i].attr);
	}
	usb_put_dev(egpms->udev);
	return ret;
}

static void egpms_disconnect(struct usb_interface *intf)
{
	struct egpms_data *egpms = usb_get_intfdata(intf);
	int slots;
	int i;

	switch (egpms->type) {
	case SISPM:
	case SISPM_FLASH_NEW:
		slots = 4;
		break;
	case MSISPM_OLD:
	case MSISPM_FLASH:
		slots = 1;
		break;
	default:
		slots = 0;
		break;
	}

	device_remove_file(&intf->dev, &dev_attr_serial_id);

	slots *= 2;

	for (i = 0; i != slots; ++i)
		device_remove_file(&intf->dev, &egpms_attrs[i].attr);

	usb_put_dev(egpms->udev);

	dev_info(&intf->dev, "USB Gembird Energenie PMS disconnected\n");
}

static struct usb_driver egpms_driver = {
	.name =		"usbegpms",
	.probe =	egpms_probe,
	.disconnect =	egpms_disconnect,
	.id_table =	supported_devices
};

static int __init egpms_init(void)
{
	int i;

	for (i = 0; i != EGPMS_MAX_SLOTS; ++i) {
		int attr_i = i * 2;
		struct device_attribute *attr;

		attr = &egpms_attrs[attr_i].attr;

		sprintf(egpms_attr_names[attr_i], "outlet%d", i + 1);
		attr->attr.name = egpms_attr_names[attr_i];
		attr->attr.mode = S_IWUSR | S_IRUGO;
		attr->show = egpms_outlet_show;
		attr->store = egpms_outlet_store;
		egpms_attrs[attr_i].outlet_id = i;

		++attr_i;
		attr = &egpms_attrs[attr_i].attr;

		sprintf(egpms_attr_names[attr_i], "outlet%d_schedule", i + 1);
		attr->attr.name = egpms_attr_names[attr_i];
		attr->attr.mode = S_IWUSR | S_IRUGO;
		attr->show = egpms_outlet_sched_show;
		attr->store = egpms_outlet_sched_store;
		egpms_attrs[attr_i].outlet_id = i;
	}

	return usb_register(&egpms_driver);
}
module_init(egpms_init);

static void __exit egpms_exit(void)
{
	return usb_deregister(&egpms_driver);
}
module_exit(egpms_exit);

MODULE_AUTHOR("Markus Pargmann <mpargmann@allfex.org>");
MODULE_DESCRIPTION("USB Gembird Energenie PMS driver");
MODULE_LICENSE("GPL");
