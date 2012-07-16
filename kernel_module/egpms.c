/*
 * USB Gembird Energenie PMS driver
 *
 * Copyright (C) 2012 Markus Pargmann (mpargmann@allfex.org)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 *
 * Thanks to the sispmctl project, which was used to get the protocol for
 * this device. (http://sispmctl.sourceforge.net/)
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/usb.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/time.h>




#define DRIVER_AUTHOR "Markus Pargmann, mpargmann@allfex.org"
#define DRIVER_DESC "USB Gembird Energenie PMS driver"

#define GEMBIRD_ID 0x04B4
#define PRODUCT_ID_SISPM 0xFD11
#define PRODUCT_ID_SISPM_FLASH_NEW 0xFD13
#define PRODUCT_ID_MSISPM_OLD 0xFD10
#define PRODUCT_ID_MSISPM_FLASH 0xFD12

enum energenie_type {
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

struct energenie_device {
	struct usb_device	*udev;
	enum energenie_type	type;
};

/*
 * Get current outlet state, 0 = off, 1 = on
 */
static int get_outlet(struct usb_interface *intf, struct energenie_device *ener,
		      unsigned int outlet_id)
{
	int ret;
	unsigned char cmd[2];

	cmd[0] = (outlet_id + 1) * 3;
	cmd[1] = 0x03;
	ret = usb_control_msg(ener->udev,
				usb_rcvctrlpipe(ener->udev, 0),
				0x01,
				0x21 | USB_DIR_IN,
				(0x03 << 8) | cmd[0],
				0,
				cmd,
				5,
				2000);
	if (ret != 2) {
		dev_err(&ener->udev->dev,
				"Failed to get outlet %u state (%d)\n",
				outlet_id, ret);
		return -1;
	}
	return cmd[1] & 1;
}

/*
 * Parse buf and set outlet_id accordingly
 */
static void set_outlet(struct usb_interface *intf,
		       struct energenie_device *ener,
		       unsigned int outlet_id, const char *buf)
{
	unsigned int state;
	int ret;
	unsigned char cmd[2];

	ret = kstrtouint(buf, 10, &state);
	if (ret < 0)
		goto error_out;
	if (state != 0 && state != 1)
		goto error_out;

	if (state)
		cmd[1] = 0x03;
	else
		cmd[1] = 0x00;
	cmd[0] = (outlet_id + 1) * 3;

	ret = usb_control_msg(ener->udev,
				usb_sndctrlpipe(ener->udev, 0),
				0x09,
				0x21,
				(0x03 << 8) | cmd[0],
				0,
				cmd,
				2,
				2000);
	if (ret != 2)
		dev_err(&ener->udev->dev, "Failed setting outlet, retval: %d\n",
				ret);


	return;

error_out:
	dev_err(&ener->udev->dev, "Invalid state, only '0' and '1' allowed");
}

static ssize_t show_serial_id(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int ret;
	struct usb_interface *intf = to_usb_interface(dev);
	struct energenie_device *ener = usb_get_intfdata(intf);
	unsigned char data[5] = {0, 0, 0, 0, 0};

	ret = usb_control_msg(ener->udev,
				usb_rcvctrlpipe(ener->udev, 0),
				0x01,
				0xa1,
				(0x03 << 8) | 1,
				0,
				data,
				5,
				2000);
	if (ret != 5)
		goto error;

	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x\n",
			data[0], data[1], data[2], data[3], data[4]);

error:
	dev_err(&ener->udev->dev, "Failed getting serial id (%d)\n", ret);
	return sprintf(buf, "unknown\n");
}
DEVICE_ATTR(serial_id, S_IRUGO, show_serial_id, NULL);


static inline u16 read_word(unsigned char *data, int *pos)
{
	u16 ret = data[*pos] | ((u16)data[*pos + 1] << 8);
	*pos += 2;
	return ret;
}

static inline void write_word(unsigned char *data, int *pos, u16 word)
{
	data[*pos] = word & 0xff;
	data[*pos + 1] = word >> 8;
	*pos += 2;
}

/*
 * Reads additional time information, only used if one time field is not
 * sufficient.
 */
static inline u64 read_time_extension(unsigned char *data, int *pos)
{
	int loc_pos = *pos;
	u64 time = 0;
	u64 next_time;
	do {
		next_time = read_word(data, &loc_pos);
		if ((next_time & 0x4000) == 0x4000) {
			time += next_time & ~0x4000;
		} else {
			loc_pos -= 2;
			break;
		}
	} while (next_time == 0x7fff);
	*pos = loc_pos;
	return time;
}


static ssize_t show_outlet_schedule(struct device *dev,
					struct device_attribute *attr,
					char *buf, unsigned int outlet_id)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct energenie_device *ener = usb_get_intfdata(intf);
	int data_pos = 1;
	int ret;
	int count = 0;
	unsigned char data[0x27];
	u64 timestamp; /* Timestamp in seconds */
	u64 next_time; /* minutes */
	u16 word;

	ret = usb_control_msg(ener->udev,
				usb_rcvctrlpipe(ener->udev, 0),
				0x01,
				0x21 | USB_DIR_IN,
				((0x03 << 8) | (3 * (outlet_id + 1))) + 1,
				0,
				data,
				0x27,
				2000);
	if (ret != 0x27)
		goto error;

	word = read_word(data, &data_pos);
	timestamp = word;
	word = read_word(data, &data_pos);
	timestamp |= (u64)word << 16;

	count = sprintf(buf, "%llu\n", timestamp);

	/*
	 * The event time of the first event is located at the end of the
	 * data.
	 */
	data_pos = 0x25;
	word = read_word(data, &data_pos);
	data_pos = 5; /* Jump back to the original position in the data */
	next_time = word;
	if (word == 0xfd21)
		next_time += read_time_extension(data, &data_pos);

	while (data_pos < 0x25) {
		unsigned int state;

		word = read_word(data, &data_pos);
		if (word == 0x3fff)
			continue;

		if (next_time) {
			state = word >> 15;
			count += sprintf(buf + count, "%llu %d\n",
					next_time, state);
		}

		next_time = word & 0x7fff;
		if (next_time == 0x3ffe)
			next_time += read_time_extension(data, &data_pos);
	}

	/*
	 * This is the repeat information of the device. It repeats all events
	 * after next_time minutes.
	 */
	if (next_time)
		count += sprintf(buf + count, "%llu\n", next_time);

	return count;
error:
	dev_err(&ener->udev->dev, "Failed getting schedule (%d)\n", ret);
	return sprintf(buf, "unknown\n");
}

static inline int write_event(unsigned char *data, unsigned int *data_pos,
				int *first, u64 time_diff, unsigned int state)
{
	/*
	 * For the first event, we have to write the event time to the end of
	 * the data
	 */
	if (*first) {
		int loc_pos = 0x25;
		if (time_diff > 0xfd21) {
			write_word(data, &loc_pos, 0xfd21);
			time_diff -= 0xfd21;

			for (; time_diff > 0x3fff; time_diff -= 0x3fff) {
				if (*data_pos >= 0x25)
					goto error;
				write_word(data, data_pos, 0x3fff | 0x4000);
			}

			if (*data_pos >= 0x25)
				goto error;
			write_word(data, data_pos, time_diff | 0x4000);
		} else {
			write_word(data, &loc_pos, time_diff);
		}
		*first = 0;
	} else {
		if (time_diff > 0x3ffe) {
			if (*data_pos >= 0x25)
				goto error;
			/*
			 * This is not the first event, so in data at *data_pos
			 * is already the action data for the previous event.
			 * We may not overwrite that data, so read and OR before
			 */
			write_word(data, data_pos,
				((u16)data[*data_pos + 1] << 8) | 0x3ffe);
			time_diff -= 0x3ffe;

			for (; time_diff > 0x3fff; time_diff -= 0x3fff) {
				if (*data_pos >= 0x25)
					goto error;
				write_word(data, data_pos, 0x3fff | 0x4000);
			}

			if (*data_pos >= 0x25)
				goto error;
			write_word(data, data_pos, time_diff | 0x4000);
		} else {
			if (*data_pos >= 0x25)
				goto error;
			write_word(data, data_pos,
					time_diff | (data[*data_pos + 1] << 8));
		}
	}
	if (*data_pos >= 0x23)
		goto error;
	/* Write the action data to the next, not yet filled data word */
	data[*data_pos] = 0;
	data[*data_pos + 1] = state << 7;
	return 0;
error:
	return -1;
}

static ssize_t set_outlet_schedule(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count,
				   unsigned int outlet_id)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct energenie_device *ener = usb_get_intfdata(intf);
	int ret;
	unsigned char data[0x27];
	unsigned int data_pos = 0;
	const char *line_split;
	const char *space_split;
	const char *line_start;
	int first = 1;
	u64 event_time_delta; /* Minutes */
	u64 timestamp = current_kernel_time().tv_sec;

	data[0] = (3 * (outlet_id + 1)) + 1;
	data_pos = 1;
	write_word(data, &data_pos, timestamp);
	write_word(data, &data_pos, timestamp >> 16);


	for (line_start = buf,
	     line_split = strnstr(line_start, "\n", buf - line_start + count);
	     line_split < buf + count;
	     line_start = line_split + 1,
	     line_split = strnstr(line_start, "\n", buf - line_start + count)) {

		unsigned int state;
		char int_parse_buf[32];
		if (line_split == NULL)
			line_split = buf + count;

		space_split = strnstr(line_start, " ", line_split - line_start);

		if (space_split == NULL) {
			/* Writing the repeat information */
			memcpy(int_parse_buf, line_start,
					line_split - line_start);
			int_parse_buf[line_split - line_start] = '\0';
			ret = kstrtou64(int_parse_buf, 10, &event_time_delta);
			if (ret)
				break;

			ret = write_event(data, &data_pos, &first,
					event_time_delta, 0);
			if (ret)
				goto user_error;

			data_pos -= 2;
			break;
		}

		/* Parse timedelta */
		memcpy(int_parse_buf, line_start, space_split - line_start);
		int_parse_buf[space_split - line_start] = '\0';
		ret = kstrtou64(int_parse_buf, 10, &event_time_delta);
		if (ret < 0)
			goto user_error;

		/* Parse action (turn on = 1, turn off = 0) */
		memcpy(int_parse_buf, space_split + 1,
				line_split - space_split - 1);
		int_parse_buf[line_split - space_split - 1] = '\0';
		ret = kstrtouint(int_parse_buf, 10, &state);
		if (ret < 0)
			goto user_error;

		ret = write_event(data, &data_pos, &first, event_time_delta,
				state);
		if (ret)
			goto user_error;
	}

	/* Fill the rest of the data with empty words (0x3fff) */
	data_pos += 2;
	while (data_pos < 0x25)
		write_word(data, &data_pos, 0x3fff);

	ret = usb_control_msg(ener->udev,
				usb_sndctrlpipe(ener->udev, 0),
				0x09,
				0x21,
				(0x03 << 8) | ((3 * (outlet_id + 1)) + 1),
				0,
				data,
				0x27,
				2000);
	if (ret != 0x27)
		goto error;

	return count;
user_error:
	dev_err(&ener->udev->dev, "Failed parsing schedule\n");
	return count;
error:
	dev_err(&ener->udev->dev, "Failed setting schedule (%d)\n", ret);
	return count;
}

#define show_set_outlet(outlet_id)					\
static ssize_t show_outlet_##outlet_id(struct device *dev,		\
				struct device_attribute *attr,		\
				char *buf)				\
{									\
	struct usb_interface *intf = to_usb_interface(dev);		\
	struct energenie_device *ener = usb_get_intfdata(intf);		\
	int ret = get_outlet(intf, ener, outlet_id);			\
									\
	if (ret != 0 && ret != 1) {					\
		dev_err(&ener->udev->dev,				\
				"Failed getting outlet state (%d)\n",	\
				ret);					\
		return sprintf(buf, "unknown\n");			\
	}								\
									\
	return sprintf(buf, "%d\n", ret);				\
}									\
									\
static ssize_t set_outlet_##outlet_id(struct device *dev,		\
			       struct device_attribute *attr,		\
			       const char *buf, size_t count)		\
{									\
	struct usb_interface *intf = to_usb_interface(dev);		\
	struct energenie_device *ener = usb_get_intfdata(intf);		\
	set_outlet(intf, ener, outlet_id, buf);				\
	return count;							\
}									\
static DEVICE_ATTR(outlet_##outlet_id, S_IRUGO | S_IWUSR,		\
		   show_outlet_##outlet_id, set_outlet_##outlet_id);	\
									\
static ssize_t show_outlet_##outlet_id##_schedule(struct device *dev,	\
				struct device_attribute *attr,		\
				char *buf)				\
{									\
	return show_outlet_schedule(dev, attr, buf, outlet_id);		\
}									\
									\
static ssize_t set_outlet_##outlet_id##_schedule(struct device *dev,	\
				struct device_attribute *attr,		\
				const char *buf, size_t count)		\
{									\
	return set_outlet_schedule(dev, attr, buf, count, outlet_id);	\
}									\
static DEVICE_ATTR(outlet_##outlet_id##_schedule, S_IRUGO | S_IWUSR,	\
		show_outlet_##outlet_id##_schedule,			\
		set_outlet_##outlet_id##_schedule);

show_set_outlet(0)
show_set_outlet(1)
show_set_outlet(2)
show_set_outlet(3)


static int energenie_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct energenie_device *dev = NULL;
	int ret = -ENOMEM;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&intf->dev, "out of memory\n");
		goto error_mem;
	}

	dev->udev = usb_get_dev(udev);
	dev->type = id->driver_info;

	usb_set_intfdata(intf, dev);

	ret = device_create_file(&intf->dev, &dev_attr_serial_id);
	if (ret)
		goto error;

	switch (dev->type) {
	case SISPM:
	case SISPM_FLASH_NEW:
		ret = device_create_file(&intf->dev, &dev_attr_outlet_1);
		if (ret)
			goto error;
		ret = device_create_file(&intf->dev,
				&dev_attr_outlet_1_schedule);
		if (ret)
			goto error;

		ret = device_create_file(&intf->dev, &dev_attr_outlet_2);
		if (ret)
			goto error;
		ret = device_create_file(&intf->dev,
				&dev_attr_outlet_2_schedule);
		if (ret)
			goto error;

		ret = device_create_file(&intf->dev, &dev_attr_outlet_3);
		if (ret)
			goto error;
		ret = device_create_file(&intf->dev,
				&dev_attr_outlet_3_schedule);
		if (ret)
			goto error;
	case MSISPM_OLD:
	case MSISPM_FLASH:
		ret = device_create_file(&intf->dev, &dev_attr_outlet_0);
		if (ret)
			goto error;
		ret = device_create_file(&intf->dev,
				&dev_attr_outlet_0_schedule);
		if (ret)
			goto error;
		break;
	default:
		dev_err(&dev->udev->dev, "unknown device type %d\n", dev->type);
		goto error;
	}

	dev_info(&intf->dev, "USB Gembird Energenie PMS connected\n");
	return 0;

error:
	device_remove_file(&intf->dev, &dev_attr_serial_id);
	device_remove_file(&intf->dev, &dev_attr_outlet_0);
	device_remove_file(&intf->dev, &dev_attr_outlet_1);
	device_remove_file(&intf->dev, &dev_attr_outlet_2);
	device_remove_file(&intf->dev, &dev_attr_outlet_3);
	device_remove_file(&intf->dev, &dev_attr_outlet_0_schedule);
	device_remove_file(&intf->dev, &dev_attr_outlet_1_schedule);
	device_remove_file(&intf->dev, &dev_attr_outlet_2_schedule);
	device_remove_file(&intf->dev, &dev_attr_outlet_3_schedule);
	usb_set_intfdata(intf, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);
error_mem:
	return -ENOMEM;
}

static void energenie_disconnect(struct usb_interface *intf)
{
	struct energenie_device *dev = usb_get_intfdata(intf);

	device_remove_file(&intf->dev, &dev_attr_serial_id);

	switch (dev->type) {
	case SISPM:
	case SISPM_FLASH_NEW:
		device_remove_file(&intf->dev, &dev_attr_outlet_1);
		device_remove_file(&intf->dev, &dev_attr_outlet_2);
		device_remove_file(&intf->dev, &dev_attr_outlet_3);
		device_remove_file(&intf->dev, &dev_attr_outlet_1_schedule);
		device_remove_file(&intf->dev, &dev_attr_outlet_2_schedule);
		device_remove_file(&intf->dev, &dev_attr_outlet_3_schedule);
	case MSISPM_OLD:
	case MSISPM_FLASH:
		device_remove_file(&intf->dev, &dev_attr_outlet_0);
		device_remove_file(&intf->dev, &dev_attr_outlet_0_schedule);
		break;
	default:
		dev_err(&dev->udev->dev, "unknown device type %d\n", dev->type);
	}

	usb_set_intfdata(intf, NULL);
	usb_put_dev(dev->udev);

	kfree(dev);

	dev_info(&intf->dev, "USB Gembird Energenie PMS disconnected\n");
}

static struct usb_driver energenie_driver = {
	.name =		"usbegpms",
	.probe =	energenie_probe,
	.disconnect =	energenie_disconnect,
	.id_table =	supported_devices
};

module_usb_driver(energenie_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
