// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hardware Monitoring driver for Advantech EIO embedded controller.
 *
 * Copyright (C) 2025 Advantech Corporation. All rights reserved.
 */

#include <linux/errno.h>
#include <linux/hwmon.h>
#include <linux/mfd/core.h>
#include <linux/mfd/eio.h>
#include <linux/module.h>
#include <linux/thermal.h>

#define MAX_CMD_SIZE 4
#define MAX_NAME 32

#define DECI_KELVIN_TO_MILLI_CELSIUS(t) (((t) - 2731) * 100)
#define MILLI_CELSIUS_TO_DECI_KELVIN(t) (((t) / 100) + 2731)
#define MIN_TEMP	1000
#define MAX_TEMP	125000

#define THERM_STS_AVAIL           BIT(0)
#define THERM_STS_THROTTLE_AVAIL  BIT(1)
#define THERM_STS_POWEROFF_AVAIL  BIT(2)
#define THERM_STS_SHUTDOWN_AVAIL  BIT(3)
#define CTRL_STATE                0x00
#define CTRL_TYPE                 0x01
#define CTRL_CTRL		0x02
#define CTRL_ERROR		0x04
#define CTRL_VALUE		0x10
#define CTRL_INVERT		0x11
#define CTRL_FREQ		0x12
#define CTRL_THERM_HIGH		0x13
#define CTRL_THERM_LOW		0x14
#define CTRL_THERM_STOP		0x15
#define CTRL_PWM_HIGH		0x16
#define CTRL_PWM_LOW		0x17
#define CTRL_THERM_SRC		0x20
#define CTRL_SHUTDOWN_HI	  0x41
#define CTRL_POWEROFF_HI	  0x31
#define CTRL_THROTTLE_HI	  0x21

#define CTRLMODE_STOP   0x00
#define CTRLMODE_FULL   0x01
#define CTRLMODE_MANUAL 0x02
#define CTRLMODE_AUTO   0x03

#define FAN_MODE_MASK   GENMASK(1, 0)
#define FAN_SCM_BIT     BIT(2)
#define FAN_FRAME_BIT   BIT(3)
#define FAN_SRC_MASK    GENMASK(7, 4)
#define FAN_SRC(val)    (((int)(val)) >> 4)

#define THERMAL_POLLING_DELAY	  2000 /* millisecond */
#define THERMAL_PASSIVE_DELAY	  1000

#define TRIP_NUM		 3
#define TRIP_SHUTDOWN		 0
#define TRIP_POWEROFF		 1
#define TRIP_THROTTLE		 2
#define CTRL_STS_THROTTLE	 0x20
#define CTRL_STS_POWEROFF	 0x30
#define CTRL_STS_SHUTDOWN	 0x40

#define EIO_ITEM_INPUT		1
#define EIO_ITEM_MAX		2
#define EIO_ITEM_MIN		3
#define EIO_ITEM_CRIT		4
#define EIO_ITEM_EMERGENCY	5

#define EIO_MAX_IN   8
#define EIO_MAX_CURR 2
#define EIO_MAX_TEMP 4
#define EIO_MAX_FAN  4
#define EIO_MAX_INTRUSION  1

enum _sen_type {
	NONE,
	VOLTAGE,
	CURRENT,
	TEMP,
	FAN,
	CASEOPEN,
};

struct eio_chan {
	bool available;
	u8   label_idx;
};

struct eio_hwmon_dev {
	struct device *mfd;
	struct device *hwmon;
	struct eio_chan in[EIO_MAX_IN];
	struct eio_chan curr[EIO_MAX_CURR];
	struct eio_chan temp[EIO_MAX_TEMP];
	struct eio_chan fan[EIO_MAX_FAN];
	struct eio_chan intrusion;

	struct thermal_zone_device *tzd[EIO_MAX_TEMP];

	struct thermal_zone_device *fan_tzd[EIO_MAX_FAN];
	struct thermal_cooling_device *fan_cdev[EIO_MAX_FAN];
};


static const u8 ctrl_sts_map[] = {
	CTRL_STS_SHUTDOWN, CTRL_STS_POWEROFF, CTRL_STS_THROTTLE
};

static const struct {
	u8 cmd;
	u8 max;
	signed int shift;
	char name[MAX_NAME];
	u8 ctrl[16];
	u16 multi[16];
	char item[16][MAX_NAME];
	char labels[32][MAX_NAME];
} sen_info[] = {
	{ 0x00, 0, 0, "none" },
	{ 0x12, 8, 0, "in",
		{ 0xFF, 0x10, 0x11, 0x12 },
		{ 1,    10,   10,   10 },
		{ "label", "input", "max", "min" },
		{ "5V", "5Vs5", "12V", "12Vs5",
		  "3V3", "3V3", "5Vsb", "3Vsb",
		  "Vcmos", "Vbat", "Vdc", "Vstb",
		  "Vcore_a", "Vcore_b", "", "",
		  "Voem0", "Voem1", "Voem2", "Voem3"
		},
	},
	{ 0x1a, 2, 0, "curr",
		{ 0xFF, 0x10, 0x11, 0x12 },
		{ 1,    10,   10,   10 },
		{ "label", "input", "max", "min" },
		{ "dc", "oem0" },
	},
	{ 0x10, 4, -2731, "temp",
		{ 0xFF, 0x10, 0x11, 0x12, 0x21, 0x41 },
		{ 1,    100,  100,  100,  100,  100 },
		{ "label", "input", "max", "min", "crit", "emergency" },
		{ "cpu0", "cpu1", "cpu2", "cpu3",
		  "sys0", "sys1", "sys2", "sys3",
		  "aux0", "aux1", "aux2", "aux3",
		  "dimm0", "dimm1", "dimm2", "dimm3",
		  "pch", "gpu", "", "",
		  "", "", "", "",
		  "", "", "", "",
		  "oem0", "oem1", "oem", "oem3" },
	},
	{ 0x24, 4, 0, "fan",
		{ 0xFF, 0x1A },
		{ 1, 1 },
		{ "label", "input"},
		{ "cpu0", "cpu1", "cpu2", "cpu3",
		  "sys0", "sys1", "sys2", "sys3",
		  "", "", "", "", "", "", "", "",
		  "", "", "", "", "", "", "", "",
		  "", "", "", "",
		  "oem0", "oem1", "oem2", "oem3",
		},
	},
	{ 0x28, 1, 0, "intrusion",
		{ 0xFF, 0x02 },
		{ 1, 1 },
		{ "label", "input" },
		{ "case_open" }
	}
};

static const struct {
	enum _sen_type type;
	u8   ctrl;
	int  size;
	bool write;
} eio_hwmon_ctrl_param[] = {
	{ NONE,     0x00, 0, false },

	{ VOLTAGE,  0x00, 1, false }, { VOLTAGE,  0x01, 1, false },
	{ VOLTAGE,  0x10, 2, false }, { VOLTAGE,  0x11, 2, false },
	{ VOLTAGE,  0x12, 2, false },

	{ CURRENT,  0x00, 1, false }, { CURRENT,  0x01, 1, false },
	{ CURRENT,  0x10, 2, false }, { CURRENT,  0x11, 2, false },
	{ CURRENT,  0x12, 2, false },

	{ TEMP,	    0x00, 2, false }, { TEMP,	  0x01, 1, false },
	{ TEMP,     0x04, 1, false }, { TEMP,	  0x10, 2, false },
	{ TEMP,     0x11, 2, false }, { TEMP,	  0x12, 2, false },
	{ TEMP,     0x20, 2, false }, { TEMP,	  0x21, 2, false },
	{ TEMP,     0x30, 2, false }, { TEMP,	  0x31, 2, false },
	{ TEMP,     0x40, 2, false }, { TEMP,	  0x41, 2, false },

	{ FAN,      0x00, 1, false }, { FAN,      0x01, 1, false },
	{ FAN,      0x02, 1, true  }, { FAN,      0x03, 1, true  },
	{ FAN,      0x10, 1, false }, { FAN,      0x11, 1, true  },
	{ FAN,      0x12, 4, false }, { FAN,      0x13, 2, true  },
	{ FAN,      0x14, 2, true  }, { FAN,      0x15, 2, true  },
	{ FAN,      0x16, 1, true  }, { FAN,      0x17, 1, true  },
	{ FAN,      0x1A, 2, false }, { FAN,      0x20, 1, false },

	{ CASEOPEN, 0x00, 1, false }, { CASEOPEN, 0x02, 1, true  },
};

static int para_idx(enum _sen_type type, u8 ctrl)
{
	int i;

	for (i = 1 ; i < ARRAY_SIZE(eio_hwmon_ctrl_param) ; i++)
		if (type == eio_hwmon_ctrl_param[i].type &&
		    ctrl == eio_hwmon_ctrl_param[i].ctrl)
			return i;

	return 0;
}

static int pmc_read(struct device *mfd, enum _sen_type type, u8 dev_id, u8 ctrl, void *data)
{
	int idx = para_idx(type, ctrl);
	int ret = 0;

	if (idx == 0)
		return -EINVAL;

	if (WARN_ON(!data))
		return -EINVAL;

	struct pmc_op op = {
		 .cmd       = sen_info[type].cmd | EIO_FLAG_PMC_READ,
		 .control   = ctrl,
		 .device_id = dev_id,
		 .size	    = eio_hwmon_ctrl_param[idx].size,
		 .payload   = (u8 *)data,
	};

	ret = eio_core_pmc_operation(mfd, &op);
	return ret;
}

static int pmc_write(struct device *mfd, enum _sen_type type, u8 dev_id, u8 ctrl, void *data)
{
	int idx = para_idx(type, ctrl);

	if (idx == 0)
		return -EINVAL;
	struct pmc_op op = {
		.cmd       = sen_info[type].cmd,
		.control   = ctrl,
		.device_id = dev_id,
		.size      = eio_hwmon_ctrl_param[idx].size,
		.payload   = (u8 *)data,
	};
	return eio_core_pmc_operation(mfd, &op);
}

/* Thermal zone operations */
static int eio_tz_get_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct eio_hwmon_dev *eio = thermal_zone_device_priv(tzd);
	u16 val = 0;
	int ret, i;

	/* Find which channel this thermal zone belongs to */
	for (i = 0; i < EIO_MAX_TEMP; i++) {
		if (eio->tzd[i] == tzd)
			break;
	}

	if (i >= EIO_MAX_TEMP)
		return -ENODEV;

	ret = pmc_read(eio->mfd, TEMP, i, sen_info[TEMP].ctrl[EIO_ITEM_INPUT], &val);
	if (ret)
		return ret;

	*temp = DECI_KELVIN_TO_MILLI_CELSIUS(val);
	return 0;
}

static int eio_tz_set_trip_temp(struct thermal_zone_device *tzd,
		const struct thermal_trip *trip, int temp)
{
	struct eio_hwmon_dev *eio = thermal_zone_device_priv(tzd);
	u16 val = MILLI_CELSIUS_TO_DECI_KELVIN(temp);
	u8 ctrl;
	int i;

	for (i = 0; i < EIO_MAX_TEMP; i++) {
		if (eio->tzd[i] == tzd)
			break;
	}
	if (i >= EIO_MAX_TEMP)
		return -ENODEV;

	if ((temp < MIN_TEMP) || (temp > MAX_TEMP))
		return -EINVAL;

	switch (trip->type) {
	case THERMAL_TRIP_CRITICAL:
		ctrl = CTRL_SHUTDOWN_HI;
		break;
	case THERMAL_TRIP_HOT:
		ctrl = CTRL_POWEROFF_HI;
		break;
	case THERMAL_TRIP_PASSIVE:
		ctrl = CTRL_THROTTLE_HI;
		break;
	default:
		return -EINVAL;
	}
	return pmc_write(eio->mfd, TEMP, i, ctrl, &val);
}

static int eio_tz_change_mode(struct thermal_zone_device *tzd,
			      enum thermal_device_mode mode)
{
	struct eio_hwmon_dev *eio = thermal_zone_device_priv(tzd);
	int trip, ret, i;

	for (i = 0; i < EIO_MAX_TEMP; i++) {
		if (eio->tzd[i] == tzd)
			break;
	}
	if (i >= EIO_MAX_TEMP)
		return -ENODEV;

	for (trip = 0; trip < TRIP_NUM; trip++) {
		ret = pmc_write(eio->mfd, TEMP, i, ctrl_sts_map[trip], &mode);
		if (ret)
			return ret;
	}

	return ret;
}

static const struct thermal_zone_params eio_thermal_params = {
	.no_hwmon = true,
};

static const struct thermal_zone_device_ops eio_thermal_ops = {
	.get_temp = eio_tz_get_temp,
	.set_trip_temp = eio_tz_set_trip_temp,
	.change_mode   = eio_tz_change_mode,
};

static int eio_fan_tz_get_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct eio_hwmon_dev *eio = thermal_zone_device_priv(tzd);
	int i;
	u8 sensor = 0;
	u16 val = 0;
	int ret;

	for (i = 0; i < EIO_MAX_FAN; i++) {
		if (eio->fan_tzd[i] == tzd)
			break;
	}
	if (i >= EIO_MAX_FAN)
		return -ENODEV;

	ret = pmc_read(eio->mfd, FAN, i, CTRL_CTRL, &sensor);
	if (ret)
		return ret;

	ret = pmc_read(eio->mfd, TEMP, FAN_SRC(sensor), sen_info[TEMP].ctrl[EIO_ITEM_INPUT], &val);
	if (ret)
		return ret;

	*temp = DECI_KELVIN_TO_MILLI_CELSIUS(val);
	return 0;
}

static int eio_fan_tz_set_trip_temp(struct thermal_zone_device *tzd, const struct thermal_trip *trip, int temp)
{
	struct eio_hwmon_dev *eio = thermal_zone_device_priv(tzd);
	u16 val = MILLI_CELSIUS_TO_DECI_KELVIN(temp);
	u8 ctrl;
	int i;

	for (i = 0; i < EIO_MAX_FAN; i++) {
		if (eio->fan_tzd[i] == tzd)
			break;
	}
	if (i >= EIO_MAX_FAN)
		return -ENODEV;

	if ((temp < MIN_TEMP) || (temp > MAX_TEMP))
		return -EINVAL;

	switch (trip->type) {
	case THERMAL_TRIP_HOT:
		ctrl = CTRL_THERM_HIGH;
		break;
	case THERMAL_TRIP_ACTIVE:
		ctrl = CTRL_THERM_LOW;
		break;
	case THERMAL_TRIP_PASSIVE:
		ctrl = CTRL_THERM_STOP;
		break;
	default:
		return -EINVAL;
	}

	return pmc_write(eio->mfd, FAN, i, ctrl, &val);
}

static bool eio_fan_should_bind(struct thermal_zone_device *tzd,
				const struct thermal_trip *trip,
				struct thermal_cooling_device *cdev,
				struct cooling_spec *spec)
{
	struct eio_hwmon_dev *eio = thermal_zone_device_priv(tzd);

	if (!eio)
		return false;

	for (int i = 0; i < EIO_MAX_FAN; i++) {
		if (eio->fan_cdev[i] == cdev)
			return true;
	}

	return false;
}

static const struct thermal_zone_device_ops eio_fan_tz_ops = {
	.get_temp = eio_fan_tz_get_temp,
	.set_trip_temp = eio_fan_tz_set_trip_temp,
	.should_bind   = eio_fan_should_bind,
};

static int eio_fan_cdev_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = 100;
	return 0;
}

static int eio_fan_cdev_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct eio_hwmon_dev *eio = cdev->devdata;
	int i;
	u8 fan_mode = 0;
	u8 duty = 0;
	int ret;

	for (i = 0; i < EIO_MAX_FAN; i++) {
		if (eio->fan_cdev[i] == cdev)
			break;
	}
	if (i >= EIO_MAX_FAN)
		return -ENODEV;

	ret = pmc_read(eio->mfd, FAN, i, CTRL_CTRL, &fan_mode);
	if (ret)
		return ret;

	switch (fan_mode & FAN_MODE_MASK) {
	case CTRLMODE_STOP:
		*state = 0;
		break;
	case CTRLMODE_FULL:
		*state = 100;
		break;
	case CTRLMODE_AUTO:
		*state = 0;
		break;
	case CTRLMODE_MANUAL:
		ret = pmc_read(eio->mfd, FAN, i, CTRL_VALUE, &duty);
		if (ret)
			return ret;
		*state = clamp_val(duty, 0, 100);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int eio_fan_cdev_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct eio_hwmon_dev *eio = cdev->devdata;
	int i;
	u8 ctrl = 0;
	u8 duty;
	int ret;

	for (i = 0; i < EIO_MAX_FAN; i++) {
		if (eio->fan_cdev[i] == cdev)
			break;
	}
	if (i >= EIO_MAX_FAN)
		return -ENODEV;

	ret = pmc_read(eio->mfd, FAN, i, CTRL_CTRL, &ctrl);
	if (ret)
		return ret;

	if ((ctrl & FAN_MODE_MASK) != CTRLMODE_MANUAL)
		return -EOPNOTSUPP;

	duty = clamp_val((u8)state, 0, 100);
	return pmc_write(eio->mfd, FAN, i, CTRL_VALUE, &duty);
}

static const struct thermal_cooling_device_ops eio_fan_cdev_ops = {
	.get_max_state = eio_fan_cdev_get_max_state,
	.get_cur_state = eio_fan_cdev_get_cur_state,
	.set_cur_state = eio_fan_cdev_set_cur_state,
};

static ssize_t fan_mode_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	static const char * const names[] = { "Stop", "Full", "Manual", "Auto" };
	struct thermal_cooling_device *cdev = dev_get_drvdata(dev);
	struct eio_hwmon_dev *eio = cdev->devdata;
	u8 mode = 0;
	int ret, i;

	for (i = 0; i < EIO_MAX_FAN; i++) {
		if (eio->fan_cdev[i] == cdev)
			break;
	}
	if (i >= EIO_MAX_FAN)
		return -ENODEV;

	ret = pmc_read(eio->mfd, FAN, i, CTRL_CTRL, &mode);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n", names[mode & FAN_MODE_MASK]);
}

static ssize_t fan_mode_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	static const char * const names[] = { "Stop", "Full", "Manual", "Auto" };
	struct thermal_cooling_device *cdev = dev_get_drvdata(dev);
	struct eio_hwmon_dev *eio = cdev->devdata;
	u8 ctrl, newc;
	int mode_idx, ret, i;

	for (i = 0; i < EIO_MAX_FAN; i++) {
		if (eio->fan_cdev[i] == cdev)
			break;
	}
	if (i >= EIO_MAX_FAN)
		return -ENODEV;

	for (mode_idx = 0; mode_idx < ARRAY_SIZE(names); mode_idx++) {
		if (strncasecmp(buf, names[mode_idx], strlen(names[mode_idx])))
			continue;

		ret = pmc_read(eio->mfd, FAN, i, CTRL_CTRL, &ctrl);
		if (ret)
			return -EIO;

		newc  = ctrl & FAN_SRC_MASK;

		switch (mode_idx) {
		case CTRLMODE_AUTO:
			newc |= FAN_FRAME_BIT;
			newc &= ~FAN_SCM_BIT;
			newc |= CTRLMODE_AUTO;
			break;
		case CTRLMODE_MANUAL:
			newc &= ~FAN_FRAME_BIT;
			newc &= ~FAN_SCM_BIT;
			newc |= CTRLMODE_MANUAL;
			break;
		case CTRLMODE_FULL:
			newc &= ~FAN_FRAME_BIT;
			newc &= ~FAN_SCM_BIT;
			newc |= CTRLMODE_FULL;
			break;
		case CTRLMODE_STOP:
		default:
			newc &= ~FAN_FRAME_BIT;
			newc &= ~FAN_SCM_BIT;
			newc |= CTRLMODE_STOP;
			break;
		}

		ret = pmc_write(eio->mfd, FAN, i, CTRL_CTRL, &newc);
		return ret ? ret : count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_RW(fan_mode);

static umode_t eio_is_visible(const void *drvdata, enum hwmon_sensor_types type,
			      u32 attr, int channel)
{
	const struct eio_hwmon_dev *eio = drvdata;

	switch (type) {
	case hwmon_in:
		return (channel < EIO_MAX_IN && eio->in[channel].available) ? 0444 : 0;
	case hwmon_curr:
		return (channel < EIO_MAX_CURR && eio->curr[channel].available) ? 0444 : 0;
	case hwmon_temp:
		return (channel < EIO_MAX_TEMP && eio->temp[channel].available) ? 0444 : 0;
	case hwmon_fan:
		return (channel < EIO_MAX_FAN && eio->fan[channel].available) ? 0444 : 0;
	case hwmon_intrusion:
		return (channel < EIO_MAX_INTRUSION && eio->intrusion.available) ? 0444 : 0;
	default:
		return 0;
	}
}

static int eio_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	struct eio_hwmon_dev *eio = dev_get_drvdata(dev);
	u8 data[MAX_CMD_SIZE] = {};
	enum _sen_type ec_type;
	u8 item;
	u32 raw;
	int ret;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			item = EIO_ITEM_INPUT;
			break;
		case hwmon_in_max:
			item = EIO_ITEM_MAX;
			break;
		case hwmon_in_min:
			item = EIO_ITEM_MIN;
			break;
		default:
			return -EOPNOTSUPP;
		}
		ec_type = VOLTAGE;
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			item = EIO_ITEM_INPUT;
			break;
		case hwmon_curr_max:
			item = EIO_ITEM_MAX;
			break;
		case hwmon_curr_min:
			item = EIO_ITEM_MIN;
			break;
		default:
			return -EOPNOTSUPP;
		}
		ec_type = CURRENT;
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			item = EIO_ITEM_INPUT;
			break;
		case hwmon_temp_max:
			item = EIO_ITEM_MAX;
			break;
		case hwmon_temp_min:
			item = EIO_ITEM_MIN;
			break;
		case hwmon_temp_crit:
			item = EIO_ITEM_CRIT;
			break;
		case hwmon_temp_emergency:
			item = EIO_ITEM_EMERGENCY;
			break;
		default:
			return -EOPNOTSUPP;
		}
		ec_type = TEMP;
		break;
	case hwmon_fan:
		if (attr != hwmon_fan_input)
			return -EOPNOTSUPP;
		item = EIO_ITEM_INPUT;
		ec_type = FAN;
		break;
	case hwmon_intrusion:
		if (attr != hwmon_intrusion_alarm)
			return -EOPNOTSUPP;
		item = EIO_ITEM_INPUT;
		ec_type = CASEOPEN;
		channel = 0;
		break;
	default:
		return -EOPNOTSUPP;
	}

	ret = pmc_read(eio->mfd, ec_type, channel, sen_info[ec_type].ctrl[item], data);
	if (ret)
		return ret;

	raw = (u32)data[0] | (u32)data[1] << 8 | (u32)data[2] << 16 | (u32)data[3] << 24;
	*val = ((long)(signed int)raw + sen_info[ec_type].shift) *
	       (long)sen_info[ec_type].multi[item];
	return 0;
}

static int eio_read_string(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, const char **str)
{
	struct eio_hwmon_dev *eio = dev_get_drvdata(dev);
	enum _sen_type ec_type;
	u8 label_idx;

	switch (type) {
	case hwmon_in:
		if (attr != hwmon_in_label)
			return -EOPNOTSUPP;
		ec_type = VOLTAGE;
		label_idx = eio->in[channel].label_idx;
		break;
	case hwmon_curr:
		if (attr != hwmon_curr_label)
			return -EOPNOTSUPP;
		ec_type = CURRENT;
		label_idx = eio->curr[channel].label_idx;
		break;
	case hwmon_temp:
		if (attr != hwmon_temp_label)
			return -EOPNOTSUPP;
		ec_type = TEMP;
		label_idx = eio->temp[channel].label_idx;
		break;
	case hwmon_fan:
		if (attr != hwmon_fan_label)
			return -EOPNOTSUPP;
		ec_type = FAN;
		label_idx = eio->fan[channel].label_idx;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (label_idx >= ARRAY_SIZE(sen_info[ec_type].labels))
		return -EOPNOTSUPP;

	*str = sen_info[ec_type].labels[label_idx];
	return 0;
}

static const struct hwmon_ops eio_hwmon_ops = {
	.is_visible  = eio_is_visible,
	.read        = eio_read,
	.read_string = eio_read_string,
};

static const struct hwmon_channel_info * const eio_hwmon_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN,
			   HWMON_I_LABEL | HWMON_I_INPUT | HWMON_I_MAX | HWMON_I_MIN),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_LABEL | HWMON_C_INPUT | HWMON_C_MAX | HWMON_C_MIN,
			   HWMON_C_LABEL | HWMON_C_INPUT | HWMON_C_MAX | HWMON_C_MIN),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_LABEL | HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_MIN | HWMON_T_CRIT | HWMON_T_EMERGENCY,
			   HWMON_T_LABEL | HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_MIN | HWMON_T_CRIT | HWMON_T_EMERGENCY,
			   HWMON_T_LABEL | HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_MIN | HWMON_T_CRIT | HWMON_T_EMERGENCY,
			   HWMON_T_LABEL | HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_MIN | HWMON_T_CRIT | HWMON_T_EMERGENCY),
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_LABEL | HWMON_F_INPUT,
			   HWMON_F_LABEL | HWMON_F_INPUT,
			   HWMON_F_LABEL | HWMON_F_INPUT,
			   HWMON_F_LABEL | HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(intrusion, HWMON_INTRUSION_ALARM),
	NULL,
};

static const struct hwmon_chip_info eio_chip_info = {
	.ops  = &eio_hwmon_ops,
	.info = eio_hwmon_info,
};


static int thermal_zone_init(struct device *mfd, struct eio_hwmon_dev *eio, u8 ch)
{
	u16 state = 0, name_idx = 0;
	u16 hi_shutdown = 0, hi_poweroff = 0, hi_throttle = 0;
	struct thermal_trip trips[3];
	int ntrips = 0, throttle_temp, ret;

	if (pmc_read(mfd, TEMP, ch, CTRL_STATE, &state) ||
	    pmc_read(mfd, TEMP, ch, CTRL_TYPE, &name_idx)) {
		return -EIO;
	}

	if (!(state & THERM_STS_AVAIL) ||
	    !((state & THERM_STS_THROTTLE_AVAIL) ||
	      (state & THERM_STS_POWEROFF_AVAIL) ||
	      (state & THERM_STS_SHUTDOWN_AVAIL))) {
		return -ENODEV;
	}

	if (name_idx >= ARRAY_SIZE(sen_info[TEMP].labels) || !sen_info[TEMP].labels[name_idx][0])
		return -EINVAL;

	throttle_temp = MILLI_CELSIUS_TO_DECI_KELVIN(60000);
	pmc_write(mfd, TEMP, ch, CTRL_THROTTLE_HI, &throttle_temp);

	pmc_read(mfd, TEMP, ch, CTRL_SHUTDOWN_HI, &hi_shutdown);
	pmc_read(mfd, TEMP, ch, CTRL_POWEROFF_HI, &hi_poweroff);
	pmc_read(mfd, TEMP, ch, CTRL_THROTTLE_HI, &hi_throttle);

	if (hi_shutdown) {
		trips[ntrips++] = (struct thermal_trip){
			.type = THERMAL_TRIP_CRITICAL,
			.temperature = DECI_KELVIN_TO_MILLI_CELSIUS(hi_shutdown),
			.flags = THERMAL_TRIP_FLAG_RW_TEMP,
			.priv = THERMAL_INT_TO_TRIP_PRIV(TRIP_SHUTDOWN),
		};
	}
	if (hi_poweroff) {
		trips[ntrips++] = (struct thermal_trip){
			.type = THERMAL_TRIP_HOT,
			.temperature = DECI_KELVIN_TO_MILLI_CELSIUS(hi_poweroff),
			.flags = THERMAL_TRIP_FLAG_RW_TEMP,
			.priv = THERMAL_INT_TO_TRIP_PRIV(TRIP_POWEROFF),
		};
	}
	if (hi_throttle) {
		trips[ntrips++] = (struct thermal_trip){
			.type = THERMAL_TRIP_PASSIVE,
			.temperature = DECI_KELVIN_TO_MILLI_CELSIUS(hi_throttle),
			.flags = THERMAL_TRIP_FLAG_RW_TEMP,
			.priv = THERMAL_INT_TO_TRIP_PRIV(TRIP_THROTTLE),
		};
	}
	if (!ntrips)
		return -ENODEV;

	 eio->tzd[ch] = thermal_zone_device_register_with_trips(
		sen_info[TEMP].labels[name_idx],
		trips, ntrips, eio, &eio_thermal_ops, &eio_thermal_params,
		THERMAL_PASSIVE_DELAY, THERMAL_POLLING_DELAY);

	if (IS_ERR(eio->tzd[ch]))
		return PTR_ERR(eio->tzd[ch]);

	ret = thermal_zone_device_disable(eio->tzd[ch]);
	if (ret)
		goto err_zone;

	return 0;

err_zone:
	thermal_zone_device_unregister(eio->tzd[ch]);
	return ret;
}

static int fan_thermal_init(struct device *mfd, struct eio_hwmon_dev *eio, u8 fan_id)
{
	u8 state, name_idx, pwm_hi, pwm_lo;
	u16 trip_hi, trip_lo, trip_stop;
	struct thermal_trip trips[3];
	int ntrips = 0;
	int ret;

	if (pmc_read(mfd, FAN, fan_id, CTRL_STATE, &state) ||
		pmc_read(mfd, FAN, fan_id, CTRL_TYPE, &name_idx) ||
		pmc_read(mfd, FAN, fan_id, CTRL_THERM_HIGH, &trip_hi) ||
		pmc_read(mfd, FAN, fan_id, CTRL_THERM_LOW, &trip_lo) ||
		pmc_read(mfd, FAN, fan_id, CTRL_THERM_STOP, &trip_stop) ||
		pmc_read(mfd, FAN, fan_id, CTRL_PWM_HIGH, &pwm_hi) ||
		pmc_read(mfd, FAN, fan_id, CTRL_PWM_LOW, &pwm_lo))
		return -EIO;

	if (!(state & 0x01))
		return -ENODEV;

	if (name_idx >= ARRAY_SIZE(sen_info[FAN].labels) ||
		!sen_info[FAN].labels[name_idx][0])
		return -EINVAL;

	if (trip_hi) {
		trips[ntrips++] = (struct thermal_trip) {
			.type = THERMAL_TRIP_HOT,
			.temperature = DECI_KELVIN_TO_MILLI_CELSIUS(trip_hi),
			.flags = THERMAL_TRIP_FLAG_RW_TEMP,
		};
	}
	if (trip_lo) {
		trips[ntrips++] = (struct thermal_trip) {
			.type = THERMAL_TRIP_ACTIVE,
			.temperature = DECI_KELVIN_TO_MILLI_CELSIUS(trip_lo),
			.flags = THERMAL_TRIP_FLAG_RW_TEMP,
		};
	}
	if (trip_stop) {
		trips[ntrips++] = (struct thermal_trip) {
			.type = THERMAL_TRIP_PASSIVE,
			.temperature = DECI_KELVIN_TO_MILLI_CELSIUS(trip_stop),
			.flags = THERMAL_TRIP_FLAG_RW_TEMP,
		};
	}

	if (!ntrips)
		return -ENODEV;

	eio->fan_cdev[fan_id] = devm_thermal_cooling_device_register(eio->hwmon,
			sen_info[FAN].labels[name_idx], eio, &eio_fan_cdev_ops);
	if (IS_ERR(eio->fan_cdev[fan_id]))
		return PTR_ERR(eio->fan_cdev[fan_id]);

	dev_set_drvdata(&eio->fan_cdev[fan_id]->device, eio->fan_cdev[fan_id]);
	ret = device_create_file(&eio->fan_cdev[fan_id]->device, &dev_attr_fan_mode);
	if (ret)
		goto cdev_err;

	eio->fan_tzd[fan_id] = thermal_zone_device_register_with_trips(
			sen_info[FAN].labels[name_idx],
			trips, ntrips, eio, &eio_fan_tz_ops, &eio_thermal_params,
			THERMAL_PASSIVE_DELAY, THERMAL_POLLING_DELAY);
	if (IS_ERR(eio->fan_tzd[fan_id])) {
		ret = PTR_ERR(eio->fan_tzd[fan_id]);
		goto cdev_err;
	}

	ret = thermal_zone_device_enable(eio->fan_tzd[fan_id]);
	if (ret)
		goto thermal_err;

	return 0;

thermal_err:
	thermal_zone_device_unregister(eio->fan_tzd[fan_id]);
cdev_err:
	thermal_cooling_device_unregister(eio->fan_cdev[fan_id]);
	return ret;
}


static int hwmon_init(struct device *mfd, struct eio_hwmon_dev *eio)
{
	enum _sen_type type;
	u8 i, data[MAX_CMD_SIZE];
	int n = 0;
	int ret;

	for (type = VOLTAGE ; type <= TEMP ; type++) {
		struct eio_chan *chans = (type == VOLTAGE) ? eio->in :
					(type == CURRENT) ? eio->curr : eio->temp;

		for (i = 0 ; i < sen_info[type].max ; i++) {
			memset(data, 0, sizeof(data));
			if (pmc_read(mfd, type, i, 0x00, data) ||
			    (data[0] & 0x01) == 0)
				continue;

			memset(data, 0, sizeof(data));
			ret = pmc_read(mfd, type, i, 0x01, data);
			if (ret != 0 && ret != -EINVAL)
				continue;

			chans[i].available = true;
			chans[i].label_idx = data[0];
			n++;
		}
	}

	for (i = 0 ; i < sen_info[FAN].max ; i++) {
		memset(data, 0, sizeof(data));
		if (pmc_read(mfd, FAN, i, 0x00, data) ||
		    (data[0] & 0x01) == 0)
			continue;

		memset(data, 0, sizeof(data));
		ret = pmc_read(mfd, FAN, i, 0x01, data);
		if (ret != 0 && ret != -EINVAL)
			continue;

		eio->fan[i].available = true;
		eio->fan[i].label_idx = data[0];
		n++;
	}

	memset(data, 0, sizeof(data));
	if (!pmc_read(mfd, CASEOPEN, 0, 0x00, data) && (data[0] & 0x01)) {
		eio->intrusion.available = true;
		n++;
	}

	return n;
}

static int hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct eio_hwmon_dev *eio;
	struct eio_dev *eio_dev = dev_get_drvdata(dev->parent);
	int ret;

	if (!eio_dev) {
		dev_err(dev, "Error contact eio_core\n");
		return -ENODEV;
	}

	eio = devm_kzalloc(dev, sizeof(*eio), GFP_KERNEL);
	if (!eio)
		return -ENOMEM;

	eio->mfd = dev->parent;

	if (hwmon_init(dev->parent, eio) <= 0)
		return -ENODEV;

	eio->hwmon = devm_hwmon_device_register_with_info(dev, KBUILD_MODNAME,
						     eio, &eio_chip_info, NULL);
	if (IS_ERR(eio->hwmon))
		return PTR_ERR_OR_ZERO(eio->hwmon);

	for (int i = 0 ; i < sen_info[TEMP].max ; i++) {
		if (eio->temp[i].available) {
			ret = thermal_zone_init(eio->mfd, eio, i);
			if (ret)
				dev_warn(eio->hwmon, "Failed to init thermal zone for channel %d: %d\n", i, ret);
		}
	}
	for (int i = 0; i < sen_info[FAN].max; i++) {
		if (eio->fan[i].available) {
			ret = fan_thermal_init(eio->mfd, eio, i);
			if (ret)
				dev_warn(eio->hwmon, "Failed to init fan thermal %d: %d\n", i, ret);
	}
}
	return 0;

}

static struct platform_driver eio_hwmon_driver = {
	.probe  = hwmon_probe,
	.driver = {
		.name = "eio_hwmon",
	},
};

module_platform_driver(eio_hwmon_driver);

MODULE_AUTHOR("Wenkai Chung <wenkai.chung@advantech.com.tw>");
MODULE_AUTHOR("Ramiro Oliveira <ramiro.oliveira@advantech.com>");
MODULE_DESCRIPTION("Hardware monitor driver for Advantech EIO embedded controller");
MODULE_LICENSE("GPL");
