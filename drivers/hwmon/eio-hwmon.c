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

#define MAX_CMD_SIZE 4
#define MAX_NAME 32

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
	struct eio_chan in[EIO_MAX_IN];
	struct eio_chan curr[EIO_MAX_CURR];
	struct eio_chan temp[EIO_MAX_TEMP];
	struct eio_chan fan[EIO_MAX_FAN];
	struct eio_chan intrusion;
};

static struct {
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

static struct {
	enum _sen_type type;
	u8   ctrl;
	int  size;
	bool write;
} ctrl_para[] = {
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
	{ TEMP,     0x21, 2, false }, { TEMP,	  0x41, 2, false },

	{ FAN,      0x00, 1, false }, { FAN,	  0x01, 1, false },
	{ FAN,      0x03, 1, true  }, { FAN,	  0x1A, 2, false },

	{ CASEOPEN, 0x00, 1, false }, { CASEOPEN, 0x02, 1, true  },
};

static int para_idx(enum _sen_type type, u8 ctrl)
{
	int i;

	for (i = 1 ; i < ARRAY_SIZE(ctrl_para) ; i++)
		if (type == ctrl_para[i].type &&
		    ctrl == ctrl_para[i].ctrl)
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
		 .size	    = ctrl_para[idx].size,
		 .payload   = (u8 *)data,
	};

	ret = eio_core_pmc_operation(mfd, &op);
	return ret;
}


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
	struct device *hwmon;

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

	hwmon = devm_hwmon_device_register_with_info(dev, KBUILD_MODNAME,
						     eio, &eio_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon);
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
