// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for Advantech EIO Embedded controller.
 *
 * Copyright (C) 2025 Advantech Corporation. All rights reserved.
 */

#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/mfd/core.h>
#include <linux/mfd/eio.h>
#include <linux/module.h>

#define EIO_GPIO_MAX_PINS	48
#define EIO_GPIO_WRITE		0x18
#define EIO_GPIO_READ		0x19

struct eio_gpio_dev {
	u64 avail;
	int max;
	struct gpio_chip chip;
	struct device *dev;
};

static struct {
	int size;
	bool write;
} ctrl_para[] = {
	{ 0x01, false }, { 0x00, false }, { 0x00, false }, { 0x02, false },
	{ 0x01, false }, { 0x00, false }, { 0x00, false }, { 0x00, false },
	{ 0x00, false }, { 0x00, false }, { 0x00, false }, { 0x00, false },
	{ 0x00, false }, { 0x00, false }, { 0x00, false }, { 0x00, false },
	{ 0x01, true  }, { 0x01, true  }, { 0x02, true  }, { 0x02, true  },
	{ 0x02, false }, { 0x10, false }
};

enum gpio_ctrl {
	EIO_GPIO_STATUS	 	= 0x0,
	EIO_GPIO_GROUP_AVAIL 	= 0x3,
	EIO_GPIO_ERROR	 	= 0x04,
	EIO_GPIO_PIN_DIR	= 0x10,
	EIO_GPIO_PIN_LEVEL	= 0x11,
	EIO_GPIO_GROUP_DIR	= 0x12,
	EIO_GPIO_GROUP_LEVEL 	= 0x13,
	EIO_GPIO_MAPPING	= 0x14,
	EIO_GPIO_NAME	 	= 0x15
};

static struct {
	int group;
	int port;
} group_map[] = {
	{ 0, 0 }, { 0, 1 },
	{ 1, 0 }, { 1, 1 },
	{ 2, 0 }, { 2, 1 },
	{ 3, 0 }, { 3, 1 },
	{ 3, 2 }, { 3, 3 },
	{ 3, 4 }, { 3, 5 },
	{ 3, 6 }, { 3, 7 }
};

static int pmc_write(struct device *mfd_dev, u8 ctrl, u8 dev_id, void *data)
{
	struct pmc_op op = {
		 .cmd       = EIO_GPIO_WRITE,
		 .control   = ctrl,
		 .device_id = dev_id,
		 .payload   = (u8 *)data,
	};

	if (ctrl >= ARRAY_SIZE(ctrl_para))
		return -ENOMEM;

	if (!ctrl_para[ctrl].write)
		return -EINVAL;

	op.size = ctrl_para[ctrl].size;

	return eio_core_pmc_operation(mfd_dev, &op);
}

static int pmc_read(struct device *mfd_dev, u8 ctrl, u8 dev_id, void *data)
{
	struct pmc_op op = {
		 .cmd       = EIO_GPIO_READ,
		 .control   = ctrl,
		 .device_id = dev_id,
		 .payload   = (u8 *)data,
	};

	if (ctrl > ARRAY_SIZE(ctrl_para))
		return -ENOMEM;

	op.size = ctrl_para[ctrl].size;

	return eio_core_pmc_operation(mfd_dev, &op);
}

static int get_dir(struct gpio_chip *chip, unsigned int offset)
{
	u8 dir;
	int ret;

	ret = pmc_read(chip->parent, EIO_GPIO_PIN_DIR, offset, &dir);
	if (ret)
		return ret;

	return dir ? 0 : 1;
}

static int dir_input(struct gpio_chip *chip, unsigned int offset)
{
	u8 dir = 0;

	return pmc_write(chip->parent, EIO_GPIO_PIN_DIR, offset, &dir);
}

static int dir_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	u8 dir = 1;
	u8 val = value;

	pmc_write(chip->parent, EIO_GPIO_PIN_DIR, offset, &dir);

	return pmc_write(chip->parent, EIO_GPIO_PIN_LEVEL, offset, &val);
}

static int gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	u8 level;
	int ret;

	ret = pmc_read(chip->parent, EIO_GPIO_PIN_LEVEL, offset, &level);
	if (ret)
		return ret;

	return level;
}

static int gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	u8 val = value;

	return pmc_write(chip->parent, EIO_GPIO_PIN_LEVEL, offset, &val);
}

static int check_support(struct device *dev)
{
	u8  data;
	int ret;

	ret = pmc_read(dev, EIO_GPIO_STATUS, 0, &data);
	if (ret)
		return ret;

	if ((data & 0x01) == 0)
		return -EOPNOTSUPP;

	return 0;
}

static int check_pin(struct device *dev, int pin)
{
	int ret;
	int group, bit;
	u16 data;

	/* Get pin mapping */
	ret = pmc_read(dev, EIO_GPIO_MAPPING, pin, &data);
	if (ret)
		return ret;

	if ((data & 0xFF) > ARRAY_SIZE(group_map))
		return -EINVAL;

	group = group_map[data & 0xFF].group;
	bit   = data >> 8;

	/* Check mapped pin */
	ret = pmc_read(dev, EIO_GPIO_GROUP_AVAIL, group, &data);
	if (ret)
		return ret;

	return data & BIT(bit) ? 0 : -EOPNOTSUPP;
}

static int gpio_init(struct device *mfd, struct eio_gpio_dev *eio_gpio)
{
	int ret, i;

	ret = check_support(mfd);
	if (ret)
		return dev_err_probe(eio_gpio->dev, ret, "GPIO not supported\n");

	eio_gpio->avail = 0;

	for (i = 0 ; i < EIO_GPIO_MAX_PINS ; i++) {
		ret = check_pin(mfd, i);
		if (ret)
			continue;

		eio_gpio->avail |= BIT(i);
		eio_gpio->max = i + 1;
	}

	return eio_gpio->max ? 0 : -EOPNOTSUPP;
}

static int gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct eio_gpio_dev *eio_gpio;
	struct eio_dev *eio_dev = dev_get_drvdata(dev->parent);

	if (!eio_dev)
		return dev_err_probe(dev, -ENODEV, "Error contact eio_core\n");

	eio_gpio = devm_kzalloc(dev, sizeof(*eio_gpio), GFP_KERNEL);
	if (!eio_gpio)
		return -ENOMEM;

	eio_gpio->dev = dev;

	if (gpio_init(dev->parent, eio_gpio))
		return -EIO;

	eio_gpio->chip.parent = dev->parent;
	eio_gpio->chip.ngpio = eio_gpio->max;
	eio_gpio->chip.label = KBUILD_MODNAME;
	eio_gpio->chip.owner = THIS_MODULE;
	eio_gpio->chip.direction_input = dir_input;
	eio_gpio->chip.get = gpio_get;
	eio_gpio->chip.direction_output = dir_output;
	eio_gpio->chip.set = gpio_set;
	eio_gpio->chip.get_direction = get_dir;
	eio_gpio->chip.base = -1;
	eio_gpio->chip.can_sleep = true;

	return devm_gpiochip_add_data(dev, &eio_gpio->chip, eio_gpio);
}

static struct platform_driver gpio_driver = {
	.probe  = gpio_probe,
	.driver = { .name = KBUILD_MODNAME, },
};

module_platform_driver(gpio_driver);

MODULE_AUTHOR("Wenkai Chung <wenkai.chung@advantech.com.tw>");
MODULE_AUTHOR("Ramiro Oliveira <ramiro.oliveira@advantech.com>");
MODULE_DESCRIPTION("GPIO driver for Advantech EIO embedded controller");
MODULE_LICENSE("GPL");
