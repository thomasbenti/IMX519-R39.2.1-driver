// SPDX-License-Identifier: GPL-2.0
// Source code software license
#include <linux/module.h> // Definitions for creating a Linux kernel module
#include <linux/i2c.h> // Interface for communicating with I2C devices
#include <linux/of.h> // Interface for device tree

// Clock, GPIO descriptor, and regulator interfaces
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include<linux/slab.h> // Provides declaration needed for memory allocation
#include <linux/delay.h>

#define IMX519_CHIP_ID_REG 0x0016 // Register address containing the sensor's identification value
#define IMX519_CHIP_ID 0x0519 // Identification value we expect to read from the sensor

#define IMX519_XCLK_FREQ 24000000

#define IMX519_XCLR_MIN_DELAY_US 8000
#define IMX519_XCLR_DELAY_RANGE_US 1000

/* Defining 3 regulator names
 * Each element points to a string that is immutable through the pointer.
 */
static const char * const imx519_supply_names[] = {
    "VANA",
    "VDIG",
    "VDDL",
};


#define IMX519_NUM_SUPPLIES ARRAY_SIZE(imx519_supply_names)

// Contains sensor's hardware resources
struct imx519 {
    struct clk *xclk;
    struct gpio_desc *reset_gpio;
    struct regulator_bulk_data supplies[IMX519_NUM_SUPPLIES];
};


/*
1.) Function receives a pointer to the sensor struct
2.) Loops through the 3 supplies; for each iteration, copies a supply-name pointer to the corresponding regulator_bulk_data struct
*/
static void imx519_init_supplies(struct imx519 *imx519)
{
    int i;

    for (i = 0; i < IMX519_NUM_SUPPLIES; i++) {
        imx519->supplies[i].supply = imx519_supply_names[i];
    }
}


// Requesting regulators from Linux
static int imx519_get_supplies(struct device *dev, struct imx519 *imx519)
{
    imx519_init_supplies(imx519);

    // Asks Linux's regulator subsystem to get these resources
    return devm_regulator_bulk_get(
        dev,
        IMX519_NUM_SUPPLIES,
        imx519->supplies
    );
}


static int imx519_enable_supplies(struct imx519 *imx519)
{
    int ret;

    ret = regulator_bulk_enable(
        IMX519_NUM_SUPPLIES,
        imx519->supplies
    );

    return ret;
}


static void imx519_disable_supplies(struct imx519 *imx519)
{
    regulator_bulk_disable(
        IMX519_NUM_SUPPLIES,
        imx519->supplies
    );
}


static int imx519_get_clock(struct device *dev, struct imx519 *imx519)
{
    imx519->xclk = devm_clk_get(dev, "mclk");

    if (IS_ERR(imx519->xclk))
        return PTR_ERR(imx519->xclk);

    return 0;
}


static int imx519_enable_clock(struct imx519 *imx519)
{
    int ret;
    unsigned long rate;

    ret = clk_set_rate(imx519->xclk, IMX519_XCLK_FREQ);
    if (ret)
        return ret;

    rate = clk_get_rate(imx519->xclk);

    if (rate != IMX519_XCLK_FREQ)
        return -EINVAL;

    return clk_prepare_enable(imx519->xclk);
}


static void imx519_disable_clock(struct imx519 *imx519)
{
    clk_disable_unprepare(imx519->xclk);
}


static int imx519_get_reset_gpio(struct device *dev,
                                struct imx519 *imx519)
{
    // Request the GPIO and initially hold XCLR low
    imx519->reset_gpio = devm_gpiod_get_optional(
        dev, "reset", GPIOD_OUT_LOW
    );

    // Check whether Linux returned an error pointer
    if (IS_ERR(imx519->reset_gpio))
        return PTR_ERR(imx519->reset_gpio);

    return 0;
}


// Read one byte from a sensor register
static int imx519_read_reg8(struct i2c_client *client,
                           u16 reg, u8 *value)
{
    u8 addr[2] = {
        reg >> 8,
        reg & 0xff
    };

    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,
            .len = sizeof(addr),
            .buf = addr,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = value,
        }
    };

    int ret = i2c_transfer(client->adapter, msgs, 2);

    if (ret != 2)
        return ret < 0 ? ret : -EIO;

    return 0;
}


// Write one byte to a sensor register
static int imx519_write_reg8(struct i2c_client *client,
                            u16 reg, u8 value)
{
    u8 data[3] = {
        reg >> 8,
        reg & 0xff,
        value
    };

    int ret = i2c_master_send(client, data, sizeof(data));

    if (ret != sizeof(data))
        return ret < 0 ? ret : -EIO;

    return 0;
}


static int imx519_read_id(struct i2c_client *client /* Pointer to a Linux structure representing the camera on I2C */,
                         u16 *chip_id /* Pointer to a 16-bit unsigned integer to store the sensor's ID */)
{
    u8 addr[2] = {
        IMX519_CHIP_ID_REG >> 8,
        IMX519_CHIP_ID_REG & 0xff
    };

    u8 data[2]; // Array of 2 unsigned 8-bit integers received from the sensor

    /*
    Declares an array of two i2c_msg structures, each describing an I2C message
    First one sends the register address
    Second one reads the register value
    */
    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,
            .len = sizeof(addr),
            .buf = addr,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD, // Flag that tells Linux whether an I2C message should read or write data
            .len = sizeof(data),
            .buf = data,
        }
    };

    int ret;
    ret = i2c_transfer(client->adapter, msgs, 2); // Returns 2 if both messages were processed correctly
    if (ret != 2)
        return ret < 0 ? ret : -EIO;

    *chip_id = (data[0] << 8) | data[1]; // Combine two 8-bit I2C values into one 16-bit value
    return 0;
}


/*
1.) Enable the external sensor clock
2.) Release XCLR to initialize the sensor
3.) Wait 8-9 milliseconds before accessing the sensor

For initial testing, sensor supply control is handled
outside this driver. Board-level power must be verified.
*/
static int imx519_power_on(struct imx519 *imx519)
{
    int ret;

    // Configure and enable the external clock
    ret = imx519_enable_clock(imx519);
    if (ret)
        return ret;

    // Release XCLR high (candidate active-high mapping)
    if (imx519->reset_gpio)
        gpiod_set_value_cansleep(imx519->reset_gpio, 1);

    // Allow the sensor to initialize
    usleep_range(
        IMX519_XCLR_MIN_DELAY_US,
        IMX519_XCLR_MIN_DELAY_US + IMX519_XCLR_DELAY_RANGE_US
    );

    return 0;
}


/*
1.) Assert XCLR to hold the sensor in reset
2.) Disable the external sensor clock
*/
static void imx519_power_off(struct imx519 *imx519)
{
    // Hold XCLR low
    if (imx519->reset_gpio)
        gpiod_set_value_cansleep(imx519->reset_gpio, 0);

    // Disable the external clock
    imx519_disable_clock(imx519);
}



/*
    imx519_probe is called when Linux finds a device
    that matches the driver's registered device information.
*/
static int imx519_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct imx519 *imx519;
    u16 chip_id;
    int ret;

    u8 id_high;
    u8 id_low;

    u8 mode_select;

    dev_info(dev, "IMX519 probe started\n");

    // Allocate memory for the sensor's hardware resources
    imx519 = devm_kzalloc(dev, sizeof(*imx519), GFP_KERNEL);
    if (!imx519)
        return -ENOMEM;

    // Acquire the external clock
    ret = imx519_get_clock(dev, imx519);
    if (ret)
        return dev_err_probe(dev, ret, "failed to get clock\n");

    // Acquire the reset GPIO
    ret = imx519_get_reset_gpio(dev, imx519);
    if (ret)
        return dev_err_probe(dev, ret, "failed to get reset GPIO\n");

    // Require reset control for this identification test
    if (!imx519->reset_gpio)
        return dev_err_probe(dev, -ENODEV, "reset GPIO missing\n");

    // Enable the clock and release the sensor from reset
    ret = imx519_power_on(imx519);
    if (ret)
        return dev_err_probe(dev, ret, "failed to power on sensor\n");


    // Check reset GPIO before communicating with the sensor
    ret = gpiod_get_value_cansleep(imx519->reset_gpio);
    if (ret < 0)
        goto power_off;

    // Print the current clock and reset states
    dev_info(dev, "Clock: %lu Hz\n",
             clk_get_rate(imx519->xclk));

    dev_info(dev, "XCLR logical value: %d\n", ret);

    // Read the sensor's chip ID
    ret = imx519_read_id(client, &chip_id);
    if (ret) {
        dev_err(dev, "failed to read chip ID: %d\n", ret);
        goto power_off;
    }

    // Verify the chip ID
    if (chip_id != IMX519_CHIP_ID) {
        dev_err(dev, "unexpected chip ID: 0x%04x\n", chip_id);
        ret = -ENODEV;
        goto power_off;
    }

    dev_info(dev, "IMX519 detected: 0x%04x\n", chip_id);

    // Verify the reusable register-read function
    ret = imx519_read_reg8(client, 0x0016, &id_high);
    if (ret)
        goto power_off;

    ret = imx519_read_reg8(client, 0x0017, &id_low);
    if (ret)
        goto power_off;

    dev_info(dev, "ID registers: 0x%02x 0x%02x\n",
            id_high, id_low);

    if (((u16)id_high << 8 | id_low) != chip_id) {
        ret = -EIO;
        goto power_off;
    }

    // Identification succeeded
    ret = 0;

    // Put the sensor into standby mode
    ret = imx519_write_reg8(client, 0x0100, 0x00);
    if (ret) {
        dev_err(dev, "failed to write mode select: %d\n", ret);
        goto power_off;
    }

    // Read the register back to verify the write
    ret = imx519_read_reg8(client, 0x0100, &mode_select);
    if (ret)
        goto power_off;

    dev_info(dev, "Mode select register: 0x%02x\n", mode_select);

    if (mode_select != 0x00) {
        ret = -EIO;
        goto power_off;
    }

power_off:
    // Restore reset and disable the external clock
    imx519_power_off(imx519);

    return ret;
}


/* Register device with Linux
struct of_device_id: A Linux structure used to match drivers with devices described in the device tree
imx519_of_match: An array of supported device identifiers
*/
static const struct of_device_id imx519_of_match[] = {
    { .compatible = "sony,imx519" }, // Tells Linux that the driver supports a device-tree node containing sony,imx519
    {}
};


MODULE_DEVICE_TABLE(of, imx519_of_match);

static struct i2c_driver imx519_driver = {
    .driver = {
        .name = "nv_imx519",
        .of_match_table = imx519_of_match,
    },
    .probe = imx519_probe,
};

module_i2c_driver(imx519_driver);

MODULE_DESCRIPTION("Experimental IMX519 identification driver");
MODULE_AUTHOR("IMX519 Jetson Port Project");
MODULE_LICENSE("GPL");
