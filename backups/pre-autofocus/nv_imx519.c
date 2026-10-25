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

#include <linux/regmap.h>
#include <linux/of_gpio.h>        // Read GPIO definitions from the device tree
#include <linux/gpio.h>
#include <media/camera_common.h>  // NVIDIA camera platform configuration
#include <media/tegracam_core.h>  // NVIDIA Tegra camera device interface

#include "imx519_mode_tbls.h"
#include "imx519_common_tbls.h"

#include <linux/math64.h>

#define IMX519_CHIP_ID_REG 0x0016 // Register address containing the sensor's identification value
#define IMX519_CHIP_ID 0x0519 // Identification value we expect to read from the sensor

#define IMX519_XCLK_FREQ 24000000

#define IMX519_XCLR_MIN_DELAY_US 8000
#define IMX519_XCLR_DELAY_RANGE_US 1000

#define IMX519_REG_ANALOG_GAIN 0x0204

// Gain
#define IMX519_GAIN_FACTOR 16
#define IMX519_MIN_GAIN 16
#define IMX519_MAX_GAIN 256

// Exposure 
#define IMX519_REG_EXPOSURE 0x0202

#define IMX519_PIXEL_RATE 686000000ULL
#define IMX519_LINE_LENGTH 9689ULL
#define IMX519_FRAME_LENGTH 1180
#define IMX519_EXPOSURE_OFFSET 32

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

    struct regmap *regmap;

    // NVIDIA camera framework device
    struct tegracam_device *tc_dev;
};


static const struct regmap_config imx519_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
    .cache_type = REGCACHE_NONE,
};


// Our first supported IMX519 camera mode
#define IMX519_MODE_1920X1080_60FPS 0

// Supported frame rates for this mode
static const int imx519_60fps[] = {
    60,
};


// Describe the image format to NVIDIA's camera framework
static const struct camera_common_frmfmt imx519_frmfmt[] = {
    {
        {1920, 1080},
        imx519_60fps,
        ARRAY_SIZE(imx519_60fps),
        0,
        IMX519_MODE_1920X1080_60FPS
    },
};


// List of camera controls supported by our IMX519 driver
static const u32 imx519_ctrl_cid_list[] = {
    TEGRA_CAMERA_CID_GAIN,
    TEGRA_CAMERA_CID_EXPOSURE,
    TEGRA_CAMERA_CID_FRAME_RATE,
    TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};


/*
1.) Function receives a pointer to the sensor struct
2.) Loops through the 3 supplies; for each iteration, copies a supply-name pointer to the corresponding regulator_bulk_data struct
*/
static void imx519_init_supplies(struct imx519 *imx519) {
    int i;

    for (i = 0; i < IMX519_NUM_SUPPLIES; i++) {
        imx519->supplies[i].supply = imx519_supply_names[i];
    }
}


// Requesting regulators from Linux
static int imx519_get_supplies(struct device *dev, struct imx519 *imx519) {
    imx519_init_supplies(imx519);

    // Asks Linux's regulator subsystem to get these resources
    return devm_regulator_bulk_get(
        dev,
        IMX519_NUM_SUPPLIES,
        imx519->supplies
    );
}


static int imx519_enable_supplies(struct imx519 *imx519) {
    int ret;

    ret = regulator_bulk_enable(
        IMX519_NUM_SUPPLIES,
        imx519->supplies
    );

    return ret;
}


static void imx519_disable_supplies(struct imx519 *imx519) {
    regulator_bulk_disable(
        IMX519_NUM_SUPPLIES,
        imx519->supplies
    );
}


static int imx519_get_clock(struct device *dev, struct imx519 *imx519) {
    imx519->xclk = devm_clk_get(dev, "mclk");
    if (IS_ERR(imx519->xclk)) return PTR_ERR(imx519->xclk);

    return 0;
}


static int imx519_enable_clock(struct imx519 *imx519) {
    int ret;
    unsigned long rate;

    ret = clk_set_rate(imx519->xclk, IMX519_XCLK_FREQ);
    if (ret) return ret;
    rate = clk_get_rate(imx519->xclk);
    if (rate != IMX519_XCLK_FREQ) return -EINVAL;

    return clk_prepare_enable(imx519->xclk);
}


static void imx519_disable_clock(struct imx519 *imx519) {
    clk_disable_unprepare(imx519->xclk);
}


static int imx519_get_reset_gpio(struct device *dev, struct imx519 *imx519) {
    // Request the GPIO and initially hold XCLR low
    imx519->reset_gpio = devm_gpiod_get_optional(
        dev, "reset", GPIOD_OUT_LOW
    );

    // Check whether Linux returned an error pointer
    if (IS_ERR(imx519->reset_gpio)) return PTR_ERR(imx519->reset_gpio);

    return 0;
}


// Read one byte from a sensor register
static int imx519_read_reg8(struct i2c_client *client, u16 reg, u8 *value) {
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

    if (ret != 2) return ret < 0 ? ret : -EIO;

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

    if (ret != sizeof(data)) return ret < 0 ? ret : -EIO;

    return 0;
}

/* 
i2c_client *client  Pointer to a Linux structure representing the camera on I2C
u16 *chip_id Pointer to a 16-bit unsigned integer to store the sensor's ID
 */
static int imx519_read_id(struct i2c_client *client, u16 *chip_id ) {
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
    if (ret != 2) return ret < 0 ? ret : -EIO;
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
static int imx519_power_on(struct imx519 *imx519) {
    int ret;

    // Configure and enable the external clock
    ret = imx519_enable_clock(imx519);
    if (ret) return ret;

    // Release XCLR high (candidate active-high mapping)
    if (imx519->reset_gpio) gpiod_set_value_cansleep(imx519->reset_gpio, 1);

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
static void imx519_power_off(struct imx519 *imx519) {
    // Hold XCLR low
    if (imx519->reset_gpio) gpiod_set_value_cansleep(imx519->reset_gpio, 0);

    // Disable the external clock
    imx519_disable_clock(imx519);
}


// Read the camera's hardware configuration from the device tree
static struct camera_common_pdata *imx519_parse_dt(struct tegracam_device *tc_dev) {
    struct device *dev = tc_dev->dev;
    struct device_node *np = dev->of_node;
    struct camera_common_pdata *pdata;
    int gpio;

    if (!np) return ERR_PTR(-EINVAL);

    // Allocate memory for camera platform configuration
    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata) return ERR_PTR(-ENOMEM);

    // Read the board enable GPIO from our current overlay.
    // NVIDIA calls this reset_gpio, but our board uses POWER_EN.
    gpio = of_get_named_gpio(np, "reset-gpios", 0);
    if (gpio < 0) return ERR_PTR(gpio);

    pdata->reset_gpio = gpio;

    // Match the clock name defined in our device tree
    pdata->mclk_name = "mclk";

    return pdata;
}


// Read one byte from a sensor register using NVIDIA's camera framework
static int imx519_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val) {
    unsigned int value;
    int ret;

    ret = regmap_read(s_data->regmap, addr, &value);
    if (ret) return ret;
    *val = value & 0xff;
    return 0;
}


// Write one byte to a sensor register using NVIDIA's camera framework
static int imx519_write_reg(struct camera_common_data *s_data, u16 addr, u8 val) {
    int ret;

    ret = regmap_write(s_data->regmap, addr, val);
    if (ret)
        dev_err(s_data->dev, 
                "Failed to write register 0x%04x: %d\n",
                addr, ret);

    return ret;
}


// Write a 16-bit value to two consecutive IMX519 registers
static int imx519_write_reg16(struct camera_common_data *s_data, u16 addr, u16 value) {
    int ret;

    // Write the upper 8 bits to the first register
    ret = imx519_write_reg(s_data, addr, value >> 8);
    if (ret) return ret;

    // Write the lower 8 bits to the next register
    return imx519_write_reg(s_data, addr + 1, value & 0xff);
}


// Set the IMX519's analog gain
static int imx519_set_gain(struct tegracam_device *tc_dev, s64 val) {
    u16 gain_reg;

    // Check that the requested gain is between 1x and 16x
    if (val < IMX519_MIN_GAIN || val > IMX519_MAX_GAIN) return -EINVAL;

    // Convert NVIDIA's scaled gain into an IMX519 register value
    gain_reg = 1024 - (1024 * IMX519_GAIN_FACTOR) / val;

    // Write the 16-bit gain value to the sensor
    return imx519_write_reg16(
        tc_dev->s_data,
        IMX519_REG_ANALOG_GAIN,
        gain_reg
    );
}


// Set the IMX519 exposure time
static int imx519_set_exposure(struct tegracam_device *tc_dev, s64 val) {
    u64 exposure_lines;
    u64 max_exposure = IMX519_FRAME_LENGTH - IMX519_EXPOSURE_OFFSET;

    // Exposure time must be positive
    if (val <= 0)
        return -EINVAL;

    // Convert exposure time from microseconds into sensor lines
    exposure_lines = div64_u64(
        (u64)val * IMX519_PIXEL_RATE,
        IMX519_LINE_LENGTH * 1000000ULL
    );

    // Limit exposure to the current 60 FPS frame length
    exposure_lines = clamp_val(exposure_lines, 1, max_exposure);

    // Log the requested exposure and calculated integration time
    dev_info(tc_dev->s_data->dev,
            "IMX519 exposure callback: %lld us -> %llu lines\n",
            (long long)val,
            (unsigned long long)exposure_lines);

    // Write the 16-bit exposure value to the sensor
    return imx519_write_reg16(
        tc_dev->s_data,
        IMX519_REG_EXPOSURE,
        (u16)exposure_lines
    );
}


// Configure the IMX519 frame rate
static int imx519_set_frame_rate(struct tegracam_device *tc_dev, s64 val) {
    // NVIDIA represents FPS using a scaling factor of 1,000,000
    // For our first mode, only 60 FPS is supported
    if (val != 60000000LL) return -EINVAL;

    // The 1920x1080 mode table already sets the frame length
    return 0;
}


// Temporary group-hold handler required by NVIDIA's camera framework
static int imx519_set_group_hold(struct tegracam_device *tc_dev, bool val) {
    // Group hold is not implemented during initial bring-up
    (void)tc_dev;
    (void)val;

    return 0;
}


// Connect our IMX519 controls to NVIDIA's camera framework
static struct tegracam_ctrl_ops imx519_ctrl_ops = {

    // Number of supported controls
    .numctrls = ARRAY_SIZE(imx519_ctrl_cid_list),

    // List of supported control IDs
    .ctrl_cid_list = imx519_ctrl_cid_list,

    // Camera control functions
    .set_gain = imx519_set_gain,
    .set_exposure = imx519_set_exposure,
    .set_frame_rate = imx519_set_frame_rate,
    .set_group_hold = imx519_set_group_hold,
};


// Write an entire register table to configure the IMX519
static int imx519_write_table(struct camera_common_data *s_data,
                              const struct imx519_reg *table,
                              size_t num_regs)
{
    size_t i;
    int ret;

    // Loop through every register in the table
    for (i = 0; i < num_regs; i++) {

        // Write the register address and its value
        ret = regmap_write(
            s_data->regmap,
            table[i].address,
            table[i].value
        );

        // Stop if any register write fails
        if (ret) {
            dev_err(s_data->dev,
                    "Failed to write register 0x%04x: %d\n",
                    table[i].address, ret);

            return ret;
        }
    }

    return 0;
}



// Configure the IMX519 for the selected camera mode
static int imx519_set_mode(struct tegracam_device *tc_dev) {
    struct camera_common_data *s_data = tc_dev->s_data;
    u8 lanes;
    u8 frame_high, frame_low;
    u8 line_high, line_low;
    int ret;

    // Currently, only supports the 1920x1080 mode
    if (s_data->mode != IMX519_MODE_1920X1080_60FPS) return -EINVAL;

    dev_info(s_data->dev, "IMX519 configuring 1920x1080 mode\n");

    // Apply the IMX519 common initialization registers
    ret = imx519_write_table(
        s_data,
        mode_common_regs,
        ARRAY_SIZE(mode_common_regs)
    );

    if (ret) {
        dev_err(s_data->dev, "Failed to write IMX519 common registers: %d\n", ret);
        return ret;
    }
    dev_info(s_data->dev, "IMX519 common initialization complete\n");

    // Write the register configuration to the sensor
    ret = imx519_write_table(
        s_data,
        mode_1920x1080_regs,
        ARRAY_SIZE(mode_1920x1080_regs)
    );

    if (ret) return ret;
    dev_info(s_data->dev, "IMX519 mode table written successfully\n");

    // Read back the CSI lane configuration
    ret = imx519_read_reg(s_data, 0x0114, &lanes);
    if (ret) return ret;

    // Read back the frame length
    ret = imx519_read_reg(s_data, 0x0340, &frame_high);
    if (ret) return ret;
    ret = imx519_read_reg(s_data, 0x0341, &frame_low);
    if (ret) return ret;

    // Read back the line length
    ret = imx519_read_reg(s_data, 0x0342, &line_high);
    if (ret) return ret;
    ret = imx519_read_reg(s_data, 0x0343, &line_low);
    if (ret) return ret;

    dev_info(s_data->dev, "CSI lane register: 0x%02x\n", lanes);
    dev_info(s_data->dev, "Frame length: %u\n", ((u16)frame_high << 8) | frame_low);
    dev_info(s_data->dev, "Line length: %u\n", ((u16)line_high << 8) | line_low);

    return 0;
}


#define IMX519_REG_MODE_SELECT 0x0100

// Tell the IMX519 to begin transmitting image data

// Tell the IMX519 to begin transmitting image data
static int imx519_start_streaming(struct tegracam_device *tc_dev) {
    struct camera_common_data *s_data = tc_dev->s_data;
    u8 value;
    u8 exposure_high, exposure_low;
    int ret;

    dev_info(s_data->dev, "IMX519 starting streaming\n");

    // Set the mode select register to streaming
    ret = imx519_write_reg(
        s_data,
        IMX519_REG_MODE_SELECT,
        0x01
    );

    if (ret) {
        dev_err(s_data->dev, "Failed to start streaming: %d\n", ret);
        return ret;
    }

    // Read the register back to verify the write
    ret = imx519_read_reg(
        s_data,
        IMX519_REG_MODE_SELECT,
        &value
    );

    if (ret) return ret;
    dev_info(s_data->dev,
             "IMX519 mode select: 0x%02x\n", value);

    // Confirm the sensor accepted the streaming command
    if (value != 0x01)return -EIO;

    // Read sesnors actual exposure register
    ret = imx519_read_reg(s_data, 0x0202, &exposure_high);
    if (ret) return ret;
    ret = imx519_read_reg(s_data, 0x0203, &exposure_low);
    if (ret) return ret;

    // Display the exposure currently programmed in sensor
    dev_info(s_data->dev, "IMX519 actual exposure: %u lines\n", ((u16)exposure_high) << 8 | exposure_low);

    return 0;
}


// Tell the IMX519 to stop transmitting image data
static int imx519_stop_streaming(struct tegracam_device *tc_dev) {
    struct camera_common_data *s_data = tc_dev->s_data;

    // Set the mode select register to standby
    return imx519_write_reg(
        s_data,
        IMX519_REG_MODE_SELECT,
        0x00
    );
}


// Enable the IMX519 camera through NVIDIA's camera framework
static int imx519_v4l2_power_on(struct camera_common_data *s_data) {
    struct camera_common_power_rail *pw = s_data->power;

    if (!pw) return -EINVAL;

    // Verify the camera board's POWER_EN GPIO
    if (!gpio_is_valid(pw->reset_gpio)) return -EINVAL;

    // Avoid powering on an already enabled camera
    if (pw->state == SWITCH_ON) return 0;

    // Enable the camera board's onboard voltage regulators
    gpio_set_value_cansleep(pw->reset_gpio, 1);

    // Allow the camera to initialize
    usleep_range(
        IMX519_XCLR_MIN_DELAY_US,
        IMX519_XCLR_MIN_DELAY_US + IMX519_XCLR_DELAY_RANGE_US
    );

    pw->state = SWITCH_ON;

    return 0;
}


// Disable the IMX519 camera through NVIDIA's camera framework
static int imx519_v4l2_power_off(struct camera_common_data *s_data) {
    struct camera_common_power_rail *pw = s_data->power;

    if (!pw) return -EINVAL;
    if (!gpio_is_valid(pw->reset_gpio)) return -EINVAL;

    // Avoid powering off an already disabled camera
    if (pw->state == SWITCH_OFF) return 0;

    // Disable the camera board's onboard voltage regulators
    gpio_set_value_cansleep(pw->reset_gpio, 0);

    pw->state = SWITCH_OFF;

    return 0;
}


// Acquire hardware resources needed by camera
static int imx519_v4l2_power_get(struct tegracam_device *tc_dev) {
    struct device *dev = tc_dev->dev;
    struct camera_common_data *s_data = tc_dev->s_data;
    struct camera_common_power_rail *pw = s_data->power;
    struct camera_common_pdata *pdata = s_data->pdata;
    int ret;

    if (!pw || !pdata) return -EINVAL;
    // Acquire the clock from our device tree

    pw->mclk = devm_clk_get(dev, "mclk");
    if (IS_ERR(pw->mclk)) return PTR_ERR(pw->mclk);

    // Get the camera board's POWER_EN GPIO
    pw->reset_gpio = pdata->reset_gpio;
    if (!gpio_is_valid(pw->reset_gpio)) return -EINVAL;

    // Request the GPIO and initially disable camera power
    ret = gpio_request_one(
        pw->reset_gpio,
        GPIOF_OUT_INIT_LOW,
        "imx519_power_en"
    );

    if (ret) return ret;

    pw->state = SWITCH_OFF;

    return 0;
}


// Release the camera's hardware resources
static int imx519_v4l2_power_put(struct tegracam_device *tc_dev) {
    struct camera_common_power_rail *pw = tc_dev->s_data->power;

    if (!pw) return -EINVAL;

    // Release the POWER_EN GPIO
    gpio_free(pw->reset_gpio);

    // The clock was acquired with devm_clk_get,
    // so Linux releases it automatically when the device is removed

    return 0;
}


// Connect the IMX519 functions to NVIDIA's camera framework
static struct camera_common_sensor_ops imx519_common_ops = {

    // Supported camera resolutions and frame rates
    .numfrmfmts = ARRAY_SIZE(imx519_frmfmt),
    .frmfmt_table = imx519_frmfmt,

    // Camera power management
    .power_on = imx519_v4l2_power_on,
    .power_off = imx519_v4l2_power_off,
    .power_get = imx519_v4l2_power_get,
    .power_put = imx519_v4l2_power_put,

    // Sensor register communication
    .read_reg = imx519_read_reg,
    .write_reg = imx519_write_reg,

    // Device-tree configuration
    .parse_dt = imx519_parse_dt,

    // Camera operating modes
    .set_mode = imx519_set_mode,

    // Video streaming
    .start_streaming = imx519_start_streaming,
    .stop_streaming = imx519_stop_streaming,
};


// Initialize the camera and verify the IMX519 sensor
static int imx519_board_setup(struct tegracam_device *tc_dev) {
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;

    u8 id_high;
    u8 id_low;
    u16 chip_id;

    int ret;
    int power_ret;

    // Enable the camera clock before powering the sensor
    ret = camera_common_mclk_enable(s_data);
    if (ret) {
        dev_err(dev, "Failed to enable camera clock: %d\n", ret);
        return ret;
    }

    // Enable the camera board's power
    ret = imx519_v4l2_power_on(s_data);
    if (ret) {
        dev_err(dev, "Failed to power on IMX519: %d\n", ret);
        goto clock_off;
    }

    // Read the upper byte of the chip ID
    ret = imx519_read_reg(s_data, 0x0016, &id_high);
    if (ret) goto power_off;

    // Read the lower byte of the chip ID
    ret = imx519_read_reg(s_data, 0x0017, &id_low);
    if (ret) goto power_off;

    // Combine the two bytes into the 16-bit chip ID
    chip_id = ((u16)id_high << 8) | id_low;

    // Verify that the connected sensor is an IMX519
    if (chip_id != IMX519_CHIP_ID) {
        dev_err(dev, "Unexpected chip ID: 0x%04x\n", chip_id);
        ret = -ENODEV;
        goto power_off;
    }

    dev_info(dev, "IMX519 board setup successful: 0x%04x\n", chip_id);
    power_off:

    // Disable sensor power before disabling the clock
    power_ret = imx519_v4l2_power_off(s_data);

    if (!ret) ret = power_ret;

    clock_off:
    // Release the clock enabled during board setup
    camera_common_mclk_disable(s_data);

    return ret;
}


// Called when the V4L2 subdevice is opened
static int imx519_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh) { return 0; }

// Register the IMX519 V4L2 subdevice operations
static const struct v4l2_subdev_internal_ops imx519_subdev_internal_ops = {
    .open = imx519_open,
};


// Register the IMX519 with NVIDIA's camera framework
static int imx519_probe(struct i2c_client *client) {
    struct device *dev = &client->dev;
    struct tegracam_device *tc_dev;
    struct imx519 *imx519;
    int ret;

    dev_info(dev, "IMX519 V4L2 probe started\n");

    // Verify that the device was described by the device tree
    if (!IS_ENABLED(CONFIG_OF) || !dev->of_node) return -EINVAL;

    // Allocate memory for our IMX519 private data
    imx519 = devm_kzalloc(dev, sizeof(*imx519), GFP_KERNEL);
    if (!imx519) return -ENOMEM;

    // Allocate NVIDIA's camera framework structure
    tc_dev = devm_kzalloc(dev, sizeof(*tc_dev), GFP_KERNEL);
    if (!tc_dev) return -ENOMEM;

    // Associate the NVIDIA camera device with this I2C sensor
    tc_dev->client = client;
    tc_dev->dev = dev;

    // Register the sensor name
    strscpy(tc_dev->name, "imx519", sizeof(tc_dev->name));

    // Provide the sensor's register-map configuration
    tc_dev->dev_regmap_config = &imx519_regmap_config;

    // Connect our sensor and camera control callbacks
    tc_dev->sensor_ops = &imx519_common_ops;
    tc_dev->tcctrl_ops = &imx519_ctrl_ops;
    tc_dev->v4l2sd_internal_ops = &imx519_subdev_internal_ops;

    // Register our sensor with NVIDIA's camera framework
    ret = tegracam_device_register(tc_dev);
    if (ret) {
        dev_err(dev, "tegracam registration failed: %d\n", ret);
        return ret;
    }

    // Save the framework device in our private structure
    imx519->tc_dev = tc_dev;

    // Associate our private data with NVIDIA's camera device
    tegracam_set_privdata(tc_dev, imx519);

    // Power up the sensor temporarily and verify its chip ID
    ret = imx519_board_setup(tc_dev);
    if (ret) {
        dev_err(dev, "IMX519 board setup failed: %d\n", ret);
        goto unregister_camera;
    }

    // Register the sensor as a V4L2 subdevice
    ret = tegracam_v4l2subdev_register(tc_dev, true);
    if (ret) {
        dev_err(dev, "V4L2 subdevice registration failed: %d\n", ret);
        goto unregister_camera;
    }

    dev_info(dev, "IMX519 V4L2 registration successful\n");

    return 0;

    unregister_camera:
    // Release resources if initialization fails
    tegracam_device_unregister(tc_dev);

    return ret;
}


// Unregister the IMX519 when the driver is removed
static void imx519_remove(struct i2c_client *client) {
    struct camera_common_data *s_data;
    struct imx519 *imx519;

    // Retrieve the camera data registered during probe
    s_data = to_camera_common_data(&client->dev);
    if (!s_data) return;

    imx519 = s_data->priv;
    if (!imx519 || !imx519->tc_dev) return;

    // Unregister the V4L2 subdevice first
    tegracam_v4l2subdev_unregister(imx519->tc_dev);

    // Release NVIDIA camera framework resources
    tegracam_device_unregister(imx519->tc_dev);
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
    .remove = imx519_remove,
};

module_i2c_driver(imx519_driver);

MODULE_DESCRIPTION("Experimental IMX519 identification driver");
MODULE_AUTHOR("IMX519 Jetson Port Project");
MODULE_LICENSE("GPL");