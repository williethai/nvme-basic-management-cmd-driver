#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#define CMD_CODE_0_7_BYTE 	0
#define CMD_CODE_8_31_BYTE	8

#define LEN_0_7_BYTE	8
#define LEN_8_31_BYTE	24

static const uint8_t CRC_8_TABLE[256] = 
{
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B,
	0x12, 0x15, 0x38, 0x3F, 0x36, 0x31,
	0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77,
	0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53,
	0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9,
	0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF,
	0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B,
	0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1,
	0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0,
	0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4,
	0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE,
	0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88,
	0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C,
	0x35, 0x32, 0x1F, 0x18, 0x11, 0x16,
	0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50,
	0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74,
	0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80,
	0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6,
	0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2,
	0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8,
	0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E,
	0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A,
	0x43, 0x44, 0x19, 0x1E, 0x17, 0x10,
	0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26,
	0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55,
	0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F,
	0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39,
	0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D,
	0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7,
	0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91,
	0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5,
	0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF,
	0xFA, 0xFD, 0xF4, 0xF3
};

struct subsystem_mng_data
{
	u8 len_of_ident;
	u8 resv0:2;
	u8 port1_pcie_link_active:1;
	u8 port0_pcie_link_active:1;
	u8 reset_not_required:1;
	u8 drive_functional:1;
	u8 drive_not_ready:1;
	u8 smbus_arbitration:1;
	u8 smart_warning;
	u8 c_temp;
	u8 percent_drive_life_used;
	u8 resv1[2];
	u8 pec0;
	u8 len_of_identification;
	u8 vendor_id[2];
	u8 ser_num[20];
	u8 pec1;

}__attribute__((packed));

struct my_attr {
    	struct attribute attr;
	struct i2c_client       *client;
    	int value;
	int code;
};

struct nvme_basic_mng_data{
	struct i2c_client       *client;
	struct subsystem_mng_data mng_data;
	u8 slave_w_addr;
	u8 slave_r_addr;
};

static struct my_attr i2c_temp_nvme = {
    	.attr.name="temp_nvme",
    	.attr.mode = 0644,
    	.value = 0,
	.code = 0,
};

static struct my_attr i2c_device_id = {
    	.attr.name="device_id",
    	.attr.mode = 0644,
    	.value = 0,
	.code = 1,
};

static struct my_attr i2c_result = {
    	.attr.name="result",
    	.attr.mode = 0644,
    	.value = 0,
	.code = 2,
};

static struct my_attr i2c_echo = {
        .attr.name="test",
        .attr.mode = 0644,
        .value = 0,
        .code = 3,
};

static struct attribute * myattr[] = {
    	&i2c_temp_nvme.attr,
	&i2c_device_id.attr,
	&i2c_result.attr,
	&i2c_echo.attr,
    	NULL
};

static int nvme_i2c_write(struct i2c_client *client, u32 off, u16 len, u8 *buf)
{
    	int ret, i;
	u8 off_tmp[4];
	u8 _off[4];
	u8 buf_tmp[32] = {0};

	memcpy(off_tmp, &off, 4);
	
	for(i=0;i<=3;i++)_off[i] = off_tmp[3-i];
	memcpy(buf_tmp, _off, 4);
	memcpy(&buf_tmp[4], buf, len);
    	struct i2c_msg msg[1] = {
        	{
            		.addr = client->addr,
            		.len = 4 + len,
            		.buf = buf_tmp,
        	},
    	};

    	ret = i2c_transfer(client->adapter, msg, 1);
    	if (ret < 0) {
        	dev_err(&client->dev, "I2C write failed\n");
        	return ret;
    	}

    	return 0;
}

static int basic_mng_i2c_read(struct i2c_client *client, u8 cmd_code, u16 len, u8 *buf)
{
    	int ret, i;
    	
	struct i2c_msg msg[2] = {
        	{
            	.addr = client->addr,
            	.len = 1,
            	.buf = &cmd_code,
        	},
        	{
            	.addr = client->addr,
            	.flags = I2C_M_RD,
            	.len = len,
            	.buf = buf,
        	}
	};

    	ret = i2c_transfer(client->adapter, msg, 2);
    	if (ret < 0) {
        	dev_err(&client->dev, "I2C read failed\n");
        	return ret;
    	}

    	return 0;
}

static int basic_mng_write(struct i2c_client *client, struct mrpc_regs * _mrpc_regs)
{
	u8 *buf;
	u8 cmd[4] = {0};
	u8 cmd_tmp[4] = {0};

	int res, i;
	

	return 0;
} 

static int basic_mng_read(struct i2c_client *client, u8 cmd_code, u16 len, u8* out_data)
{
        u8 *buf;
        u8 tmp[4] = {0};
        int res, i;

	
	return 0;
}

static ssize_t default_show(struct kobject *kobj, struct attribute *attr,
        char *buf)
{
    	struct my_attr *a = container_of(attr, struct my_attr, attr);
	//char offset[4] = {0x00, 0x20, 0x00, 0x00};
	char *ret;
	unsigned char reg[4] = {0};
	int res = 0;
	int i = 0;
	int time_out = 0;
	u8 out_data[32];
	/*
	if(a->code == 3)
	{
		basic_mng_i2c_read(a->client, CMD_CODE_0_7_BYTE, LEN_0_7_BYTE, out_data);
		for(i=0; i<LEN_0_7_BYTE; i++)printk("out_data: %x\n", out_data[i]);	
		basic_mng_i2c_read(a->client, CMD_CODE_8_31_BYTE, LEN_8_31_BYTE, out_data);
                for(i=0; i<LEN_8_31_BYTE; i++)printk("out_data: %x\n", out_data[i]);

		memcpy((u8*)a->mng_data, out_data, 32);
		printk("temp: %d pec: %d\n", a->mng_data.ctemp, a->mng_data.pec2);
	}*/

        return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t default_store(struct kobject *kobj, struct attribute *attr,
        const char *buf, size_t len)
{
    	struct my_attr *a = container_of(attr, struct my_attr, attr);
    	long value;
	int res = 0;

    	return sizeof(int);
}

/*-----------------------------------------------------------------------*/
static int nvme_temp_read(struct device *dev, enum hwmon_sensor_types type,
                     u32 attr, int channel, long *val)
{
	struct nvme_basic_mng_data *data = dev_get_drvdata(dev);
	int res, ret, time_out;
	u8 out_data[32], out_len, pec;
	int i = 0;

	switch (type) {
                case hwmon_temp:
                        switch (attr) {
                                case hwmon_temp_input:
						ret = basic_mng_i2c_read(data->client, CMD_CODE_0_7_BYTE, LEN_0_7_BYTE, out_data);
						if(ret == 0)
						{
							out_len = out_data[0];
							pec = 0;
							pec = CRC_8_TABLE[pec ^ data->slave_w_addr];
							pec = CRC_8_TABLE[pec ^ CMD_CODE_0_7_BYTE];
							pec = CRC_8_TABLE[pec ^ data->slave_r_addr];
							for(i=0; i<=out_len; i++)pec = CRC_8_TABLE[pec ^ out_data[i]];
							if(pec != out_data[out_len + 1])
								return -EINVAL;

						}
						else return -EINVAL;
						ret = basic_mng_i2c_read(data->client, CMD_CODE_8_31_BYTE, LEN_8_31_BYTE, &out_data[LEN_0_7_BYTE]);
						if(ret == 0)
						{
							out_len = out_data[LEN_0_7_BYTE];
							pec = 0;
							pec = CRC_8_TABLE[pec ^ data->slave_w_addr];
							pec = CRC_8_TABLE[pec ^ CMD_CODE_8_31_BYTE];
							pec = CRC_8_TABLE[pec ^ data->slave_r_addr];
							for(i=LEN_0_7_BYTE; i<=(LEN_0_7_BYTE + out_len); i++)pec = CRC_8_TABLE[pec ^ out_data[i]];
							if(pec != out_data[LEN_0_7_BYTE + out_len + 1])
								return -EINVAL;

						}
						else return -EINVAL;

						memcpy((u8*)&data->mng_data, out_data, 32);
						*val = data->mng_data.c_temp * 1000;
						return 0;
					break;
                        }
                        break;

                default: break;
        }
        return 0;
}

static int nvme_temp_write(struct device *dev, enum hwmon_sensor_types type,
                      u32 attr, int channel, long temp)
{
	return 0;
}

static umode_t nvme_temp_is_visible(const void *data, enum hwmon_sensor_types type,
                               u32 attr, int channel)
{

        switch (type) {
        case hwmon_temp:
                switch (attr) {
                case hwmon_temp_input:
                        return S_IRUGO;

                }
                break;

        default:
                break;
        }

        return 0;
}

/* nvme temp configuration */

static const u32 nvme_temp_chip_config[] = {
        HWMON_C_REGISTER_TZ,
        0
};

static const struct hwmon_channel_info nvme_temp_chip = {
        .type = hwmon_chip,
        .config = nvme_temp_chip_config,
};

static const u32 nvme_temp_config[] = {
        HWMON_T_INPUT,
        0
};

static const struct hwmon_channel_info nvme_temp = {
        .type = hwmon_temp,
        .config = nvme_temp_config,
};

static const struct hwmon_channel_info *nvme_temp_info[] = {
        &nvme_temp_chip,
        &nvme_temp,
        NULL
};

static const struct hwmon_ops nvme_temp_hwmon_ops = {
        .is_visible = nvme_temp_is_visible,
        .read = nvme_temp_read,
        .write = nvme_temp_write,
};

static const struct hwmon_chip_info nvme_temp_chip_info = {
        .ops = &nvme_temp_hwmon_ops,
        .info = nvme_temp_info,
};

static struct sysfs_ops myops = {
    .show = default_show,
    .store = default_store,
};

static struct kobj_type mytype = {
    .sysfs_ops = &myops,
    .default_attrs = myattr,
};

static int _module_init(struct i2c_client *client, const struct i2c_device_id *id)
{
    	int err = -1;
	
	struct device *dev = &client->dev;
        struct device *hwmon_dev;
        struct nvme_basic_mng_data *data;

	data = devm_kzalloc(dev, sizeof(struct nvme_basic_mng_data), GFP_KERNEL);
        if (!data)
                return -ENOMEM;	
	
	data->client = client;
	data->slave_w_addr = client->addr << 1;
	data->slave_r_addr = (client->addr << 1) | 1;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
                                                         data, &nvme_temp_chip_info,
                                                         NULL);
        if (IS_ERR(hwmon_dev))
                return PTR_ERR(hwmon_dev);

        dev_info(dev, "%s: sensor '%s'\n", dev_name(hwmon_dev), client->name);
    	return 0;
}

static int _module_exit(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id i2c_switchtec_gas_id[] = {
        { "nvme_bs_mng" },
        { }
};
MODULE_DEVICE_TABLE(i2c, i2c_switchtec_gas_id);

static struct i2c_driver i2c_switchtec_gas_driver = {
        .driver = {
                .name = "i2c_nvme_basic_mng",
        },
        .probe = _module_init,
        .remove = _module_exit,
        .id_table = i2c_switchtec_gas_id,
};

module_i2c_driver(i2c_switchtec_gas_driver);

MODULE_AUTHOR("WILLIE");
MODULE_DESCRIPTION("I2C driver to communicate with nvme ssd with mctp protocol");
MODULE_LICENSE("GPL");

