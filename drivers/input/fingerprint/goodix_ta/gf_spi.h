/*
 * driver definition for sensor driver
 *
 * Coypright (c) 2017 Goodix
 */
#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
/**********************************************************/
enum FP_MODE {
	GF_IMAGE_MODE = 0,
	GF_KEY_MODE,
	GF_SLEEP_MODE,
	GF_FF_MODE,
	GF_DEBUG_MODE = 0x56
};

struct gf_ioc_chip_info {
	unsigned char vendor_id;
	unsigned char mode;
	unsigned char operation;
	unsigned char reserved[5];
};

#define GF_IOC_MAGIC	'g'			/*define magic number*/
#define GF_IOC_INIT					_IOR(GF_IOC_MAGIC, 0, uint8_t)
#define GF_IOC_EXIT					_IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET				_IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ			_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ			_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK		_IOW(GF_IOC_MAGIC, 5, uint32_t)
#define GF_IOC_DISABLE_SPI_CLK		_IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER			_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER		_IO(GF_IOC_MAGIC, 8)
#define GF_IOC_ENTER_SLEEP_MODE		_IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO			_IOR(GF_IOC_MAGIC, 11, uint8_t)
#define GF_IOC_REMOVE				_IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO			_IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)

#define USE_PLATFORM_BUS		1
#define GF_PW_CTL 1
#define GF_NETLINK_ENABLE 1
#define GF_NET_EVENT_IRQ 1
#define GF_NET_EVENT_FB_BLACK 2
#define GF_NET_EVENT_FB_UNBLACK 3
#define NETLINK_TEST 25

struct gf_dev {
	dev_t devt;
	struct list_head device_entry;
	struct platform_device *spi;
	struct clk *core_clk;
	struct clk *iface_clk;

	struct input_dev *input;
	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	signed pwr_gpio;
	int irq;
	int irq_enabled;
	int clk_enabled;
	struct notifier_block notifier;
	char device_available;
	char fb_black;
	char wait_finger_down;
	struct work_struct work;
};

int gf_parse_dts(struct gf_dev *gf_dev);
void gf_cleanup(struct gf_dev *gf_dev);

int gf_power_on(struct gf_dev *gf_dev);
int gf_power_off(struct gf_dev *gf_dev);

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms);
int gf_irq_num(struct gf_dev *gf_dev);

void sendnlmsg(char *message);
int netlink_init(void);
void netlink_exit(void);
#endif /*__GF_SPI_H*/
