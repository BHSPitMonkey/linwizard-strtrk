#ifndef _SPI100K_H
#define _SPI100K_H

struct omap_spi100k_platform_config {
	unsigned short	num_cs;
};

struct omap_spi100k_device_config {
	unsigned turbo_mode:1;

	/* Do we want one channel enabled at the same time? */
	unsigned single_channel:1;
};
#endif
