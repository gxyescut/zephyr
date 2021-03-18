#include <zephyr/types.h>
#include <sys/util.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*fpga_api_get_status)(const struct device *dev);
typedef int (*fpga_api_load)(const struct device *dev, uint32_t *image_ptr,
			     uint32_t img_size);
typedef int (*fpga_api_reset)(const struct device *dev);
typedef int (*fpga_api_on)(const struct device *dev);
typedef int (*fpga_api_off)(const struct device *dev);
typedef const char *(*fpga_api_get_info)(const struct device *dev);

__subsystem struct fpga_driver_api {
	fpga_api_get_status get_status;
	fpga_api_reset reset;
	fpga_api_load load;
	fpga_api_on on;
	fpga_api_off off;
	fpga_api_get_info get_info;
};

static inline uint32_t fpga_get_status(const struct device *dev)
{
	const struct fpga_driver_api *api =
		(const struct fpga_driver_api *)dev->api;

	return api->get_status(dev);
}

static inline int fpga_reset(const struct device *dev)
{
	const struct fpga_driver_api *api =
		(const struct fpga_driver_api *)dev->api;

	return api->reset(dev);
}

static inline int fpga_load(const struct device *dev, uint32_t *image_ptr,
			    uint32_t img_size)
{
	const struct fpga_driver_api *api =
		(const struct fpga_driver_api *)dev->api;

	return api->load(dev, image_ptr, img_size);
}

static inline int fpga_on(const struct device *dev)
{
	const struct fpga_driver_api *api =
		(const struct fpga_driver_api *)dev->api;

	return api->on(dev);
}

static inline const char *fpga_get_info(const struct device *dev)
{
	const struct fpga_driver_api *api =
		(const struct fpga_driver_api *)dev->api;

	return api->get_info(dev);
}

static inline int fpga_off(const struct device *dev)
{
	const struct fpga_driver_api *api =
		(const struct fpga_driver_api *)dev->api;

	return api->off(dev);
}

#ifdef __cplusplus
}
#endif
