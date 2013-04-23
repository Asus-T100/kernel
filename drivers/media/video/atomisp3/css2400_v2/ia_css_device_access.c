#include "ia_css_device_access.h"
#include "device_access.h"
#include "assert_support.h"

static struct ia_css_hw_access_env my_env;

void
ia_css_device_access_init(const struct ia_css_hw_access_env *env)
{
	my_env = *env;
}

uint8_t
device_load_uint8(const hrt_address addr)
{
	return my_env.load_8(addr);
}

uint16_t
device_load_uint16(const hrt_address addr)
{
	return my_env.load_16(addr);
}

uint32_t
device_load_uint32(const hrt_address addr)
{
	return my_env.load_32(addr);
}

uint64_t
device_load_uint64(const hrt_address addr)
{
	(void)addr;
	assert(0);
	return 0;
}

void
device_store_uint8(const hrt_address addr, const uint8_t data)
{
	my_env.store_8(addr, data);
}

void
device_store_uint16(const hrt_address addr, const uint16_t data)
{
	my_env.store_16(addr, data);
}

void
device_store_uint32(const hrt_address addr, const uint32_t data)
{
	my_env.store_32(addr, data);
}

void
device_store_uint64(const hrt_address addr, const uint64_t data)
{
	(void)addr;
	(void)data;
	assert(0);
}

void
device_load(const hrt_address addr, void *data, const size_t size)
{
	my_env.load(addr, data, size);
}

void
device_store(const hrt_address addr, const void *data, const size_t size)
{
	my_env.store(addr, data, size);
}
