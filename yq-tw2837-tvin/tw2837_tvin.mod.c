#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

MODULE_INFO(intree, "Y");

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x75193b8b, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0xb3298fb0, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x9ff63823, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0xd9860367, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x809c16a1, __VMLINUX_SYMBOL_STR(v4l2_int_device_unregister) },
	{ 0xbb35d2b1, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x86ee9077, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xbccc4474, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0x2e03f339, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x8851fd43, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0xf8401dac, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0x970c2e89, __VMLINUX_SYMBOL_STR(cdev_alloc) },
	{ 0x2c34f2ef, __VMLINUX_SYMBOL_STR(kmem_cache_alloc) },
	{ 0xf7e4c997, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
	{ 0x815588a6, __VMLINUX_SYMBOL_STR(clk_enable) },
	{ 0xb077e70a, __VMLINUX_SYMBOL_STR(clk_unprepare) },
	{ 0xb6e6d99d, __VMLINUX_SYMBOL_STR(clk_disable) },
	{ 0x834b2fa1, __VMLINUX_SYMBOL_STR(v4l2_int_device_register) },
	{ 0x7c9a7371, __VMLINUX_SYMBOL_STR(clk_prepare) },
	{ 0x7d43ffac, __VMLINUX_SYMBOL_STR(of_property_read_u32_array) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x5d2f2370, __VMLINUX_SYMBOL_STR(gpiod_set_raw_value) },
	{ 0xab2fec6c, __VMLINUX_SYMBOL_STR(gpiod_direction_output_raw) },
	{ 0x1a2a0b81, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x47229b5c, __VMLINUX_SYMBOL_STR(gpio_request) },
	{ 0x6dfe2a9d, __VMLINUX_SYMBOL_STR(of_get_named_gpio_flags) },
	{ 0x60e51c2f, __VMLINUX_SYMBOL_STR(of_count_phandle_with_args) },
	{ 0x3bea5325, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xfbc74f64, __VMLINUX_SYMBOL_STR(__copy_from_user) },
	{ 0x8aa5d8f5, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0xdda4a1b6, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x470b0a4c, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0xfe990052, __VMLINUX_SYMBOL_STR(gpio_free) },
	{ 0xdaccb431, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xfa2a45e, __VMLINUX_SYMBOL_STR(__memzero) },
	{ 0x9d669763, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=v4l2-int-device";


MODULE_INFO(srcversion, "B198FFED88EE1947A8CC4FE");
