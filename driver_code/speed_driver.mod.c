#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xf3ebda75, "__platform_driver_register" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x122c3a7e, "_printk" },
	{ 0xcf825a00, "platform_driver_unregister" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0x3da4bb8, "devm_gpiod_get" },
	{ 0xdb7a9fc1, "gpiod_set_debounce" },
	{ 0x21ac4a94, "gpiod_to_irq" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xc3f920e3, "param_ops_long" },
	{ 0x47e64c59, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "A532625BDE6E8586777589D");
