#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
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

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xc1514a3b, "free_irq" },
	{ 0x57f427e2, "usb_alloc_urb" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x931f4cb0, "usb_free_urb" },
	{ 0x5a6efd55, "param_ops_uint" },
	{ 0x2ce213b2, "param_ops_ulong" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x67620c6, "pci_enable_device" },
	{ 0x5e515be6, "ktime_get_ts64" },
	{ 0xed197515, "usb_get_current_frame_number" },
	{ 0xe1688e24, "proc_create" },
	{ 0xf0090d06, "param_ops_ushort" },
	{ 0xcf17517d, "consume_skb" },
	{ 0x94a18028, "pci_iomap" },
	{ 0xb0e602eb, "memmove" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x6fdfc0b2, "i2c_bit_add_bus" },
	{ 0xd8baaaf4, "sysfs_add_file_to_group" },
	{ 0x1b3f97e5, "alloc_canfd_skb" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x805f3047, "usb_register_driver" },
	{ 0x355f4c49, "__pci_register_driver" },
	{ 0xedc03953, "iounmap" },
	{ 0x817e8e14, "param_array_ops" },
	{ 0xd4defbf9, "pci_request_regions" },
	{ 0x69acdf38, "memcpy" },
	{ 0x37a0cba, "kfree" },
	{ 0x7ab1f0cb, "pcpu_hot" },
	{ 0x7369f212, "seq_lseek" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0xe2964344, "__wake_up" },
	{ 0x10ed95a8, "pci_irq_vector" },
	{ 0x9adfe4ee, "param_ops_byte" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xa248be7, "open_candev" },
	{ 0xadc0e922, "__dev_get_by_name" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0xe0872c22, "pci_unregister_driver" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x1035c7c2, "__release_region" },
	{ 0xf13d9347, "netdev_err" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0x122c3a7e, "_printk" },
	{ 0x14f5c1f1, "usb_clear_halt" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x1000e51, "schedule" },
	{ 0xf5a069f6, "usb_bulk_msg" },
	{ 0xe1f59815, "usb_reset_device" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0x240be9bc, "alloc_can_skb" },
	{ 0x8769894e, "unregister_candev" },
	{ 0x6383b27c, "__x86_indirect_thunk_rdx" },
	{ 0x814e7be6, "usb_submit_urb" },
	{ 0x4985390f, "_dev_info" },
	{ 0x6080f4e2, "i2c_del_adapter" },
	{ 0xec3a8035, "can_change_mtu" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xceb19f3e, "_dev_err" },
	{ 0xa280f4a3, "init_net" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x55385e2e, "__x86_indirect_thunk_r14" },
	{ 0x3adc709e, "alloc_candev_mqs" },
	{ 0x1e6d26a8, "strstr" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x3641c37, "kfree_skb_reason" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x7475b515, "dma_alloc_attrs" },
	{ 0x670ecece, "__x86_indirect_thunk_rbx" },
	{ 0x2e5f3dce, "pci_read_config_word" },
	{ 0x5a921311, "strncmp" },
	{ 0x61410f19, "usb_control_msg" },
	{ 0x9166fada, "strncpy" },
	{ 0x4d94dd12, "free_netdev" },
	{ 0xe39c7e45, "usb_set_interface" },
	{ 0x1edb69d6, "ktime_get_raw_ts64" },
	{ 0xde80cd09, "ioremap" },
	{ 0x4785ec73, "class_unregister" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0x449ad0a7, "memcmp" },
	{ 0x67b2497e, "sysfs_remove_file_from_group" },
	{ 0xfef216eb, "_raw_spin_trylock" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x3cf0639e, "usb_deregister" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x71cf81bb, "pci_iounmap" },
	{ 0x538d25fd, "netif_tx_wake_queue" },
	{ 0x65929cae, "ns_to_timespec64" },
	{ 0xe9249b1a, "close_candev" },
	{ 0xdbdf6c92, "ioport_resource" },
	{ 0xfb578fc5, "memset" },
	{ 0x36d2243, "_dev_warn" },
	{ 0xa4d1849d, "pci_alloc_irq_vectors_affinity" },
	{ 0x6794def6, "pci_set_master" },
	{ 0xfc8dfd45, "param_ops_charp" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xb238baad, "dma_get_required_mask" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xacd7be29, "netif_rx" },
	{ 0xc787da5e, "can_bus_off" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x42b507fc, "dma_set_coherent_mask" },
	{ 0x6df5afa9, "pv_ops" },
	{ 0xc1d9b323, "seq_read" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0xc688b77e, "device_create_with_groups" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0xa9962270, "dma_free_attrs" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x8bf35ae8, "pci_release_regions" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x2f4a328e, "driver_for_each_device" },
	{ 0x66cca4f9, "__x86_indirect_thunk_rcx" },
	{ 0xb329595b, "__register_chrdev" },
	{ 0xe9b868f, "device_destroy" },
	{ 0x9feaf322, "remove_proc_entry" },
	{ 0xa78ce5b7, "usb_kill_urb" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x56470118, "__warn_printk" },
	{ 0xc00e2b80, "seq_printf" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0x20000329, "simple_strtoul" },
	{ 0x33b30418, "netif_carrier_on" },
	{ 0xd7c0f588, "i2c_transfer" },
	{ 0x2e4b8e3f, "pci_disable_device" },
	{ 0xe32066fc, "usb_reset_endpoint" },
	{ 0x8e66928c, "single_release" },
	{ 0x935c45ad, "dma_set_mask" },
	{ 0x362f9a8, "__x86_indirect_thunk_r12" },
	{ 0xa4ec7e8f, "alloc_can_err_skb" },
	{ 0xbf55f104, "kmalloc_trace" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x754d539c, "strlen" },
	{ 0x77358855, "iomem_resource" },
	{ 0x46b0be46, "single_open" },
	{ 0x349cba85, "strchr" },
	{ 0x1c7c3948, "pci_write_config_word" },
	{ 0x4503bd8e, "pci_free_irq_vectors" },
	{ 0x5c8093dc, "register_candev" },
	{ 0xf90a1e85, "__x86_indirect_thunk_r8" },
	{ 0xd4ec10e6, "BUG_func" },
	{ 0x52a2116a, "class_register" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xe2c17b5d, "__SCT__might_resched" },
	{ 0x1004e946, "kmalloc_caches" },
	{ 0xea2485ce, "netdev_info" },
	{ 0x85bd1608, "__request_region" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x73776b79, "module_layout" },
};

MODULE_INFO(depends, "i2c-algo-bit,can-dev");

MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000016sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000017sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000019sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000001Asv*sd*bc*sc*i*");
MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0014d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "E384788D80BD3973893A78B");
