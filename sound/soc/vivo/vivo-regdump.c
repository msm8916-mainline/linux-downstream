#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/vivo-regdump.h>

struct vivo_regdump_callback {
	struct list_head list;
	struct vivo_regdump_handler *handler;
};

static struct vivo_regdump_callback vivo_callbacks;

int vivo_audio_regdump_register_callback(struct vivo_regdump_handler *handler) 
{
	struct vivo_regdump_callback *new_callback = NULL;
	
	new_callback = kzalloc(sizeof(struct vivo_regdump_callback), GFP_KERNEL);
	if (!new_callback) {
		printk(KERN_ERR "%s: Failed at allocate callback struct\n", __func__);
		return -ENOMEM;
	}
	if (!handler->name) {
		printk(KERN_ERR "%s: register failed, handler->name is NULL\n", __func__);
		return -EINVAL;
	}
	new_callback->handler= handler;
	INIT_LIST_HEAD(&new_callback->list);
	list_add_tail(&new_callback->list, &vivo_callbacks.list);

	return 0;
}
EXPORT_SYMBOL_GPL(vivo_audio_regdump_register_callback);

void vivo_audio_regdump_deregister_callback(char *callback_name)
{
	struct vivo_regdump_callback *entry;
	if (!callback_name){
		printk(KERN_ERR "%s() callback_name is NULL, deregister failed\n", __func__);
		return;
	}
	if (!list_empty(&vivo_callbacks.list)) {
		list_for_each_entry(entry, &vivo_callbacks.list, list)
			if (!strcmp(entry->handler->name, callback_name)) {
				list_del(&entry->list);
				kfree(entry);
				return;
			}
	}
}
EXPORT_SYMBOL_GPL(vivo_audio_regdump_deregister_callback);

void vivo_audio_regdump_do_callback()
{
	struct vivo_regdump_callback *entry;

	printk(KERN_INFO "%s() is called\n", __func__);
	if (!list_empty(&vivo_callbacks.list)) {
		list_for_each_entry(entry, &vivo_callbacks.list, list) {
			if (entry->handler->callback)
			entry->handler->callback();
		}
	}
}
EXPORT_SYMBOL_GPL(vivo_audio_regdump_do_callback);

static int __init vivo_regdump_init(void)
{
	printk(KERN_INFO "%s enter\n", __func__);
	INIT_LIST_HEAD(&vivo_callbacks.list);
	return 0;
}

static void __exit vivo_regdump_exit(void)
{
	/*nothing to do */
}

module_init(vivo_regdump_init);
module_exit(vivo_regdump_exit);
