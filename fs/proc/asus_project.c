#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/init.h>

static struct proc_dir_entry *project_id_proc_file;
static int project_id_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", asus_PRJ_ID);
	return 0;
}

static int project_id_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_id_proc_read, NULL);
}


static struct file_operations project_id_proc_ops = {
	.open = project_id_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_id_proc_file(void)
{
    printk("create_project_id_proc_file\n");
    project_id_proc_file = proc_create("apid", 0444,NULL, &project_id_proc_ops);
    if(project_id_proc_file){
        printk("create project_id_proc_file sucessed!\n");
    }else{
		printk("create project_id_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_stage_proc_file;
static int project_stage_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_stage);
	return 0;
}

static int project_stage_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_stage_proc_read, NULL);
}


static struct file_operations project_stage_proc_ops = {
	.open = project_stage_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_stage_proc_file(void)
{
    printk("create_project_stage_proc_file\n");
    project_stage_proc_file = proc_create("apsta", 0444,NULL, &project_stage_proc_ops);
    if(project_stage_proc_file){
        printk("create project_stage_proc_file sucessed!\n");
    }else{
		printk("create project_stage_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_RFsku_proc_file;
static int project_RFsku_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_RFsku);
	return 0;
}

static int project_RFsku_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_RFsku_proc_read, NULL);
}


static struct file_operations project_RFsku_proc_ops = {
	.open = project_RFsku_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_RFsku_proc_file(void)
{
    printk("create_project_RFsku_proc_file\n");
    project_RFsku_proc_file = proc_create("aprf", 0444,NULL, &project_RFsku_proc_ops);
    if(project_RFsku_proc_file){
        printk("create project_RFsku_proc_file sucessed!\n");
    }else{
		printk("create project_RFsku_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_lte_proc_file;
static int project_lte_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_lte);
	return 0;
}

static int project_lte_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_lte_proc_read, NULL);
}


static struct file_operations project_lte_proc_ops = {
	.open = project_lte_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_lte_proc_file(void)
{
    printk("create_project_lte_proc_file\n");
    project_lte_proc_file = proc_create("aplte", 0444,NULL, &project_lte_proc_ops);
    if(project_lte_proc_file){
        printk("create project_lte_proc_file sucessed!\n");
    }else{
		printk("create project_lte_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_mem_proc_file;
static int project_mem_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_mem);
	return 0;
}

static int project_mem_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_mem_proc_read, NULL);
}


static struct file_operations project_mem_proc_ops = {
	.open = project_mem_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_mem_proc_file(void)
{
    printk("create_project_mem_proc_file\n");
    project_mem_proc_file = proc_create("apmem", 0444,NULL, &project_mem_proc_ops);
    if(project_mem_proc_file){
        printk("create project_mem_proc_file sucessed!\n");
    }else{
		printk("create project_mem_proc_file failed!\n");
    }
}

extern char asus_project_hd[2];

static struct proc_dir_entry *project_hd_proc_file;
static int project_hd_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n", asus_project_hd);
	return 0;
}

static int project_hd_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_hd_proc_read, NULL);
}


static struct file_operations project_hd_proc_ops = {
	.open = project_hd_proc_open,
	.read = seq_read,
	.release = single_release,	
};

static void create_project_hd_proc_file(void)
{
    printk("create_project_hd_proc_file\n");
    project_hd_proc_file = proc_create("aphd", 0444,NULL, &project_hd_proc_ops);
    if(project_hd_proc_file){
        printk("create project_hd_proc_file sucessed!\n");
    }else{
		printk("create project_hd_proc_file failed!\n");
    }
}



static int __init proc_asusPRJ_init(void)
{
	create_project_id_proc_file();
	create_project_lte_proc_file();
	create_project_RFsku_proc_file();
	create_project_stage_proc_file();
	create_project_mem_proc_file();
	create_project_hd_proc_file();
	return 0;
}
module_init(proc_asusPRJ_init);
