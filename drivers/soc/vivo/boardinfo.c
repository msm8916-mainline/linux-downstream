/*
 * Copyright (C) 2016 vivo Co., Ltd.
 * YangChun <yangchun@vivo.com.cn>
 *
 * This driver is used to export hardware board information for users
 *
**/

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <soc/qcom/socinfo.h>
#include <soc/qcom/smem.h>
#include <linux/cpufreq.h>
#include <asm/io.h>
#include <linux/cpumask.h>
#define BOARD_REV_LEN 16
#define BOARD_NAME_LEN 24
#define VIVO_VENDOR_LEN 8
#define VIVO_CPU_TYPE_LEN 8
#define CPU_REVISION_ADDR 0x58004
#define VIVO_HW_VERSION_MASK (0x3<<8) //[9:8]


//
struct boardinfo_ext{
	char vendor[VIVO_VENDOR_LEN];
	unsigned int cpu_freq;
	char cpu_type[VIVO_CPU_TYPE_LEN];
	unsigned int core_num;
} *boardinfo_ext;




static ssize_t vivo_show_vendor(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->vendor);
}
static ssize_t vivo_show_cpu_freq(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_ext->cpu_freq);
}
static ssize_t vivo_show_cpu_type(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->cpu_type);
}

static ssize_t vivo_show_core_num(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_ext->core_num);
}

static struct device_attribute vivo_vendor = __ATTR(vendor, S_IRUGO, vivo_show_vendor,  NULL);

static struct device_attribute vivo_cpu_freq = __ATTR(cpu_freq, S_IRUGO,vivo_show_cpu_freq, NULL);

static struct device_attribute vivo_cpu_type = __ATTR(cpu_type, S_IRUGO, vivo_show_cpu_type, NULL);	

static struct device_attribute vivo_core_num = __ATTR(core_num, S_IRUGO, vivo_show_core_num, NULL);			

static void __init populate_soc_sysfs_files(struct device *vivo_soc_device)
{
	device_create_file(vivo_soc_device, &vivo_vendor);
	device_create_file(vivo_soc_device, &vivo_cpu_freq);
	device_create_file(vivo_soc_device, &vivo_cpu_type);
	device_create_file(vivo_soc_device, &vivo_core_num);
	return;
}
	
static int vivo_get_max_freq(unsigned int cpu_id){
	int max_freq = 0;
	int cur_freq = 0;
	int i = 0 ;
	struct cpufreq_frequency_table *table = NULL;
	struct cpufreq_policy *policy = NULL;
	policy = cpufreq_cpu_get(cpu_id);
	if(policy == NULL){
		return 0;
	}
	table = cpufreq_frequency_get_table(policy->cpu);
	cpufreq_cpu_put(policy);
	if(table == NULL){
		pr_err("vivo get frequency of CPU%u fail\n",cpu_id);
		return 0;
	}
	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		cur_freq = table[i].frequency;
		if (cur_freq == CPUFREQ_ENTRY_INVALID)
			continue;
		if(cur_freq > max_freq){
			max_freq = cur_freq;
		}
		//pr_err("CPU%u:table[%d]=%d\n",cpu_id,i,cur_freq);
	}
	return max_freq;
}


static unsigned int vivo_get_cpu_freq(void ){
	int cpu_id = 0;
	unsigned int max_freq = 0;
	unsigned int cur_freq = 0;
	int num_cpus = num_possible_cpus();
	for(cpu_id = 0;cpu_id < num_cpus;cpu_id++){
		if(cpu_online(cpu_id)){
			cur_freq = vivo_get_max_freq(cpu_id);
			if(cur_freq > max_freq){
				max_freq = cur_freq;
			}
		}
	}
	
	return max_freq;
}

static int vivo_get_cpu_type(char *cpu_type,int size){
	void __iomem *p_cpu_revision = NULL;
	unsigned int cpu_revision = 0;
	enum msm_cpu cpu_id = socinfo_get_msm_cpu();
	switch(cpu_id){
		case MSM_CPU_8939:{
			p_cpu_revision = (void __iomem *)ioremap(CPU_REVISION_ADDR,0x8);
			if(!p_cpu_revision){
				return 0;
			}
			cpu_revision = ioread32(p_cpu_revision);
			iounmap(p_cpu_revision);
			if(((cpu_revision&VIVO_HW_VERSION_MASK)>>8) == 0x3){  //hw_revision [9:8]
				strncpy(boardinfo_ext->cpu_type,"616",VIVO_CPU_TYPE_LEN); //616
			}else{
				strncpy(boardinfo_ext->cpu_type,"615",VIVO_CPU_TYPE_LEN); //615
			}
			break;
		}
		case MSM_CPU_8939_BC:{
			strncpy(boardinfo_ext->cpu_type,"615",size);
			break;
		}
		case MSM_CPU_8916:{
			strncpy(boardinfo_ext->cpu_type,"410",size);//410/412
			break;
		}
		case MSM_CPU_8929:{
			strncpy(boardinfo_ext->cpu_type,"415",size);
			break;
		}
		default:{
			pr_err("get cpu id error!\n");
			return 0;
		}
	}
	return 1;
}


static void vivo_boardinfo_ext_init(void){
	
	boardinfo_ext = kzalloc(sizeof(*boardinfo_ext), GFP_KERNEL);
	if (!boardinfo_ext) {
		pr_err("boardinfo_ext alloc failed!\n");
		return;
	}
	//frequency of cpu
	boardinfo_ext->cpu_freq = vivo_get_cpu_freq();
	//core number
	boardinfo_ext->core_num = num_possible_cpus();
	//type of cpu
	if(!vivo_get_cpu_type(boardinfo_ext->cpu_type,VIVO_CPU_TYPE_LEN)){
		strncpy(boardinfo_ext->cpu_type,"616",VIVO_CPU_TYPE_LEN); //default
	}
	//vendor
	strncpy(boardinfo_ext->vendor,"vivo",VIVO_VENDOR_LEN); //vivo
	pr_err("vivo cpu_freq:%u core_num=%u cpu_type=%s\n",boardinfo_ext->cpu_freq ,
			boardinfo_ext->core_num,
			boardinfo_ext->cpu_type);
}
static int __init vivo_boardinfo_init_sysfs(void)
{
	struct device *vivo_soc_device;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		pr_err("Soc Device alloc failed!\n");
		return -ENOMEM;
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		kfree(soc_dev_attr);
		pr_err("Soc device register failed\n");
		return -EIO;
	}
		
	vivo_soc_device = soc_device_to_device(soc_dev);
	
	populate_soc_sysfs_files(vivo_soc_device);
	
	//extra information init
	vivo_boardinfo_ext_init();
	
	return 0;
}
late_initcall(vivo_boardinfo_init_sysfs);

