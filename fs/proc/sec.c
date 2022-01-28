#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/ioport.h>    
#include <asm/io.h> 
#include <linux/libfdt_env.h>
#include <linux/string.h>
#include <linux/types.h>

#define SEC_SERIAL_NUM_ADDR 		0x0005C008
#define SEC_ENABLE_REG_ADDR 		0x00058098
#define SEC_KEY_VER_ADDR 			0x00058030
#define SEC_ROOT_CERT_HASH_ADDR  	0x000580a8
#define SEC_MASK_BIT_56			 	0XFFFFFFFFFFFFFF00
#define SEC_MASK_BIT_32			 	0XFFFFFFFF00000000
#define SEC_RPMB_KEY_PROVISION_ADDR 0x0005C018



u32 serial_num,sec_enable,sec_key_ver_reg;
char sec_key_hash[80];
void __iomem *rpmb_key_prov_ptr = NULL;


static int  get_secureboot_root_cert_hash(void __iomem *key_hash_addr,char *hash,const int size){
	uint64_t hash_row[5];
	uint8_t byte_val = 0;
	char byte_hex_str[8];
	void __iomem * reg_addr = key_hash_addr;
	uint64_t reg_val = 0;
	int i = 0;
	int row_idx = 0;
	int shift_bit = 0;
	int len = 0;
	
	if(NULL == hash || size < 64){
		return 0;
	}
	//read hash data from gfprom
	for(i = 0; i< 5;i++){
		//reg_val = ioread64(reg_addr);
		reg_val = readq(reg_addr);
		//le64_to_cpu
		//be64_to_cpu
		reg_val = cpu_to_fdt64(reg_val); //converting val to big-endian
		if(i == 4){
			hash_row[i] = (reg_val&SEC_MASK_BIT_32)>>32;
		}else{
			hash_row[i] = (reg_val&SEC_MASK_BIT_56)>>8;	
		}
		reg_addr += 8;
	}
	//hash data to string
	strcpy(hash,"");
	for(row_idx = 0;row_idx < 5;row_idx++){
		if(row_idx == 4){
			shift_bit = 24;
			len = 4;
		}else{
			shift_bit = 48;
			len = 7;
		}
		for(i = 0;i < len;i++){
			byte_val = hash_row[row_idx]>>shift_bit;
			snprintf(byte_hex_str,sizeof(byte_hex_str),"%02x",byte_val);
			strcat(hash,byte_hex_str);
			shift_bit -= 8;
		}
	}
	return 1;
}


static int sec_enable_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m,"0x%08x",
		sec_enable);
	return 0;
}

static int serial_num_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m,"0x%08x",
		serial_num);
	return 0;
}

static int rpmb_prov_proc_show(struct seq_file *m, void *v)
{
	if(NULL != rpmb_key_prov_ptr){
		seq_printf(m,"0x%08x",ioread32(rpmb_key_prov_ptr));
	}else{
		printk(KERN_ERR "Sec:read rpmb key provision register error!\n");
		seq_printf(m,"0x0");
	}
	
	return 0;
}

static int key_ver_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m,"%d",(sec_key_ver_reg&0x0F800000)>>23);//key version bit 27:23
	return 0;
}

static int key_hash_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m,"%s",sec_key_hash);	
	return 0;
}



static int serial_num_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, serial_num_proc_show, NULL);
}
static int sec_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_enable_proc_show, NULL);
}
static int key_ver_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, key_ver_proc_show, NULL);
}
static int key_hash_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, key_hash_proc_show, NULL);
}

static int rpmb_prov_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rpmb_prov_proc_show, NULL);
}



static const struct file_operations sec_enable_proc_fops = {
	.open		= sec_enable_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations serial_num_proc_fops = {
	.open		= serial_num_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations key_ver_proc_fops = {
	.open		= key_ver_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations key_hash_proc_fops = {
	.open		= key_hash_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations rpmb_prov_proc_fops = {
	.open		= rpmb_prov_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init proc_sec_reg_init(void)
{
	void __iomem *sec_enable_ptr,*sec_serial_num_ptr,*key_ver_ptr, *key_hash_addr_ptr;
	//create node 
	proc_create("sec_serial_num", 0, NULL, &serial_num_proc_fops);
	proc_create("sec_enable", 0, NULL, &sec_enable_proc_fops);
	proc_create("sec_key_ver", 0, NULL, &key_ver_proc_fops);
	proc_create("sec_key_hash", 0, NULL, &key_hash_proc_fops);
	proc_create("sec_rpmb_prov", 0, NULL, &rpmb_prov_proc_fops);
	
	//request memery
	request_mem_region(SEC_ENABLE_REG_ADDR,0x8,"sec_enable"); 
	request_mem_region(SEC_SERIAL_NUM_ADDR,0x8,"serial_num");  
	request_mem_region(SEC_KEY_VER_ADDR,0x8,"sec_key_ver");
	request_mem_region(SEC_ROOT_CERT_HASH_ADDR,40,"sec_key_hash");	
	request_mem_region(SEC_RPMB_KEY_PROVISION_ADDR,0x8,"sec_rpmb_prov");
	
	sec_enable_ptr = ioremap(SEC_ENABLE_REG_ADDR,0x8);
	sec_serial_num_ptr = ioremap(SEC_SERIAL_NUM_ADDR,0x8);
	key_ver_ptr = ioremap(SEC_KEY_VER_ADDR,0x8);
	key_hash_addr_ptr = ioremap(SEC_ROOT_CERT_HASH_ADDR,40);
	rpmb_key_prov_ptr = ioremap(SEC_RPMB_KEY_PROVISION_ADDR,0x8);
	//get secure boot info 
	sec_enable = ioread32(sec_enable_ptr);
	serial_num = ioread32(sec_serial_num_ptr);
	sec_key_ver_reg = ioread32(key_ver_ptr);
	strcpy(sec_key_hash,"");
	get_secureboot_root_cert_hash(key_hash_addr_ptr,sec_key_hash,sizeof(sec_key_hash));
	//release
	release_mem_region(SEC_ENABLE_REG_ADDR,0x8); 
	release_mem_region(SEC_SERIAL_NUM_ADDR,0x8); 
	release_mem_region(SEC_KEY_VER_ADDR,0x8); 
	release_mem_region(SEC_ROOT_CERT_HASH_ADDR,0x40); 
    iounmap(sec_enable_ptr);
    iounmap(sec_serial_num_ptr);
	iounmap(key_ver_ptr);
	iounmap(key_hash_addr_ptr);
	return 0;
}
module_init(proc_sec_reg_init);
