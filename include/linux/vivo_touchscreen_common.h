#ifndef __BBK_TOUCHSCREEN_H__
#define __BBK_TOUCHSCREEN_H__

typedef struct vivo_touchscreen_common_data{
    char *driver_name;
	void (*charge_connect_judge)(char on_or_off);
	int (*get_ts_log_switch)(void);
}vivo_touchscreen_common_data;


enum {
	TOUCHSCREEN_REQ_ID_RESUME_SUSPEND = 0,
	TOUCHSCREEN_REQ_ID_LCD_STATE, 
	TOUCHSCREEN_REQ_ID_PROX_STATE, 
	TOUCHSCREEN_REQ_ID_USB_CHARGE,
	
	TOUCHSCREEN_REQ_ID_MAX
};

typedef int (*RESP_FUNC)(void*,int);

void touchscreen_set_priv_data(void * data);
int touchscreen_register_resp_func(int id,RESP_FUNC  pfun);
int touchscreen_request_send(int id,int state);



int register_touchscreen_common_interface(vivo_touchscreen_common_data *common_data);
void unregister_touchscreen_common_interface(vivo_touchscreen_common_data *common_data);
int get_ts_log_switch(void);
void vivo_get_file_path_str(char **pp_file,char *file_path);
#define VIVO_TS_LOG_ERR(fmt,param...) \
do {\
	char *file_name = NULL;\
	vivo_get_file_path_str(&file_name,__FILE__);\
	printk(KERN_ERR "VIVO_TS_ERR^%d^"fmt,__LINE__,##param);\
}while(0)

#define VIVO_TS_LOG_INF(fmt,param...) \
do {\
	char *file_name = NULL;\
	vivo_get_file_path_str(&file_name,__FILE__);\
	printk(KERN_ERR "VIVO_TS_INF^%d^"fmt,__LINE__,##param);\
}while(0)

#define VIVO_TS_LOG_DBG(fmt,param...) \
do {\
	if(get_ts_log_switch()) {\
		char *file_name = NULL;\
		vivo_get_file_path_str(&file_name,__FILE__);\
		printk(KERN_ERR "VIVO_TS_DBG^%d^"fmt,__LINE__,##param);\
	}\
}while(0)

#endif


