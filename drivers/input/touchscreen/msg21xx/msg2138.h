#ifndef __LINUX_MSG21XX_TS_H__
#define __LINUX_MSG21XX_TS_H__

#define    MSG21XX_TOUCH_KEY

//#define MSTAR_DEBUG		0
#ifdef MSTAR_DEBUG
#define MSTAR_DBG(format, ...)	\
		printk(KERN_INFO "MSTAR_TS " format "\n", ## __VA_ARGS__)
#define	TP_DEBUG(format, ...)	\
		printk(KERN_INFO "MSTAR_TS " format "\n", ## __VA_ARGS__)
#else
#define MSTAR_DBG(fmt, ...)
#define TP_DEBUG(fmt, ...)
#endif
#define MAX_TOUCH_FINGER 2

struct msg21xx_ts_platform_data {
    u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 disp_minx;
	u32 disp_miny;
	u32 disp_maxx;
	u32 disp_maxy;
	//const char *name;
	int *keycodes;
	int num_keys;
	int y_offset;
    bool no_force_update;
	bool i2c_pull_up;    
	//int disp_maxx;
	//int disp_maxy;
	int pan_maxx;
	int pan_maxy;	
	unsigned irq_gpio;
	unsigned reset_gpio;
	u32 irq_flags;
	u32 reset_flags;

};
typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;

struct msg21xx_ts_data
{
	uint16_t            addr;
	struct i2c_client  *client;
	struct input_dev   *input_dev;
	int                 use_irq;
	struct work_struct  work;
	int (*power)(int on);
	int (*get_int_status)(void);
	void (*reset_ic)(void);
};
#endif
