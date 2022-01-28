#include"across.h"

#include <linux/timer.h>
#include <linux/input.h>

static unsigned int GetTickCount(void)     //ms
{
	struct timeval tv;

	do_gettimeofday(&tv);
	return ((tv.tv_sec * 1000) + (tv.tv_usec / 1000));       // return in msec
}

int proximity_across_detect(unsigned int ps_data,struct across_data *data){
  unsigned int ps_across_data = ps_data;

  if(data->across_status == PS_ACROSS_FARAWAY && ps_across_data > atomic_read(&data->in_threshold)){
  	data->across_status = PS_ACROSS_NEAR;
  	data->across_in_time = GetTickCount();
  }
  
  if(data->across_status == PS_ACROSS_NEAR && ps_across_data < atomic_read(&data->out_threshold)){
  	data->across_status = PS_ACROSS_FARAWAY;
  	data->across_out_time = GetTickCount();
  }
  
  if(data->debug)
  printk(KERN_INFO"[Across] ps_across_data is %d,ps_across_status is %d,ps_across_in_time is %d,ps_across_out_time is %d\n",
  ps_across_data,data->across_status,data->across_in_time,data->across_out_time);
  
  if(data->across_in_time != -1 && data->across_out_time != -1 && 
    (data->across_out_time - data->across_in_time) > 0 &&
    (data->across_out_time - data->across_in_time) < 500){
  	//input report
  	printk(KERN_INFO"[Across] yes we get a proximity across event\n");
  	data->across_in_time = -1;
  	data->across_out_time = -1;
  	//1 represent a across motion detect
  	//input_report_rel(data->input_dev_ps_across, REL_X, 1);
  	//input_sync(data->input_dev_ps_across);
    return 1;  
  }

  return 0;
}
//EXPORT_SYMBOL_GPL(proximity_across_detect);
void set_across_threshold(int base,int out_coefficient,struct across_data *data){

  atomic_set(&data->in_threshold,  base + out_coefficient);//treat ps far away threshold as across close threshold
  atomic_set(&data->out_threshold,  base + out_coefficient*(data->across_gain)/100);
}

void init_across_data(unsigned int across_status,struct across_data *data,unsigned int debug){
  data->debug = debug;
  data->poll_time = PS_ACROSS_DEFAULT_POLL_TIME;
  data->across_gain = PS_ACROSS_DEFAULT_LOW_GAIN;
  data->across_status = across_status;
  data->across_in_time = -1;
  data->across_out_time = -1;
  printk(KERN_INFO"[Across] initial across data ,first ps status %d\n",data->across_status);
}
