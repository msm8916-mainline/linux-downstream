#include <asm/atomic.h>

#define PS_ACROSS_NEAR 0
#define PS_ACROSS_FARAWAY 1
#define PS_ACROSS_DEFAULT_GAIN 50 //range 1~100
//new method for ensure the across threshold
//#define PS_ACROSS_DEFAULT_HIGH_GAIN 37
#define PS_ACROSS_DEFAULT_LOW_GAIN 50//across low thres is normal_ps_low_threshold * 0.5

#define PS_ACROSS_DEFAULT_POLL_TIME 10  //ms

struct across_data{
  unsigned int poll_time;
  unsigned int across_status;
  unsigned int across_gain;
  unsigned int across_in_time;
  unsigned int across_out_time;
  unsigned int debug;
  
  atomic_t in_threshold;
  atomic_t out_threshold;

  unsigned int across_enable_first;
};

int proximity_across_detect(unsigned int ps_data,struct across_data *data);
void init_across_data(unsigned int across_status,struct across_data *data,unsigned int debug);
void set_across_threshold(int base,int out_coefficient,struct across_data *data);//out coefficient is normal ps far away threshold -- 5cm usually

