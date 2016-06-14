/**
 * @file upi_ug31xx.c
 *
 * UPI uG31xx gauge driver
 *
 * @version $Revision$
 * @author AllenTeng <allen.kuoliang.teng@gmail.com>
 * @note
 */

#include "upi_ug31xx.h"

//#define UG31XX_REGISTER_I2C                         ///< 註册 I2C driver
#ifdef  UG31XX_REGISTER_I2C
#define UG31XX_I2C_ADAPTER  (1)                     ///< 设定 I2C 设备串口
#endif  ///< end of UG31XX_REGISTER_I2C

#define UG31XX_REGISTER_POWER_SUPPLY                ///< 註册 power supply 檔案系统
#define UG31XX_REGISTER_PROC                        ///< 註册 proc 檔案系统 

//#define UG31XX_REGISTER_EARLY_SUSPEND               ///< 開啟 Early Suspend 程序
#ifdef  UG31XX_REGISTER_EARLY_SUSPEND
#include <linux/earlysuspend.h>
#endif  ///< end of UG31XX_REGISTER_EARLY_SUSPEND

#define UG31XX_BOARD_OFFSET_CALIBRATION             ///< 開啟 Board Offset 校正功能

#define UG31XX_ALARM_FUNCTION                       ///< 開啟 UV ALARM IRQ 功能
#define UG31XX_ATTACH_DETECTION                     ///< 開啟 Gauge Attach IRQ 功能
#ifdef  UG31XX_ATTACH_DETECTION
  #define UG31XX_DELAY_INIT                         ///< 開啟 Delay Initialize 功能
#endif  ///< end of UG31XX_ATTACH_DETECTION

#define UG31XX_BAT_FULL_VOLT        (4350)          ///< 设定电池满充电压
#define UG31XX_BAT_SHUTDOWN_VOLT    (3000)          ///< 设定系统关机电压

#define UG31XX_CALI_RSOC_TIME       (3600)          ///< 设定 RSOC 校正时间间隔
#define UG31XX_SUSPEND_CAL_TIME     (60)            ///< 设定 suspend 运算间隔
#define UG31XX_SUSPEND_CNT_MAX      (10)

#define UG31XX_RSOC_VALUE_IF_DRV_NOT_READY  (-1)

typedef struct upi_kbo_st {
  struct delayed_work work;
  int                 result;
  int                 result_sum;
  int                 pass_cnt;
  int                 fail_cnt;
  int                 current_now;
} upi_kbo_type;

typedef struct upi_ug31xx_st {
  struct i2c_client       *client;
  struct device           *dev;
  struct delayed_work      info_update_work;
  struct delayed_work      power_change_work;
  struct wake_lock         info_update_wake_lock;
  struct wake_lock         i2c_wake_lock;
  #ifdef  UG31XX_REGISTER_EARLY_SUSPEND
  struct early_suspend     early_suspend;
  struct rtc_time          tm_early_suspend;
  struct rtc_time          tm_late_resume;
  #endif  ///< end of UG31XX_REGISTER_EARLY_SUSPEND
  struct rtc_time          tm_suspend;
  struct rtc_time          tm_resume;
  struct rtc_time          tm_last_task;
  #ifdef  UG31XX_ATTACH_DETECTION
  struct mutex             attach_lock;
  #endif  ///< end of UG31XX_ATTACH_DETECTION
  #ifdef  UG31XX_DELAY_INIT
  struct hrtimer           delay_init_timer;
  struct delayed_work      delay_init_work;
  #endif  ///< end of UG31XX_DELAY_INIT

  int                      curr_cable_status;
  int                      prev_cable_status;
  int                      cable_change_cnt;
  bool                     charging_stopped;
  #ifdef  UG31XX_REGISTER_EARLY_SUSPEND
  unsigned long            sec_early_suspend;
  unsigned long            sec_late_resume;
  #endif  ///< end of UG31XX_REGISTER_EARLY_SUSPEND
  unsigned long            sec_suspend;
  unsigned long            sec_resume;
  unsigned long            sec_delta_suspend_resume;
  unsigned long            sec_last_task;
  bool                     first_run;
  int                      last_report_rsoc;
  int                      in_suspend_cnt;
  #ifdef  UG31XX_DELAY_INIT 
  atomic_t                 wait_delay_init;
  atomic_t                 wait_delay_state;
  atomic_t                 wait_delay_toggle;
  #endif  ///< end of UG31XX_DELAY_INIT 
  upi_kbo_type             kbo_data;
} upi_ug31xx_type;

static upi_ug31xx_type *upi_ug31xx = NULL;
static unsigned char debug_level = 0;
static unsigned char alarm_enable = 0;
static unsigned char gauge_attached = 0;

typedef struct polling_time_st {
  int volt;
  int sec;
} polling_time_type;

#define POLL_TIME_V_STEP  (UG31XX_BAT_FULL_VOLT - UG31XX_BAT_SHUTDOWN_VOLT) / 100

static polling_time_type upi_ug31xx_polling_time[] =
{
  { UG31XX_BAT_SHUTDOWN_VOLT + POLL_TIME_V_STEP * 50, 30, },
  { UG31XX_BAT_SHUTDOWN_VOLT + POLL_TIME_V_STEP * 40, 25, },
  { UG31XX_BAT_SHUTDOWN_VOLT + POLL_TIME_V_STEP * 30, 20, },
  { UG31XX_BAT_SHUTDOWN_VOLT + POLL_TIME_V_STEP * 20, 15, },
  { UG31XX_BAT_SHUTDOWN_VOLT + POLL_TIME_V_STEP * 10, 10, },
  { UG31XX_BAT_SHUTDOWN_VOLT,                         5,  },
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~chris add for Z380 project~~~~~~~~~~~~~~~~~~~~~~~~ */
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
extern bool COVER_ATTACHED_UPI(void);
extern bool CHARGING_FULL(void);
extern bool STOP_CHARGING(void);
extern bool START_CHARGING(void);
extern bool IS_CA81(void);
extern bool IS_CB81(void);
extern int WRITE_EEPROM(u8 address, u8 value);
extern int READ_EEPROM(u8 address);
extern int smb345c_get_charging_status(void);
#else
bool COVER_ATTACHED_UPI(void) { return false; }
#endif

/// ===============================================================================================
/// Link to System
/// ===============================================================================================

static int write_eeprom(GU1 addr, GU1 value)
{
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  return WRITE_EEPROM(addr, value);
#else
  GLOGE("[%s]: Not implemented -> Always return 0, which means PASS\n", __func__);
  return (0);
#endif
}

static int read_eeprom(GU1 addr)
{
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  return READ_EEPROM(addr);
#else
  GLOGE("[%s]: Not implemented -> Always return 0, which means data is 0\n", __func__);
  return (0);
#endif
}

/// ===============================================================================================
/// Charging control API
/// ===============================================================================================

static void stop_charging(void)
{
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  STOP_CHARGING();
#else
  GLOGE("[%s]: Not implemented\n", __func__);
#endif
}

static void start_charging(void)
{
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  START_CHARGING();
#else
  GLOGE("[%s]: Not implemented\n", __func__);
#endif
}

static bool is_charging_full(void)
{
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  bool is_full;
  GWORD wData;
  int curr_now;
  int volt_now;
  int rsoc_now;

  /// [AT-PM] : Get charger full status ; 05/25/2015
  is_full = CHARGING_FULL();
  if(is_full == true)
  {
    if(gauge_attached == 1)
    {
      /// [AT-PM] : Check RSOC ; 05/26/2015
      SmbReadCommand(SBS_RSOC);
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      rsoc_now = (int)wData;

      /// [AT-PM] : No need to check voltage and current if RSOC is 100% ; 05/26/2015
      if(rsoc_now != 100)
      {
        /// [AT-PM] : Update gauge status ; 05/25/2015
        GMainTask();

        /// [AT-PM] : Get current ; 05/25/2015
        SmbReadCommand(SBS_CURR);
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
        curr_now = (int)wData;
        if(curr_now > 32767)
        {
          curr_now = curr_now - 65536;
        }

        /// [AT-PM] : Set the is_full to false if current is larger than taper current ; 05/25/2015
        if(curr_now > GGB->cChargeControl.scTerminationCfg.wTPCurrent)
        {
          is_full = false;
          GLOGE("[%s]: current = %d > taper current = %d\n", __func__, curr_now, GGB->cChargeControl.scTerminationCfg.wTPCurrent);
        }
        else
        {
          /// [AT-PM] : Get voltage ; 05/25/2015
          SmbReadCommand(SBS_VOLT);
          wData = (GWORD)(*(SBSCmd->pbData + 1));
          wData = (wData << 8) | (*SBSCmd->pbData);
          volt_now = (int)wData;

          if(volt_now < GGB->cChargeControl.scTerminationCfg.wTPVoltage)
          {
            is_full = false;
            GLOGE("[%s]: voltage = %d < taper voltage = %d\n", __func__, volt_now, GGB->cChargeControl.scTerminationCfg.wTPVoltage);
          }
        }
      }
    }
  }
  return (is_full);
#else
  GLOGE("[%s]: Not implemented -> Always returns false\n", __func__);
  return (false);
#endif
}

/// ===============================================================================================
/// Link to gauge driver
/// ===============================================================================================

GBOOL _CFGWriteByte(GWORD addr, GCHAR buf)
{
  int rtn;

#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  if (!COVER_ATTACHED_UPI()) return (GFALSE);
#endif

  rtn = write_eeprom((GU1)addr, (GU1)buf);
  return ((rtn == 0) ? GTRUE : GFALSE);
}

GBOOL _CFGReadByte(GWORD addr, GCHAR *buf)
{
  int rtn;

#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  if (!COVER_ATTACHED_UPI()) return (GFALSE);
#endif

  rtn = read_eeprom((GU1)addr);
  *buf = (GCHAR)rtn;
  return (GTRUE);
}

GBOOL _I2cExeCmd(GI2CCmdType *pI2CCmd)
{
  struct i2c_msg msg[2];
  int rtn;
  int msg_size;

#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  if (!COVER_ATTACHED_UPI())
  {
    pI2CCmd->bStatus |= I2C_STS_SLAVE_NOT_EXIST;
    return (GFALSE);
  }
#endif
  if((!(upi_ug31xx->client)) || (!(upi_ug31xx->client->adapter)))
  {
    return (GFALSE);
  }

  if(pI2CCmd->bRdCnt > 0)
  {
    /// [AT-PM] : I2C read operation ; 06/10/2014
    msg[0].addr = pI2CCmd->bSlaveAddr;
    msg[0].flags = 0 | I2C_M_NOSTART;
    msg[0].len = pI2CCmd->bWtCnt;
    msg[0].buf = (unsigned char *)(&(pI2CCmd->bWtBuf[0]));

    msg[1].addr = pI2CCmd->bSlaveAddr; 
    msg[1].flags = I2C_M_RD;
    msg[1].len = pI2CCmd->bRdCnt;
    msg[1].buf = (unsigned char *)(&(pI2CCmd->bRdBuf[0]));

    msg_size = 2;
    #if defined(CONFIG_Z380C)
    wake_lock(&upi_ug31xx->i2c_wake_lock);
    rtn = i2c_transfer(upi_ug31xx->client->adapter, msg, msg_size);
    wake_unlock(&upi_ug31xx->i2c_wake_lock);
    #endif  ///< end of defined(CONFIG_Z380C)
  }
  else
  {
    #if defined(CONFIG_Z380C)
    if(pI2CCmd->bWtCnt > 2)
    {
      wake_lock(&upi_ug31xx->i2c_wake_lock);
  	  rtn = i2c_smbus_write_word_data(upi_ug31xx->client, pI2CCmd->bWtBuf[0], (pI2CCmd->bWtBuf[2] << 8) | pI2CCmd->bWtBuf[1]);
      wake_unlock(&upi_ug31xx->i2c_wake_lock);
    }
    else
    {
      wake_lock(&upi_ug31xx->i2c_wake_lock);
  	  rtn = i2c_smbus_write_byte_data(upi_ug31xx->client, pI2CCmd->bWtBuf[0], pI2CCmd->bWtBuf[1]);
      wake_unlock(&upi_ug31xx->i2c_wake_lock);
    }
    #else   ///< else of defined(CONFIG_Z380C)
     /// [AT-PM] : I2C write operation ; 06/10/2014
    msg[0].addr = pI2CCmd->bSlaveAddr;
    msg[0].flags = 0 | I2C_M_NOSTART;
    msg[0].len = pI2CCmd->bWtCnt;
    msg[0].buf = (unsigned char *)(&(pI2CCmd->bWtBuf[0]));

    msg_size = 1;
    #endif  ///< end of defined(CONFIG_Z380C)
  }
  #if !defined(CONFIG_Z380C)
  rtn = i2c_transfer(upi_ug31xx->client->adapter, msg, msg_size);
  #endif  ///< end of defined(CONFIG_Z380C)
  return ((rtn < 0) ? GFALSE : GTRUE);
}

static unsigned long ug31xx_get_rtc_time(struct rtc_time *obj)
{
  struct timeval rawtime;
  unsigned long local_time;

  do_gettimeofday(&rawtime);
  local_time = (unsigned long)(rawtime.tv_sec - (sys_tz.tz_minuteswest * 60));
  rtc_time_to_tm(local_time, obj);
  return (local_time);
}

static void upi_ug31xx_main_init(void)
{
  upi_ug31xx->curr_cable_status = UPI_UG31XX_CABLE_OUT;
  upi_ug31xx->prev_cable_status = UPI_UG31XX_CABLE_OUT;
  upi_ug31xx->cable_change_cnt  = UPI_UG31XX_CABLE_POLL_CNT;
  upi_ug31xx->first_run         = true;
  upi_ug31xx->last_report_rsoc  = -1;
  upi_ug31xx->kbo_data.result   = 0;

  wake_lock(&upi_ug31xx->info_update_wake_lock);
  
//  UPI_MEMCPY(FactoryGGBXFile, FactoryGGBXFile_A023Cl, sizeof(GlobalDFVarType));     ///< CB71
//  UPI_MEMCPY(FactoryGGBXFile, FactoryGGBXFile_A0249l, sizeof(GlobalDFVarType));     ///< CB71
//  UPI_MEMCPY(FactoryGGBXFile, FactoryGGBXFile_A025Jl, sizeof(GlobalDFVarType));     ///< CB71
//  UPI_MEMCPY(FactoryGGBXFile, FactoryGGBXFile_A0262h, sizeof(GlobalDFVarType));     ///< CA71 and CA81
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  if (IS_CA81())
    UPI_MEMCPY(FactoryGGBXFile, FactoryGGBXFile_A0262h, sizeof(GlobalDFVarType));     ///< CA71 and CA81
  else
#endif
  UPI_MEMCPY(FactoryGGBXFile, FactoryGGBXFile_A027Co, sizeof(GlobalDFVarType));     ///< CB81

  GMainInit();

  stop_charging();
  upi_ug31xx->charging_stopped = true;

  wake_unlock(&upi_ug31xx->info_update_wake_lock);
}

static void upi_ug31xx_main_task(struct work_struct *work)
{
  GWORD wSysMode;
  GWORD wVolt;
  GWORD wRsoc;
  GBYTE buf[2];
  int idx;

  #ifdef  UG31XX_ATTACH_DETECTION
  if(gauge_attached == 0)
  {
    GLOGE("[%s]: No gauge IC attached\n", __func__);
    return;
  }
  #endif  ///< end of UG31XX_ATTACH_DETECTION

  wake_lock(&upi_ug31xx->info_update_wake_lock);

  if(is_charging_full() == true)
  {
    buf[0] = CAP_SBS_CMD_SET_FULL % 256;
    buf[1] = CAP_SBS_CMD_SET_FULL / 256;
    SmbWriteCommand(SBS_CAP_DATA, &buf[0], 2);

    buf[0] = VCAP_SBS_CMD_SET_FULL % 256;
    buf[1] = VCAP_SBS_CMD_SET_FULL / 256;
    SmbWriteCommand(SBS_VCAP_DATA, &buf[0], 2);
  }

  #ifdef  UG31XX_ALARM_FUNCTION
  /// [AT-PM] : Enable/disable alarm function ; 04/01/2015
  if(alarm_enable == 0)
  {
    buf[0] = SBS_ALARM_UV_DISABLE % 256;
    buf[1] = SBS_ALARM_UV_DISABLE / 256;
  }
  else
  {
    buf[0] = SBS_ALARM_UV_ENABLE % 256;
    buf[1] = SBS_ALARM_UV_ENABLE / 256;
  }
  SmbWriteCommand(SBS_ALARM, &buf[0], 2);
  #endif  ///< end of UG31XX_ALARM_FUNCTION

  GMainTask();

  SmbReadCommand(SBS_SYS_MODE);
  wSysMode = (GWORD)(*(SBSCmd->pbData + 1));
  wSysMode = (wSysMode << 8) | (*(SBSCmd->pbData));
  if(wSysMode & SYS_MODE_INIT_CAP)
  {
    if(upi_ug31xx->charging_stopped == true)
    {
      upi_ug31xx->charging_stopped = false;
      start_charging();
    }

    if(upi_ug31xx->first_run == true)
    {
      upi_ug31xx->first_run = false;

      schedule_delayed_work(&upi_ug31xx->info_update_work, msecs_to_jiffies(TIME_RESET_NEXT_ROUND));
    }
    else
    {
      /// [AT-PM] : Set next round ; 10/03/2014
      upi_ug31xx->in_suspend_cnt = (upi_ug31xx->in_suspend_cnt == 0) ? 0 : upi_ug31xx->in_suspend_cnt - 1;

      if(upi_ug31xx->cable_change_cnt)
      {
        /// [AT-PM] : Due to cable status changed ; 10/14/2014
        upi_ug31xx->cable_change_cnt = upi_ug31xx->cable_change_cnt - 1;

        schedule_delayed_work(&upi_ug31xx->info_update_work, UPI_UG31XX_CABLE_POLL_TIME*HZ);
        GLOGE("[%s]: Busy polling due to cable changed. (%d)\n", __func__, upi_ug31xx->cable_change_cnt);

        cancel_delayed_work_sync(&upi_ug31xx->power_change_work);
        schedule_delayed_work(&upi_ug31xx->power_change_work, 1*HZ);
      }
      else
      {
        /// [AT-PM] : Set according to voltage ; 10/14/2014
        SmbReadCommand(SBS_VOLT);
        wVolt = (GWORD)(*(SBSCmd->pbData + 1));
        wVolt = (wVolt << 8) | (*SBSCmd->pbData);

        idx = 0;
        while(upi_ug31xx_polling_time[idx].volt > UG31XX_BAT_SHUTDOWN_VOLT)
        {
          if(upi_ug31xx_polling_time[idx].volt <= wVolt)
          {
            break;
          }

          idx = idx + 1;
        }

        schedule_delayed_work(&upi_ug31xx->info_update_work, upi_ug31xx_polling_time[idx].sec*HZ);
        GLOGE("[%s]: Wait %d seconds for next round (%d)\n", __func__, upi_ug31xx_polling_time[idx].sec, wVolt);
      }
    }

    /// [AT-PM] : Report to system if RSOC is updated ; 10/31/2014
    SmbReadCommand(SBS_RSOC);
    wRsoc = (GWORD)(*(SBSCmd->pbData + 1));
    wRsoc = (wRsoc << 8) | (*SBSCmd->pbData);

    if(wRsoc != upi_ug31xx->last_report_rsoc)
    {
      cancel_delayed_work_sync(&upi_ug31xx->power_change_work);
      schedule_delayed_work(&upi_ug31xx->power_change_work, 0*HZ);
    }
    upi_ug31xx->last_report_rsoc = (int)wRsoc;

    /// [YL] : Set debug level ; 20150907
    if(debug_level == 0)
    {
      buf[0] = SYS_MODE_CMD_SIMPLE_DEBUG_MSG % 256;
      buf[1] = SYS_MODE_CMD_SIMPLE_DEBUG_MSG / 256;
      SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);
    }
    else 
    {
      buf[0] = SYS_MODE_CMD_FULL_DEBUG_MSG % 256;
      buf[1] = SYS_MODE_CMD_FULL_DEBUG_MSG / 256;
      SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);
    }
  }
  else
  {
    schedule_delayed_work(&upi_ug31xx->info_update_work, msecs_to_jiffies(TIME_RESET_NEXT_ROUND));
    GLOGE("[%s]: Wait for next initial round\n", __func__);
  }

  upi_ug31xx->sec_last_task = ug31xx_get_rtc_time(&upi_ug31xx->tm_last_task);

  wake_unlock(&upi_ug31xx->info_update_wake_lock);
}

static void upi_ug31xx_main_exit(void)
{
  wake_lock(&upi_ug31xx->info_update_wake_lock);

  GMainExit();

  wake_unlock(&upi_ug31xx->info_update_wake_lock);
}

/// ===============================================================================================
/// Attach detection
/// ===============================================================================================

#ifdef  UG31XX_ATTACH_DETECTION

#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
void upi_ug31xx_alarm_callback_for_asus(void)
{
  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);
  schedule_delayed_work(&upi_ug31xx->info_update_work, 0*HZ);
}
EXPORT_SYMBOL(upi_ug31xx_alarm_callback_for_asus);
#endif

static bool upi_ug31xx_is_attach(void)
{
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  GLOGD("[%s]: %s\n", __func__, COVER_ATTACHED_UPI() ? "true" : "false");
  return COVER_ATTACHED_UPI();
#else
  GLOGE("[%s]: Not implemented -> Always return false\n", __func__);
  return ((gauge_attached == 1) ? true : false);
#endif
}

#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
void upi_ug31xx_attach(bool attach)
#else
static void upi_ug31xx_attach(bool attach)
#endif
{
  #ifdef  UG31XX_DELAY_INIT
  atomic_set(&upi_ug31xx->wait_delay_toggle, 1);
  atomic_set(&upi_ug31xx->wait_delay_state, (attach == true) ? 1 : 0);

  if(atomic_read(&upi_ug31xx->wait_delay_init) == 1)
  {
    schedule_delayed_work(&upi_ug31xx->delay_init_work, 1*HZ);
    GLOGE("[%s]: Wait for the RTC timer stable\n", __func__);
    return;
  }
  #endif  ///< end of UG31XX_DELAY_INIT

  cancel_delayed_work_sync(&upi_ug31xx->power_change_work);
  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);

  mutex_lock(&upi_ug31xx->attach_lock);

  if(attach == true)
  {
    /// [AT-PM] : Operation for gauge attached ; 04/01/2015
    if(gauge_attached == 0)
    {
      GLOGE("[%s]: uG31xx attached without detached event.\n", __func__);
      upi_ug31xx_main_init();
    }

    schedule_delayed_work(&upi_ug31xx->info_update_work, 1*HZ);
    schedule_delayed_work(&upi_ug31xx->power_change_work, 10*HZ);

    gauge_attached = 1;
    GLOGE("[%s]: uG31xx attached.\n", __func__);
  }
  else
  {
    /// [AT-PM] : Operation for gauge removed ; 04/01/2015
    if(gauge_attached == 1)
    {
      GLOGE("[%s]: uG31xx removed without attached event.\n", __func__);
      upi_ug31xx_main_exit();
    }

    gauge_attached = 0;
    GLOGE("[%s]: uG31xx removed.\n", __func__);
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
    /* immediately update POWER_SUPPLY_PROP_PRESENT to android framework */
    cancel_delayed_work_sync(&upi_ug31xx->power_change_work);
    schedule_delayed_work(&upi_ug31xx->power_change_work, 0);
#endif
  }

  mutex_unlock(&upi_ug31xx->attach_lock);
}
EXPORT_SYMBOL(upi_ug31xx_attach);

#endif  ///< end of UG31XX_ATTACH_DETECTION

#ifdef  UG31XX_DELAY_INIT
static void upi_ug31xx_delay_init_work_func(struct work_struct *work)
{
  GLOGE("[%s]: Restart attach event\n", __func__);
  upi_ug31xx_attach(atomic_read(&upi_ug31xx->wait_delay_state) == 1 ? true : false);
}

static enum hrtimer_restart upi_ug31xx_delay_init_timer_func(struct hrtimer *timer)
{
  atomic_set(&upi_ug31xx->wait_delay_init, 0);
  GLOGE("[%s]: RTC time is stable\n", __func__);
  return (HRTIMER_NORESTART);
}
#endif  ////< end of UG31XX_DELAY_INIT

/// ===============================================================================================
/// Alarm function
/// ===============================================================================================

#ifdef  UG31XX_ALARM_FUNCTION

static void upi_ug31xx_alarm_callback(void)
{
  GWORD wAlarmSts;

  #ifdef  UG31XX_ATTACH_DETECTION
  if(upi_ug31xx_is_attach() == false)
  {
    GLOGE("[%s]: No gauge IC attached.\n", __func__);
    return;
  }
  #endif  ///< end of UG31XX_ATTACH_DETECTION

  cancel_delayed_work_sync(&upi_ug31xx->power_change_work);
  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);

  wake_lock(&upi_ug31xx->info_update_wake_lock);

  GMainTask();

  SmbReadCommand(SBS_ALARM);
  wAlarmSts = (GWORD)(*(SBSCmd->pbData + 1));
  wAlarmSts = (wAlarmSts << 8) | (*(SBSCmd->pbData));
  GLOGE("[%s]: Alarm Status = %04x\n", __func__, wAlarmSts);

  wake_unlock(&upi_ug31xx->info_update_wake_lock);

  schedule_delayed_work(&upi_ug31xx->info_update_work, 0*HZ);
  schedule_delayed_work(&upi_ug31xx->power_change_work, 0*HZ);
}
EXPORT_SYMBOL(upi_ug31xx_alarm_callback);

#endif  ///< end of UG31XX_ALARM_FUNCTION

/// ===============================================================================================
/// Board Offset Calibration
/// ===============================================================================================

#ifdef  UG31XX_BOARD_OFFSET_CALIBRATION

static void kbo_start(void)
{
  stop_charging();

  schedule_delayed_work(&upi_ug31xx->kbo_data.work, 1*HZ);
}

static void kbo_stop(void)
{
  cancel_delayed_work_sync(&upi_ug31xx->kbo_data.work);

  start_charging();
}

static bool kbo_chk_charging_stop(void)
{
  bool rtn;

  /// [AT-PM] : Return true if charging is stopped ; 04/01/2015
  rtn = true;

  return (rtn);
}

static int kbo_save_to_eeprom(void)
{
  GWORD wResult;
  GBYTE buf[UPI_UG31XX_KBO_EEPROM_SIZE];
  u8 idx;
  u8 addr;
  int rtn;
  int retry;

  wResult = 0;
  wResult = wResult + upi_ug31xx->kbo_data.result;

  buf[0] = 0x46;      ///< [AT-PM] : 'F' ; 04/01/2015
  buf[1] = wResult % 256;
  buf[2] = wResult / 256;
  buf[3] = buf[0] + buf[1] + buf[2];

  idx = 0;
  addr = UPI_UG31XX_KBO_EEPROM_ADDR;
  while(1)
  {
    retry = 0;
    while(1)
    {
      rtn = write_eeprom(addr, buf[idx]);
      if(rtn != 0)
      {
        GLOGE("[%s]: Write EEPROM fail. (%d)\n", __func__, rtn);
        return (UPI_UG31XX_KBO_RTN_WT_EEP_FAIL);
      }

      rtn = read_eeprom(addr);
      if(rtn == buf[idx])
      {
        break;
      }

      retry = retry + 1;
      if(retry > UPI_UG31XX_KBO_CMP_EEP_RETRY)
      {
        GLOGE("[%s]: Check EEPROM written data %d fail. (%02x != %02x)\n", __func__, idx, rtn, buf[idx]);
        return (UPI_UG31XX_KBO_RTN_CMP_EEP_FAIL);
      }
    }

    idx = idx + 1;
    addr = addr + 1;
    if(idx >= UPI_UG31XX_KBO_EEPROM_SIZE)
    {
      break;
    }
  }
  return (UPI_UG31XX_KBO_RTN_PASS);
}

#endif  ///< end of UG31XX_BOARD_OFFSET_CALIBRATION

static void upi_ug31xx_kbo_work(struct work_struct *work)
{
  GWORD wData;

  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);

  wake_lock(&upi_ug31xx->info_update_wake_lock);

  /// [AT-PM] : Reset default board offset ; 03/25/2015
  GGB->cCalibration.scCaliData.sAdc1PosOffset = 0;

  /// [AT-PM] : Initialize kbo data ; 03/25/2015
  upi_ug31xx->kbo_data.result       = 0;
  upi_ug31xx->kbo_data.result_sum   = 0;
  upi_ug31xx->kbo_data.pass_cnt     = 0;
  upi_ug31xx->kbo_data.fail_cnt     = 0;
  upi_ug31xx->kbo_data.current_now  = 0;

  /// [AT-PM] : Board offset calibration ; 03/25/2015
  while(1)
  {
    GMainTask();

    /// [AT-PM] : Get current ; 03/25/2015
    SmbReadCommand(SBS_CURR);
    wData = (GWORD)(*(SBSCmd->pbData + 1));
    wData = (wData << 8) | (*SBSCmd->pbData);
    upi_ug31xx->kbo_data.current_now = wData;
    if(upi_ug31xx->kbo_data.current_now > 32767)
    {
      upi_ug31xx->kbo_data.current_now = upi_ug31xx->kbo_data.current_now - 65536;
    }

    /// [AT-PM] : Get the board offset ; 03/25/2015
    if(upi_ug31xx->kbo_data.current_now > UPI_UG31XX_KBO_UPPER_BOUND)
    {
      upi_ug31xx->kbo_data.fail_cnt = upi_ug31xx->kbo_data.fail_cnt + 1;
    }
    else if(upi_ug31xx->kbo_data.current_now < UPI_UG31XX_KBO_LOWER_BOUND)
    {
      upi_ug31xx->kbo_data.fail_cnt = upi_ug31xx->kbo_data.fail_cnt + 1;
    }
    else
    {
      upi_ug31xx->kbo_data.pass_cnt = upi_ug31xx->kbo_data.pass_cnt + 1;

      upi_ug31xx->kbo_data.result_sum = upi_ug31xx->kbo_data.result_sum + upi_ug31xx->kbo_data.current_now;
      upi_ug31xx->kbo_data.result = upi_ug31xx->kbo_data.result_sum / upi_ug31xx->kbo_data.pass_cnt;
      GLOGE("kbo result = %d (%d / %d)\n", upi_ug31xx->kbo_data.result, upi_ug31xx->kbo_data.result_sum, upi_ug31xx->kbo_data.pass_cnt);
    }

    /// [AT-PM] : Check the calibration is finished ; 03/25/2015
    if(upi_ug31xx->kbo_data.pass_cnt >= UPI_UG31XX_KBO_PASS_CNT)
    {
      GGB->cCalibration.scCaliData.sAdc1PosOffset = (GSHORT)upi_ug31xx->kbo_data.result;
      GLOGE("Set board offset = %d\n", GGB->cCalibration.scCaliData.sAdc1PosOffset);
      break;
    }
    if(upi_ug31xx->kbo_data.fail_cnt >= UPI_UG31XX_KBO_FAIL_CNT)
    {
      break;
    }

    /// [AT-PM] : Wait for next round ; 03/25/2015
    msleep(UPI_UG31XX_KBO_INTERVAL_MS);
  }

  wake_unlock(&upi_ug31xx->info_update_wake_lock);

  schedule_delayed_work(&upi_ug31xx->info_update_work, 0*HZ);
}

/// ===============================================================================================
/// PROC
/// ===============================================================================================

#ifdef  UG31XX_REGISTER_PROC

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)

static int ug31xx_get_proc_kbo_start(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
  int len;

  #ifdef  UG31XX_BOARD_OFFSET_CALIBRATION
  kbo_start();
  #endif  ///< end of UG31XX_BOARD_OFFSET_CALIBRATION

  len = 0;
  len += sprintf(buf + len, "Start board offset calibration.\n");
  return (len);
}

static int ug31xx_get_proc_kbo_result(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
  int len;

  len = 0;
  len += sprintf(buf + len, "Board offset = %d ", upi_ug31xx->kbo_data.result);
  if(upi_ug31xx->kbo_data.pass_cnt >= UPI_UG31XX_KBO_PASS_CNT)
  {
    len += sprintf(buf + len, "(PASS)\n");
  }
  else if(upi_ug31xx->kbo_data.fail_cnt >= UPI_UG31XX_KBO_FAIL_CNT)
  {
    len += sprintf(buf + len, "(FAIL)\n");
  }
  else
  {
    len += sprintf(buf + len, "(RUNNING)\n");
  }
  return (len);
}

static int ug31xx_get_proc_kbo_stop(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
  int len;

  #ifdef  UG31XX_BOARD_OFFSET_CALIBRATION
  kbo_stop();
  #endif  ///< end of UG31XX_BOARD_OFFSET_CALIBRATION

  len = 0;
  len += sprintf(buf + len, "Stop board offset calibration.\n");
  return (len);
}

static int ug31xx_get_proc_kbo_full(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
  int len;
  int timeout;
  int err;

  len = 0;
  err = UPI_UG31XX_KBO_RTN_PASS;

  #ifdef  UG31XX_BOARD_OFFSET_CALIBRATION
  /// [AT-PM] : Polling for charging stopped from hardware ; 04/01/2015
  timeout = 0;
  while(1)
  {
    if(kbo_chk_charging_stop() == true)
    {
      break;
    }

    timeout = timeout + 1;
    if(timeout > UPI_UG31XX_KBO_CHK_TIMEOUT)
    {
      err = UPI_UG31XX_KBO_RTN_CHK_TIMEOUT;
      break;
    }

    /// [AT-PM] : Wait for next round ; 03/25/2015
    msleep(UPI_UG31XX_KBO_CHK_TIME_MS);
  }
  if(err != UPI_UG31XX_KBO_RTN_PASS)
  {
    goto kbo_full_exit;
  }

  /// [AT-PM] : Start calibration and wait for finished ; 04/01/2015
  upi_ug31xx_kbo_work(&upi_ug31xx->kbo_data.work.work);

  /// [AT-PM] : Check result range ; 04/01/2015
  if(upi_ug31xx->kbo_data.pass_cnt >= UPI_UG31XX_KBO_PASS_CNT)
  {
    err = UPI_UG31XX_KBO_RTN_PASS;
  }
  else if(upi_ug31xx->kbo_data.fail_cnt >= UPI_UG31XX_KBO_FAIL_CNT)
  {
    err = UPI_UG31XX_KBO_RTN_OUT_OF_RANGE;
  }
  else
  {
    err = UPI_UG31XX_KBO_RTN_UNKNOWN;
  }
  if(err != UPI_UG31XX_KBO_RTN_PASS)
  {
    goto kbo_full_exit;
  }

  /// [AT-PM] : Save result to EEPROM ; 04/01/2015
  err = kbo_save_to_eeprom();
  if(err != UPI_UG31XX_KBO_RTN_PASS)
  {
    goto kbo_full_exit;
  }

kbo_full_exit:
  #endif  ///< end of UG31XX_BOARD_OFFSET_CALIBRATION

  len += sprintf(buf + len, "Board offset = %d ", upi_ug31xx->kbo_data.result);
  if(err == UPI_UG31XX_KBO_RTN_PASS)
  {
    len += sprintf(buf + len, "(PASS)\n");
  }
  else
  {
    len += sprintf(buf + len, "(FAIL:%d)\n", err);
  }
  return (len);
}

static int ug31xx_get_proc_upi_gauge_toggle(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
  int len;

  #ifdef  UG31XX_ATTACH_DETECTION
  gauge_attached = (gauge_attached == 0) ? 1 : 0;
  upi_ug31xx_attach((gauge_attached == 0) ? false : true);
  #endif  ///< end of UG31XX_ATTACH_DETECTION

  len = 0;
  len += sprintf(buf + len, "Gauge %s.\n", (gauge_attached == 0) ? "removed" : "attached");
  return (len);
}

static int ug31xx_get_proc_upi_gauge_attached(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
  int len;

  len = 0;
  len += sprintf(buf + len, "Gauge %s.\n", (gauge_attached == 0) ? "removed" : "attached");
  return (len);
}

static int ug31xx_get_proc_reset_capacity(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
  int len;
  GBYTE buf[2];

  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);

  /// [AT-PM] : Reset capacity status ; 04/22/2015
  buf[0] = SYS_MODE_CMD_FULL_DEBUG_MSG % 256;
  buf[1] = SYS_MODE_CMD_FULL_DEBUG_MSG / 256;
  SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);
    
  buf[0] = SYS_MODE_CMD_INIT_CAP_CLR % 256;
  buf[1] = SYS_MODE_CMD_INIT_CAP_CLR / 256;
  SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);

  buf[0] = SYS_MODE_CMD_RESTORED_CLR % 256;
  buf[1] = SYS_MODE_CMD_RESTORED_CLR / 256;
  SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);

  stop_charging();
  upi_ug31xx->charging_stopped = true;
  schedule_delayed_work(&upi_ug31xx->info_update_work, 2*HZ);

  len = 0;
  len += sprintf(buf + len, "Reset capacity done in 2 seconds.\n");
  return (len);
}

static int ug31xx_create_proc(void)
{
  struct proc_dir_entry *ent;

  ent = create_proc_read_entry("kbo_start", 0744, NULL, ug31xx_get_proc_kbo_start, NULL);
  if(!ent)
  {
    GLOGE("create /proc/kbo_start fail\n");
  }
  ent = create_proc_read_entry("kbo_result", 0744, NULL, ug31xx_get_proc_kbo_result, NULL);
  if(!ent)
  {
    GLOGE("create /proc/kbo_result fail\n");
  }
  ent = create_proc_read_entry("kbo_stop", 0744, NULL, ug31xx_get_proc_kbo_stop, NULL);
  if(!ent)
  {
    GLOGE("create /proc/kbo_stop fail\n");
  }
  ent = create_proc_read_entry("kbo_full", 0744, NULL, ug31xx_get_proc_kbo_full, NULL);
  if(!ent)
  {
    GLOGE("create /proc/kbo_full fail\n");
  }
  ent = create_proc_read_entry("upi_gauge_toggle", 0744, NULL, ug31xx_get_proc_kbo_full, NULL);
  if(!ent)
  {
    GLOGE("create /proc/upi_gauge_toggle fail\n");
  }
  ent = create_proc_read_entry("upi_gauge_attached", 0744, NULL, ug31xx_get_proc_kbo_full, NULL);
  if(!ent)
  {
    GLOGE("create /proc/upi_gauge_attached fail\n");
  }
  ent = create_proc_read_entry("upi_reset_capacity", 0744, NULL, ug31xx_get_proc_reset_capacity, NULL);
  if(!ent)
  {
    GLOGE("create /proc/upi_reset_capacity fail\n");
  }
  return (0);
}

#else   /// else of LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)

static ssize_t ug31xx_proc_write(struct file *file, const char __user *buf, size_t count, loff_t * ppos)
{
  GLOGE("[%s]\n", __func__);
  return (count);
}

static int ug31xx_get_proc_kbo_start(struct seq_file *m, void *v)
{
  #ifdef  UG31XX_BOARD_OFFSET_CALIBRATION
  kbo_start();
  #endif  ///< end of UG31XX_BOARD_OFFSET_CALIBRATION

  seq_printf(m, "Start board offset calibration.\n");
  return (0);
}

static int ug31xx_get_proc_kbo_start_open(struct inode *inode, struct file *file)
{
  return (single_open(file, ug31xx_get_proc_kbo_start, NULL));
}

static const struct file_operations ug31xx_get_proc_kbo_start_fops = {
  .owner = THIS_MODULE,
  .open = ug31xx_get_proc_kbo_start_open,
  .read = seq_read,
  .write = ug31xx_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ug31xx_get_proc_kbo_result(struct seq_file *m, void *v)
{
  if(upi_ug31xx->kbo_data.pass_cnt >= UPI_UG31XX_KBO_PASS_CNT)
  {
    seq_printf(m, "Board offset = %d (PASS)\n", upi_ug31xx->kbo_data.result);
  }
  else if(upi_ug31xx->kbo_data.fail_cnt >= UPI_UG31XX_KBO_FAIL_CNT)
  {
    seq_printf(m, "Board offset = %d (FAIL)\n", upi_ug31xx->kbo_data.result);
  }
  else
  {
    seq_printf(m, "Board offset = %d (RUNNING)\n", upi_ug31xx->kbo_data.result);
  }
  return (0);
}

static int ug31xx_get_proc_kbo_result_open(struct inode *inode, struct file *file)
{
  return (single_open(file, ug31xx_get_proc_kbo_result, NULL));
}

static const struct file_operations ug31xx_get_proc_kbo_result_fops = {
  .owner = THIS_MODULE,
  .open = ug31xx_get_proc_kbo_result_open,
  .read = seq_read,
  .write = ug31xx_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ug31xx_get_proc_kbo_stop(struct seq_file *m, void *v)
{
  #ifdef  UG31XX_BOARD_OFFSET_CALIBRATION
  kbo_stop();
  #endif  ///< end of UG31XX_BOARD_OFFSET_CALIBRATION

  seq_printf(m, "Stop board offset calibration.\n");
  return (0);
}

static int ug31xx_get_proc_kbo_stop_open(struct inode *inode, struct file *file)
{
  return (single_open(file, ug31xx_get_proc_kbo_stop, NULL));
}

static const struct file_operations ug31xx_get_proc_kbo_stop_fops = {
  .owner = THIS_MODULE,
  .open = ug31xx_get_proc_kbo_stop_open,
  .read = seq_read,
  .write = ug31xx_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ug31xx_get_proc_kbo_full(struct seq_file *m, void *v)
{
  int timeout;
  int err;

  err = UPI_UG31XX_KBO_RTN_PASS;

  #ifdef  UG31XX_BOARD_OFFSET_CALIBRATION
  /// [AT-PM] : Polling for charging stopped from hardware ; 04/01/2015
  timeout = 0;
  while(1)
  {
    if(kbo_chk_charging_stop() == true)
    {
      break;
    }

    timeout = timeout + 1;
    if(timeout > UPI_UG31XX_KBO_CHK_TIMEOUT)
    {
      err = UPI_UG31XX_KBO_RTN_CHK_TIMEOUT;
      break;
    }

    /// [AT-PM] : Wait for next round ; 03/25/2015
    msleep(UPI_UG31XX_KBO_CHK_TIME_MS);
  }
  if(err != UPI_UG31XX_KBO_RTN_PASS)
  {
    goto kbo_full_exit;
  }

  /// [AT-PM] : Start calibration and wait for finished ; 04/01/2015
  upi_ug31xx_kbo_work(&upi_ug31xx->kbo_data.work.work);

  /// [AT-PM] : Check result range ; 04/01/2015
  if(upi_ug31xx->kbo_data.pass_cnt >= UPI_UG31XX_KBO_PASS_CNT)
  {
    err = UPI_UG31XX_KBO_RTN_PASS;
  }
  else if(upi_ug31xx->kbo_data.fail_cnt >= UPI_UG31XX_KBO_FAIL_CNT)
  {
    err = UPI_UG31XX_KBO_RTN_OUT_OF_RANGE;
  }
  else
  {
    err = UPI_UG31XX_KBO_RTN_UNKNOWN;
  }
  if(err != UPI_UG31XX_KBO_RTN_PASS)
  {
    goto kbo_full_exit;
  }

  /// [AT-PM] : Save result to EEPROM ; 04/01/2015
  err = kbo_save_to_eeprom();
  if(err != UPI_UG31XX_KBO_RTN_PASS)
  {
    goto kbo_full_exit;
  }

kbo_full_exit:
  #endif  ///< end of UG31XX_BOARD_OFFSET_CALIBRATION

  if(err == UPI_UG31XX_KBO_RTN_PASS)
  {
    seq_printf(m, "Board offset = %d (PASS)\n", upi_ug31xx->kbo_data.result);
  }
  else
  {
    seq_printf(m, "Board offset = %d (FAIL:%d)\n", upi_ug31xx->kbo_data.result, err);
  }
  return (0);
}

static int ug31xx_get_proc_kbo_full_open(struct inode *inode, struct file *file)
{
  return (single_open(file, ug31xx_get_proc_kbo_full, NULL));
}

static const struct file_operations ug31xx_get_proc_kbo_full_fops = {
  .owner = THIS_MODULE,
  .open = ug31xx_get_proc_kbo_full_open,
  .read = seq_read,
  .write = ug31xx_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ug31xx_get_proc_upi_gauge_toggle(struct seq_file *m, void *v)
{

  #ifdef  UG31XX_ATTACH_DETECTION
  gauge_attached = (gauge_attached == 0) ? 1 : 0;
  upi_ug31xx_attach((gauge_attached == 0) ? false : true);
  #endif  ///< end of UG31XX_ATTACH_DETECTION

  seq_printf(m, "Gauge %s.\n", (gauge_attached == 0) ? "removed" : "attached");
  return (0);
}

static int ug31xx_get_proc_upi_gauge_toggle_open(struct inode *inode, struct file *file)
{
  return (single_open(file, ug31xx_get_proc_upi_gauge_toggle, NULL));
}

static const struct file_operations ug31xx_get_proc_upi_gauge_toggle_fops = {
  .owner = THIS_MODULE,
  .open = ug31xx_get_proc_upi_gauge_toggle_open,
  .read = seq_read,
  .write = ug31xx_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ug31xx_get_proc_upi_gauge_attached(struct seq_file *m, void *v)
{
  seq_printf(m, "Gauge %s.\n", (gauge_attached == 0) ? "removed" : "attached");
  return (0);
}

static int ug31xx_get_proc_upi_gauge_attached_open(struct inode *inode, struct file *file)
{
  return (single_open(file, ug31xx_get_proc_upi_gauge_attached, NULL));
}

static const struct file_operations ug31xx_get_proc_upi_gauge_attached_fops = {
  .owner = THIS_MODULE,
  .open = ug31xx_get_proc_upi_gauge_attached_open,
  .read = seq_read,
  .write = ug31xx_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ug31xx_get_proc_reset_capacity(struct seq_file *m, void *v)
{
  GBYTE buf[2];

  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);

  /// [YL] : Set debug level ; 20150907
  if(debug_level == 0)
  {
    buf[0] = SYS_MODE_CMD_SIMPLE_DEBUG_MSG % 256;
    buf[1] = SYS_MODE_CMD_SIMPLE_DEBUG_MSG / 256;
    SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);     
  }
  else 
  {
    buf[0] = SYS_MODE_CMD_FULL_DEBUG_MSG % 256;
    buf[1] = SYS_MODE_CMD_FULL_DEBUG_MSG / 256;
    SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);
  }
  buf[0] = SYS_MODE_CMD_INIT_CAP_CLR % 256;
  buf[1] = SYS_MODE_CMD_INIT_CAP_CLR / 256;
  SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);

  buf[0] = SYS_MODE_CMD_RESTORED_CLR % 256;
  buf[1] = SYS_MODE_CMD_RESTORED_CLR / 256;
  SmbWriteCommand(SBS_SYS_MODE, &buf[0], 2);

  stop_charging();
  upi_ug31xx->charging_stopped = true;
  schedule_delayed_work(&upi_ug31xx->info_update_work, 2*HZ);

  seq_printf(m, "Reset capacity done in 2 seconds!\n");
  return (0);
}

static int ug31xx_get_proc_reset_capacity_open(struct inode *inode, struct file *file)
{
  return (single_open(file, ug31xx_get_proc_reset_capacity, NULL));
}

static const struct file_operations ug31xx_get_proc_reset_capacity_fops = {
  .owner = THIS_MODULE,
  .open = ug31xx_get_proc_reset_capacity_open,
  .read = seq_read,
  .write = ug31xx_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ug31xx_create_proc(void)
{
  struct proc_dir_entry *ent;

  ent = proc_create("kbo_start", 0744, NULL, &ug31xx_get_proc_kbo_start_fops);
  if(!ent)
  {
    GLOGE("create /proc/kbo_start fail\n");
  }
  ent = proc_create("kbo_result", 0744, NULL, &ug31xx_get_proc_kbo_result_fops);
  if(!ent)
  {
    GLOGE("create /proc/kbo_result fail\n");
  }
  ent = proc_create("kbo_stop", 0744, NULL, &ug31xx_get_proc_kbo_stop_fops);
  if(!ent)
  {
    GLOGE("create /proc/kbo_stop fail\n");
  }
  ent = proc_create("kbo_full", 0744, NULL, &ug31xx_get_proc_kbo_full_fops);
  if(!ent)
  {
    GLOGE("create /proc/kbo_full fail\n");
  }
  ent = proc_create("upi_gauge_toggle", 0744, NULL, &ug31xx_get_proc_upi_gauge_toggle_fops);
  if(!ent)
  {
    GLOGE("create /proc/upi_gauge_toggle fail\n");
  }
  ent = proc_create("upi_gauge_attached", 0744, NULL, &ug31xx_get_proc_upi_gauge_attached_fops);
  if(!ent)
  {
    GLOGE("create /proc/upi_gauge_attached fail\n");
  }
  ent = proc_create("upi_reset_capacity", 0744, NULL, &ug31xx_get_proc_reset_capacity_fops);
  if(!ent)
  {
    GLOGE("create /proc/upi_reset_capacity fail\n");
  }
  return (0);
}

#endif  ///< end of LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)

#endif  ///< end of UG31XX_REGISTER_PROC

/// ===============================================================================================
/// IOCTL
/// ===============================================================================================

static int upi_ug31xx_misc_open(struct inode *inode, struct file *file)
{
  GLOGE("[%s]\n", __func__);
  return (0);
}

static int upi_ug31xx_misc_release(struct inode *inode, struct file *file)
{
  GLOGE("[%s]\n", __func__);
  return (0);
}

#define UG31XX_IOCTL_BUF_SIZE   (512)

static long upi_ug31xx_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  int rtn = 0;
  GBYTE buf[UG31XX_IOCTL_BUF_SIZE];

  if(_IOC_TYPE(cmd) != UPI_UG31XX_IOC_MAGIC)
  {
    GLOGE("[%s]: Invaild type (%x)\n", __func__,
          cmd);
    return (-ENOTTY);
  }

  memset(buf, 0, UG31XX_IOCTL_BUF_SIZE);

  switch(UPI_UG31XX_IOC_CMD(cmd))
  {
    case UPI_UG31XX_IOC_CMD(UPI_UG31XX_IOC_SBS_RW):
      if(copy_from_user((void *)&buf[0], (void __user *)arg, sizeof(GBYTE)))
      {
        GLOGE("[%s]: copy_from_user fail\n", __func__);
        return (-EINVAL);
      }

      buf[1] = SmbReadCommand(buf[0]);
      memcpy(&buf[2], SBSCmd->pbData, 2);

      GLOGE("[%s]: UPI_UG31XX_IOC_SBS_RW command (%x) : %02x %02x %02x%02x\n", __func__,
            cmd,
            buf[0],     ///< [AT-PM] : SBS Command ; 07/11/2014
            buf[1],     ///< [AT-PM] : Data Length ; 07/11/2014
            buf[3],
            buf[2]);

      if(copy_to_user((void __user *)arg, (void *)&buf[0], sizeof(GBYTE) * (buf[1] + 2)))
      {
        GLOGE("[%s]: copy_to_user fail\n", __func__);
        return (-EINVAL);
      }
      break;
    case UPI_UG31XX_IOC_CMD(UPI_UG31XX_IOC_SBS_WW):
      if(copy_from_user((void *)&buf[0], (void __user *)arg, sizeof(GBYTE) * 4))
      {
        GLOGE("[%s]: copy_from_user fail\n", __func__);
        return (-EINVAL);
      }

      SmbWriteCommand(buf[0], &buf[2], 2);

      GLOGE("[%s]: UPI_UG31XX_IOC_SBS_WW command (%x) : %02x %02x %02x%02x\n", __func__,
            cmd,
            buf[0],     ///< [AT-PM] : SBS Command ; 07/11/2014
            buf[1],     ///< [AT-PM] : Data Length ; 07/11/2014
            buf[3],
            buf[2]);
      break;
    case UPI_UG31XX_IOC_CMD(UPI_UG31XX_IOC_SBS_RP):
      if(copy_from_user((void *)&buf[0], (void __user *)arg, sizeof(GBYTE)))
      {
        GLOGE("[%s]: copy_from_user fail\n", __func__);
        return (-EINVAL);
      }

      buf[1] = SmbReadCommand(buf[0]);
      memcpy(&buf[2], SBSCmd->pbData, buf[1]);

      GLOGE("[%s]: UPI_UG31XX_IOC_SBS_RP command (%x) : %02x %02x\n", __func__,
            cmd,
            buf[0],     ///< [AT-PM] : SBS Command ; 07/11/2014
            buf[1]);    ///< [AT-PM] : Data Length ; 07/11/2014

      if(copy_to_user((void __user *)arg, (void *)&buf[0], sizeof(GBYTE) * (buf[1] + 2)))
      {
        GLOGE("[%s]: copy_to_user fail\n", __func__);
        return (-EINVAL);
      }
      break;
    case UPI_UG31XX_IOC_CMD(UPI_UG31XX_IOC_SBS_WP):
      if(copy_from_user((void *)&buf[0], (void __user *)arg, sizeof(GBYTE) * 257))
      {
        GLOGE("[%s]: copy_from_user fail\n", __func__);
        return (-EINVAL);
      }

      SmbWriteCommand(buf[0], &buf[2], buf[1]);

      GLOGE("[%s]: UPI_UG31XX_IOC_SBS_WP command (%x) : %02x %02x\n", __func__,
            cmd,
            buf[0],     ///< [AT-PM] : SBS Command ; 07/11/2014
            buf[1]);    ///< [AT-PM] : Data Length ; 07/11/2014
      break;
    default:
      GLOGE("[%s]: Un-known ioctl command (%x)\n", __func__,
            cmd);
      break;
  }

  return (rtn);
}

static struct file_operations upi_ug31xx_fops = {
  .owner          = THIS_MODULE,
  .open           = upi_ug31xx_misc_open,
  .release        = upi_ug31xx_misc_release,
  .unlocked_ioctl = upi_ug31xx_misc_ioctl,
  .compat_ioctl   = upi_ug31xx_misc_ioctl,
};

struct miscdevice upi_ug31xx_misc_dev = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = UPI_UG31XX_IOC_NAME,
  .fops  = &upi_ug31xx_fops
};

/// ===============================================================================================
/// Register power supply class
/// ===============================================================================================

#ifdef  UG31XX_REGISTER_POWER_SUPPLY

static enum power_supply_property upi_ug31xx_batt_props[] = {
  POWER_SUPPLY_PROP_STATUS,
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  POWER_SUPPLY_PROP_PACK_TYPE,
#endif
  POWER_SUPPLY_PROP_PRESENT,
  POWER_SUPPLY_PROP_VOLTAGE_NOW,
  POWER_SUPPLY_PROP_CURRENT_NOW,
  POWER_SUPPLY_PROP_CAPACITY,
  POWER_SUPPLY_PROP_TEMP,
  POWER_SUPPLY_PROP_CHARGE_NOW,
  POWER_SUPPLY_PROP_CHARGE_FULL,
  POWER_SUPPLY_PROP_MANUFACTURER,
  POWER_SUPPLY_PROP_TECHNOLOGY,
};

static bool upi_ug31xx_check_low_voltage(void)
{
  GWORD wData;

  #ifdef  UG31XX_ATTACH_DETECTION
  if(gauge_attached == 1)
  {
    SmbReadCommand(SBS_ERROR_STATUS);
    wData = (GWORD)(*(SBSCmd->pbData + 1));
    wData = (wData << 8) | (*SBSCmd->pbData);
  }
  #else   ///< else of UG31XX_ATTACH_DETECTION
  SmbReadCommand(SBS_ERROR_STATUS);
  wData = (GWORD)(*(SBSCmd->pbData + 1));
  wData = (wData << 8) | (*SBSCmd->pbData);
  #endif  ///< end of UG31XX_ATTACH_DETECTION
  return ((wData & GAUGE_ERR_LV_FAIL) ? true : false);
}

static bool upi_ug31xx_check_driver_ready(void)
{
  GWORD wData;

  #ifdef  UG31XX_ATTACH_DETECTION
  if(gauge_attached == 1)
  {
    SmbReadCommand(SBS_SYS_MODE);
    wData = (GWORD)(*(SBSCmd->pbData + 1));
    wData = (wData << 8) | (*(SBSCmd->pbData));
  }
  else
  {
    wData = 0;
  }
  #else   ///< else of UG31XX_ATTACH_DETECTION
  SmbReadCommand(SBS_SYS_MODE);
  wData = (GWORD)(*(SBSCmd->pbData + 1));
  wData = (wData << 8) | (*(SBSCmd->pbData));
  #endif  ///< end of UG31XX_ATTACH_DETECTION
  return ((wData & SYS_MODE_INIT_CAP) ? true : false);
}

static int upi_ug31xx_battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
  int ret = 0;
  GWORD wData = 0;

  switch (psp)
  {
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
    case POWER_SUPPLY_PROP_PACK_TYPE:
      if (gauge_attached == 0) {
        val->intval = 0;
      }
      else {
        if (IS_CB81())
          val->intval = 1;
        else if (IS_CA81())
          val->intval = 2;
        else
          val->intval = 0;
      }
      break;
#endif
    case POWER_SUPPLY_PROP_PRESENT:
      val->intval = gauge_attached;
      break;
    case POWER_SUPPLY_PROP_STATUS:
      #ifdef  UG31XX_ATTACH_DETECTION
      if(gauge_attached == 1)
      {
        #if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
        SmbReadCommand(SBS_RSOC);
        #else   ///< else of defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
        SmbReadCommand(SBS_BS);
        #endif  ///< end of defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
      }
      #else   ///< else of UG31XX_ATTACH_DETECTION
      #if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
      SmbReadCommand(SBS_RSOC);
      #else   ///< else of defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
      SmbReadCommand(SBS_BS);
      #endif  ///< end of defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      #endif  ///< end of UG31XX_ATTACH_DETECTION

#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
      if(wData == 100)
      {
        val->intval = POWER_SUPPLY_STATUS_FULL;
        GLOGD("[%s]: POWER_SUPPLY_PROP_STATUS: POWER_SUPPLY_STATUS_FULL\n", __func__);
      }
      else
      {
        val->intval = smb345c_get_charging_status();
      }
#else
      if(wData & FC_BS)
      {
        val->intval = POWER_SUPPLY_STATUS_FULL;
        GLOGD("[%s]: POWER_SUPPLY_PROP_STATUS: POWER_SUPPLY_STATUS_FULL\n", __func__);
      }
      else if(upi_ug31xx->curr_cable_status == UPI_UG31XX_CABLE_IN)
      {
        val->intval = POWER_SUPPLY_STATUS_CHARGING;
        GLOGD("[%s]: POWER_SUPPLY_PROP_STATUS: POWER_SUPPLY_STATUS_CHARGING\n", __func__);
      }
      else
      {
        val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        GLOGD("[%s]: POWER_SUPPLY_PROP_STATUS: POWER_SUPPLY_STATUS_DISCHARGING\n", __func__);
      }
#endif
      break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
      #ifdef  UG31XX_ATTACH_DETECTION
      if(gauge_attached == 1)
      {
        SmbReadCommand(SBS_VOLT);
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
      }
      #else   ///< else of UG31XX_ATTACH_DETECTION
      SmbReadCommand(SBS_VOLT);
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      #endif  ///< end of UG31XX_ATTACH_DETECTION
      val->intval = wData;
      GLOGD("[%s]: POWER_SUPPLY_PROP_VOLTAGE_NOW: %d\n", __func__, val->intval);
      break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
      #ifdef  UG31XX_ATTACH_DETECTION
      if(gauge_attached == 1)
      {
        SmbReadCommand(SBS_CURR);
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
      }
      #else   ///< else of UG31XX_ATTACH_DETECTION
      SmbReadCommand(SBS_CURR);
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      #endif  ///< end of UG31XX_ATTACH_DETECTION
      val->intval = wData;
      if(val->intval > 32767)
      {
        val->intval = val->intval - 65536;
      }
      GLOGD("[%s]: POWER_SUPPLY_PROP_CURRENT_NOW: %d\n", __func__, val->intval);
      break;
    case POWER_SUPPLY_PROP_CAPACITY:
      #ifdef  UG31XX_ATTACH_DETECTION
      if(gauge_attached == 1)
      {
        SmbReadCommand(SBS_RSOC);
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
      }
      #else   ///< else of UG31XX_ATTACH_DETECTION
      SmbReadCommand(SBS_RSOC);
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      #endif  ///< end of UG31XX_ATTACH_DETECTION
      if(upi_ug31xx_check_driver_ready() == true)
      {
        val->intval = (upi_ug31xx_check_low_voltage() == true) ? 0 : wData;
      }
      else
      {
        val->intval = UG31XX_RSOC_VALUE_IF_DRV_NOT_READY;
      }
      GLOGD("[%s]: POWER_SUPPLY_PROP_CAPACITY: %d\n", __func__, val->intval);
      break;
    case POWER_SUPPLY_PROP_TEMP:
      #ifdef  UG31XX_ATTACH_DETECTION
      if(gauge_attached == 1)
      {
        SmbReadCommand(SBS_TEMP);
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
      }
      #else   ///< else of UG31XX_ATTACH_DETECTION
      SmbReadCommand(SBS_TEMP);
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      #endif  ///< end of UG31XX_ATTACH_DETECTION
      val->intval = wData;
      val->intval = val->intval - SBS_TEMP_C_2_K;
      GLOGD("[%s]: POWER_SUPPLY_PROP_TEMP: %d\n", __func__, val->intval);
      break;
    case POWER_SUPPLY_PROP_CHARGE_NOW:
      #ifdef  UG31XX_ATTACH_DETECTION
      if(gauge_attached == 1)
      {
        SmbReadCommand(SBS_RM);
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
      }
      #else   ///< else of UG31XX_ATTACH_DETECTION
      SmbReadCommand(SBS_RM);
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      #endif  ///< end of UG31XX_ATTACH_DETECTION
      if(upi_ug31xx_check_driver_ready() == true)
      {
        val->intval = (upi_ug31xx_check_low_voltage() == true) ? 0 : wData;
      }
      else
      {
        val->intval = -1;
      }
      GLOGD("[%s]: POWER_SUPPLY_PROP_CHARGE_NOW: %d\n", __func__, val->intval);
      break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
      #ifdef  UG31XX_ATTACH_DETECTION
      if(gauge_attached == 1)
      {
        SmbReadCommand(SBS_FCC);
        wData = (GWORD)(*(SBSCmd->pbData + 1));
        wData = (wData << 8) | (*SBSCmd->pbData);
      }
      #else   ///< else of UG31XX_ATTACH_DETECTION
      SmbReadCommand(SBS_FCC);
      wData = (GWORD)(*(SBSCmd->pbData + 1));
      wData = (wData << 8) | (*SBSCmd->pbData);
      #endif  ///< end of UG31XX_ATTACH_DETECTION
      val->intval = wData;
      GLOGD("[%s]: POWER_SUPPLY_PROP_CHARGE_FULL: %d\n", __func__, val->intval);
      break;
    case POWER_SUPPLY_PROP_MANUFACTURER:
#ifdef  FEATURE_BUFFER_ENABLE
      val->strval = (char *)DebugLogBuf;
#else   ///< else of FEATURE_BUFFER_ENABLE
      val->strval = "NSoftLab";
#endif  ///< end of FEATURE_BUFFER_ENABLE
      break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
      val->intval = (upi_ug31xx_check_driver_ready() == true) ? POWER_SUPPLY_TECHNOLOGY_LION : POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
      break;
    default:
      ret = -EINVAL;
      break;
  }

  return (ret);
}

static void upi_ug31xx_battery_external_power_changed(struct power_supply *psy)
{
  GBYTE  buf[2];
  upi_ug31xx->prev_cable_status = upi_ug31xx->curr_cable_status;
  if(power_supply_am_i_supplied(psy))
  {
    upi_ug31xx->curr_cable_status = UPI_UG31XX_CABLE_IN;
    buf[0] = RSOC_FREE_RUN_CMD_MFA % 256;
    buf[1] = RSOC_FREE_RUN_CMD_MFA / 256;
    if(gauge_attached  == 1)
    {
      SmbWriteCommand(SBS_MFA, &buf[0], 2);
    }
  }
  else
  {
    upi_ug31xx->curr_cable_status = UPI_UG31XX_CABLE_OUT;
    buf[0] = RSOC_KEEP_DEC_CMD_MFA % 256;
    buf[1] = RSOC_KEEP_DEC_CMD_MFA / 256;
    if(gauge_attached  == 1)
    {
      SmbWriteCommand(SBS_MFA, &buf[0], 2);
    }
  }

  if(upi_ug31xx->prev_cable_status != upi_ug31xx->curr_cable_status)
  {
    upi_ug31xx->cable_change_cnt  = UPI_UG31XX_CABLE_POLL_CNT;
    cancel_delayed_work_sync(&upi_ug31xx->info_update_work);
    schedule_delayed_work(&upi_ug31xx->info_update_work, 1*HZ);
  }

  GLOGE("<BATT> ++++++++++++++++++++ cable status changed %d -> %d (%d) ++++++++++++++++++++\n",
        upi_ug31xx->prev_cable_status,
        upi_ug31xx->curr_cable_status,
        upi_ug31xx->cable_change_cnt);
}

static struct power_supply upi_ug31xx_supply = {
  .name                   = UPI_UG31XX_POWER_NAME,
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
  .type                   = POWER_SUPPLY_TYPE_PACK_BATTERY,
#else
  .type                   = POWER_SUPPLY_TYPE_BATTERY,
#endif
  .properties             = upi_ug31xx_batt_props,
  .num_properties         = ARRAY_SIZE(upi_ug31xx_batt_props),
  .get_property           = upi_ug31xx_battery_get_property,
  .external_power_changed = upi_ug31xx_battery_external_power_changed,
};

#endif  ///< end of UG31XX_REGISTER_POWER_SUPPLY

static void upi_ug31xx_power_supply_change(struct work_struct *work)
{
  #ifdef  UG31XX_REGISTER_POWER_SUPPLY
  power_supply_changed(&upi_ug31xx_supply);
  #endif  ///< end of UG31XX_REGISTER_POWER_SUPPLY
  schedule_delayed_work(&upi_ug31xx->power_change_work, 60*HZ);
  GLOGE("[%s]: power supply changed.\n", __func__);
}

/// ===============================================================================================
/// Register early_suspend and late_resume
/// ===============================================================================================

#ifdef  UG31XX_REGISTER_EARLY_SUSPEND

static void ug31xx_early_suspend(struct early_suspend *obj)
{
  upi_ug31xx->sec_early_suspend = ug31xx_get_rtc_time(&upi_ug31xx->tm_early_suspend);

  GLOGE("[%s]: Enter early_suspend at %d/%d/%d %d:%d:%d (%d)\n", __func__,
        upi_ug31xx->tm_early_suspend.tm_year + 1900,
        upi_ug31xx->tm_early_suspend.tm_mon + 1,
        upi_ug31xx->tm_early_suspend.tm_mday,
        upi_ug31xx->tm_early_suspend.tm_hour,
        upi_ug31xx->tm_early_suspend.tm_min,
        upi_ug31xx->tm_early_suspend.tm_sec,
        upi_ug31xx->sec_early_suspend);
}

static void ug31xx_late_resume(struct early_suspend *obj)
{
  unsigned long delta_sec;
  GBYTE buf[2];

  upi_ug31xx->sec_late_resume = ug31xx_get_rtc_time(&upi_ug31xx->tm_late_resume);

  GLOGE("[%s]: Enter late_resume at %d/%d/%d %d:%d:%d (%d)\n", __func__,
        upi_ug31xx->tm_late_resume.tm_year + 1900,
        upi_ug31xx->tm_late_resume.tm_mon + 1,
        upi_ug31xx->tm_late_resume.tm_mday,
        upi_ug31xx->tm_late_resume.tm_hour,
        upi_ug31xx->tm_late_resume.tm_min,
        upi_ug31xx->tm_late_resume.tm_sec,
        upi_ug31xx->sec_late_resume);

  delta_sec = upi_ug31xx->sec_late_resume - upi_ug31xx->sec_early_suspend;
  if(delta_sec > UG31XX_CALI_RSOC_TIME)
  {
    buf[0] = CAP_SBS_CMD_RESET % 256;
    buf[1] = CAP_SBS_CMD_RESET / 256;
    SmbWriteCommand(SBS_CAP_DATA, &buf[0], 2);
  }

  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);
  schedule_delayed_work(&upi_ug31xx->info_update_work, 0*HZ);
}

#endif  ///< end of UG31XX_REGISTER_EARLY_SUSPEND

/// ===============================================================================================
/// Register I2C driver
/// ===============================================================================================

static int ug31xx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

  GLOGE("<BATT> ++++++++++++++++++++ %s ++++++++++++++++++++\n", __func__);

  if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
  {
    GLOGE("<BATT> %s: I2C check functionality failed.\n", __func__);
    return (-EIO);
  }

  upi_ug31xx = kzalloc(sizeof(struct upi_ug31xx_st), GFP_KERNEL);
  if (!upi_ug31xx)
  {
    return (-ENOMEM);
  }

  upi_ug31xx->client                   =  client;
  upi_ug31xx->dev                      = &(client->dev);
  upi_ug31xx->sec_delta_suspend_resume =  0;
  upi_ug31xx->in_suspend_cnt           =  false;
  
  i2c_set_clientdata(upi_ug31xx->client, upi_ug31xx);

  INIT_DEFERRABLE_WORK(&upi_ug31xx->info_update_work, upi_ug31xx_main_task);
  INIT_DEFERRABLE_WORK(&upi_ug31xx->power_change_work, upi_ug31xx_power_supply_change);
  INIT_DEFERRABLE_WORK(&upi_ug31xx->kbo_data.work, upi_ug31xx_kbo_work);

  wake_lock_init(&upi_ug31xx->info_update_wake_lock, WAKE_LOCK_SUSPEND, "upi-ug31xx_update");
#if defined(CONFIG_Z380C)
  wake_lock_init(&upi_ug31xx->i2c_wake_lock, WAKE_LOCK_SUSPEND, "upi-i2c-wakelock");
#endif

  #ifdef  UG31XX_ATTACH_DETECTION
  mutex_init(&upi_ug31xx->attach_lock);
  #endif  ///< end of UG31XX_ATTACH_DETECTION

  #ifdef  UG31XX_DELAY_INIT
  /// [AT-PM] : Delay initialization if cover attached at power-on ; 09/04/2015
  atomic_set(&upi_ug31xx->wait_delay_init, 1);
  atomic_set(&upi_ug31xx->wait_delay_state, 0);
  atomic_set(&upi_ug31xx->wait_delay_toggle, 0);
  INIT_DEFERRABLE_WORK(&upi_ug31xx->delay_init_work, upi_ug31xx_delay_init_work_func);
  hrtimer_init(&upi_ug31xx->delay_init_timer, HRTIMER_BASE_BOOTTIME, HRTIMER_MODE_REL);
  upi_ug31xx->delay_init_timer.function = upi_ug31xx_delay_init_timer_func;
  hrtimer_start(&upi_ug31xx->delay_init_timer,
                ktime_set(UPI_UG31XX_INIT_DELAY_WAIT_MS / 1000,
                          (UPI_UG31XX_INIT_DELAY_WAIT_MS % 1000) * 1000000),
                HRTIMER_MODE_REL);
  #endif  ///< end of UG31XX_DELAY_INIT

  #ifdef  UG31XX_ATTACH_DETECTION
  if(upi_ug31xx_is_attach() == true)
  {
    upi_ug31xx_main_init();
  }
  #else   ///< else of UG31XX_ATTACH_DETECTION
  upi_ug31xx_main_init();
  #endif  ///< end o fUG31XX_ATTACH_DETECTION

  #ifdef  UG31XX_REGISTER_EARLY_SUSPEND
  upi_ug31xx->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
  upi_ug31xx->early_suspend.suspend = ug31xx_early_suspend;
  upi_ug31xx->early_suspend.resume  = ug31xx_late_resume;
  register_early_suspend(&upi_ug31xx->early_suspend);
  #endif  ///< end of UG31XX_REGISTER_EARLY_SUSPEND

  if(misc_register(&upi_ug31xx_misc_dev) < 0)
  {
    GLOGE("[%s]: Unable to register misc device.\n", __func__);
    misc_deregister(&upi_ug31xx_misc_dev);
    kfree(upi_ug31xx);
    return (-EINVAL);
  }

  #ifdef  UG31XX_REGISTER_POWER_SUPPLY
  if(power_supply_register(upi_ug31xx->dev, &upi_ug31xx_supply) != 0)
  {
    GLOGE("[%s]: Unable to register power_supply class.\n", __func__);
    kfree(upi_ug31xx);
    return (-EINVAL);
  }
  #endif  ///< end of UG31XX_REGISTER_POWER_SUPPLY

  #ifdef  UG31XX_REGISTER_PROC
  ug31xx_create_proc();
  #endif  ///< end of UG31XX_REGISTER_PROC

  #ifdef  UG31XX_ATTACH_DETECTION
  if(upi_ug31xx_is_attach() == true)
  {
    schedule_delayed_work(&upi_ug31xx->info_update_work, 1*HZ);
    schedule_delayed_work(&upi_ug31xx->power_change_work, 10*HZ);
  }
  #else   ///< else of UG31XX_ATTACH_DETECTION
  schedule_delayed_work(&upi_ug31xx->info_update_work, 1*HZ);
  schedule_delayed_work(&upi_ug31xx->power_change_work, 10*HZ);
  #endif  ///< end o fUG31XX_ATTACH_DETECTION
  return (0);
}

static int ug31xx_i2c_remove(struct i2c_client *client)
{
  cancel_delayed_work_sync(&upi_ug31xx->info_update_work);

  upi_ug31xx_main_exit();

  #ifdef  UG31XX_ATTACH_DETECTION
  mutex_destroy(&upi_ug31xx->attach_lock);
  #endif  ///< end of UG31XX_ATTACH_DETECTION

  wake_lock_destroy(&upi_ug31xx->info_update_wake_lock);

  misc_deregister(&upi_ug31xx_misc_dev);

  #ifdef  UG31XX_REGISTER_POWER_SUPPLY
  power_supply_unregister(&upi_ug31xx_supply);
  #endif  ///< end of UG31XX_REGISTER_POWER_SUPPLY

  kfree(upi_ug31xx);

  return (0);
}

static int ug31xx_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
  upi_ug31xx->sec_suspend = ug31xx_get_rtc_time(&upi_ug31xx->tm_suspend);

  if(upi_ug31xx->in_suspend_cnt == 0)
  {
    upi_ug31xx->sec_delta_suspend_resume = 0;
    upi_ug31xx->in_suspend_cnt           = 1;
  }
  upi_ug31xx->in_suspend_cnt = upi_ug31xx->in_suspend_cnt + 1;
  if(upi_ug31xx->in_suspend_cnt > UG31XX_SUSPEND_CNT_MAX)
  {
    upi_ug31xx->in_suspend_cnt = UG31XX_SUSPEND_CNT_MAX;
  }

  GLOGE("[%s]: Enter suspend at %d/%d/%d %d:%d:%d (%d-%d)\n", __func__,
        upi_ug31xx->tm_suspend.tm_year + 1900,
        upi_ug31xx->tm_suspend.tm_mon + 1,
        upi_ug31xx->tm_suspend.tm_mday,
        upi_ug31xx->tm_suspend.tm_hour,
        upi_ug31xx->tm_suspend.tm_min,
        upi_ug31xx->tm_suspend.tm_sec,
        upi_ug31xx->sec_suspend,
        upi_ug31xx->sec_delta_suspend_resume);
  return (0);
}

static int ug31xx_i2c_resume(struct i2c_client *client)
{
  unsigned long delta_sec;
  GBYTE         buf[2];
  GWORD         wData;

  upi_ug31xx->sec_resume = ug31xx_get_rtc_time(&upi_ug31xx->tm_resume);

  GLOGE("[%s]: Enter resume at %d/%d/%d %d:%d:%d (%d)\n", __func__,
        upi_ug31xx->tm_resume.tm_year + 1900,
        upi_ug31xx->tm_resume.tm_mon + 1,
        upi_ug31xx->tm_resume.tm_mday,
        upi_ug31xx->tm_resume.tm_hour,
        upi_ug31xx->tm_resume.tm_min,
        upi_ug31xx->tm_resume.tm_sec,
        upi_ug31xx->sec_resume);

  /// [YL] : Check inital finish ; 20150825
  #ifdef  UG31XX_ATTACH_DETECTION
  if(gauge_attached == 0)
  {
    GLOGE("[%s]: Initail operation not finish\n",__func__);
    return 0;
  }
  #endif  ///< end of UG31XX_ATTACH_DETECTION
  
  delta_sec = upi_ug31xx->sec_resume - upi_ug31xx->sec_suspend;
  upi_ug31xx->sec_delta_suspend_resume = upi_ug31xx->sec_delta_suspend_resume + delta_sec;

  /// [AT-PM] : Set suspend time ; 07/21/2015
  wData = 0;
  #ifdef  UG31XX_ATTACH_DETECTION
  if(upi_ug31xx_is_attach() == true)
  {
    SmbReadCommand(SBS_SUSPEND_TIME);
    wData = (GWORD)(*(SBSCmd->pbData + 1));
    wData = (wData << 8) | (*(SBSCmd->pbData));
  }
  #else   ///< else of UG31XX_ATTACH_DETECTION
  SmbReadCommand(SBS_SUSPEND_TIME);
  wData = (GWORD)(*(SBSCmd->pbData + 1));
  wData = (wData << 8) | (*(SBSCmd->pbData));
  #endif  ///< end of UG31XX_ATTACH_DETECTION
  delta_sec = delta_sec + wData;
  wData = (delta_sec > 65535) ? 65535 : (GWORD)delta_sec;
  buf[0] = wData % 256;
  buf[1] = wData / 256;
  #ifdef  UG31XX_ATTACH_DETECTION
  if(upi_ug31xx_is_attach() == true)
  {
    SmbWriteCommand(SBS_SUSPEND_TIME, &buf[0], 2);
  }
  #else   ///< else of UG31XX_ATTACH_DETECTION
  SmbWriteCommand(SBS_SUSPEND_TIME, &buf[0], 2);
  #endif  ///< end of UG31XX_ATTACH_DETECTION

  #ifndef UG31XX_REGISTER_EARLY_SUSPEND
  if(upi_ug31xx->sec_delta_suspend_resume > UG31XX_CALI_RSOC_TIME)
  {
    upi_ug31xx->in_suspend_cnt           = 0;
    upi_ug31xx->sec_delta_suspend_resume = 0;

    buf[0] = CAP_SBS_CMD_RESET % 256;
    buf[1] = CAP_SBS_CMD_RESET / 256;
    #ifdef  UG31XX_ATTACH_DETECTION
    if(upi_ug31xx_is_attach() == true)
    {
      SmbWriteCommand(SBS_CAP_DATA, &buf[0], 2);
      /// [YL] : capacity sbs update ; 20150622
      GMainTask();
    }
    #else   ///< else of UG31XX_ATTACH_DETECTION
    SmbWriteCommand(SBS_CAP_DATA, &buf[0], 2);
    /// [YL] : capacity sbs update ; 20150622
    GMainTask();
    #endif  ///< end of UG31XX_ATTACH_DETECTION
  }
  #endif  ///< end of UG31XX_REGISTER_EARLY_SUSPEND

  delta_sec = upi_ug31xx->sec_resume - upi_ug31xx->sec_last_task;
  GLOGE("[%s]: Last Task = %d (%d)\n", __func__, upi_ug31xx->sec_last_task, delta_sec);

  if(delta_sec > UG31XX_SUSPEND_CAL_TIME)
  {
    cancel_delayed_work_sync(&upi_ug31xx->info_update_work);
    schedule_delayed_work(&upi_ug31xx->info_update_work, 0*HZ);
  }
  return (0);
}

static void ug31xx_i2c_shutdown(struct i2c_client *client)
{
  return;
}

#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
static const struct of_device_id ug31xx_match[] = {
    { .compatible = "upi,ug31xx" },
    { },
};
#endif

static const struct i2c_device_id ug31xx_i2c_id[] = {
  { UPI_UG31XX_DEV_NAME, 0 },
  { },
};

MODULE_DEVICE_TABLE(i2c, ug31xx_i2c_id);

static struct i2c_driver ug31xx_i2c_driver = {
  .driver    = {
    .name  = UPI_UG31XX_DEV_NAME,
    .owner = THIS_MODULE,
#if defined(CONFIG_Z380C) || defined(CONFIG_Z380KL)
    .of_match_table = of_match_ptr(ug31xx_match),
#endif
  },
  .probe     = ug31xx_i2c_probe,
  .remove    = ug31xx_i2c_remove,
  .suspend   = ug31xx_i2c_suspend,
  .resume    = ug31xx_i2c_resume,
  .shutdown  = ug31xx_i2c_shutdown,
  .id_table  = ug31xx_i2c_id,
};

#ifdef UG31XX_REGISTER_I2C

static struct i2c_board_info ug31xx_i2c_board_info = {
  .type          = UPI_UG31XX_DEV_NAME,
  .flags         = 0x00,
  .addr          = 0x70,
  .platform_data = NULL,
  .archdata      = NULL,
  .irq           = -1,
};

static struct i2c_client *i2c_client;
static struct i2c_adapter *i2c_adap;

static int ug31xx_i2c_init(void)
{
  int ret = 0;

  i2c_adap = i2c_get_adapter(UG31XX_I2C_ADAPTER);
  if (!i2c_adap) 
  {
    GLOGD("[%s] Cannot get i2c adapter %d\n", __func__, UG31XX_I2C_ADAPTER);
    ret = -ENODEV;
    goto err1;
  }

  i2c_client = i2c_new_device(i2c_adap, &ug31xx_i2c_board_info);
  if (!i2c_client) 
  {
    GLOGD("[%s] Unable to add I2C device for 0x%x\n", __func__,
              ug31xx_i2c_board_info.addr);
    ret = -ENODEV;
    goto err2;
  }

  ret =  i2c_add_driver(&ug31xx_i2c_driver);
  if (ret)
  {
    goto err3;
  }

  return 0;

err3:
  i2c_unregister_device(i2c_client);

err2:
  i2c_put_adapter(i2c_adap);

err1:
  return (ret);
}

static void ug31xx_i2c_exit(void)
{
  i2c_put_adapter(i2c_adap);
  i2c_del_driver(&ug31xx_i2c_driver);
  i2c_unregister_device(i2c_client);
}

#else  ///< else of UG31XX_REGISTER_I2C

static int ug31xx_i2c_init(void)
{
  int ret = 0;

  ret =  i2c_add_driver(&ug31xx_i2c_driver);
  if (ret)
  {
    goto err1;
  }

  return 0;

err1:
  return (ret);
}

static void ug31xx_i2c_exit(void)
{
  i2c_del_driver(&ug31xx_i2c_driver);
}

#endif ///< end of UG31XX_REGISTER_I2C

module_init(ug31xx_i2c_init);
module_exit(ug31xx_i2c_exit);

MODULE_AUTHOR("Allen Teng");
MODULE_DESCRIPTION("UG31XX Fuel Gauge");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION_ASUS);

module_param(debug_level, byte, 0644);
MODULE_PARM_DESC(debug_level, "Debug level for uG31xx driver.");
module_param(alarm_enable, byte, 0644);
MODULE_PARM_DESC(alarm_enable, "Enable/disable alarm function of uG31xx driver.");
module_param(gauge_attached, byte, 0644);
MODULE_PARM_DESC(gauge_attached, "Gauge IC attached or not");

