/**
 * @file Platform.c
 *
 *  Platform API
 *
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/

#ifdef __cplusplus
extern "C" {
#endif ///< for __cplusplus

#include"UpiDef.h"

#ifdef FEATURE_PLAT_WINDOWS
	CRITICAL_SECTION CriticalSection;
#endif ///< for FEATURE_PLAT_WINDOWS

/**
* @brief UpiPrintDbg
*
*  print degug message
*
* @return NULL
*
*/
GVOID UpiPrintDbg(char *format, ...)
{
#if defined(FEATURE_PLAT_WINDOWS)
  va_list arg;
  char buf[1024];
#ifdef FEATURE_DUMP_LOG
  char name[64];
  FILE *fpLogFile;
  DWORD dwSize;
#endif ///< end of FEATURE_DUMP_LOG
  DWORD dwMaxSize = DUMP_FILE_SIZE;   ///< 10MB
  static int idx = 0;

  memset(buf, 0, sizeof(buf));
  
  va_start(arg, format);
  vsprintf(buf + strlen(buf),  format, arg);
  va_end(arg);
#ifdef FEATURE_DUMP_LOG
  sprintf(name, "%s_%d.TXT", DUMP_FILE_NAME, idx);
  fpLogFile = fopen(name, "a+");
  if (fpLogFile)
  {
    fprintf(fpLogFile, "%s", buf);

    fseek(fpLogFile, 0L, SEEK_END);
    dwSize = ftell(fpLogFile);
    fseek(fpLogFile, 0L, SEEK_SET);
    /// check the file size is bug then 100MB
    if (dwSize > dwMaxSize)
    {
      idx++;
    }
    fclose(fpLogFile);
  }
#endif ///< end of FEATURE_DUMP_LOG

  printf("%s", buf);
#elif defined(FEATURE_PLAT_LINUX)
  char buf[1008];
  va_list arg;
  memset(buf, 0, sizeof(buf));
  
  va_start(arg, format);
  vsprintf(buf + strlen(buf),  format, arg);
  va_end(arg);
  if(UpiGauge.wSysMode & SYS_MODE_FULL_DEBUG_MSG)
  {
    printk(KERN_INFO "%s %s", DUMP_KEY_WORD, buf);
  }

#ifdef  FEATURE_BUFFER_ENABLE
  if(strlen(DebugLogBuf) < DUMP_BUFFER_SIZE)
  {
    sprintf(buf, "%10d:%s", UpiGauge.tmNow.absTime, buf);
    UPI_MEMCPY(&DebugLogBuf[strlen(DebugLogBuf)], 
               &buf[0], 
               (((strlen(DebugLogBuf) + strlen(buf)) < DUMP_BUFFER_SIZE) ? 
                strlen(buf) : 
                (DUMP_BUFFER_SIZE - strlen(DebugLogBuf))));
  }
#endif  ///< end of FEATURE_BUFFER_ENABLE
#elif defined(FEATURE_PLAT_ARM_M0)
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
#else  ///< end of FEATURE_PLAT_LINUX
  char buf[1024];
  va_list arg;
  memset(buf, 0, sizeof(buf));
  
  va_start(arg, format);
  vsprintf(buf + strlen(buf),  format, arg);
  va_end(arg);
  printf("%s", buf);
#endif ///< end of FEATURE_PLAT_LINUX
}

/**
* @brief UpiPrintDbgE
*
*  print degug message
*
* @return NULL
*
*/
GVOID UpiPrintDbgE(char *format, ...)
{
#if defined(FEATURE_PLAT_WINDOWS)
  char buf[1024];
  char name[64];
  va_list arg;
#ifdef FEATURE_DUMP_LOG
  FILE *fpLogFile;
  DWORD dwSize;
#endif ///< end of FEATURE_DUMP_LOG
  DWORD dwMaxSize = DUMP_FILE_SIZE;   ///< 10MB
  static int idx = 0;

  memset(buf, 0, sizeof(buf));
  
  va_start(arg, format);
  vsprintf(buf + strlen(buf),  format, arg);
  va_end(arg);
  sprintf(name, "%s_%d.TXT", DUMP_FILE_NAME, idx);
#ifdef FEATURE_DUMP_LOG
  fpLogFile = fopen(name, "a+");
  if (fpLogFile)
  {
    fprintf(fpLogFile, "%s", buf);

    fseek(fpLogFile, 0L, SEEK_END);
    dwSize = ftell(fpLogFile);
    fseek(fpLogFile, 0L, SEEK_SET);
    /// check the file size is bug then 100MB
    if (dwSize > dwMaxSize)
    {
      idx++;
    }
    fclose(fpLogFile);
  }
#endif ///< end of FEATURE_DUMP_LOG

  printf("%s", buf);
#elif defined(FEATURE_PLAT_LINUX)
  char buf[1008];
  va_list arg;
  memset(buf, 0, sizeof(buf));
  
  va_start(arg, format);
  vsprintf(buf + strlen(buf),  format, arg);
  va_end(arg);
  printk(KERN_INFO "%s %s", DUMP_KEY_WORD, buf);

#ifdef  FEATURE_BUFFER_ENABLE
  if(strlen(DebugLogBuf) < DUMP_BUFFER_SIZE)
  {
    sprintf(buf, "%10d:%s", UpiGauge.tmNow.absTime, buf);
    UPI_MEMCPY(&DebugLogBuf[strlen(DebugLogBuf)], 
               &buf[0], 
               (((strlen(DebugLogBuf) + strlen(buf)) < DUMP_BUFFER_SIZE) ? 
                strlen(buf) : 
                (DUMP_BUFFER_SIZE - strlen(DebugLogBuf))));
  }
#endif  ///< end of FEATURE_BUFFER_ENABLE
#elif defined(FEATURE_PLAT_ARM_M0)
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
#else  ///< end of FEATURE_PLAT_LINUX
  char buf[1024];
  va_list arg;
  memset(buf, 0, sizeof(buf));
  
  va_start(arg, format);
  vsprintf(buf + strlen(buf),  format, arg);
  va_end(arg);
  printf("%s", buf);
#endif ///< end of FEATURE_PLAT_LINUX
}

/**
 * @brief UpiAbs
 *
 *  Get ABS value of a short
 *
 * @return NULL
 *
 */
GWORD UpiAbs(GSHORT sVal)
{
  if(sVal < 0)
  {
    sVal = sVal * -1;
    return (GWORD)sVal;
  }
  return (GWORD) sVal;
}

/**
 * @brief UpiMemset
 *
 *  Sleep Mini-Second
 *
 * @return NULL
 *
 */
GVOID UpiMemset(GVOID *dst, GCHAR ch, GINT32 cnt)
{
  GINT64 idx;
  GBYTE *ptrDest;

  idx = 0;
  ptrDest = (GBYTE *)dst;
  while(idx < cnt)
  {
    *(ptrDest) = ch;
    ptrDest++;
    idx++;
  }
}

/**
 * @brief UpiSleep
 *
 *  Sleep Mini-Second
 *
 * @return NULL
 *
 */
GVOID UpiMemcpy(GVOID *dst, GVOID *src, GINT32 cnt)
{
  GINT64 idx;
  GBYTE *ptrDest;
  GBYTE *ptrSrc;

  idx = 0;
  ptrDest = (GBYTE *)dst;
  ptrSrc = (GBYTE *)src;
  while(idx < cnt)
  {
    *(ptrDest) = *(ptrSrc);
    ptrDest++;
    ptrSrc++; 
    idx++;
  }
}

/**
 * @brief UpiSleep
 *
 *  Sleep Mini-Second
 *
 * @return NULL
 *
 */
GVOID UpiSleep(GDWORD msec)
{
#if defined(FEATURE_PLAT_WINDOWS)
  Sleep(msec);
#elif defined(FEATURE_PLAT_LINUX)
  mdelay(msec);
#elif defined(FEATURE_PLAT_ARM_M0)
  SYSDelay(msec);
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
  SYSDelay(msec);
#else ///< for FEATURE_PLAT_LINUX
#endif ///< for FEATURE_PLAT_LINUX
}

/**
 * @brief UpiGetLocalTime
 *
 *  Get local time
 *
 * @return NULL
 *
 */
GVOID UpiGetLocalTime(GTimeDataType *pTimeData)
{

#if defined(FEATURE_PLAT_WINDOWS)
  time_t rawtime; 
  struct tm * timeInfo; 
  SYSTEMTIME systime;

  time(&rawtime); 
  timeInfo = localtime(&rawtime); 
  GetLocalTime(&systime);
  ///----------------------------------///
  /// Set time objects
  ///----------------------------------///
  pTimeData->absTime  = UpiGetTickCount();
  pTimeData->sec     = systime.wSecond;
  pTimeData->min     = systime.wMinute;
  pTimeData->hour    = systime.wHour;
  pTimeData->day     = systime.wDay;
  pTimeData->mon     = systime.wMonth;
  pTimeData->year    = systime.wYear - 1980;
  pTimeData->rawTime  = (GINT32)rawtime;  
#elif defined(FEATURE_PLAT_LINUX)
  struct timeval rawtime;
  struct rtc_time tm;
  GU4 localTime;

  do_gettimeofday(&rawtime);
  localTime = (GU4)(rawtime.tv_sec);
  rtc_time_to_tm(localTime, &tm);

  ///----------------------------------///
  /// Set time objects
  ///----------------------------------///
  pTimeData->absTime  = UpiGetTickCount();
  pTimeData->sec      = tm.tm_sec;
  pTimeData->min      = tm.tm_min;
  pTimeData->hour     = tm.tm_hour;
  pTimeData->day      = tm.tm_mday;
  pTimeData->mon      = tm.tm_mon + 1;
  pTimeData->year     = tm.tm_year + 1900;
  pTimeData->rawTime  = (GINT32)localTime;  
#elif defined(FEATURE_PLAT_ARM_M0)
  S_DRVRTC_TIME_DATA_T sCurTime;
  DrvRTC_Read(DRVRTC_CURRENT_TIME, &sCurTime);
  
  pTimeData->absTime  = UpiGetTickCount();
  pTimeData->sec      = sCurTime.u32cSecond;
  pTimeData->min      = sCurTime.u32cMinute;
  pTimeData->hour     = sCurTime.u32cHour;
  pTimeData->day      = sCurTime.u32cDay;
  pTimeData->mon      = sCurTime.u32cMonth;
  pTimeData->year     = sCurTime.u32Year-1980;
  pTimeData->rawTime  = SYSRawTime();  
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
  S_DRVRTC_TIME_DATA_T sCurTime;
  DrvRTC_Read(DRVRTC_CURRENT_TIME, &sCurTime);
  
  pTimeData->absTime  = UpiGetTickCount();
  pTimeData->sec      = sCurTime.u32cSecond;
  pTimeData->min      = sCurTime.u32cMinute;
  pTimeData->hour     = sCurTime.u32cHour;
  pTimeData->day      = sCurTime.u32cDay;
  pTimeData->mon      = sCurTime.u32cMonth;
  pTimeData->year     = sCurTime.u32Year-1980;
  pTimeData->rawTime  = SYSRawTime();  
#else  ///< for FEATURE_PLAT_LINUX

#endif ///< for FEATURE_PLAT_LINUX
}

/**
 * @brief UpiGetJiffies
 *
 *  Get mini-second from system
 *
 * @return NULL
 *
 */
GDWORD UpiGetTickCount(GVOID)
{
  GDWORD ms = 0;

#if defined(FEATURE_PLAT_WINDOWS)
  ms = GetTickCount();
#elif defined(FEATURE_PLAT_LINUX)
  ms = jiffies_to_msecs(jiffies);
#elif defined(FEATURE_PLAT_ARM_M0)
  ms = uSysSecCnt*1000;
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
  ms = uSysOneEighthSecCnt*1000;
#else  ///< for FEATURE_PLAT_LINUX
  struct timeval tv;

  if(gettimeofday(&tv, NULL) != 0)
  {
    ms = 0;
  }
  else
  {
    ms = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
  }
#endif ///< for FEATURE_PLAT_LINUX
  return ms;
}

/**
 * @brief UpiMutexInit
 *
 *  Mutex init
 *
 * @return NULL
 *
 */
GVOID UpiMutexInit(GVOID **pObj)
{
#if defined(FEATURE_PLAT_WINDOWS)
  
	InitializeCriticalSection(&CriticalSection);
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *ptr;

  #ifdef PLATFORM_ASUS_Z370KL_COVER
  if (*pObj != NULL)
    return;
  #endif ///< end of PLATFORM_ASUS_Z370KL_COVER
	
  ptr = kzalloc(sizeof(struct mutex), GFP_KERNEL);
  mutex_init(ptr);   ///< linux
  *pObj = (GVOID *)ptr;
#else  ///< for FEATURE_PLAT_WINDOWS
#endif ///< for FEATURE_PLAT_WINDOWS
}

/**
 * @brief UpiMutexLock
 *
 *  Mutex lock
 *
 * @return NULL
 *
 */
GVOID UpiMutexLock(GVOID **pObj)
{
#if defined(FEATURE_PLAT_WINDOWS)

	EnterCriticalSection(&CriticalSection);
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *ptr;

  ptr = (struct mutex *)(*pObj);
  mutex_lock(ptr);   ///< linux
#else  ///< for FEATURE_PLAT_WINDOWS
#endif ///< for FEATURE_PLAT_WINDOWS
}

/**
 * @brief UpiMutexUnLock
 *
 *  Mutex unlock
 *
 * @return NULL
 *
 */
GVOID UpiMutexUnLock(GVOID **pObj)
{
#if defined(FEATURE_PLAT_WINDOWS)
	LeaveCriticalSection(&CriticalSection);
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *ptr;

  ptr = (struct mutex *)(*pObj);
  mutex_unlock(ptr);   ///< linux
#else  ///< for FEATURE_PLAT_WINDOWS
#endif ///< for FEATURE_PLAT_WINDOWS
}

/**
 * @brief UpiMutexInit
 *
 *  Mutex init
 *
 * @return NULL
 *
 */
GVOID UpiMutexExit(GVOID **pObj)
{
#if defined(FEATURE_PLAT_WINDOWS)
  
	DeleteCriticalSection(&CriticalSection);
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *ptr;

  #ifdef PLATFORM_ASUS_Z370KL_COVER
  if (1)
    return;
  #endif ///< end of PLATFORM_ASUS_Z370KL_COVER
	
  ptr = (struct mutex *)(*pObj);
  mutex_destroy(ptr);   ///< linux
  kfree(ptr);
#else  ///< for FEATURE_PLAT_WINDOWS
#endif ///< for FEATURE_PLAT_WINDOWS
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

