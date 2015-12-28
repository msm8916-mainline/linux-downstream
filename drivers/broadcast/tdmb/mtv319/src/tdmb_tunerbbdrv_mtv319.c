/*****************************************************************************
 Copyright(c) 2009 LG Electronics Inc. All Rights Reserved
 
 File name : Tunerbb_drv_mtv319.c
 
 Description : mtv319 made by RAONTECH Driver Code
 
 History : 
 ----------------------------------------------------------------------
 Mar. 25,  2013 :      RAONTECH release for LG MC
*******************************************************************************/

#include "tdmb_tunerbbdrv_mtv319def.h"
#include "broadcast_mtv319.h"

#include <linux/string.h>

#include "mtv319.h"
#include "mtv319_internal.h"
#ifdef RTV_CIF_MODE_ENABLED
	#include "mtv319_cifdec.h"
#endif

/*============================================================
**    1.   DEFINITIONS

*============================================================*/
/* Example of Return value */
#define MTV319_RESULT_ERROR		(int8) 0
#define MTV319_RESULT_SUCCESS	(int8) 1

/* -----------------------------------------------------------------------
< HOST Interface FEATURE Usage>
  1. STREAM_TS_UPLOAD  : HOST Interface between MSM and MTV319 is TSIF(I2C)
  2. STREAM_SLAVE_PARALLEL_UPLOAD : HOST Interface between MSM and MTV319 is EBI2
  3. STREAM_SPI_UPLOAD : HOST Interface between MSM and MTV319 is SPI
  ------------------------------------------------------------------------- */

/*============================================================
**    2.   External Variables
*============================================================*/

/*============================================================
**    3.   External Functions
*============================================================*/

/*============================================================
**    4.   Local constant variables
*============================================================*/


/*============================================================
**    5.   Local Typedef
*============================================================*/
typedef enum	mtv319_service_type
{
	MTV319_DAB = 1,
	MTV319_DMB = 2,
	MTV319_VISUAL =3,
	MTV319_DATA,
	MTV319_ENSQUERY = 6,	/* LGE Added */
	MTV319_BLT_TEST = 9,
	MTV319_SERVICE_MAX
} mtv319_service_type;

typedef struct{
	UINT				subch_id;
	mtv319_service_type	svc_type;
	TDMB_BB_DATA_TYPE	data_type;
} TDMB_OPENED_SUBCH_INFO;

/*============================================================
**    6.   Global Variables
*============================================================*/

/*============================================================
**    7.   Static Variables
*============================================================*/
static TDMB_OPENED_SUBCH_INFO opened_subch_info[RTV_MAX_NUM_USE_SUBCHANNEL+1];

static const uint32 tdmb_korea_freq_tbl[] =
{
	/*7A:71, 7B:72, 7C:73*/
	0, 175280, 177008, 178736, 0, 0, 0, 0, 0, 0/*9*/,
	0, 181280, 183008, 184736, 0, 0, 0, 0, 0, 0/*9*/,
	0, 187280, 189008, 190736, 0, 0, 0, 0, 0, 0/*9*/,
	0, 193280, 195008, 196736, 0, 0, 0, 0, 0, 0/*9*/,
	0, 199280, 201008, 202736, 0, 0, 0, 0, 0, 0/*9*/,
	0, 205280, 207008, 208736, 0, 0, 0, 0, 0, 0/*9*/,
	0, 211280, 213008, 214736
};

static BOOL fic_timeout_retry_cnt = 0;

/*============================================================
**    8.   Local Function Prototype
*============================================================*/
static inline uint32 get_freq_from_table(int32 freq_num)
{
	return tdmb_korea_freq_tbl[freq_num - 70];
}

static inline void print_tsp(uint8 *tspb, UINT size)
{
#if 1
	unsigned int i, cnt = 4;
	const uint8 *tsp_buf_ptr = (const uint8 *)tspb;

	for (i = 0; i < cnt; i++, tsp_buf_ptr += 188)
	{
		DMBMSG("[%d] 0x%02X 0x%02X 0x%02X 0x%02X | 0x%02X\n",
			i, tsp_buf_ptr[0], tsp_buf_ptr[1],
			tsp_buf_ptr[2], tsp_buf_ptr[3],
			tsp_buf_ptr[187]);
	}
#endif
}

int8 tunerbb_drv_mtv319_power_on(void)
{
	return tdmb_mtv319_power_on();
}

int8 tunerbb_drv_mtv319_power_off(void)
{
	return tdmb_mtv319_power_off();
}

int8 tunerbb_drv_mtv319_select_antenna(unsigned int sel)
{
	return tdmb_mtv319_select_antenna(sel);
}

int8 tunerbb_drv_mtv319_reset_ch(void)
{
	rtvTDMB_CloseFIC();

	rtvTDMB_CloseAllSubChannels();

	return MTV319_RESULT_SUCCESS;
}

int8 tunerbb_drv_mtv319_re_syncdetector(uint8 op_mode)
{
	return MTV319_RESULT_SUCCESS;
}

int8 tunerbb_drv_mtv319_set_channel(int32 freq_num, uint8 subch_id, uint8 op_mode)
{
	int8 ret_val;

	ret_val = tunerbb_drv_mtv319_multi_set_channel(freq_num, 1, &subch_id, &op_mode);

	return ret_val;
}

void tunerbb_drv_mtv319_set_userstop(int mode)
{
	tdmb_mtv319_set_userstop(mode);
}

int tunerbb_drv_mtv319_is_on(void)
{
	return tdmb_mtv319_tdmb_is_on();
}

/*--------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_init(void)
    (1)   Initializing the MTV319 Chip after power on.
    (2)   Return Value
           SUCCESS : 1
           FAIL : 0 or negative interger (If there is error code)
    (3)   Argument
           VOID
---------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_init(void)
{
	int ret;

	/* Common Code */
#if defined(STREAM_SLAVE_PARALLEL_UPLOAD)
	/* EBI2 Specific Code */
#elif defined(STREAM_TS_UPLOAD)
	/* TSIF Specific Code */
#elif defined(STREAM_SPI_UPLOAD)
	/* SPI Specific. Code */
	//BBM_HOSTIF_SELECT(NULL, BBM_SPI);
#else
#error code not present
#endif

	RTV_GUARD_INIT;

	ret = rtvTDMB_Initialize();
	if (ret == RTV_SUCCESS)
	{
		tdmb_mtv319_interrupt_free();
		return MTV319_RESULT_SUCCESS;
	}
	else
	{
		DMBERR("MTV319 Init error (%d)\n", ret);
		return MTV319_RESULT_ERROR;  /* return 0 : in case of FAIL */
	}
}

/*--------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_stop(void)
    (1)   Stopping the MTV319 Chip Operation
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           VOID
---------------------------------------------------------------------------- */

int8	tunerbb_drv_mtv319_stop(void)
{
	rtvTDMB_CloseFIC();

	rtvTDMB_CloseAllSubChannels();

	return MTV319_RESULT_SUCCESS;
}


/*--------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info)
    (1)   Getting the RF/BB Information 
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
            struct broadcast_tdmb_sig_info *dmb_bb_info (IN/OUT)
           struct broadcast_tdmb_sig_info
           {
                 uint32 dab_ok;
                 uint32 msc_ber;
                 uint32 sync_lock;
                 uint32 afc_ok;
                 uint32 cir;
                 uint32 fic_ber;
                 uint32 tp_lock;
                 uint32 sch_ber;
                 uint32 tp_err_cnt;
                 uint32 va_ber;
                 byte   srv_state_flag;
                 uint32 antenna_level;
           };

           These paramters are dependent on Information supplied by Device.
---------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info)
{
	uint32 lock_mask, msc_cer, fic_cer, per, va_ber;

	msc_cer = rtvTDMB_GetCER();
	dmb_bb_info->msc_ber = msc_cer;
	dmb_bb_info->antenna_level = rtvTDMB_GetAntennaLevel(msc_cer);

	lock_mask = rtvTDMB_GetLockStatus();
	if (lock_mask & RTV_TDMB_OFDM_LOCK_MASK)
	{
		dmb_bb_info->sync_lock = 1;
		dmb_bb_info->dab_ok = 1;
		dmb_bb_info->cir = 1;
		dmb_bb_info->sch_ber = 1;
	}
	else
	{
		dmb_bb_info->sync_lock = 0;
		dmb_bb_info->dab_ok = 0;
		dmb_bb_info->cir = 0;
		dmb_bb_info->sch_ber = 0;
	}

	if (lock_mask & RTV_TDMB_AGC_LOCK_MASK)
		dmb_bb_info->afc_ok = 1;
	else
		dmb_bb_info->afc_ok = 0;

	if ((opened_subch_info[0].svc_type == MTV319_DMB)
	|| (opened_subch_info[0].svc_type == MTV319_VISUAL))
	{
		if (lock_mask & RTV_TDMB_FEC_LOCK_MASK)
			dmb_bb_info->tp_lock = 1;
		else
			dmb_bb_info->tp_lock = 0;

		va_ber = rtvTDMB_GetBER();
		dmb_bb_info->va_ber = va_ber;

		per = rtvTDMB_GetPER();
		dmb_bb_info->tp_err_cnt = per;
	}
	else
	{
		dmb_bb_info->tp_lock = 0;
		dmb_bb_info->tp_err_cnt = 0;
		dmb_bb_info->va_ber = 0;
	}

	fic_cer = rtvTDMB_GetFicCER();
	dmb_bb_info->fic_ber = fic_cer;

	return MTV319_RESULT_SUCCESS;
}


/*--------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_get_msc_ber(void)
    (1)   Getting the msc ber
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           uint32* pmsc_ber (IN/OUT)
---------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_get_msc_ber(uint32* pmsc_ber )
{
	*pmsc_ber = rtvTDMB_GetCER();

	return MTV319_RESULT_SUCCESS;
}

/*-------------------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[ ], uint8 op_mode[ ])
    (1)   Setting the frequency , subch_id and op_mode.
            This function is used in Single Service and Mulitiple Service
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           int32 freq_num (IN)
                - TDMB Frequency index(e.g 7A(71), 13C(133) etc). Convert frequency if needed
           uint8 subch_cnt (IN)
                - The number of multiple service. This value is 1 in case of Single Service
           uint8 subch_id[ ] (IN)
                - Service Componet Sub-Channel ID
           uint8  op_mode[ ] (IN)
                - Service Operation Mode
                DAB  = 1;
                DMB = 2;
                VISUAL = 3;
                DATA = 4;
                TPEG = 5;
                ENSQUERY = 6

           <notice> The size of subch_cnt[ ] and op_mode[ ] is the maximum number being supported by MTV319
--------------------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[ ], uint8 op_mode[ ])
{
	uint8 i;
	uint32 freq_khz;
	enum E_RTV_SERVICE_TYPE svc_type;
	int ret;
	UINT intr_size; /* TS interrupt size. */
	bool fic_open_in_play = FALSE;

	freq_khz = get_freq_from_table(freq_num);
	//printk("tunerbb_drv_mtv319_multi_set_channel, freq_khz = %d, freq_num = %d\n",freq_khz, freq_num);

	if (op_mode[0] == MTV319_ENSQUERY)
	{
		fic_timeout_retry_cnt = 0;

		rtvTDMB_CloseFIC();
		rtvTDMB_CloseAllSubChannels();

		ret = rtvTDMB_ScanFrequency(freq_khz);
		if (ret == RTV_SUCCESS)
			return MTV319_RESULT_SUCCESS; /* Channel found and FIC opened */
		else 
		{
			if(ret != RTV_CHANNEL_NOT_DETECTED)
				DMBERR("Device error: %d\n", ret);

			return MTV319_RESULT_ERROR;
		}
	}

	for (i = 0; i < subch_cnt; i++)
	{
		opened_subch_info[i].svc_type = op_mode[i];
		opened_subch_info[i].subch_id = subch_id[i];

		switch (op_mode[i])
		{
		case MTV319_DMB:
		case MTV319_VISUAL:
		case MTV319_BLT_TEST:
			svc_type = RTV_SERVICE_DMB;
			intr_size = MTV319_DMB_INTERRUPT_SIZE;
			opened_subch_info[i].data_type = TDMB_BB_DATA_TS;
			break;

		case MTV319_DAB:
			svc_type = RTV_SERVICE_DAB;
			intr_size = 10 * 188;//MTV319_DMB_INTERRUPT_SIZE;
			opened_subch_info[i].data_type = TDMB_BB_DATA_DAB;
			break;

		case MTV319_DATA:
			svc_type = RTV_SERVICE_DAB;
			intr_size = 10 * 188;//MTV319_DMB_INTERRUPT_SIZE;
			opened_subch_info[i].data_type = TDMB_BB_DATA_PACK;
			break;

		case MTV319_ENSQUERY:
			fic_open_in_play = TRUE;
			opened_subch_info[i].data_type = TDMB_BB_DATA_FIC;
			break;

		default:
			DMBERR("Invalid op mode (%d)\n", op_mode[i]);
			goto svc_open_err;
		}

		if (op_mode[i] != MTV319_ENSQUERY)
		{
			ret = rtvTDMB_OpenSubChannel(freq_khz, subch_id[i],
										svc_type, intr_size);
			if (ret != RTV_SUCCESS)
			{
				if (ret != RTV_ALREADY_OPENED_SUBCHANNEL_ID)
				{
					DMBERR("Sub channel open failed: %d\n", ret);
					goto svc_open_err;
				}
			}
		}
	}

	if (fic_open_in_play)
	{	/* Must open fic after sub channel in play state. */
		ret = rtvTDMB_OpenFIC();
		if (ret != RTV_SUCCESS)
		{
			DMBERR("FIC open failed: %d\n", ret);
			goto svc_open_err;
		}
	}

	return MTV319_RESULT_SUCCESS;

svc_open_err:
	rtvTDMB_CloseAllSubChannels();

	return MTV319_RESULT_ERROR;
}


/*-------------------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_get_fic(uint8* buffer, uint32* buffer_size)
    (1)   Getting the FIC data after calling tunerbb_drv_mtv319_multi_set_channel(freq, 1, ignore, ENSQUERY)
            In case of ENSQUERY, set_channel function must return Channel LOCKING or Not.
            Get_FIC is called in case of LOCKING Success
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           uint8* buffer (IN/OUT)
               - buffer for FIC data
           uint32* buffer_size (IN /OUT)
              - FIC Data Size

        <notice> This function is used in All HOST Interface
--------------------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_get_fic(uint8* buffer, uint32* buffer_size  /*, uint8 crc_on_off */)
{
	int ret;
	static uint8 fic_buf[MTV319_FIC_BUF_SIZE];

	if (fic_timeout_retry_cnt < 10)
		ret = rtvTDMB_ReadFIC(fic_buf);
	else
		ret = RTV_FIC_READ_TIMEOUT;
	
	if (ret > 0)
	{
		memcpy(buffer, fic_buf, ret);
		*buffer_size = ret;
		return MTV319_RESULT_SUCCESS;
	}
	else
	{
		fic_timeout_retry_cnt++;

		DMBMSG("read fic failed (%d)\n", ret);
		*buffer_size = 0;
		return MTV319_RESULT_ERROR;
	}
}

/*-------------------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_read_data(uint8* buffer, uint32* buffer_size)
    (1)   Reading MSC or MSC + FIC etc Data.
            This function is used in EBI2 HOST Interface
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           uint8* buffer (IN/OUT)
               - buffer for Data
           uint32* buffer_size (IN /OUT)
              - Data Size

        <notice> This function is used in only EBI2 HOST Interface
--------------------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_read_data(uint8* buffer, uint32* buffer_size)
{
#if defined(STREAM_SLAVE_PARALLEL_UPLOAD) || defined(STREAM_SPI_UPLOAD)
#if defined(SPI_INTERFACE_CHECK)
	uint8 ifreg, ifreg2;
	UINT i;
#endif
	uint8 istatus;
	UINT intr_size, num_read_ts_chunk, tsp_read_len = 0;
	TDMB_BB_HEADER_TYPE *dmb_header = (TDMB_BB_HEADER_TYPE *)buffer;
	uint8 *tspb_ptr = (uint8 *)(buffer + sizeof(TDMB_BB_HEADER_TYPE));
	uint8 intr_cnt;
#define MAX_NUM_READ_THRESHOLD_INTR		4

	*buffer_size = 0; /* Init */

	RTV_GUARD_LOCK;

	intr_size = rtvTDMB_GetInterruptLevelSize();

	RTV_REG_MAP_SEL(SPI_CTRL_PAGE);
#if defined(SPI_INTERFACE_CHECK)
	for (i = 0; i < 10; i++)
	{
		ifreg = RTV_REG_GET(0x55);
		ifreg2 = RTV_REG_GET(0x56);
		if ((ifreg == 0xAA) && (ifreg2 == 0xAA))
			break;
		else
		{
			mtv319_spi_recover(buffer, MTV319_SPI_CMD_SIZE + intr_size);
			DMBMSG("(%u) Interface error 1\n", i);
		}
	}
#endif
	istatus = RTV_REG_GET(0x10);
	if (istatus & (U8)(~SPI_INTR_BITS)) {
		mtv319_spi_recover(buffer, MTV319_SPI_CMD_SIZE + intr_size);
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		DMBMSG("Interface error 2 (0x%02X)\n", istatus);
		goto exit_read_mem;
	}

	if (istatus & SPI_UNDERFLOW_INTR)
	{
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		DMBMSG("UDF: 0x%02X\n", istatus);
		goto exit_read_mem;
	}

	if (istatus & SPI_THRESHOLD_INTR)
	{
	#if 0
		if (!(istatus & SPI_OVERFLOW_INTR))
			num_read_ts_chunk = 1;
		else
			num_read_ts_chunk = MAX_NUM_READ_THRESHOLD_INTR; /* Overflow */
	#else
		num_read_ts_chunk = 1;
	#endif

		//DMBMSG("num_read_ts_chunk (%u)\n", num_read_ts_chunk);
    	RTV_REG_MAP_SEL(SPI_CTRL_PAGE);
		intr_cnt = RTV_REG_GET(0x12);
		if (intr_cnt > 1)
			DMBMSG("1 more threshold interrupt(%d)\n", intr_cnt);

		RTV_REG_MAP_SEL(SPI_MEM_PAGE);
		do {
			RTV_REG_BURST_GET(0x10, tspb_ptr, intr_size);
			//print_tsp(tspb_ptr, intr_size); /* To debug */

			tspb_ptr += intr_size;
			tsp_read_len += intr_size;
		} while (--num_read_ts_chunk);

		if (istatus & SPI_OVERFLOW_INTR)
			DMBMSG("OVF: 0x%02X\n", istatus); /* To debug */

		dmb_header->size = tsp_read_len;
		dmb_header->data_type = opened_subch_info[0].data_type;

		*buffer_size = sizeof(TDMB_BB_HEADER_TYPE) + tsp_read_len;
	}
	else
	{
		RTV_REG_MAP_SEL(SPI_CTRL_PAGE);
		mtv319_spi_recover(buffer, MTV319_SPI_CMD_SIZE + intr_size);
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);	
		DMBMSG("No data interrupt (0x%02X)\n", istatus);
	}

exit_read_mem:
	RTV_GUARD_FREE;

	return MTV319_RESULT_SUCCESS;

#elif defined(STREAM_TS_UPLOAD)
	return MTV319_RESULT_ERROR;
#endif
}

/*-------------------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_process_multi_data(uint8 subch_cnt, uint8* input_buf, uint32 input_size, uint32* read_size)
    (1)   Process Multi or Single Service Data. The Driver must process multi or single data and stroe them in other buffer
           for supplying data requested by tunerbb_drv_mtv319_get_multi_data( ) function
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           uint8 subch_cnt (IN)
                - Service Sub-Channel Count
           uint8* input_buf (IN)
               - The buffer pointer  containing Multi or Single Data(FIC/DMB/DAB or Mixed data) read from TSIF or EBI2 buffer
           uint32 input_size (IN)
              - input_buf has input_size data
           uint32* read_size (IN /OUT)
             - data size + subch_id header size supply to Application

        <notice> 
             (1) read_size is the multi or single data + header size.
             (2) LGE supply the headr type
             (3) For example
                 - DMB Single Service case : read_size = DMB MSC Data size + dmb_header size
                 - FIC + DMB + PACKET multi case : 
                       read_size FIC data size + dmb_header + DMB data size + dmb_header + Packet data size + dmb_header 
--------------------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_process_multi_data(uint8 subch_cnt, uint8* input_buf, uint32 input_size, uint32* read_size)
{
	return MTV319_RESULT_SUCCESS;
}


/*-------------------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_start_tii(void)
    (1)   Starting TII
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           VOID
--------------------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_start_tii(void)
{
	return MTV319_RESULT_ERROR;
}

/*-------------------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_stop_tii(void)
    (1)   Stopping TII
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           VOID
--------------------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_stop_tii(void)
{
	return MTV319_RESULT_ERROR;
}

/*-------------------------------------------------------------------------------------
int8 tunerbb_drv_mtv319_check_tii(uint8* main_tii_ptr, uint8* sub_tii_ptr)
    (1)   Stopping TII
    (2)   Return Value
           Sucess : 1
           Fail : 0 or negative interger (If there is error code)
    (3)   Argument
           uint8* main_tii_ptr
              - Main TII value
           uint8* sub_tii_ptr
              - SUB TII value
--------------------------------------------------------------------------------------- */
int8	tunerbb_drv_mtv319_check_tii(uint8* main_tii_ptr, uint8* sub_tii_ptr)
{
	if(( NULL == main_tii_ptr) ||( NULL == sub_tii_ptr))
	{
		return MTV319_RESULT_ERROR;
	}

	*main_tii_ptr = 0xFF;
	*sub_tii_ptr = 0xFF;

	return MTV319_RESULT_ERROR;
}
