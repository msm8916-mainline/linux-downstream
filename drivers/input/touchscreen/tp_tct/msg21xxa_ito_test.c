
#include "msg21xx.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kobject.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>


#include "open_test_ANA1_MUTTO.h"
#include "open_test_ANA2_MUTTO.h"
#include "open_test_ANA1_B_MUTTO.h"
#include "open_test_ANA2_B_MUTTO.h"
#include "open_test_ANA3_MUTTO.h"

#include "short_test_ANA1_MUTTO.h"
#include "short_test_ANA2_MUTTO.h"
#include "short_test_ANA3_MUTTO.h"
#include "short_test_ANA4_MUTTO.h"

#if 0
#include "open_test_ANA1_X.h"
#include "open_test_ANA2_X.h"
#include "open_test_ANA1_B_X.h"
#include "open_test_ANA2_B_X.h"
#include "open_test_ANA3_X.h"

#include "short_test_ANA1_X.h"
#include "short_test_ANA2_X.h"
#include "short_test_ANA3_X.h"
#include "short_test_ANA4_X.h"


#include "open_test_ANA1_Y.h"
#include "open_test_ANA2_Y.h"
#include "open_test_ANA1_B_Y.h"
#include "open_test_ANA2_B_Y.h"
#include "open_test_ANA3_Y.h"

#include "short_test_ANA1_Y.h"
#include "short_test_ANA2_Y.h"
#include "short_test_ANA3_Y.h"
#include "short_test_ANA4_Y.h"
#endif

///////////////////////////////////////////////////////////////////////////

u8 bItoTestDebug = 0;
#define ITO_TEST_DEBUG(format, ...) \
{ \
    if(bItoTestDebug) \
    { \
        printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__); \
        mdelay(5); \
    } \
}
#define ITO_TEST_DEBUG_MUST(format, ...)	printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__);mdelay(5)

#define MAX_CHNL_NUM (48)
#define PIN_GUARD_RING (46)
#define GPIO_SETTING_SIZE (3)

#if 0
#define	OPEN_TEST_NON_BORDER_AREA_THRESHOLD (25)        
#define	OPEN_TEST_BORDER_AREA_THRESHOLD     (25)
#else
#define	OPEN_TEST_NON_BORDER_AREA_THRESHOLD (40)        
#define	OPEN_TEST_BORDER_AREA_THRESHOLD     (40)
#endif

#define OPEN_TEST_NON_BORDER_AREA_MINI_THRESHOLD (1000)

#define	SHORT_TEST_THRESHOLD                (3500)

#define	ITO_TEST_MODE_OPEN_TEST              (0x01)
#define	ITO_TEST_MODE_SHORT_TEST             (0x02)

s16 s16_raw_data_1[MAX_CHNL_NUM] = {0};
s16 s16_raw_data_2[MAX_CHNL_NUM] = {0};
s16 s16_raw_data_3[MAX_CHNL_NUM] = {0};
s16 s16_raw_data_4[MAX_CHNL_NUM] = {0};
s8 data_flag_1[MAX_CHNL_NUM] = {0};
s8 data_flag_2[MAX_CHNL_NUM] = {0};
s8 data_flag_3[MAX_CHNL_NUM] = {0};
s8 data_flag_4[MAX_CHNL_NUM] = {0};
u8 ito_test_keynum = 0;
u8 ito_test_dummynum = 0;
u8 ito_test_trianglenum = 0;
u8 ito_test_2r = 0;
u8 g_LTP = 1;	
uint16_t *open_1 = NULL;
uint16_t *open_1B = NULL;
uint16_t *open_2 = NULL;
uint16_t *open_2B = NULL;
uint16_t *open_3 = NULL;
u8 *MAP1 = NULL;
u8 *MAP2 = NULL;
u8 *MAP3 = NULL;
u8 *MAP40_1 = NULL;
u8 *MAP40_2 = NULL;
u8 *MAP40_3 = NULL;
u8 *MAP40_4 = NULL;
u8 *MAP41_1 = NULL;
u8 *MAP41_2 = NULL;
u8 *MAP41_3 = NULL;
u8 *MAP41_4 = NULL;

u16 *short_1 = NULL;
u16 *short_2 = NULL;
u16 *short_3 = NULL;
u16 *short_4 = NULL;
u8 *SHORT_MAP1 = NULL;
u8 *SHORT_MAP2 = NULL;
u8 *SHORT_MAP3 = NULL;
u8 *SHORT_MAP4 = NULL;
u16 *short_1_GPO = NULL;
u16 *short_2_GPO = NULL;
u16 *short_3_GPO = NULL;
u16 *short_4_GPO = NULL;


static u8 g_fail_channel[MAX_CHNL_NUM] = {0};
static int fail_channel_count = 0;
static u8 ito_test_mode = 0;

#define ITO_TEST_ADDR_TP  (0x4C>>1)
#define ITO_TEST_ADDR_REG (0xC4>>1)
#define REG_INTR_FIQ_MASK           0x04
#define FIQ_E_FRAME_READY_MASK      ( 1 << 8 )

#define BIT0  (1<<0)
#define BIT1  (1<<1)
#define BIT2  (1<<2)
#define BIT5  (1<<5)
#define BIT11 (1<<11)
#define BIT15 (1<<15)

struct i2c_client *g_I2cClient = NULL;

static int ito_test_i2c_read(u8 addr, u8* read_data, u16 size)
{
    int rc;
    unsigned short addr_before = g_I2cClient->addr;
    g_I2cClient->addr = addr;

    #ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
    {
        int i = 0;
        g_I2cClient->ext_flag = g_I2cClient->ext_flag | I2C_DMA_FLAG ;
        rc = i2c_master_recv(g_I2cClient, (unsigned char *)I2CDMABuf_pa, size);
        for(i = 0; i < size; i++)
   		{
        	read_data[i] = I2CDMABuf_va[i];
    	}
    }
    else
    {
        rc = i2c_master_recv(g_I2cClient, read_data, size);
    }
    g_I2cClient->ext_flag = g_I2cClient->ext_flag & (~I2C_DMA_FLAG);	
    #else
    rc = i2c_master_recv(g_I2cClient, read_data, size);
    #endif

    g_I2cClient->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_read error %d,addr=%d\n", rc,addr);
    }
    return rc;
}

static int ito_test_i2c_write(u8 addr, u8* data, u16 size)
{
    int rc;
    unsigned short addr_before = g_I2cClient->addr;
    g_I2cClient->addr = addr;

#ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
	{
	    int i = 0;
	    for(i=0;i<size;i++)
    	{
    		 I2CDMABuf_va[i]=data[i];
    	}
		g_I2cClient->ext_flag = g_I2cClient->ext_flag | I2C_DMA_FLAG ;
		rc = i2c_master_send(g_I2cClient, (unsigned char *)I2CDMABuf_pa, size);
	}
	else
	{
		rc = i2c_master_send(g_I2cClient, data, size);
	}
    g_I2cClient->ext_flag = g_I2cClient->ext_flag & (~I2C_DMA_FLAG);	
#else
    rc = i2c_master_send(g_I2cClient, data, size);
#endif

    g_I2cClient->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_write error %d,addr = %d,data[0]=%d\n", rc, addr,data[0]);
    }
    return rc;
}

static void ito_test_reset(void)
{
	struct msg21xx_ts_data *ts_data = NULL;

	ts_data = i2c_get_clientdata(g_I2cClient);

	ITO_TEST_DEBUG("reset tp\n");

	if (NULL == ts_data) {
		printk("%s %d failed ! \n", __func__, __LINE__);
	} else {
		gpio_direction_output(ts_data->pdata->reset_gpio, 1);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(100);     /* Note that the RST must be in LOW 10ms at least */
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(100);     /* Enable the interrupt service thread/routine for INT after 50ms */
	}
}

static void ito_test_disable_irq(void)
{
	struct msg21xx_ts_data *ts_data = NULL;
	ts_data = i2c_get_clientdata(g_I2cClient);

	if (NULL == ts_data) {
		printk("%s %d failed ! \n", __func__, __LINE__);
	} else
		disable_irq_nosync(ts_data->pdata->irq_gpio);
}

static void ito_test_enable_irq(void)
{
	struct msg21xx_ts_data *ts_data = NULL;
	ts_data = i2c_get_clientdata(g_I2cClient);

	if (NULL == ts_data) {
		printk("%s %d failed ! \n", __func__, __LINE__);
	} else
		enable_irq(ts_data->pdata->irq_gpio);
}

static void ito_test_set_iic_rate(u32 iicRate)
{
	#ifdef CONFIG_I2C_SPRD//չѶƽ̨
        sprd_i2c_ctl_chg_clk(g_I2cClient->adapter->nr, iicRate);
        mdelay(100);
	#endif
    #ifdef MTK//MTKƽ̨
        g_I2cClient->timing = iicRate/1000;
    #endif
}

static void ito_test_WriteReg( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 5 );
}

static void ito_test_WriteReg8Bit( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    ito_test_i2c_write ( ITO_TEST_ADDR_REG, &tx_data[0], 4 );
}

static unsigned short ito_test_ReadReg( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 3 );
    ito_test_i2c_read ( ITO_TEST_ADDR_REG, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static u32 ito_test_get_TpType(void)
{
    u8 tx_data[3] = {0};
    u8 rx_data[4] = {0};
    u32 Major = 0, Minor = 0;

    ITO_TEST_DEBUG("GetTpType\n");
        
    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x2A;
    ito_test_i2c_write(ITO_TEST_ADDR_TP, &tx_data[0], 3);
    mdelay(50);
    ito_test_i2c_read(ITO_TEST_ADDR_TP, &rx_data[0], 4);
    Major = (rx_data[1]<<8) + rx_data[0];
    Minor = (rx_data[3]<<8) + rx_data[2];

    ITO_TEST_DEBUG("***TpTypeMajor = %d ***\n", Major);
    ITO_TEST_DEBUG("***TpTypeMinor = %d ***\n", Minor);
    
    return Major;
}

//modify:ע\D2\E2\B8\C3\CF\EEĿtp\CA\FDĿ
//#define TP_OF_X    (2)
/* MUTTO is 1, JUNDA 2 , YEJI 5 */
#define TP_OF_X    (1)
#define TP_OF_Y    (4)

static u32 ito_test_choose_TpType(void)
{
    u32 tpType = 0;
    u8 i = 0;
    open_1 = NULL;
    open_1B = NULL;
    open_2 = NULL;
    open_2B = NULL;
    open_3 = NULL;
    MAP1 = NULL;
    MAP2 = NULL;
    MAP3 = NULL;
    MAP40_1 = NULL;
    MAP40_2 = NULL;
    MAP40_3 = NULL;
    MAP40_4 = NULL;
    MAP41_1 = NULL;
    MAP41_2 = NULL;
    MAP41_3 = NULL;
    MAP41_4 = NULL;
    short_1 = NULL;
    short_2 = NULL;
    short_3 = NULL;
    short_4 = NULL;
    SHORT_MAP1 = NULL;
    SHORT_MAP2 = NULL;
    SHORT_MAP3 = NULL;
    SHORT_MAP4 = NULL;
    short_1_GPO = NULL;
    short_2_GPO = NULL;
    short_3_GPO = NULL;
    short_4_GPO = NULL;
    ito_test_keynum = 0;
    ito_test_dummynum = 0;
    ito_test_trianglenum = 0;
    ito_test_2r = 0;

    for(i=0;i<10;i++)
    {
        tpType = ito_test_get_TpType();
        ITO_TEST_DEBUG("tpType=%d;i=%d;\n",tpType,i);
        if(TP_OF_X==tpType
           ||TP_OF_Y==tpType)//modify:ע\D2\E2\B8\C3\CF\EEĿtp\CA\FDĿ
        {
            break;
        }
        else if(i<5)
        {
            mdelay(100);  
        }
        else
        {
            ito_test_reset();
        }
    }

    if(TP_OF_X==tpType)//modify:ע\D2\E2\B8\C3\CF\EEĿtp\CA\FDĿ
    {
        open_1 = open_1_MUTTO;
        open_1B = open_1B_MUTTO;
        open_2 = open_2_MUTTO;
        open_2B = open_2B_MUTTO;
        open_3 = open_3_MUTTO;
        MAP1 = MAP1_MUTTO;
        MAP2 = MAP2_MUTTO;
        MAP3 = MAP3_MUTTO;
        MAP40_1 = MAP40_1_MUTTO;
        MAP40_2 = MAP40_2_MUTTO;
        MAP40_3 = MAP40_3_MUTTO;
        MAP40_4 = MAP40_4_MUTTO;
        MAP41_1 = MAP41_1_MUTTO;
        MAP41_2 = MAP41_2_MUTTO;
        MAP41_3 = MAP41_3_MUTTO;
        MAP41_4 = MAP41_4_MUTTO;
        short_1 = short_1_MUTTO;
        short_2 = short_2_MUTTO;
        short_3 = short_3_MUTTO;
        short_4 = short_4_MUTTO;
        SHORT_MAP1 = SHORT_MAP1_MUTTO;
        SHORT_MAP2 = SHORT_MAP2_MUTTO;
        SHORT_MAP3 = SHORT_MAP3_MUTTO;
        SHORT_MAP4 = SHORT_MAP4_MUTTO;
        short_1_GPO = short_1_MUTTO_GPO;
        short_2_GPO = short_2_MUTTO_GPO;
        short_3_GPO = short_3_MUTTO_GPO;
        short_4_GPO = short_4_MUTTO_GPO;
        ito_test_keynum = NUM_KEY_MUTTO;
        ito_test_dummynum = NUM_DUMMY_MUTTO;
        ito_test_trianglenum = NUM_SENSOR_MUTTO;
        ito_test_2r = ENABLE_2R_MUTTO;
    }
#if 0
    if(TP_OF_X==tpType)//modify:ע\D2\E2\B8\C3\CF\EEĿtp\CA\FDĿ
    {
        open_1 = open_1_X;
        open_1B = open_1B_X;
        open_2 = open_2_X;
        open_2B = open_2B_X;
        open_3 = open_3_X;
        MAP1 = MAP1_X;
        MAP2 = MAP2_X;
        MAP3 = MAP3_X;
        MAP40_1 = MAP40_1_X;
        MAP40_2 = MAP40_2_X;
        MAP40_3 = MAP40_3_X;
        MAP40_4 = MAP40_4_X;
        MAP41_1 = MAP41_1_X;
        MAP41_2 = MAP41_2_X;
        MAP41_3 = MAP41_3_X;
        MAP41_4 = MAP41_4_X;
        short_1 = short_1_X;
        short_2 = short_2_X;
        short_3 = short_3_X;
        short_4 = short_4_X;
        SHORT_MAP1 = SHORT_MAP1_X;
        SHORT_MAP2 = SHORT_MAP2_X;
        SHORT_MAP3 = SHORT_MAP3_X;
        SHORT_MAP4 = SHORT_MAP4_X;
        short_1_GPO = short_1_X_GPO;
        short_2_GPO = short_2_X_GPO;
        short_3_GPO = short_3_X_GPO;
        short_4_GPO = short_4_X_GPO;
        ito_test_keynum = NUM_KEY_X;
        ito_test_dummynum = NUM_DUMMY_X;
        ito_test_trianglenum = NUM_SENSOR_X;
        ito_test_2r = ENABLE_2R_X;
    }
    else if(TP_OF_Y==tpType)
    {
        open_1 = open_1_Y;
        open_1B = open_1B_Y;
        open_2 = open_2_Y;
        open_2B = open_2B_Y;
        open_3 = open_3_Y;
        MAP1 = MAP1_Y;
        MAP2 = MAP2_Y;
        MAP3 = MAP3_Y;
        MAP40_1 = MAP40_1_Y;
        MAP40_2 = MAP40_2_Y;
        MAP40_3 = MAP40_3_Y;
        MAP40_4 = MAP40_4_Y;
        MAP41_1 = MAP41_1_Y;
        MAP41_2 = MAP41_2_Y;
        MAP41_3 = MAP41_3_Y;
        MAP41_4 = MAP41_4_Y;
        short_1 = short_1_Y;
        short_2 = short_2_Y;
        short_3 = short_3_Y;
        short_4 = short_4_Y;
        SHORT_MAP1 = SHORT_MAP1_Y;
        SHORT_MAP2 = SHORT_MAP2_Y;
        SHORT_MAP3 = SHORT_MAP3_Y;
        SHORT_MAP4 = SHORT_MAP4_Y;
        short_1_GPO = short_1_Y_GPO;
        short_2_GPO = short_2_Y_GPO;
        short_3_GPO = short_3_Y_GPO;
        short_4_GPO = short_4_Y_GPO;
        ito_test_keynum = NUM_KEY_Y;
        ito_test_dummynum = NUM_DUMMY_Y;
        ito_test_trianglenum = NUM_SENSOR_Y;
        ito_test_2r = ENABLE_2R_Y;
    }
    else
    {
        tpType = 0;
    }
#endif
    return tpType;
}

static void ito_test_EnterSerialDebugMode(void)
{
    u8 data[5];

    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 5);

    data[0] = 0x37;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x35;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x71;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);
}

static void ito_test_disable_filter_noise_detect(void)
{
    u16 reg_value;

    ITO_TEST_DEBUG("ito_test_disable_filter_noise_detect()\n");
     
    // Disable DIG/ANA drop
    reg_value = ito_test_ReadReg( 0x13, 0x02 ); 
      
    ito_test_WriteReg( 0x13, 0x02, reg_value & (~(BIT2 | BIT1 | BIT0)) );      
}

static uint16_t ito_test_get_num( void )
{
    uint16_t    num_of_sensor,i;
    uint16_t 	RegValue1,RegValue2;
 
    num_of_sensor = 0;
        
    RegValue1 = ito_test_ReadReg( 0x11, 0x4A); //bank:ana, addr:h0025  
    ITO_TEST_DEBUG("ito_test_get_num,RegValue1=%d\n",RegValue1);
    if ( ( RegValue1 & BIT1) == BIT1 )
    {
    	RegValue1 = ito_test_ReadReg( 0x12, 0x0A); //bank:ana2, addr:h0005  			
    	RegValue1 = RegValue1 & 0x0F;
    	
    	RegValue2 = ito_test_ReadReg( 0x12, 0x16); //bank:ana2, addr:h000b    		
    	RegValue2 = (( RegValue2 >> 1 ) & 0x0F) + 1;
    	
    	num_of_sensor = RegValue1 * RegValue2;
    }
	else
	{
	    for(i=0;i<4;i++)
	    {
	        num_of_sensor+=(ito_test_ReadReg( 0x12, 0x0A)>>(4*i))&0x0F; //bank:ana2, addr:h0005  
	    }
	}
    ITO_TEST_DEBUG("ito_test_get_num() num_of_sensor=%d\n", num_of_sensor);
    return num_of_sensor;        
}

static void ito_test_polling( void )
{
    uint16_t    reg_int = 0x0000;
    uint16_t    reg_value;


    reg_int = 0;

    ito_test_WriteReg( 0x13, 0x0C, BIT15 ); //bank:fir, addr:h0006         
    ito_test_WriteReg( 0x12, 0x14, (ito_test_ReadReg(0x12,0x14) | BIT0) ); //bank:ana2, addr:h000a        
            
    ITO_TEST_DEBUG("polling start\n");

    do
    {
        reg_int = ito_test_ReadReg(0x3D, 0x18); //bank:intr_ctrl, addr:h000c
    } while( ( reg_int & FIQ_E_FRAME_READY_MASK ) == 0x0000 );

    ITO_TEST_DEBUG("polling end\n");
    reg_value = ito_test_ReadReg( 0x3D, 0x18 ); 
    ito_test_WriteReg( 0x3D, 0x18, reg_value & (~FIQ_E_FRAME_READY_MASK) );      
}

static uint16_t ito_test_get_data_out( int16_t* s16_raw_data )
{
    uint8_t     i,dbbus_tx_data[8];
    uint16_t    raw_data[MAX_CHNL_NUM]={0};
    uint16_t    num_of_sensor;
    uint16_t    reg_int;
    uint8_t		dbbus_rx_data[MAX_CHNL_NUM*2]={0};
  
    num_of_sensor = ito_test_get_num();
    if(num_of_sensor*2>MAX_CHNL_NUM*2)
    {
        ITO_TEST_DEBUG("danger, num_of_sensor=%d\n", num_of_sensor);
        return num_of_sensor;
    }

    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int & (uint16_t)(~FIQ_E_FRAME_READY_MASK) ) ); 
    ito_test_polling();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x13; //bank:fir, addr:h0020 
    dbbus_tx_data[2] = 0x40;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
    mdelay(20);
    ito_test_i2c_read(ITO_TEST_ADDR_REG, &dbbus_rx_data[0], (num_of_sensor * 2));
    mdelay(100);
    
    for(i=0;i<num_of_sensor * 2;i++)
    {
        ITO_TEST_DEBUG("dbbus_rx_data[%d]=%d\n",i,dbbus_rx_data[i]);
    }
 
    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int | (uint16_t)FIQ_E_FRAME_READY_MASK ) ); 

    for( i = 0; i < num_of_sensor; i++ )
    {
        raw_data[i] = ( dbbus_rx_data[ 2 * i + 1] << 8 ) | ( dbbus_rx_data[2 * i] );
        s16_raw_data[i] = ( int16_t )raw_data[i];
    }
    
    return num_of_sensor;
}

static void ito_test_send_data_in( uint8_t step )
{
    uint16_t	i;
    uint8_t 	dbbus_tx_data[512];
    uint16_t 	*Type1=NULL;        

    ITO_TEST_DEBUG("ito_test_send_data_in step=%d\n",step);

    if( step == 0 ) // 39-4 (2R)
    {
        Type1 = &short_4[0];  
    }
    else if( step == 1 ) // 39-1
    {
        Type1 = &short_1[0];      	
    }
    else if( step == 2 ) // 39-2
    {
        Type1 = &short_2[0];      	
    }
    else if( step == 3 ) // 39-3
    {
        Type1 = &short_3[0];        
    }
    else if( step == 4 )
    {
        Type1 = &open_1[0];        
    }
    else if( step == 5 )
    {
        Type1 = &open_2[0];      	
    }
    else if( step == 6 )
    {
        Type1 = &open_3[0];      	
    }
    else if( step == 9 )
    {
        Type1 = &open_1B[0];        
    }
    else if( step == 10 )
    {
        Type1 = &open_2B[0];      	
    } 
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11; //bank:ana, addr:h0000
    dbbus_tx_data[2] = 0x00;    
    for( i = 0; i <= 0x3E ; i++ )
    {
        dbbus_tx_data[3+2*i] = Type1[i] & 0xFF;
        dbbus_tx_data[4+2*i] = ( Type1[i] >> 8 ) & 0xFF;    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+0x3F*2);
 
    dbbus_tx_data[2] = 0x7A * 2; //bank:ana, addr:h007a
    for( i = 0x7A; i <= 0x7D ; i++ )
    {
        dbbus_tx_data[3+2*(i-0x7A)] = 0;
        dbbus_tx_data[4+2*(i-0x7A)] = 0;    	    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+8);  
    
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12; //bank:ana2, addr:h0005
      
    dbbus_tx_data[2] = 0x05 * 2;
    dbbus_tx_data[3] = Type1[128+0x05] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x05] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x0B * 2; //bank:ana2, addr:h000b
    dbbus_tx_data[3] = Type1[128+0x0B] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x0B] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x12 * 2; //bank:ana2, addr:h0012
    dbbus_tx_data[3] = Type1[128+0x12] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x12] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x15 * 2; //bank:ana2, addr:h0015
    dbbus_tx_data[3] = Type1[128+0x15] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x15] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        
/*
#if 1//for AC mod --showlo
        dbbus_tx_data[1] = 0x13;
        dbbus_tx_data[2] = 0x12 * 2;
        dbbus_tx_data[3] = 0X30;
        dbbus_tx_data[4] = 0X30;
        ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        

        
        dbbus_tx_data[2] = 0x14 * 2;
        dbbus_tx_data[3] = 0X30;
        dbbus_tx_data[4] = 0X30;
        ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);     

        
        dbbus_tx_data[1] = 0x12;
             for (i = 0x0D; i <= 0x10;i++ )//for AC noise(++)
	{
	    dbbus_tx_data[2] = i * 2;
	    dbbus_tx_data[3] = Type1[128+i] & 0xFF;
	    dbbus_tx_data[4] = ( Type1[128+i] >> 8 ) & 0xFF;
	    ito_test_i2c_write( ITO_TEST_ADDR_REG,  dbbus_tx_data,5 );  
	}

       for (i = 0x16; i <= 0x18; i++)//for AC noise
	{
	    dbbus_tx_data[2] = i * 2;
	    dbbus_tx_data[3] = Type1[128+i] & 0xFF;
	    dbbus_tx_data[4] = ( Type1[128+i] >> 8 ) & 0xFF;
	    ito_test_i2c_write( ITO_TEST_ADDR_REG, dbbus_tx_data,5 );  
	}
#endif
*/
}

static void ito_test_set_v( uint8_t Enable, uint8_t Prs)	
{
    uint16_t    u16RegValue;        
    
    u16RegValue = ito_test_ReadReg( 0x12, 0x08); //bank:ana2, addr:h0004
    u16RegValue = u16RegValue & 0xF1; 							
    if ( Prs == 0 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0C); 		
    }
    else if ( Prs == 1 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0E); 		     	
    }
    else
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x02); 			
    }    
    
    if ( Enable )
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06); //bank:ana, addr:h0003  
        ito_test_WriteReg( 0x11, 0x06, u16RegValue| 0x03);   	
    }
    else
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        u16RegValue = u16RegValue & 0xFC;					
        ito_test_WriteReg( 0x11, 0x06, u16RegValue);         
    }
}

static void ito_test_set_c( uint8_t Csub_Step )
{
    uint8_t i;
    uint8_t dbbus_tx_data[MAX_CHNL_NUM+3];
    uint8_t HighLevel_Csub = false;
    uint8_t Csub_new;
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11; //bank:ana, addr:h0042       
    dbbus_tx_data[2] = 0x84;        
    for( i = 0; i < MAX_CHNL_NUM; i++ )
    {
		Csub_new = Csub_Step;        
        HighLevel_Csub = false;   
        if( Csub_new > 0x1F )
        {
            Csub_new = Csub_new - 0x14;
            HighLevel_Csub = true;
        }
           
        dbbus_tx_data[3+i] = Csub_new & 0x1F;        
        if( HighLevel_Csub == true )
        {
            dbbus_tx_data[3+i] |= BIT5;
        }
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);

    dbbus_tx_data[2] = 0xB4; //bank:ana, addr:h005a        
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);
}

static void ito_test_sw( void )
{
    ito_test_WriteReg( 0x11, 0x00, 0xFFFF ); //bank:ana, addr:h0000
    ito_test_WriteReg( 0x11, 0x00, 0x0000 );
    mdelay( 50 );
}

static void ito_open_test_first(uint8_t item_id, int16_t* s16_raw_data, s8* data_flag)		
{
    uint8_t     loop;
    uint8_t     i, j;
    int16_t     s16_raw_data_tmp[MAX_CHNL_NUM] = {0};
    uint8_t     num_of_sensor, num_of_sensor2, total_sensor = 0;
    uint16_t	u16RegValue;
    uint8_t 	*pMapping = NULL;
    
    num_of_sensor = 0;
    num_of_sensor2 = 0;	
	
    ITO_TEST_DEBUG("ito_open_test_first() item_id=%d\n", item_id);
    // stop cpu
    ito_test_WriteReg( 0x0F, 0xE6, 0x01 ); //bank:mheg5, addr:h0073

    ito_test_WriteReg( 0x1E, 0x24, 0x0500 ); //bank:chip, addr:h0012
    ito_test_WriteReg( 0x1E, 0x2A, 0x0000 ); //bank:chip, addr:h0015
    ito_test_WriteReg( 0x1E, 0xE6, 0x6E00 ); //bank:chip, addr:h0073
    ito_test_WriteReg( 0x1E, 0xE8, 0x0071 ); //bank:chip, addr:h0074
	    
    if ( item_id == 40 )    			
    {
        pMapping = &MAP1[0];
        if ( ito_test_2r )
        {
            total_sensor = ito_test_trianglenum/2; 
        }
        else
        {
            total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
        }
    }
    else if( item_id == 41 )    		
    {
        pMapping = &MAP2[0];
        if ( ito_test_2r )
        {
            total_sensor = ito_test_trianglenum/2; 
        }
        else
        {
            total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
        }
    }
    else if( item_id == 42 )    		
    {
        pMapping = &MAP3[0];      
        total_sensor = ito_test_trianglenum + ito_test_keynum+ ito_test_dummynum; 
    }
        	    
    loop = 1;
    if ( item_id != 42 )
    {
	      if(total_sensor>11)
        {
            loop = 2;
        }
    }	
    
    ITO_TEST_DEBUG("loop=%d\n", loop);
	
    for ( i = 0; i < loop; i ++ )
    {
        if ( i == 0 )
        {
            ito_test_send_data_in( item_id - 36 );
        }
        else
        { 
            if ( item_id == 40 )
            { 
                ito_test_send_data_in( 9 );
            }
            else
            { 		
                ito_test_send_data_in( 10 );
            }
        }
        
        ito_test_disable_filter_noise_detect();
	
        ito_test_set_v(1,0);    
        u16RegValue = ito_test_ReadReg( 0x11, 0x0E ); //bank:ana, addr:h0007   			
        ito_test_WriteReg( 0x11, 0x0E, u16RegValue | BIT11 );				 		
	
        if ( g_LTP == 1 )
        {
	    	    ito_test_set_c( 32 );
	    }	    	
        else
        {	    	
	    	    ito_test_set_c( 0 );
        }
        
        ito_test_sw();
		
        if ( i == 0 )	 
        {      
            num_of_sensor=ito_test_get_data_out(  s16_raw_data_tmp );
            ITO_TEST_DEBUG("num_of_sensor=%d;\n",num_of_sensor);
        }
        else	
        {      
            num_of_sensor2=ito_test_get_data_out(  &s16_raw_data_tmp[num_of_sensor] );
            ITO_TEST_DEBUG("num_of_sensor=%d;num_of_sensor2=%d\n",num_of_sensor,num_of_sensor2);
        }
    }
    
    for ( j = 0; j < total_sensor; j ++ )
    {
        if ( g_LTP == 1 )
        {
            s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j] + 4096;
            data_flag[pMapping[j]] = 1;
        }
        else
        {
            s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j];	
            data_flag[pMapping[j]] = 1;
        }
    }	

    return;
}

typedef enum
{
	ITO_TEST_OK = 0,
	ITO_TEST_FAIL,
	ITO_TEST_GET_TP_TYPE_ERROR,
	ITO_TEST_UNDEFINED_ERROR

} ITO_TEST_RET;

ITO_TEST_RET ito_open_test_second(u8 item_id)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;

    ITO_TEST_DEBUG("ito_open_test_second() item_id=%d\n", item_id);

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];
		}
    }
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 + OPEN_TEST_NON_BORDER_AREA_THRESHOLD) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 - OPEN_TEST_NON_BORDER_AREA_THRESHOLD) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + OPEN_TEST_BORDER_AREA_THRESHOLD) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2 ) * ( 100 - OPEN_TEST_BORDER_AREA_THRESHOLD) / 100 ;
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min);

	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
		{
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min || s16_raw_data_1[MAP40_1[i]] < OPEN_TEST_NON_BORDER_AREA_MINI_THRESHOLD)
			{
				g_fail_channel[fail_channel_count] = MAP40_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}

		for (i=0; i<2; i++)
		{
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
		{
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min || s16_raw_data_2[MAP41_1[i]] < OPEN_TEST_NON_BORDER_AREA_MINI_THRESHOLD) 
			{
				g_fail_channel[fail_channel_count] = MAP41_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}

		for (i=0; i<2; i++)
		{
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min)
			{ 
				g_fail_channel[fail_channel_count] = MAP41_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	return ret;
}

ITO_TEST_RET ito_open_test_second_2r (u8 item_id)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  s16_raw_data_jg_tmp3 = 0;
	s32  s16_raw_data_jg_tmp4 = 0;
	
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;
	s32  jg_tmp3_avg_Th_max =0;
	s32  jg_tmp3_avg_Th_min =0;
	s32  jg_tmp4_avg_Th_max =0;
	s32  jg_tmp4_avg_Th_min =0;

	if ( item_id == 40 )    			
  {
    for (i=0; i<(ito_test_trianglenum/4)-2; i++)
    {
      s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];  //first region: non-border 
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];  //first region: border
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
		  s16_raw_data_jg_tmp3 += s16_raw_data_1[MAP40_3[i]];  //second region: non-border
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp4 += s16_raw_data_1[MAP40_4[i]];  //second region: border
		}
  }
  else if( item_id == 41 )    		
  {
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
		  s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];  //first region: non-border
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];  //first region: border
		}
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
		  s16_raw_data_jg_tmp3 += s16_raw_data_2[MAP41_3[i]];  //second region: non-border
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp4 += s16_raw_data_2[MAP41_4[i]];  //second region: border
		}
	}

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 + OPEN_TEST_NON_BORDER_AREA_THRESHOLD) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 - OPEN_TEST_NON_BORDER_AREA_THRESHOLD) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + OPEN_TEST_BORDER_AREA_THRESHOLD) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2) * ( 100 - OPEN_TEST_BORDER_AREA_THRESHOLD) / 100 ;
		jg_tmp3_avg_Th_max = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 + OPEN_TEST_NON_BORDER_AREA_THRESHOLD) / 100 ;
	    jg_tmp3_avg_Th_min = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 - OPEN_TEST_NON_BORDER_AREA_THRESHOLD) / 100 ;
        jg_tmp4_avg_Th_max = (s16_raw_data_jg_tmp4 / 2) * ( 100 + OPEN_TEST_BORDER_AREA_THRESHOLD) / 100 ;
	    jg_tmp4_avg_Th_min = (s16_raw_data_jg_tmp4 / 2) * ( 100 - OPEN_TEST_BORDER_AREA_THRESHOLD) / 100 ;
		
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d;sum3=%d;max3=%d;min3=%d;sum4=%d;max4=%d;min4=%d;\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min,s16_raw_data_jg_tmp3,jg_tmp3_avg_Th_max,jg_tmp3_avg_Th_min,s16_raw_data_jg_tmp4,jg_tmp4_avg_Th_max,jg_tmp4_avg_Th_min);


	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min || s16_raw_data_1[MAP40_1[i]] < OPEN_TEST_NON_BORDER_AREA_MINI_THRESHOLD) 
			{
				g_fail_channel[fail_channel_count] = MAP40_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_2[i];				
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		} 
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_1[MAP40_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_1[MAP40_3[i]] < jg_tmp3_avg_Th_min || s16_raw_data_1[MAP40_3[i]] < OPEN_TEST_NON_BORDER_AREA_MINI_THRESHOLD) 
			{
				g_fail_channel[fail_channel_count] = MAP40_3[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_1[MAP40_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_1[MAP40_4[i]] < jg_tmp4_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_4[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min || s16_raw_data_2[MAP41_1[i]] < OPEN_TEST_NON_BORDER_AREA_MINI_THRESHOLD) 
			{
				g_fail_channel[fail_channel_count] = MAP41_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_2[MAP41_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_2[MAP41_3[i]] < jg_tmp3_avg_Th_min || s16_raw_data_2[MAP41_3[i]] < OPEN_TEST_NON_BORDER_AREA_MINI_THRESHOLD) 
			{	
				g_fail_channel[fail_channel_count] = MAP41_3[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_2[MAP41_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_2[MAP41_4[i]] < jg_tmp4_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_4[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		} 
	}

	return ret;
}

static ITO_TEST_RET ito_open_test_interface(void)
{
    ITO_TEST_RET ret1 = ITO_TEST_OK, ret2 = ITO_TEST_OK, ret3 = ITO_TEST_OK;
    uint16_t i = 0;
#ifdef DMA_IIC
    _msg_dma_alloc();
#endif
    ITO_TEST_DEBUG("open test start\n");

    ito_test_set_iic_rate(50000);
    ito_test_disable_irq();
    ito_test_reset();
    if(!ito_test_choose_TpType())
    {
        ITO_TEST_DEBUG("choose tpType fail\n");
        ret1 = ITO_TEST_GET_TP_TYPE_ERROR;
        goto ITO_TEST_END;
    }
    ito_test_EnterSerialDebugMode();
    mdelay(100);
    ITO_TEST_DEBUG("EnterSerialDebugMode\n");
    // stop cpu
    ito_test_WriteReg8Bit ( 0x0F, 0xE6, 0x01 ); //bank:mheg5, addr:h0073
    // stop watch dog
    ito_test_WriteReg ( 0x3C, 0x60, 0xAA55 ); //bank:reg_PIU_MISC_0, addr:h0030
    ITO_TEST_DEBUG("stop mcu and disable watchdog V.005\n");   
    mdelay(50);
    
    for(i = 0;i < MAX_CHNL_NUM;i++)
    {
        s16_raw_data_1[i] = 0;
        s16_raw_data_2[i] = 0;
        s16_raw_data_3[i] = 0;
        data_flag_1[i] = 0;
        data_flag_2[i] = 0;
        data_flag_3[i] = 0;
    }	
	
    fail_channel_count = 0; // Reset fail_channel_count to 0 before test start
	
    ito_open_test_first(40, s16_raw_data_1, data_flag_1);
    ITO_TEST_DEBUG("40 get s16_raw_data_1\n");
    if(ito_test_2r)
    {
        ret2=ito_open_test_second_2r(40);
    }
    else
    {
        ret2=ito_open_test_second(40);
    }
    
    ito_open_test_first(41, s16_raw_data_2, data_flag_2);
    ITO_TEST_DEBUG("41 get s16_raw_data_2\n");
    if(ito_test_2r)
    {
        ret3=ito_open_test_second_2r(41);
    }
    else
    {
        ret3=ito_open_test_second(41);
    }
    
    //ito_open_test_first(42, s16_raw_data_3, data_flag_3);
    //ITO_TEST_DEBUG("42 get s16_raw_data_3\n");
    
    ITO_TEST_END:
#ifdef DMA_IIC
    _msg_dma_free();
#endif
    ito_test_set_iic_rate(100000);
    ito_test_reset();
    ito_test_enable_irq();
    ITO_TEST_DEBUG("open test end\n");
    
    if ((ret1 != ITO_TEST_OK) && (ret2 == ITO_TEST_OK) && (ret3 == ITO_TEST_OK))
    {
        return ITO_TEST_GET_TP_TYPE_ERROR;		
    }
    else if ((ret1 == ITO_TEST_OK) && ((ret2 != ITO_TEST_OK) || (ret3 != ITO_TEST_OK)))
    {
        return ITO_TEST_FAIL;	
    }
    else
    {
        return ITO_TEST_OK;	
    }
}

static void ito_short_test_change_GPO_setting(u8 item_id)
{
    u8 dbbus_tx_data[3+GPIO_SETTING_SIZE*2] = {0};
    u16 gpoSettings[3] = {0};
    u32 i;
    
    ITO_TEST_DEBUG("ito_short_test_change_GPO_setting() item_id=%d\n", item_id);
    
    if (item_id == 0) // 39-4
    {
        gpoSettings[0] = short_4_GPO[0];		
        gpoSettings[1] = short_4_GPO[1];		
        gpoSettings[2] = short_4_GPO[2];		
        gpoSettings[2] |= (1 << (int)(PIN_GUARD_RING % 16));
    }
    else if (item_id == 1) // 39-1
    {
        gpoSettings[0] = short_1_GPO[0];		
        gpoSettings[1] = short_1_GPO[1];		
        gpoSettings[2] = short_1_GPO[2];		
        gpoSettings[2] |= (1 << (int)(PIN_GUARD_RING % 16));
    }
    else if (item_id == 2) // 39-2
    {
        gpoSettings[0] = short_2_GPO[0];		
        gpoSettings[1] = short_2_GPO[1];		
        gpoSettings[2] = short_2_GPO[2];		
        gpoSettings[2] |= (1 << (int)(PIN_GUARD_RING % 16));
    }
    else if (item_id == 3) // 39-3
    {
        gpoSettings[0] = short_3_GPO[0];		
        gpoSettings[1] = short_3_GPO[1];		
        gpoSettings[2] = short_3_GPO[2];		
        gpoSettings[2] |= (1 << (int)(PIN_GUARD_RING % 16));
    }
    else
    {
        ITO_TEST_DEBUG("Invalid item id for changing GPIO setting of short test.\n");

        return;
    }

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12;
    dbbus_tx_data[2] = 0x48;

    for (i = 0; i < GPIO_SETTING_SIZE; i ++)
    {
        dbbus_tx_data[3+2*i] = gpoSettings[i] & 0xFF;
        dbbus_tx_data[4+2*i] = (gpoSettings[i] >> 8) & 0xFF;    	
    }

    ito_test_i2c_write(ITO_TEST_ADDR_REG, &dbbus_tx_data[0], 3+GPIO_SETTING_SIZE*2);    
}

static void ito_short_test_change_Rmode_setting(uint8_t mode)
{
    uint8_t dbbus_tx_data[6];

    ITO_TEST_DEBUG("ito_short_test_change_Rmode_setting() mode=%d\n", mode);

    // AFE R-mode enable(Bit-12)
    ito_test_WriteReg8Bit( 0x11, 0x03, 0x10 );

    // drv_mux_OV (Bit-8 1:enable)
    ito_test_WriteReg8Bit( 0x11, 0x07, 0x55 );
    
    if (mode == 1) // P_CODE: 0V
    {
        ito_test_WriteReg( 0x11, 0x0E, 0x073A );
    }
    else if (mode == 0) // N_CODE: 2.4V
    {
        ito_test_WriteReg( 0x11, 0x0E, 0x073B );
    }

    // SW2 rising & SW3 rising return to 0
    ito_test_WriteReg8Bit( 0x12, 0x27, 0x01 );
    // turn off the chopping
    ito_test_WriteReg8Bit( 0x12, 0x08, 0x0C );
    // idle driver ov
    ito_test_WriteReg8Bit( 0x12, 0x41, 0xC0 );
	  
	  // AFE ov
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12;
    dbbus_tx_data[2] = 0x44;
    dbbus_tx_data[3] = 0xFF;
    dbbus_tx_data[4] = 0xFF;
    dbbus_tx_data[5] = 0xFF;

    ito_test_i2c_write(ITO_TEST_ADDR_REG, &dbbus_tx_data[0], 6);        
}	

static void ito_short_test_first(uint8_t item_id , int16_t* s16_raw_data, s8* data_flag)		
{
    uint8_t     i;
    int16_t     s16_raw_data_tmp[MAX_CHNL_NUM] = {0};
    int16_t     s16_raw_data_tmp2[MAX_CHNL_NUM] = {0};
    uint8_t     num_of_sensor, num_of_sensor2, num_of_sensor_mapping_1, num_of_sensor_mapping_2, sensor_count = 0;
    uint8_t 	*pMapping = NULL;
    

    ITO_TEST_DEBUG("ito_short_test_first() item_id=%d\n", item_id);
    // stop cpu
    ito_test_WriteReg( 0x0F, 0xE6, 0x01 ); //bank:mheg5, addr:h0073
    // chip top op0
    ito_test_WriteReg( 0x1E, 0x24, 0x0500 ); //bank:chip, addr:h0012
    ito_test_WriteReg( 0x1E, 0x2A, 0x0000 ); //bank:chip, addr:h0015
    ito_test_WriteReg( 0x1E, 0xE6, 0x6E00 ); //bank:chip, addr:h0073
    ito_test_WriteReg( 0x1E, 0xE8, 0x0071 ); //bank:chip, addr:h0074
	    
    if ((ito_test_trianglenum + ito_test_keynum + ito_test_dummynum) % 2 != 0)
    {
        num_of_sensor_mapping_1 = (ito_test_trianglenum + ito_test_keynum + ito_test_dummynum) / 2 + 1;
        num_of_sensor_mapping_2 = num_of_sensor_mapping_1;
    }
    else
    {
        num_of_sensor_mapping_1 = (ito_test_trianglenum + ito_test_keynum + ito_test_dummynum) / 2;
        num_of_sensor_mapping_2 = num_of_sensor_mapping_1;
        if (num_of_sensor_mapping_2 % 2 != 0)
        {	
            num_of_sensor_mapping_2 ++;
        }
    }        

    if ( item_id == 0 ) // 39-4 (2R)    			
    {
        pMapping = &SHORT_MAP4[0];
        sensor_count = ito_test_trianglenum/2; 
    }
    else if( item_id == 1 ) // 39-1    			    		
    {
        pMapping = &SHORT_MAP1[0];
        sensor_count = num_of_sensor_mapping_1; 
    }
    else if( item_id == 2 ) // 39-2   		
    {
        pMapping = &SHORT_MAP2[0];      
        sensor_count = num_of_sensor_mapping_2; 
    }
    else if( item_id == 3 ) // 39-3    		
    {
        pMapping = &SHORT_MAP3[0];      
        sensor_count = ito_test_trianglenum; 
    }
    ITO_TEST_DEBUG("sensor_count=%d\n", sensor_count);
        	    
    ito_test_send_data_in( item_id );
    
    ito_test_disable_filter_noise_detect();

    ito_short_test_change_Rmode_setting(1);
    ito_short_test_change_GPO_setting(item_id);
    ito_test_sw();

    num_of_sensor = ito_test_get_data_out(  s16_raw_data_tmp );
    ITO_TEST_DEBUG("num_of_sensor=%d\n", num_of_sensor);

    ito_short_test_change_Rmode_setting(0);
    ito_short_test_change_GPO_setting(item_id);
    ito_test_sw();

    num_of_sensor2 = ito_test_get_data_out(  s16_raw_data_tmp2 );
    ITO_TEST_DEBUG("num_of_sensor2=%d\n", num_of_sensor2);
    
    for ( i = 0; i < sensor_count; i ++ )
    {
        s16_raw_data[pMapping[i]] = s16_raw_data_tmp[i] - s16_raw_data_tmp2[i];	
        data_flag[pMapping[i]] = 1;
    }	
}

static ITO_TEST_RET ito_short_test_second(u8 item_id)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
    u8 i;
    u8 num_of_sensor_mapping_1, num_of_sensor_mapping_2, sensor_count = 0;
	
    ITO_TEST_DEBUG("ito_short_test_second() item_id=%d\n", item_id);

    if ((ito_test_trianglenum + ito_test_keynum + ito_test_dummynum) % 2 != 0)
    {
        num_of_sensor_mapping_1 = (ito_test_trianglenum + ito_test_keynum + ito_test_dummynum) / 2 + 1;
        num_of_sensor_mapping_2 = num_of_sensor_mapping_1;
    }
    else
    {
        num_of_sensor_mapping_1 = (ito_test_trianglenum + ito_test_keynum + ito_test_dummynum) / 2;
        num_of_sensor_mapping_2 = num_of_sensor_mapping_1;
        if (num_of_sensor_mapping_2 % 2 != 0)
        {	
            num_of_sensor_mapping_2 ++;
        }
    }        

    if ( item_id == 0 ) // 39-4 (2R)   
    {
        sensor_count = ito_test_trianglenum/2;
        
        for (i = 0; i < sensor_count; i ++)
        {
            if (s16_raw_data_4[SHORT_MAP4[i]] > SHORT_TEST_THRESHOLD)
            {
                g_fail_channel[fail_channel_count] = SHORT_MAP4[i];
                fail_channel_count ++; 
                ret = ITO_TEST_FAIL;
            }
        }
    }
    else if ( item_id == 1 ) // 39-1
    {
        sensor_count = num_of_sensor_mapping_1;
        
        for (i = 0; i < sensor_count; i ++)
        {
            if (s16_raw_data_1[SHORT_MAP1[i]] > SHORT_TEST_THRESHOLD)
            {
                g_fail_channel[fail_channel_count] = SHORT_MAP1[i];
                fail_channel_count ++; 
                ret = ITO_TEST_FAIL;
            }
        }
    }
    else if ( item_id == 2 ) // 39-2
    {
        sensor_count = num_of_sensor_mapping_2;
        
        for (i = 0; i < sensor_count; i ++)
        {
            if (s16_raw_data_2[SHORT_MAP2[i]] > SHORT_TEST_THRESHOLD)
            {
                g_fail_channel[fail_channel_count] = SHORT_MAP2[i];
                fail_channel_count ++; 
                ret = ITO_TEST_FAIL;
            }
        }
    }
    else if ( item_id == 3 ) // 39-3
    {
        sensor_count = ito_test_trianglenum;
        
        for (i = 0; i < sensor_count; i ++)
        {
            if (s16_raw_data_3[SHORT_MAP3[i]] > SHORT_TEST_THRESHOLD)
            {
                g_fail_channel[fail_channel_count] = SHORT_MAP3[i];
                fail_channel_count ++; 
                ret = ITO_TEST_FAIL;
            }
        }
    }
    ITO_TEST_DEBUG("sensor_count=%d\n", sensor_count);

    return ret;
}

static ITO_TEST_RET ito_short_test_interface(void)
{
    ITO_TEST_RET ret1 = ITO_TEST_OK, ret2 = ITO_TEST_OK, ret3 = ITO_TEST_OK, ret4 = ITO_TEST_OK, ret5 = ITO_TEST_OK;
    u16 i = 0;
#ifdef DMA_IIC
    _msg_dma_alloc();
#endif

    ITO_TEST_DEBUG("short test start\n");

    ito_test_set_iic_rate(50000);
    
    ito_test_disable_irq();
    ito_test_reset();
    if(!ito_test_choose_TpType())
    {
        ITO_TEST_DEBUG("choose tpType fail\n");
        ret1 = ITO_TEST_GET_TP_TYPE_ERROR;
        goto ITO_TEST_END;
    }
    ito_test_EnterSerialDebugMode();
    mdelay(100);
    ITO_TEST_DEBUG("EnterSerialDebugMode\n");
    // stop cpu
    ito_test_WriteReg8Bit ( 0x0F, 0xE6, 0x01 ); //bank:mheg5, addr:h0073
    // stop watch dog
    ito_test_WriteReg ( 0x3C, 0x60, 0xAA55 ); //bank:reg_PIU_MISC_0, addr:h0030
    ITO_TEST_DEBUG("stop mcu and disable watchdog V.005\n");   
    mdelay(50);
    
    for(i = 0; i < MAX_CHNL_NUM; i ++)
    {
        s16_raw_data_1[i] = 0;
        s16_raw_data_2[i] = 0;
        s16_raw_data_3[i] = 0;
        s16_raw_data_4[i] = 0;
        data_flag_1[i] = 0;
        data_flag_2[i] = 0;
        data_flag_3[i] = 0;
        data_flag_4[i] = 0;
    }	
	
    fail_channel_count = 0; // Reset fail_channel_count to 0 before test start
	
    ito_short_test_first(1, s16_raw_data_1, data_flag_1);
    ITO_TEST_DEBUG("1 get s16_raw_data_1\n");
    ret2 = ito_short_test_second(1);
    
    ito_short_test_first(2, s16_raw_data_2, data_flag_2);
    ITO_TEST_DEBUG("2 get s16_raw_data_2\n");
    ret3 = ito_short_test_second(2);

    ito_short_test_first(3, s16_raw_data_3, data_flag_3);
    ITO_TEST_DEBUG("3 get s16_raw_data_3\n");
    ret4 = ito_short_test_second(3);
    
    if(ito_test_2r)
    {
        ito_short_test_first(0, s16_raw_data_4, data_flag_4);
        ITO_TEST_DEBUG("0 get s16_raw_data_4\n");
        ret5 = ito_short_test_second(0);
    }

    ITO_TEST_END:
#ifdef DMA_IIC
    _msg_dma_free();
#endif
    ito_test_set_iic_rate(100000);
    ito_test_reset();
    ito_test_enable_irq();
    ITO_TEST_DEBUG("short test end\n");
    
    if ((ret1 != ITO_TEST_OK) && (ret2 == ITO_TEST_OK) && (ret3 == ITO_TEST_OK) && (ret4 == ITO_TEST_OK) && (ret5 == ITO_TEST_OK))
    {
        return ITO_TEST_GET_TP_TYPE_ERROR;		
    }
    else if ((ret1 == ITO_TEST_OK) && ((ret2 != ITO_TEST_OK) || (ret3 != ITO_TEST_OK) || (ret4 != ITO_TEST_OK) || (ret5 != ITO_TEST_OK)))
    {
        return ITO_TEST_FAIL;	
    }
    else
    {
        return ITO_TEST_OK;	
    }
}

#include <linux/proc_fs.h>
#define ITO_TEST_AUTHORITY 0777 
static struct proc_dir_entry *msg_ito_test = NULL;
static struct proc_dir_entry *debug = NULL;
static struct proc_dir_entry *debug_on_off = NULL;
static struct proc_dir_entry *open_test = NULL;
static struct proc_dir_entry *short_test = NULL;
static struct proc_dir_entry *fail_channel = NULL;
static struct proc_dir_entry *data = NULL;
#define PROC_MSG_ITO_TEST      "msg-ito-test"
#define PROC_ITO_TEST_DEBUG      "debug"
#define PROC_ITO_TEST_DEBUG_ON_OFF     "debug-on-off"
#define PROC_ITO_TEST_OPEN     "open"
#define PROC_ITO_TEST_SHORT     "short"
#define PROC_ITO_TEST_FAIL_CHANNEL     "fail-channel"
#define PROC_ITO_TEST_DATA      "data"
ITO_TEST_RET g_ito_test_ret = ITO_TEST_OK;

	ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
	ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
//static ssize_t ito_test_proc_read_debug(char *page, char **start, off_t off, int count, int *eof, void *data)
static ssize_t ito_test_proc_read_debug(struct file *file, char __user *page, size_t count, loff_t *eof)
{
    int cnt = 0;
    
    cnt = sprintf(page, "%d", g_ito_test_ret);

    *eof = 1;

    return cnt;
}

//static ssize_t ito_test_proc_write_debug(struct file *file, const char *buffer, unsigned long count, void *data)
static ssize_t ito_test_proc_write_debug (struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{    
    u32 mode = 0;
    u32 i = 0;
    
    ITO_TEST_DEBUG_MUST("buffer = %s\n", buffer);

    if (buffer != NULL)
    {
        sscanf(buffer, "%x", &mode);   

        ITO_TEST_DEBUG_MUST("Mp Test Mode = 0x%x\n", mode);

        if (mode == ITO_TEST_MODE_OPEN_TEST) //open test
        {
            ito_test_mode = ITO_TEST_MODE_OPEN_TEST;
            g_ito_test_ret = ito_open_test_interface();
        }
        else if (mode == ITO_TEST_MODE_SHORT_TEST) //short test
        {
            ito_test_mode = ITO_TEST_MODE_SHORT_TEST;
            g_ito_test_ret = ito_short_test_interface();
        }
        else
        {
            ITO_TEST_DEBUG_MUST("*** Undefined MP Test Mode ***\n");

            g_ito_test_ret = ITO_TEST_UNDEFINED_ERROR;
        }
    }

    if(ITO_TEST_OK==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
    }
    else if(ITO_TEST_FAIL==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
    }
    else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
    }
    else if(ITO_TEST_UNDEFINED_ERROR==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_UNDEFINED_ERROR");
    }

    ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
    mdelay(5);

    for(i=0;i<MAX_CHNL_NUM;i++)
    {
        ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
    }
    mdelay(5);
    for(i=0;i<MAX_CHNL_NUM;i++)
    {
        ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
    }
    mdelay(5);
    for(i=0;i<MAX_CHNL_NUM;i++)
    {
        ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
    }
    mdelay(5);
    
    if (ito_test_mode == ITO_TEST_MODE_SHORT_TEST && ito_test_2r == 1)
    {
        for(i=0;i<MAX_CHNL_NUM;i++)
        {
            ITO_TEST_DEBUG_MUST("data_4[%d]=%d;\n",i,s16_raw_data_4[i]);
        }
        mdelay(5);
    }

    return count;
}

//static int ito_test_proc_read_debug_on_off(char *page, char **start, off_t off, int count, int *eof, void *data)
static ssize_t ito_test_proc_read_debug_on_off (struct file *file, char __user *page, size_t size, loff_t *eof)
{
    int cnt= 0;
    
    bItoTestDebug = 1;
    ITO_TEST_DEBUG_MUST("on debug bItoTestDebug = %d",bItoTestDebug);
    
    *eof = 1;
    return cnt;
}

//static int ito_test_proc_write_debug_on_off(struct file *file, const char *buffer, unsigned long count, void *data)
static ssize_t ito_test_proc_write_debug_on_off (struct file *file, const char __user *page, size_t count, loff_t *eof)
{    
    bItoTestDebug = 0;
    ITO_TEST_DEBUG_MUST("off debug bItoTestDebug = %d",bItoTestDebug);
    return count;
}

//static int ito_test_proc_read_open(char *page, char **start, off_t off, int count, int *eof, void *data)
static ssize_t ito_test_proc_read_open (struct file *file, char __user *page, size_t size, loff_t *eof)
{
    int cnt = 0;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_open()\n");
    
    cnt = sprintf(page, "%d", g_ito_test_ret);

    *eof = 1;

    return cnt;
}

//static int ito_test_proc_write_open(struct file *file, const char *buffer, unsigned long count, void *data)
static ssize_t ito_test_proc_write_open (struct file *file, const char __user *buffer, size_t count, loff_t *eof)
{    
    u32 i = 0;
    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_open()\n");
    ITO_TEST_DEBUG_MUST("buffer = %s\n", buffer);

    if (buffer != NULL)
    {
        ITO_TEST_DEBUG_MUST("ITO Open Test\n");

        ito_test_mode = ITO_TEST_MODE_OPEN_TEST;
        g_ito_test_ret = ito_open_test_interface();

        if(ITO_TEST_OK==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
        }
        else if(ITO_TEST_FAIL==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
        }
        else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
        }
        else if(ITO_TEST_UNDEFINED_ERROR==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_UNDEFINED_ERROR");
        }

        ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
        mdelay(5);

        for(i=0;i<MAX_CHNL_NUM;i++)
        {
            ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
        }
        mdelay(5);
        for(i=0;i<MAX_CHNL_NUM;i++)
        {
            ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
        }
        mdelay(5);
        for(i=0;i<MAX_CHNL_NUM;i++)
        {
            ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
        }
        mdelay(5);
    }

    return count;
}

//static int ito_test_proc_read_short(char *page, char **start, off_t off, int count, int *eof, void *data)
static ssize_t ito_test_proc_read_short (struct file *file, char __user *page, size_t size, loff_t *eof)
{
    int cnt = 0;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_short()\n");
    
    cnt = sprintf(page, "%d", g_ito_test_ret);

    *eof = 1;

    return cnt;
}

//static int ito_test_proc_write_short(struct file *file, const char *buffer, unsigned long count, void *data)
static ssize_t ito_test_proc_write_short (struct file *file, const char __user *buffer, size_t count, loff_t *eof)
{    
    u32 i = 0;
    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_short()\n");
    ITO_TEST_DEBUG_MUST("buffer = %s\n", buffer);

    if (buffer != NULL)
    {
        ITO_TEST_DEBUG_MUST("ITO Short Test\n");

        ito_test_mode = ITO_TEST_MODE_SHORT_TEST;
        g_ito_test_ret = ito_short_test_interface();

        if(ITO_TEST_OK==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
        }
        else if(ITO_TEST_FAIL==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
        }
        else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
        }
        else if(ITO_TEST_UNDEFINED_ERROR==g_ito_test_ret)
        {
            ITO_TEST_DEBUG_MUST("ITO_TEST_UNDEFINED_ERROR");
        }

        ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
        mdelay(5);

        for(i=0;i<MAX_CHNL_NUM;i++)
        {
            ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
        }
        mdelay(5);
        for(i=0;i<MAX_CHNL_NUM;i++)
        {
            ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
        }
        mdelay(5);
        for(i=0;i<MAX_CHNL_NUM;i++)
        {
            ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
        }
        mdelay(5);
    
        if (ito_test_mode == ITO_TEST_MODE_SHORT_TEST && ito_test_2r == 1)
        {
            for(i=0;i<MAX_CHNL_NUM;i++)
            {
                ITO_TEST_DEBUG_MUST("data_4[%d]=%d;\n",i,s16_raw_data_4[i]);
            }
            mdelay(5);
        }
    }

    return count;
}

//static int ito_test_proc_read_fail_channel(char *page, char **start, off_t off, int count, int *eof, void *data)
static ssize_t ito_test_proc_read_fail_channel (struct file *file, char __user *page, size_t size, loff_t *eof)
{
    int cnt = 0;
    int i;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_fail_channel()\n");
    ITO_TEST_DEBUG_MUST("fail_channel_count = %d\n", fail_channel_count);
    
    for (i = 0; i < fail_channel_count; i ++)
    {
    	  page[i] = g_fail_channel[i];
    }

    *eof = 1;

    cnt = fail_channel_count;

    return cnt;
}

//static int ito_test_proc_write_fail_channel(struct file *file, const char *buffer, unsigned long count, void *data)
static ssize_t ito_test_proc_write_fail_channel (struct file *file, const char __user *buffer, size_t count, loff_t *eof)
{    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_fail_channel()\n");

    return count;
}

//static int ito_test_proc_read_data(char *page, char **start, off_t off, int count, int *eof, void *data)
static ssize_t ito_test_proc_read_data (struct file *file, char __user *page, size_t count, loff_t *eof)
{
    int cnt = 0;
    int i;
    u8 high_byte, low_byte;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_data()\n");
    
    if (ito_test_mode == ITO_TEST_MODE_OPEN_TEST)
    {
        for (i = 0; i < MAX_CHNL_NUM; i ++)
        {
            high_byte = (s16_raw_data_1[i] >> 8) & 0xFF;
            low_byte = (s16_raw_data_1[i]) & 0xFF;
    	  
            if (data_flag_1[i] == 1)
            {
                page[i*4] = 1; // indicate it is a on-use channel number
            }
            else
            {
                page[i*4] = 0; // indicate it is a non-use channel number
            }
            
            if (s16_raw_data_1[i] >= 0)
            {
                page[i*4+1] = 0; // + : a positive number
            }
            else
            {
                page[i*4+1] = 1; // - : a negative number
            }
			
            page[i*4+2] = high_byte;
            page[i*4+3] = low_byte;
        }

        for (i = 0; i < MAX_CHNL_NUM; i ++)
        {
            high_byte = (s16_raw_data_2[i] >> 8) & 0xFF;
            low_byte = (s16_raw_data_2[i]) & 0xFF;
        
            if (data_flag_2[i] == 1)
            {
                page[i*4+MAX_CHNL_NUM*4] = 1; // indicate it is a on-use channel number
            }
            else
            {
                page[i*4+MAX_CHNL_NUM*4] = 0; // indicate it is a non-use channel number
            }

            if (s16_raw_data_2[i] >= 0)
            {
                page[(i*4+1)+MAX_CHNL_NUM*4] = 0; // + : a positive number
            }
            else
            {
                page[(i*4+1)+MAX_CHNL_NUM*4] = 1; // - : a negative number
            }

            page[(i*4+2)+MAX_CHNL_NUM*4] = high_byte;
            page[(i*4+3)+MAX_CHNL_NUM*4] = low_byte;
        }

        cnt = MAX_CHNL_NUM*8;
    }
    else if (ito_test_mode == ITO_TEST_MODE_SHORT_TEST)
    {
        for (i = 0; i < MAX_CHNL_NUM; i ++)
        {
            high_byte = (s16_raw_data_1[i] >> 8) & 0xFF;
            low_byte = (s16_raw_data_1[i]) & 0xFF;

            if (data_flag_1[i] == 1)
            {
                page[i*4] = 1; // indicate it is a on-use channel number
            }
            else
            {
                page[i*4] = 0; // indicate it is a non-use channel number
            }

            if (s16_raw_data_1[i] >= 0)
            {
                page[i*4+1] = 0; // + : a positive number
            }
            else
            {
                page[i*4+1] = 1; // - : a negative number
            }
			
            page[i*4+2] = high_byte;
            page[i*4+3] = low_byte;
        }

        for (i = 0; i < MAX_CHNL_NUM; i ++)
        {
            high_byte = (s16_raw_data_2[i] >> 8) & 0xFF;
            low_byte = (s16_raw_data_2[i]) & 0xFF;
        
            if (data_flag_2[i] == 1)
            {
                page[i*4+MAX_CHNL_NUM*4] = 1; // indicate it is a on-use channel number
            }
            else
            {
                page[i*4+MAX_CHNL_NUM*4] = 0; // indicate it is a non-use channel number
            }

            if (s16_raw_data_2[i] >= 0)
            {
                page[(i*4+1)+MAX_CHNL_NUM*4] = 0; // + : a positive number
            }
            else
            {
                page[(i*4+1)+MAX_CHNL_NUM*4] = 1; // - : a negative number
            }

            page[i*4+2+MAX_CHNL_NUM*4] = high_byte;
            page[i*4+3+MAX_CHNL_NUM*4] = low_byte;
        }

        for (i = 0; i < MAX_CHNL_NUM; i ++)
        {
            high_byte = (s16_raw_data_3[i] >> 8) & 0xFF;
            low_byte = (s16_raw_data_3[i]) & 0xFF;
        
            if (data_flag_3[i] == 1)
            {
                page[i*4+MAX_CHNL_NUM*8] = 1; // indicate it is a on-use channel number
            }
            else
            {
                page[i*4+MAX_CHNL_NUM*8] = 0; // indicate it is a non-use channel number
            }

            if (s16_raw_data_3[i] >= 0)
            {
                page[(i*4+1)+MAX_CHNL_NUM*8] = 0; // + : a positive number
            }
            else
            {
                page[(i*4+1)+MAX_CHNL_NUM*8] = 1; // - : a negative number
            }

            page[(i*4+2)+MAX_CHNL_NUM*8] = high_byte;
            page[(i*4+3)+MAX_CHNL_NUM*8] = low_byte;
        }

        if (ito_test_2r)
        {
            for (i = 0; i < MAX_CHNL_NUM; i ++)
            {
                high_byte = (s16_raw_data_4[i] >> 8) & 0xFF;
                low_byte = (s16_raw_data_4[i]) & 0xFF;
        
                if (data_flag_4[i] == 1)
                {
                    page[i*4+MAX_CHNL_NUM*12] = 1; // indicate it is a on-use channel number
                }
                else
                {
                    page[i*4+MAX_CHNL_NUM*12] = 0; // indicate it is a non-use channel number
                }

                if (s16_raw_data_4[i] >= 0)
                {
                    page[(i*4+1)+MAX_CHNL_NUM*12] = 0; // + : a positive number
                }
                else
                {
                    page[(i*4+1)+MAX_CHNL_NUM*12] = 1; // - : a negative number
                }

                page[(i*4+2)+MAX_CHNL_NUM*12] = high_byte;
                page[(i*4+3)+MAX_CHNL_NUM*12] = low_byte;
            }
        }
        
        cnt = MAX_CHNL_NUM*16;
    }
    else 
    {
        ITO_TEST_DEBUG_MUST("*** Undefined MP Test Mode ***\n");
    }

    *eof = 1;
    
    return cnt;
}

//static int ito_test_proc_write_data(struct file *file, const char *buffer, unsigned long count, void *data)
static ssize_t ito_test_proc_write_data (struct file *file, const char __user *buffer, size_t count, loff_t *eof)
{    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_data()\n");

    return count;
}

static const struct file_operations debug_ops = {
    .owner = THIS_MODULE,
    .read = ito_test_proc_read_debug,
    .write = ito_test_proc_write_debug,
};

static const struct file_operations debug_on_off_ops = {
    .owner = THIS_MODULE,
    .read = ito_test_proc_read_debug_on_off,
    .write = ito_test_proc_write_debug_on_off,
};

static const struct file_operations open_test_ops = {
    .owner = THIS_MODULE,
    .read = ito_test_proc_read_open,
    .write = ito_test_proc_write_open,
};

static const struct file_operations short_test_ops = {
    .owner = THIS_MODULE,
    .read = ito_test_proc_read_short,
    .write = ito_test_proc_write_short,
};

static const struct file_operations fail_channel_ops = {
    .owner = THIS_MODULE,
    .read = ito_test_proc_read_fail_channel,
    .write = ito_test_proc_write_fail_channel,
};

static const struct file_operations data_ops = {
    .owner = THIS_MODULE,
    .read = ito_test_proc_read_data,
    .write = ito_test_proc_write_data,
};

//static void ito_test_create_entry(void)
void ito_test_create_entry(struct i2c_client *i2c_client)
{
    g_I2cClient = i2c_client;

    msg_ito_test = proc_mkdir(PROC_MSG_ITO_TEST, NULL);
#if 0
    debug = create_proc_entry(PROC_ITO_TEST_DEBUG, ITO_TEST_AUTHORITY, msg_ito_test);
    debug_on_off= create_proc_entry(PROC_ITO_TEST_DEBUG_ON_OFF, ITO_TEST_AUTHORITY, msg_ito_test);
    open_test = create_proc_entry(PROC_ITO_TEST_OPEN, ITO_TEST_AUTHORITY, msg_ito_test);
    short_test = create_proc_entry(PROC_ITO_TEST_SHORT, ITO_TEST_AUTHORITY, msg_ito_test);
    fail_channel = create_proc_entry(PROC_ITO_TEST_FAIL_CHANNEL, ITO_TEST_AUTHORITY, msg_ito_test);
    data = create_proc_entry(PROC_ITO_TEST_DATA, ITO_TEST_AUTHORITY, msg_ito_test);
#endif
    debug = proc_create(PROC_ITO_TEST_DEBUG, ITO_TEST_AUTHORITY, msg_ito_test, &debug_ops);
    debug_on_off= proc_create(PROC_ITO_TEST_DEBUG_ON_OFF, ITO_TEST_AUTHORITY, msg_ito_test, &debug_on_off_ops);
    open_test = proc_create(PROC_ITO_TEST_OPEN, ITO_TEST_AUTHORITY, msg_ito_test, &open_test_ops);
    short_test = proc_create(PROC_ITO_TEST_SHORT, ITO_TEST_AUTHORITY, msg_ito_test, &short_test_ops);
    fail_channel = proc_create(PROC_ITO_TEST_FAIL_CHANNEL, ITO_TEST_AUTHORITY, msg_ito_test, &fail_channel_ops);
    data = proc_create(PROC_ITO_TEST_DATA, ITO_TEST_AUTHORITY, msg_ito_test, &data_ops);

    if (NULL==debug) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG failed\n");
    } 
    else 
    {
//        debug->read_proc = ito_test_proc_read_debug;
//        debug->write_proc = ito_test_proc_write_debug;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG OK\n");
    }

    if (NULL==debug_on_off) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF failed\n");
    } 
    else 
    {
//        debug_on_off->read_proc = ito_test_proc_read_debug_on_off;
//        debug_on_off->write_proc = ito_test_proc_write_debug_on_off;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF OK\n");
    }

    if (NULL==open_test) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST OPEN failed\n");
    } 
    else 
    {
//        open_test->read_proc = ito_test_proc_read_open;
//        open_test->write_proc = ito_test_proc_write_open;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST OPEN OK\n");
    }

    if (NULL==short_test) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST SHORT failed\n");
    } 
    else 
    {
//        short_test->read_proc = ito_test_proc_read_short;
//        short_test->write_proc = ito_test_proc_write_short;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST SHORT OK\n");
    }

    if (NULL==fail_channel) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST FAIL CHANNEL failed\n");
    } 
    else 
    {
//        fail_channel->read_proc = ito_test_proc_read_fail_channel;
//        fail_channel->write_proc = ito_test_proc_write_fail_channel;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST FAIL CHANNEL OK\n");
    }

    if (NULL==data) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DATA failed\n");
    } 
    else 
    {
//        data->read_proc = ito_test_proc_read_data;
//        data->write_proc = ito_test_proc_write_data;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DATA OK\n");
    }
}

