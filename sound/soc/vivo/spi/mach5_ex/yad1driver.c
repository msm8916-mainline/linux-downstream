/****************************************************************************
 *
 *		Copyright(c) 2013-2014 Yamaha Corporation. All rights reserved.
 *
 *		Module		: yad1driver.c
 *
 *		Description	: Yamaha Audio engine Device version 1
 *						 device driver module.
 *
 *		Version		: 1.1.0		2014.07.25
 *
 ****************************************************************************/
#include "yad1types.h"
#include "yad1driver.h"
#include "yad1driver_d.h"
#include "yad1machdep.h"

static U08	gau08Mem[ YAD1DD_MEM_SIZE ];

/*******************************************************************************
 *	get_variable_size
 ******************************************************************************/
static S32	get_variable_size
(
	const U08 *			pu08Script,
	U32					u32Size,
	U32 *				pu32Value
)
{
	U32					u32Value, u32Index;
	U08					u08Data, u08Flag;
	S32					s32Ret;

	u32Index	= 0;
	u32Value	= 0;
	u08Flag		= 0;
	s32Ret		= YAD1DD_ERROR_SYN;
	*pu32Value	= 0;

	while (( u32Index	<  u32Size )
		&& ( u32Index	<  4 )
		&& ( u08Flag	== 0 ))
	{
		u08Data		= pu08Script[ u32Index ];
		u32Value	=(U32)(( u32Value << 7 )+( u08Data & 0x7F ));
		u32Index	++;
		if (( u08Data & 0x80 )== 0x00 )
		{
			u08Flag		= 1;
			s32Ret		= u32Index;
			*pu32Value	= u32Value;
		}
	}

	return s32Ret;
}

/*******************************************************************************
 *	wait_flag
 ******************************************************************************/
static S32	wait_flag
(
	U08					u08Page,
	const U08 *			pu08Script
)
{
	U08					u08Adr, u08Mask, u08Val, u08TO, u08Reg;
	S32					s32Ret;

	u08Adr	= pu08Script[0];
	u08Mask	= pu08Script[1];
	u08Val	= pu08Script[2];
	u08TO	= pu08Script[3];
	u08Reg	= 0;
	s32Ret	= YAD1DD_SUCCESS;

	while ( s32Ret	== YAD1DD_SUCCESS )
	{
		s32Ret	= YAD1DD_Read( u08Page, u08Adr, &u08Reg, 1 );
		if (( u08Reg & u08Mask )== u08Val )
		{
			break;
		}
		if ( u08TO	== 0 )
		{
			s32Ret	= YAD1DD_ERROR_TO;
			break;
		}
		u08TO	--;
		YAD1MD_Sleep( 1 );
	}

	return s32Ret;
}

/*******************************************************************************
 *	mem_ctrl
 ******************************************************************************/
static S32	mem_ctrl
(
	const U08 *			pu08Script
)
{
	U08					u08Ope1, u08Ope2;
	U08 *				pu08Mem;
	S32					s32Ret;


	if ( pu08Script[ 0 ]< YAD1DD_MEM_SIZE )
	{
		s32Ret	= YAD1DD_SUCCESS;
		u08Ope1	= gau08Mem[ pu08Script[ 0 ]];
		u08Ope2	= pu08Script[ 2 ];
		pu08Mem	=&gau08Mem[ pu08Script[ 0 ]];

		switch ( pu08Script[ 1 ])
		{
		case YAD1DD_MEMCTRL_WRITE :
			*pu08Mem	=(U08)( u08Ope2 );
			break;
		case YAD1DD_MEMCTRL_ADD :
			*pu08Mem	=(U08)( u08Ope1 + u08Ope2 );
			break;
		case YAD1DD_MEMCTRL_SUB :
			*pu08Mem	=(U08)( u08Ope1 - u08Ope2 );
			break;
		case YAD1DD_MEMCTRL_MUL :
			*pu08Mem	=(U08)( u08Ope1 * u08Ope2 );
			break;
		case YAD1DD_MEMCTRL_OR :
			*pu08Mem	=(U08)( u08Ope1 | u08Ope2 );
			break;
		case YAD1DD_MEMCTRL_AND :
			*pu08Mem	=(U08)( u08Ope1 & u08Ope2 );
			break;
		case YAD1DD_MEMCTRL_XOR :
			*pu08Mem	=(U08)( u08Ope1 ^ u08Ope2 );
			break;
		default :
			s32Ret		= YAD1DD_ERROR_SYN;
			break;
		}
	}
	else
	{
		s32Ret	= YAD1DD_ERROR_SYN;
	}

	return s32Ret;
}

/*******************************************************************************
 *	run_1_command
 ******************************************************************************/
static S32	run_1_command
(
	const U08 *			pu08Script,
	U32					u32Size
)
{
	S32					s32Ret;
	U32					u32PrmSize, u32ComSize;
	U08					u08Page, u08Adr;
	const U08 *			pu08Data;

	s32Ret	= YAD1DD_ERROR_SYN;

	if (( pu08Script	!= NULL )
	 && ( u32Size		!= 0 ))
	{
		switch ( pu08Script[ 0 ])
		{
		case YAD1DD_COMTYPE_IF_REG_W :
		case YAD1DD_COMTYPE_A_REG_W :
		case YAD1DD_COMTYPE_MA_REG_W :
		case YAD1DD_COMTYPE_MB_REG_W :
		case YAD1DD_COMTYPE_B_REG_W :
		case YAD1DD_COMTYPE_C_REG_W :
		case YAD1DD_COMTYPE_E_REG_W :
		case YAD1DD_COMTYPE_F_REG_W :
		case YAD1DD_COMTYPE_ANA_REG_W :
		case YAD1DD_COMTYPE_CD_REG_W :
			if ( u32Size	>= 3 )
			{
				u08Page		= pu08Script[ 0 ] - YAD1DD_COMTYPE_IF_REG_W;
				u08Adr		= pu08Script[ 1 ];
				pu08Data	=&pu08Script[ 2 ];
				s32Ret		= YAD1DD_Write( u08Page, u08Adr, pu08Data, 1 );
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= 3;
				}
			}
			break;

		case YAD1DD_COMTYPE_DIRECT_W :
#if		(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_SPI )
			if ( u32Size	>= 3 )
			{
				pu08Data	=&pu08Script[ 2 ];
				s32Ret		= YAD1MD_Write( pu08Script[ 1 ], pu08Data, 1 );
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= 3;
				}
			}
#elif	(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_I2C )
			if ( u32Size	>= 4 )
			{
				pu08Data	=&pu08Script[ 3 ];
				s32Ret		= YAD1MD_WriteI2C( pu08Script[ 1 ], pu08Script[2], pu08Data, 1 );
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= 4;
				}
			}
#endif
			break;

		case YAD1DD_COMTYPE_IF_REG_W_V :
		case YAD1DD_COMTYPE_A_REG_W_V :
		case YAD1DD_COMTYPE_MA_REG_W_V :
		case YAD1DD_COMTYPE_MB_REG_W_V :
		case YAD1DD_COMTYPE_B_REG_W_V :
		case YAD1DD_COMTYPE_C_REG_W_V :
		case YAD1DD_COMTYPE_E_REG_W_V :
		case YAD1DD_COMTYPE_F_REG_W_V :
		case YAD1DD_COMTYPE_ANA_REG_W_V :
		case YAD1DD_COMTYPE_CD_REG_W_V :
			s32Ret		= get_variable_size( &pu08Script[ 1 ], u32Size - 1, &u32PrmSize );
			u32ComSize	= u32PrmSize + s32Ret + 1;
			if (( s32Ret		>  YAD1DD_SUCCESS )
			 && ( u32ComSize	<= u32Size )
			 && ( u32PrmSize	>= 2 ))
			{
				u08Page		= pu08Script[ 0 ] - YAD1DD_COMTYPE_IF_REG_W_V;
				u08Adr		= pu08Script[ s32Ret + 1 ];
				pu08Data	=&pu08Script[ s32Ret + 2 ];
				s32Ret		= YAD1DD_Write( u08Page, u08Adr, pu08Data, u32PrmSize - 1 );
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= u32ComSize;
				}
			}
			else
			{
				s32Ret	= YAD1DD_ERROR_SYN;
			}
			break;

		case YAD1DD_COMTYPE_DIRECT_W_V :
#if		(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_SPI )
			s32Ret		= get_variable_size( &pu08Script[ 1 ], u32Size - 1, &u32PrmSize );
			u32ComSize	= u32PrmSize + s32Ret + 1;
			if (( s32Ret		>  YAD1DD_SUCCESS )
			 && ( u32ComSize	<= u32Size )
			 && ( u32PrmSize	>= 2 ))
			{
				pu08Data	=&pu08Script[ s32Ret + 2 ];
				s32Ret		= YAD1MD_Write( pu08Script[ s32Ret + 1 ], pu08Data, u32PrmSize - 1 );
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= u32ComSize;
				}
			}
			else
			{
				s32Ret	= YAD1DD_ERROR_SYN;
			}
#elif	(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_I2C )
			s32Ret		= get_variable_size( &pu08Script[ 1 ], u32Size - 1, &u32PrmSize );
			u32ComSize	= u32PrmSize + s32Ret + 1;
			if (( s32Ret		>  YAD1DD_SUCCESS )
			 && ( u32ComSize	<= u32Size )
			 && ( u32PrmSize	>= 3 ))
			{
				pu08Data	=&pu08Script[ s32Ret + 3 ];
				s32Ret		= YAD1MD_WriteI2C( pu08Script[ s32Ret + 1 ], pu08Script[ s32Ret + 2 ], pu08Data, u32PrmSize - 2 );
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= u32ComSize;
				}
			}
			else
			{
				s32Ret	= YAD1DD_ERROR_SYN;
			}
#endif
			break;

		case YAD1DD_COMTYPE_IF_REG_F :
		case YAD1DD_COMTYPE_A_REG_F :
		case YAD1DD_COMTYPE_MA_REG_F :
		case YAD1DD_COMTYPE_MB_REG_F :
		case YAD1DD_COMTYPE_B_REG_F :
		case YAD1DD_COMTYPE_C_REG_F :
		case YAD1DD_COMTYPE_E_REG_F :
		case YAD1DD_COMTYPE_F_REG_F :
		case YAD1DD_COMTYPE_ANA_REG_F :
		case YAD1DD_COMTYPE_CD_REG_F :
			if ( u32Size	>= 5 )
			{
				u08Page		= pu08Script[ 0 ] - YAD1DD_COMTYPE_IF_REG_F;
				s32Ret		= wait_flag( u08Page, &pu08Script[ 1 ]);
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= 5;
				}
			}
			break;

		case YAD1DD_COMTYPE_WAIT :
			if ( u32Size	>= 2 )
			{
				YAD1MD_Sleep( pu08Script[ 1 ]);
				s32Ret		= 2;
			}
			break;

		case YAD1DD_COMTYPE_MEM_W_IF :
		case YAD1DD_COMTYPE_MEM_W_A :
		case YAD1DD_COMTYPE_MEM_W_MA :
		case YAD1DD_COMTYPE_MEM_W_MB :
		case YAD1DD_COMTYPE_MEM_W_B :
		case YAD1DD_COMTYPE_MEM_W_C :
		case YAD1DD_COMTYPE_MEM_W_E :
		case YAD1DD_COMTYPE_MEM_W_F :
		case YAD1DD_COMTYPE_MEM_W_ANA :
		case YAD1DD_COMTYPE_MEM_W_CD :
			if ( u32Size	>= 3 )
			{
				if ( pu08Script[ 2 ]< YAD1DD_MEM_SIZE )
				{
					u08Page		= pu08Script[ 0 ] - YAD1DD_COMTYPE_MEM_W_IF;
					u08Adr		= pu08Script[ 1 ];
					s32Ret		= YAD1DD_Read( u08Page, u08Adr, &gau08Mem[ pu08Script[ 2 ]], 1 );
					if ( s32Ret	== YAD1DD_SUCCESS )
					{
						s32Ret	= 3;
					}
				}
			}
			break;

		case YAD1DD_COMTYPE_MEM_CTRL :
			if ( u32Size	>= 4 )
			{
				s32Ret	= mem_ctrl( &pu08Script[ 1 ]);
				if ( s32Ret	== YAD1DD_SUCCESS )
				{
					s32Ret	= 4;
				}
			}
			break;

		case YAD1DD_COMTYPE_MEM_CP_IF :
		case YAD1DD_COMTYPE_MEM_CP_A :
		case YAD1DD_COMTYPE_MEM_CP_MA :
		case YAD1DD_COMTYPE_MEM_CP_MB :
		case YAD1DD_COMTYPE_MEM_CP_B :
		case YAD1DD_COMTYPE_MEM_CP_C :
		case YAD1DD_COMTYPE_MEM_CP_E :
		case YAD1DD_COMTYPE_MEM_CP_F :
		case YAD1DD_COMTYPE_MEM_CP_ANA :
		case YAD1DD_COMTYPE_MEM_CP_CD :
			if ( u32Size	>= 3 )
			{
				if ( pu08Script[ 1 ]< YAD1DD_MEM_SIZE )
				{
					u08Page		= pu08Script[ 0 ] - YAD1DD_COMTYPE_MEM_CP_IF;
					u08Adr		= pu08Script[ 2 ];
					pu08Data	=&gau08Mem[ pu08Script[ 1 ]];
					s32Ret		= YAD1DD_Write( u08Page, u08Adr, pu08Data, 1 );
					if ( s32Ret	== YAD1DD_SUCCESS )
					{
						s32Ret	= 3;
					}
				}
			}
			break;

		default :
			break;
		}
	}

	return s32Ret;
}

/*******************************************************************************
 *	YAD1DD_Write
 *
 *	Function:
 *			Write register.
 *	Argument:
 *			u08Page				intermediate register selector
 *			u08Adr				register address
 *			pu08Data			pointer to write data
 *			u32Size				size of write data
 *	Return:
 *			YAD1DD_SUCCESS		success
 *			YAD1DD_ERROR_SPI	error(SPI communication)
 *			YAD1DD_ERROR_ARG	error(arguments)
 *
 ******************************************************************************/
S32		YAD1DD_Write
(
	U08					u08Page,
	U08					u08Adr,
	const U08 *			pu08Data,
	U32					u32Size
)
{
#if		(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_SPI )
	S32					s32Ret;
	U08					u08Com1, u08Com2;

	if (( pu08Data	!= NULL )
	 && ( u32Size	!= 0 ))
	{
		s32Ret	= YAD1DD_SUCCESS;
		u08Com1	= YAD1DD_SPI_COM_READ;
		u08Com2	= YAD1DD_SPI_COM_READ;

		switch ( u08Page )
		{
		case YAD1DD_REGSEL_IF :
			u08Com1	= YAD1DD_SPI_COM_READ;
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |(( u08Adr & YAD1DD_SPI_COM_ADR_MASK )<< 1 )| YAD1DD_SPI_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_A :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_A_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_A_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_MA :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_MA_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_MA_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_MB :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_MB_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_MB_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_B :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_B_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_B_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_C :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_C_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_C_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_E :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_E_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_E_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_F :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_F_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_F_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_ANA :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_ANA_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_ANA_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_CD :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_CD_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_CD_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		default :
			s32Ret	= YAD1DD_ERROR_ARG;
			break;
		}

		if (( s32Ret	== YAD1DD_SUCCESS )
		 && ( u08Com1	!= YAD1DD_SPI_COM_READ ))
		{
			s32Ret	= YAD1MD_Write( u08Com1, &u08Adr, 1 );
		}
		if ( s32Ret		== YAD1DD_SUCCESS )
		{
			s32Ret	= YAD1MD_Write( u08Com2, pu08Data, u32Size );
		}
	}
	else
	{
		s32Ret	= YAD1DD_ERROR_ARG;
	}

	return s32Ret;
#elif	(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_I2C )
	S32					s32Ret;
	U08					u08Com1, u08Com2, u08Adr1, u08Adr2, u08Data;

	if (( pu08Data	!= NULL )
	 && ( u32Size	!= 0 ))
	{
		s32Ret	= YAD1DD_SUCCESS;
		u08Com1	= 0;
		u08Com2	= 0;
		u08Adr1	= 0;
		u08Adr2	= 0;
		u08Data = 0;

		switch ( u08Page )
		{
		case YAD1DD_REGSEL_IF :
			if (( u08Adr	>=  6 )
			 && ( u08Adr	<= 10 ))
			{		/* ANA/CD Reg access */
				u08Com2	=		 YAD1DD_ANALOGBLOCK_SA;
				u08Adr2	=(U08)((( u08Adr & YAD1DD_I2C_COM_ADR_MASK )<< 1 )| YAD1DD_I2C_COM_BURST );
			}
			else
			{
				u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
				u08Adr2	=(U08)((( u08Adr & YAD1DD_I2C_COM_ADR_MASK )<< 1 )| YAD1DD_I2C_COM_BURST );
			}
			break;
		case YAD1DD_REGSEL_IM_A :
			u08Com1	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_A_REG_A   << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_A_REG_D   << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_MA :
			u08Com1	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_MA_REG_A  << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_MA_REG_D  << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_MB :
			u08Com1	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_MB_REG_A  << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_MB_REG_D  << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_B :
			u08Com1	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_B_REG_A   << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_B_REG_D   << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_C :
			u08Com1	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_C_REG_A   << 1 );
			u08Data	= u08Adr;
			u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_C_REG_D   << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_E :
			u08Com1	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_E_REG_A   << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_E_REG_D   << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_F :
			u08Com1	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_F_REG_A   << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_F_REG_D   << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_ANA :
			u08Com1	=		 YAD1DD_ANALOGBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_ANA_REG_A << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_ANALOGBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_ANA_REG_D << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_CD :
			u08Com1	=		 YAD1DD_ANALOGBLOCK_SA;
			u08Adr1	=(U08)(  YAD1DD_REGADR_CD_REG_A  << 1 );
			u08Data	=(U08)(  YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		 YAD1DD_ANALOGBLOCK_SA;
			u08Adr2	=(U08)(( YAD1DD_REGADR_CD_REG_D  << 1 )| YAD1DD_I2C_COM_BURST );
			break;
		default :
			s32Ret	= YAD1DD_ERROR_ARG;
			break;
		}

		if (( s32Ret	== YAD1DD_SUCCESS )
		 && ( u08Com1	!= 0 ))
		{
			s32Ret	= YAD1MD_WriteI2C( u08Com1, u08Adr1, &u08Data, 1 );
		}
		if ( s32Ret		== YAD1DD_SUCCESS )
		{
			s32Ret	= YAD1MD_WriteI2C( u08Com2, u08Adr2, pu08Data, u32Size );
		}
	}
	else
	{
		s32Ret	= YAD1DD_ERROR_ARG;
	}

	return s32Ret;
#endif
}

/*******************************************************************************
 *	YAD1DD_Read
 *
 *	Function:
 *			Read register.
 *	Argument:
 *			u08Page				intermediate register selector
 *			u08Adr				register address
 *			pu08Data			pointer to write data
 *			u32Size				size of write data
 *	Return:
 *			YAD1DD_SUCCESS		success
 *			YAD1DD_ERROR_SPI	error(SPI communication)
 *			YAD1DD_ERROR_ARG	error(arguments)
 *
 ******************************************************************************/
S32		YAD1DD_Read
(
	U08					u08Page,
	U08					u08Adr,
	U08 *				pu08Data,
	U32					u32Size
)
{
#if		(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_SPI )
	S32					s32Ret;
	U08					u08Com1, u08Com2;

	if (( pu08Data	!= NULL )
	 && ( u32Size	!= 0 ))
	{
		s32Ret	= YAD1DD_SUCCESS;
		u08Com1	= YAD1DD_SPI_COM_READ;
		u08Com2	= YAD1DD_SPI_COM_READ;

		switch ( u08Page & 0x7F )
		{
		case YAD1DD_REGSEL_IF :
			u08Com1	= YAD1DD_SPI_COM_READ;
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |(( u08Adr & YAD1DD_SPI_COM_ADR_MASK )<< 1 )| YAD1DD_SPI_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_A :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_A_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_A_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_MA :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_MA_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_MA_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_MB :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_MB_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_MB_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_B :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_B_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_B_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_C :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_C_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_C_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			break;
		case YAD1DD_REGSEL_IM_E :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_E_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_E_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_F :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_F_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_F_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_ANA :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_ANA_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_ANA_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		case YAD1DD_REGSEL_IM_CD :
			u08Com1	=(U08)( YAD1DD_SPI_COM_WRITE |( YAD1DD_REGADR_CD_REG_A << 1 )| YAD1DD_SPI_COM_BURST );
			u08Com2	=(U08)( YAD1DD_SPI_COM_READ  |( YAD1DD_REGADR_CD_REG_D << 1 )| YAD1DD_SPI_COM_BURST );
			u08Adr	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			break;
		default :
			s32Ret	= YAD1DD_ERROR_ARG;
			break;
		}

		if (( s32Ret	== YAD1DD_SUCCESS )
		 && ( u08Com1	!= YAD1DD_SPI_COM_READ ))
		{
			s32Ret	= YAD1MD_Write( u08Com1, &u08Adr, 1 );
		}
		if ( s32Ret	== YAD1DD_SUCCESS )
		{
			s32Ret	= YAD1MD_Read( u08Com2, pu08Data, u32Size );
		}
	}
	else
	{
		s32Ret	= YAD1DD_ERROR_ARG;
	}

	return s32Ret;
#elif	(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_I2C )

	S32					s32Ret;
	U08					u08Com1, u08Com2, u08Adr1, u08Adr2, u08Data;

	if (( pu08Data	!= NULL )
	 && ( u32Size	!= 0 ))
	{
		s32Ret	= YAD1DD_SUCCESS;
		u08Com1	= 0;
		u08Com2	= 0;
		u08Adr1	= 0;
		u08Adr2	= 0;
		u08Data	= 0;

		switch ( u08Page & 0x7F )
		{
		case YAD1DD_REGSEL_IF :
			if (( u08Adr	>=  6 )
			 && ( u08Adr	<= 10 ))
			{		/* ANA/CD Reg access */
				u08Com2	=		 YAD1DD_ANALOGBLOCK_SA;
				u08Adr2	=(U08)(( u08Adr & YAD1DD_I2C_COM_ADR_MASK )<< 1 );
			}
			else
			{
				u08Com2	=		 YAD1DD_DIGITALBLOCK_SA;
				u08Adr2	=(U08)(( u08Adr & YAD1DD_I2C_COM_ADR_MASK )<< 1 );
			}
			break;
		case YAD1DD_REGSEL_IM_A :
			u08Com1	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_A_REG_A   << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_A_REG_D   << 1 );
			break;
		case YAD1DD_REGSEL_IM_MA :
			u08Com1	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_MA_REG_A  << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_MA_REG_D  << 1 );
			break;
		case YAD1DD_REGSEL_IM_MB :
			u08Com1	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_MB_REG_A  << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_MB_REG_D  << 1 );
			break;
		case YAD1DD_REGSEL_IM_B :
			u08Com1	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_B_REG_A   << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_B_REG_D   << 1 );
			break;
		case YAD1DD_REGSEL_IM_C :
			u08Com1	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_C_REG_A   << 1 );
			u08Data	= u08Adr;
			u08Com2	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_C_REG_D   << 1 );
			break;
		case YAD1DD_REGSEL_IM_E :
			u08Com1	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_E_REG_A   << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_E_REG_D   << 1 );
			break;
		case YAD1DD_REGSEL_IM_F :
			u08Com1	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_F_REG_A   << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_DIGITALBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_F_REG_D   << 1 );
			break;
		case YAD1DD_REGSEL_IM_ANA :
			u08Com1	=		YAD1DD_ANALOGBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_ANA_REG_A << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_ANALOGBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_ANA_REG_D << 1 );
			break;
		case YAD1DD_REGSEL_IM_CD :
			u08Com1	=		YAD1DD_ANALOGBLOCK_SA;
			u08Adr1	=(U08)( YAD1DD_REGADR_CD_REG_A  << 1 );
			u08Data	=(U08)( YAD1DD_REGFLAG_AINC  |( u08Adr & 0x7F ));
			u08Com2	=		YAD1DD_ANALOGBLOCK_SA;
			u08Adr2	=(U08)( YAD1DD_REGADR_CD_REG_D  << 1 );
			break;
		default :
			s32Ret	= YAD1DD_ERROR_ARG;
			break;
		}

		if (( s32Ret	== YAD1DD_SUCCESS )
		 && ( u08Com1	!= 0 ))
		{
			s32Ret	= YAD1MD_WriteI2C( u08Com1, u08Adr1, &u08Data, 1 );
		}
		while (( u32Size	!= 0 )
			&& ( s32Ret		== YAD1DD_SUCCESS ))
		{
			s32Ret	= YAD1MD_ReadI2C( u08Com2, u08Adr2, pu08Data, 1 );
			pu08Data	++;
			u32Size		--;
		}
	}
	else
	{
		s32Ret	= YAD1DD_ERROR_ARG;
	}

	return s32Ret;
#endif
}

/*******************************************************************************
 *	YAD1DD_Wait
 *
 *	Function:
 *			Wait u32Msec[msec].
 *	Argument:
 *			u32MSec				wait time[msec]
 *	Return:
 *			None
 *
 ******************************************************************************/
void	YAD1DD_Wait
(
	U32					u32MSec
)
{
	YAD1MD_Sleep( u32MSec );

	return ;
}

/*******************************************************************************
 *	YAD1DD_BatchProcess
 *
 *	Function:
 *			Run batch script.
 *	Argument:
 *			pu08Script			pointer to batch script
 *			u32Size				size of batch script
 *	Return:
 *			YAD1DD_SUCCESS		success
 *			YAD1DD_ERROR_SPI	error(SPI communication)
 *			YAD1DD_ERROR_ARG	error(arguments)
 *			YAD1DD_ERROR_SYN	error(batch script syntax)
 *			YAD1DD_ERROR_TO		error(time out)
 *
 ******************************************************************************/
S32		YAD1DD_BatchProcess
(
	const U08 *			pu08Script,
	U32					u32Size
)
{
	S32					s32Ret;

	s32Ret	= YAD1DD_ERROR_ARG;

	if (( pu08Script	!= NULL )
	 && ( u32Size		!= 0 ))
	{
		while ( u32Size	>  0 )
		{
			s32Ret		= run_1_command( pu08Script, u32Size );
			if ( s32Ret	>= 0 )
			{
				pu08Script	+= s32Ret;
				u32Size		-= s32Ret;
				s32Ret		=  YAD1DD_SUCCESS;
			}
			else
			{
				u32Size		=  0;
			}
		}
	}

	return s32Ret;
}
