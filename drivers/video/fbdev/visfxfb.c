// SPDX-License-Identifier: GPL-2.0+
/*
 * Framebuffer driver for Visualize FX cards commonly found in PA-RISC machines
 *
 * Copyright (c) 2021-2023 Sven Schnelle <svens@stackframe.org>
 * Copyright (c) 2022-2023 Helge Deller <deller@gmx.de>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <asm/grfioctl.h>
#include "sticore.h"

#define VISFX_CARDTYPE_FX5	CRT_ID_LEGO

#define UP_CONTROL_TO		BIT(13)
#define UP_CONTROL_TCE		BIT(12)
#define UP_CONTROL_SOFT_RST	BIT(8)

#define UB_STATUS_FAULT		BIT(27)
#define UB_STATUS_WPNE		BIT(24)

#define VISFX_DFP_ENABLE		0x10000
#define VISFX_HSYNC_POSITIVE		0x40000
#define VISFX_VSYNC_POSITIVE		0x80000

#define VISFX_SYNC_PLL_BASE		49383 /* 20.25MHz in ps */

#define VISFX_FB_LENGTH			0x01000000
#define VISFX_FB_OFFSET			0x01000000

#define MIN_XRES	640
#define MIN_YRES	480

#define DEFAULT_BPP	8

/* Visualize-FX hardware registers */
#define UB_CP				  0x00400000 /* Cursor Position Register (W) */
#define UB_CA				  0x00400004 /* Cursor Address Register (W) */
#define UB_CD				  0x00400008 /* Cursor Data Register (W) */
#define UB_CF				  0x0040000C /* Cursor Foreground Register (W) */
#define UB_CB				  0x00400010 /* Cursor Background Register (W) */

#define B2_BABoth			  0x00A00804 /* DBA & SBA (reads return DBA) (W) */
#define B2_DBA				  0x00A00808 /* Destination Bitmap Access Register (W) */
#define B2_SBA				  0x00A0080C /* Source Bitmap Access Register (W) */
#define B2_DSA				  0x00A00810 /* Destination Screen Address Register (W) */
#define B2_SSA				  0x00A00814 /* Source Screen Address Register (W) */
#define B2_WORG				  0x00A00818 /* Window Origin Register (W) */
#define B2_FBS				  0x00A0081C /* Front Buffer Select Register (W) */
#define B2_BPDU				  0x00A00820 /* Bitmap Pixel Data Register (Upper Half) (W) */
#define B2_BPD				  0x00A00824 /* Bitmap Pixel Data Register (W) */
#define B2_BPMU				  0x00A00828 /* Bitmap Pixel Mask Register (Upper Half) (W) */
#define B2_BPM				  0x00A0082C /* Bitmap Pixel Mask Register (W) */
#define B2_BPCU				  0x00A00830 /* Bitmap Pixel Clip Register (Upper Half) (W) */
#define B2_BPC				  0x00A00834 /* Bitmap Pixel Clip Register (W) */
#define B2_IFCU				  0x00A00838 /* Image Foreground Color Register (Uppef Half) (W) */
#define B2_IFC				  0x00A0083C /* Image Foreground Color Register (W) */
#define B2_IBCU				  0x00A00840 /* Image Background Color Register (Upper Half) (W) */
#define B2_IBC				  0x00A00844 /* Image Background Color Register (W) */
#define B2_IPMU				  0x00A00848 /* Image Plane Mask Register (Upper Half) (W) */
#define B2_IPM				  0x00A0084C /* Image Plane Mask Register (W) */
#define B2_MISC_CTL			  0x00A00850 /* FBC Miscellaneous Control Register (W) */
#define B2_FLN				  0x00A00854 /* Fill Length (in X) (W) */
#define B2_EN2D				  0x00A00858 /* Coordinate Change Register (W) */
#define B2_RPH				  0x00A0085C /* Read Prefetch Hint Register (W) */
#define B2_WCEU				  0x00A00868 /* Window Clip Enable Register (Upper Half) (W) */
#define B2_WCE				  0x00A0086C /* Window Clip Enable Register (W) */
#define B2_PCRU				  0x00A00870 /* Pattern Control Register (Upper Half) (W) */
#define B2_PCR				  0x00A00874 /* Pattern Control Register (W) */
#define B2_BLN				  0x00A00880 /* BLT Length (in X) (W) */
#define B2_SWEN				  0x00A00884 /* FBC Write Enable Register (W) */
#define B2_SREN				  0x00A00888 /* FBC Read Enable Register (W) */
#define B2_BMAP_BABoth			  0x00A008A0 /* Writes of Shared BMAP_DBA (W) */
#define B2_BMAP_DBA			  0x00A008A4 /* Writes of Shared BMAP_DBA (W) */
#define B2_BMAP_SBA			  0x00A008A8 /* Writes of Shared BMAP_SBA (W) */

/*  Video Display Processor Registers  */
#define B2_CP				  0x00800000 /* Cursor Position Register (RW) */
#define B2_CA				  0x00800004 /* Cursor Address Register (RW) */
#define B2_CD				  0x00800008 /* Cursor Data Register (RW) */
#define B2_CF				  0x0080000C /* Cursor Foreground Register (RW) */
#define B2_CB				  0x00800010 /* Cursor Background Register (RW) */
#define B2_PMASK			  0x00800018 /* LUT Plane Mask Register (RW) */
#define B2_MCS				  0x0080001C /* Midlay Color Map Select Register (RW) */
#define B2_LLCA				  0x00800020 /* LUT Load Control Address Register (RW) */
#define B2_LUTD				  0x00800024 /* Look Up Table Register (RW) */
#define B2_FATTR			  0x0080003C /* Force Attribute Bits Register (RW) */
#define B2_MV				  0x00800040 /* Misc. Video Register (RW) */
#define B2_CFG				  0x00800044 /* Configuration Register (RW) */
#define B2_SCR				  0x00800048 /* Stereo Control Register (RW) */
#define B2_MPC				  0x0080004C /* Monitor Power Control Register (RW) */
#define B2_ETG				  0x00800058 /* Early Timing Generator Register (RW) */
#define B2_VHAL				  0x0080005C /* Vertical / Horizontal Active Length Register (RW) */
#define B2_HTG				  0x00800060 /* Horizontal Timing Generator Register (RW) */
#define B2_STG				  0x00800064 /* Serration Timing Generator Register (RW) */
#define B2_VTG				  0x00800068 /* Vertical Timing Generator Register (RW) */
#define B2_HS				  0x0080006C /* Horizontal State Register (RW) */
#define B2_VS				  0x00800070 /* Vertical State Register (RW) */
#define B2_DTS				  0x00800074 /* Timing & State Register (R) */
#define B2_SRBMAP0			  0x00800078 /* Stereo Right Front Block BMAP Register (RW) */
#define B2_SRBMAP1			  0x0080007C /* Stereo Right Back Block BMAP Register (RW) */
#define B2_VOP				  0x00800080 /* Video Out Position Register (RW) */
#define B2_VOS				  0x00800084 /* Video Out Size Register (RW) */
#define B2_VOC				  0x00800088 /* Video Out Control Register (RW) */
#define B2_VOP1				  0x0080008C /* Video Out Position Register (RW) */
#define B2_STSA				  0x00800090 /* Showtime CRC Register (RW) */
#define B2_BLSA				  0x00800094 /* Blither CRC Register (RW) */
#define B2_PLL_DOT_CTL			  0x008000A0 /* Dot Clock PLL Control Register (RW) */
#define B2_PLL_DOT_CNT			  0x008000A4 /* Dot Clock PLL Counter Register (RW) */
#define B2_PLL_CORE_CTL			  0x008000A8 /* Core Clock PLL Control Register (RW) */
#define B2_PLL_CORE_CNT			  0x008000AC /* Core Clock PLL Counter Register (RW) */
#define B2_PLL_RAM_CTL			  0x008000B0 /* RAM Clock PLL Control Register (RW) */
#define B2_PLL_RAM_CNT			  0x008000B4 /* RAM Clock PLL Counter Register (RW) */
#define B2_PLL_REF_CNT			  0x008000B8 /* PLL Reference Clock Counter Register (RW) */
#define B2_BLSA_MASK			  0x008000C4 /* BL Signature Analysis Mask Register (RW) */
#define B2_SA_MASK			  0x008000C8 /* Signature Analysis Mask Register (RW) */
#define B2_SA_SIG			  0x008000CC /* Signature Analysis Register (RW) */
#define B2_MON				  0x008000D0 /* Monitor Test Register (RW) */
#define B2_DDC				  0x008000D4 /* Display Data Channel Register (RW) */
#define B2_CFS0				  0x00800100 /* Colormap Function Select Register 0 (RW) */
#define B2_CFS16			  0x00800140 /* Colormap Function Select Register 16 (RW) */
#define B2_VBC_SIG_EN			  0x008001C0 /* Vid Bus Signature Enable Register ? (RW) */
#define B2_VBC_SIG			  0x008001C4 /* Vid Bus Signature Register? (RW) */
#define B2_DUM				  0x00800240 /* Display Unit Master Register (RW) */
#define B2_ABMAP			  0x00800244 /* Attribute Block BMAP Register (RW) */
#define B2_CKEY_HI			  0x00800248 /* Color Key High Register (RW) */
#define B2_CKEY_LO			  0x0080024C /* Color Key Low Register (RW) */
#define B2_CALPHA			  0x00800250 /* Constant Alpha Register (RW) */
#define B2_CCBC				  0x00800254 /* Color Conversion Brightness & Contrast Register (RW) */
#define B2_OMC				  0x00800260 /* Overlay Miscellaneous Control Register (RW) */
#define B2_OTLS				  0x00800264 /* Overlay Transparency LUT Select Register (RW) */
#define B2_OXYO				  0x00800268 /* Overlay Visible XY Offset Register (RW) */
#define B2_OBS				  0x0080026C /* Overlay Buffer Select Register (RW) */
#define B2_OBMAP0			  0x00800270 /* Overlay Primary Buffer Block BMAP Register (RW) */
#define B2_OBMAP1			  0x00800274 /* Overlay Secondary Buffer Block BMAP Register (RW) */
#define B2_OUL				  0x00800278 /* Overlay Upper Left Register (RW) */
#define B2_OLR				  0x0080027C /* Overlay  Lower Right Register (RW) */
#define B2_ODS				  0x00800280 /* Overlay  Data Size Register (RW) */
#define B2_OSCALE			  0x00800284 /* Overlay Scale Register (RW) */
#define B2_IMD				  0x00800288 /* Image Depth Register (RW) */
#define B2_IMC0				  0x00800300 /* Image Miscellaneous Control 0 Register (RW) */
#define B2_IMC1				  0x00800304 /* Image Miscellaneous Control 1 Register (RW) */
#define B2_IMC2				  0x00800308 /* Image Miscellaneous Control 2 Register (RW) */
#define B2_IMC3				  0x0080030C /* Image Miscellaneous Control 3 Register (RW) */
#define B2_IMC4				  0x00800310 /* Image Miscellaneous Control 4 Register (RW) */
#define B2_IMC5				  0x00800314 /* Image Miscellaneous Control 5 Register (RW) */
#define B2_IMC6				  0x00800318 /* Image Miscellaneous Control 6 Register (RW) */
#define B2_IMC7				  0x0080031C /* Image Miscellaneous Control 7 Register (RW) */
#define B2_IBMAP0			  0x00800320 /* Image Primary BMAP Register (RW) */
#define B2_IBMAP1			  0x00800324 /* Image Secondary BMAP Register (RW) */
#define B2_WAIT_HSYNC			  0x00800330 /* Wait for Horizontal retrace sync Register (RW) */
#define B2_IBS0				  0x00800340 /* Image Buffer Select 0 Register (RW) */
#define B2_IBS1				  0x00800344 /* Image Buffer Select 1 Register (RW) */
#define B2_IBS2				  0x00800348 /* Image Buffer Select 2 Register (RW) */
#define B2_IBS3				  0x0080034C /* Image Buffer Select 3 Register (RW) */
#define B2_IBS4				  0x00800350 /* Image Buffer Select 4 Register (RW) */
#define B2_IBS5				  0x00800354 /* Image Buffer Select 5 Register (RW) */
#define B2_IBS6				  0x00800358 /* Image Buffer Select 6 Register (RW) */
#define B2_IBS7				  0x0080035C /* Image Buffer Select 7 Register (RW) */
#define B2_ICLR0			  0x00800360 /* Image Clear 0 Register (RW) */
#define B2_ICLR1			  0x00800364 /* Image Clear 1 Register (RW) */
#define B2_ICLR2			  0x00800368 /* Image Clear 2 Register (RW) */
#define B2_ICLR3			  0x0080036C /* Image Clear 3 Register (RW) */
#define B2_ICLR4			  0x00800370 /* Image Clear 4 Register (RW) */
#define B2_ICLR5			  0x00800374 /* Image Clear 5 Register (RW) */
#define B2_ICLR6			  0x00800378 /* Image Clear 6 Register (RW) */
#define B2_ICLR7			  0x0080037C /* Image Clear 7 Register (RW) */

#define B2_FOE				  0x00A00404 /* Fragment Operation Enable Register (W) */
#define B2_VTB				  0x00B08008 /* MFU Vector Tie Breaker Register (RW) */
#define B2_TTB				  0x00B0800C /* MFU Trapezoid Tie Breaker Register (RW) */
#define B2_TM_TSS			  0x008E5800 /* Texture Subsystem Setup (all registers) (RW) */

/*  FBC Registers  */
#define UB_IBO				  0x00521110 /* Image Bitmap Operation Register (R) */
#define UB_CPE				  0x00521114 /* Clip Plane Enable Register (R) */
#define UB_CZ				  0x00521118 /* Constant Z data Register  (R) */
#define UB_CBR				  0x0052111C /* Constant Blend Register (R) */
#define UB_ZBO				  0x00521120 /* Z Bitmap Operation Register (R) */
#define UB_SBO				  0x00521124 /* Stencil Bitmap Operation Register (R) */
#define UB_ATO				  0x00521128 /* Alpha Test Operation Register (R) */
#define UB_DOXY				  0x0052112C /* Dither Offset  XY Register (R) */
#define UB_FOG				  0x00521134 /* Fog Color Register (R) */
#define UB_FZ				  0x00521138 /* Fast  Z Clear Register (R) */
#define UB_BMAP_Z			  0x0052113C /* BMAP Z Register (R) */
#define UB_TEC0				  0x00521140 /* Texture Environment Color  0 Register (R) */
#define UB_TEC1				  0x00521144 /* Texture Environment Color 1 Register (R) */
#define UB_TEX_2D			  0x00521154 /* Texture 2D Register (R) */
#define UB_FOG_2D			  0x00521160 /* Fog 2D Register (R) */
#define UB_SCK_UR			  0x00521170 /* Source Color Key Upper Range Register (R) */
#define UB_SCK_LR			  0x00521174 /* Source Color Key Lower Range Register (R) */
#define UB_CKC				  0x00521178 /* Color Key Configuration Register (R) */
#define UB_VTR				  0x00521180 /* Visibility Test Statistics Results (R) */
#define UB_VTE				  0x00521184 /* Visibility Test Enable (R) */
#define UB_VPCNT			  0x0052118c /* Visibility Count Pass Register (R) */
#define UB_VFCNT			  0x00521190 /* Visibility Count Fail Register (R) */
#define UB_MBWB				  0x00521194 /* Multi-buffer Write Buffer B (R) */
#define UB_MBWC				  0x00521198 /* Multi-buffer Write Buffer C (R) */
#define UB_MBWD				  0x0052119c /* Multi-buffer Write Buffer D (R) */
#define UB_FCDA				  0x005211a4 /* Fast Cleat Default Alpha Register (R) */
#define UB_BTO				  0x005211a8 /* Pixel Batching Timeout Value (R) */
#define UB_RC_CONFIG0			  0x005211b0 /* Ram controller timing configuration register 0 (R) */
#define UB_RC_CONFIG1			  0x005211b4 /* Ram controller timing configuration register 1 (R) */
#define UB_RC_CONFIG2			  0x005211b8 /* Ram controller timing configuration register 2 (R) */
#define UB_FPR				  0x00522008 /* FBC Parameters Register (R) */

/*  Shadowed FBC Registers(sent to all FBC blocks)  */
#define B2_FBC_BABoth			  0x00920804 /* DBA & SBA (reads return DBA) (RW) */
#define B2_FBC_DBA			  0x00920808 /* Destination Bitmap Access Register (RW) */
#define B2_FBC_SBA			  0x0092080C /* Source Bitmap Access Register (RW) */
#define B2_FBC_DSA			  0x00920810 /* Destination Screen Address Register (RW) */
#define B2_FBC_SSA			  0x00920814 /* Source Screen Address Register (RW) */
#define B2_FBC_WORG			  0x00920818 /* FBC Window Origin (W) */
#define B2_FBC_FBS			  0x0092081C /* Front Buffer Select Register (RW) */
#define B2_FBC_BPDU			  0x00920820 /* Bitmap Pixel Data Register (Upper Half) (RW) */
#define B2_FBC_BPD			  0x00920824 /* Bitmap Pixel Data Register (RW) */
#define B2_FBC_BPMU			  0x00920828 /* Bitmap Pixel Mask Register (Upper Half) (RW) */
#define B2_FBC_BPM			  0x0092082C /* Bitmap Pixel Mask Register (RW) */
#define B2_FBC_BPCU			  0x00920830 /* Bitmap Pixel Clip Register (Upper Half) (RW) */
#define B2_FBC_BPC			  0x00920834 /* Bitmap Pixel Clip Register (RW) */
#define B2_FBC_IFCU			  0x00920838 /* Image Foreground Color Register (Upper Half) (RW) */
#define B2_FBC_IFC			  0x0092083C /* Image Foreground Color Register (RW) */
#define B2_FBC_IBCU			  0x00920840 /* Image Background Color Register (Upper Half) (RW) */
#define B2_FBC_IBC			  0x00920844 /* Image Background Color Register (RW) */
#define B2_FBC_IPMU			  0x00920848 /* Image Plane Mask Register (Upper Half) (RW) */
#define B2_FBC_IPM			  0x0092084C /* Image Plane Mask Register (RW) */
#define B2_FBC_MISC_CTL			  0x00920850 /* Miscellaneous Control Register (RW) */
#define B2_FBC_FLN			  0x00920854 /* Fill Length (in X) (RW) */
#define B2_FBC_RPH			  0x0092085C /* Read Prefetch Hint Register (R) */
#define B2_FBC_RBS			  0x00920860 /* Read Byte Swap Register (RW) */
#define B2_FBC_WCEU			  0x00920868 /* Window Clip Enable Register (Upper Half) (RW) */
#define B2_FBC_WCE			  0x0092086C /* Window Clip Enable Register (RW) */
#define B2_FBC_PCRU			  0x00920870 /* Pattern Control Register (Upper Half) (RW) */
#define B2_FBC_PCR			  0x00920874 /* Pattern Control Register (RW) */
#define B2_FBC_BLN			  0x00920880 /* BLT Length (in X) (RW) */
#define B2_FBC_WEN			  0x00920884 /* FBC Write Enable Register (RW) */
#define B2_FBC_REN			  0x00920888 /* FBC Read Enable Register (RW) */
#define B2_RTSTATUS			  0x00920890 /* RT Status Register (R) */
#define B2_FBC_BMAP_BABoth		  0x009208A0 /* BMAP DBA and SBA (reads return BMAP_DBA) (RW) */
#define B2_FBC_BMAP_DBA			  0x009208A4 /* BMAP Destination Bitmap Access register (RW) */
#define B2_FBC_BMAP_SBA			  0x009208AC /* BMAP Source Bitmap Access register (RW) */
#define B2_FBC_WUL0			  0x00920900 /* Window Clip Upper Left 0 (RW) */
#define B2_FBC_WLR0			  0x00920904 /* Window Clip Lower Right 0 (RW) */

#define B2_AMAP0			  0x00920A00 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP1			  0x00920A04 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP2			  0x00920A08 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP3			  0x00920A0C /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP4			  0x00920A10 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP5			  0x00920A14 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP6			  0x00920A18 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP7			  0x00920A1C /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP8			  0x00920A20 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP9			  0x00920A24 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP10			  0x00920A28 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP11			  0x00920A2C /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP12			  0x00920A30 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP13			  0x00920A34 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP14			  0x00920A38 /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP15			  0x00920A3C /* Direct Frame Buffer Address Mapping  (RW) */
#define B2_AMAP_BA_U			  0x00920AF0 /* Direct Frame Buffer YUV420 U Address Mapping  (RW) */
#define B2_AMAP_BA_V			  0x00920AF4 /* Direct Frame Buffer YUV420 V Address Mapping  (RW) */

#define B2_BMAP_DFB0			  0x00920B00 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB1			  0x00920B04 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB2			  0x00920B08 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB3			  0x00920B0C /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB4			  0x00920B10 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB5			  0x00920B14 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB6			  0x00920B18 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB7			  0x00920B1C /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB8			  0x00920B20 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB9			  0x00920B24 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB10			  0x00920B28 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB11			  0x00920B2C /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB12			  0x00920B30 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB13			  0x00920B34 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB14			  0x00920B38 /* Direct Frame Buffer Access BMAP registers (RW) */
#define B2_BMAP_DFB15			  0x00920B3C /* Direct Frame Buffer Access BMAP registers (RW) */

/*  Misc FBC Non-Shadowed registers  */
#define B2_FZ				  0x00921108 /* Fast Z Clear Register (RW) */
#define B2_IBO				  0x00921110 /* Image Bitmap Operation Register (RW) */
#define B2_CPE				  0x00921114 /* Clip Plane Enable Register (RW) */
#define B2_CZ				  0x00921118 /* Constant Z data Register  (RW) */
#define B2_CBR				  0x0092111C /* Constant Blend Register (RW) */
#define B2_ZBO				  0x00921120 /* Z Bitmap Operation Register (RW) */
#define B2_SBO				  0x00921124 /* Stencil Bitmap Operation Register (RW) */
#define B2_ATO				  0x00921128 /* Alpha Test Operation Register (RW) */
#define B2_DOXY				  0x0092112C /* Dither Offset XY Register (RW) */
#define B2_FOGU				  0x00921130 /* Fog Color Register (Upper Half) (RW) */
#define B2_FOG				  0x00921134 /* Fog Color Register (RW) */
#define B2_BMAP_Z			  0x0092113C /* BMAP Z register (RW) */
#define B2_TEC0				  0x00921140 /* Texture Environment Color  0 Register (RW) */
#define B2_TEC1				  0x00921144 /* Texture Environment Color 1 Register (RW) */
#define B2_OTR				  0x00921148 /* Overlay Transparency Register (RW) */
#define B2_TEX_2DU			  0x00921150 /* Texture 2D Register (Upper Half) (RW) */
#define B2_TEX_2D			  0x00921154 /* Texture 2D Register (RW) */
#define B2_FOG_2D			  0x00921160 /* Fog 2D Register (RW) */
#define B2_SCK_UR			  0x00921170 /* Source Color Key Upper Range Register (RW) */
#define B2_SCK_LR			  0x00921174 /* Source Color Key Lower Range Register (RW) */
#define B2_CKC				  0x00921178 /* Color Key Configuration Register (RW) */
#define B2_VTR				  0x00921180 /* Visibility Test Statistics Results (RW) */
#define B2_VTE				  0x00921184 /* Visibility Test Enable (RW) */
#define B2_VTC				  0x00921188 /* Visibility Test Clear (W) */
#define B2_VPCNT			  0x0092118c /* Visibility Count Pass Register (RW) */
#define B2_VFCNT			  0x00921190 /* Visibility Count Fail Register (RW) */
#define B2_MBWB				  0x00921194 /* Multi-buffer Write Buffer B (RW) */
#define B2_MBWC				  0x00921198 /* Multi-buffer Write Buffer C (RW) */
#define B2_MBWD				  0x0092119c /* Multi-buffer Write Buffer D (RW) */
#define B2_FSR				  0x009211C4 /* Fast Surface Clear Pixel Read Register (RW) */
#define B2_FSRMWA			  0x009211C8 /* Fast Surface Clear Register RMW buffer A (RW) */
#define B2_FSRMWB			  0x009211CC /* Fast Surface Clear Register RMW buffer B (RW) */
#define B2_FSRMWC			  0x009211D0 /* Fast Surface Clear Register RMW buffer C (RW) */
#define B2_FSRMWD			  0x009211D4 /* Fast Surface Clear Register RMW buffer D (RW) */
#define B2_FCDA				  0x009211D8 /* Fast Clear Default Alpha Register (RW) */
#define B2_BTO				  0x009211DC /* Pixel Batching Timeout Value (RW) */
#define B2_BIST				  0x009211e8 /* Built-in Self Test register (RW) */
#define B2_RC_CONFIG0			  0x009211f0 /* Ram controller timing configuration register 0 (RW) */
#define B2_RC_CONFIG1			  0x009211f4 /* Ram controller timing configuration register 1 (RW) */
#define B2_RC_CONFIG2			  0x009211f8 /* Ram controller timing configuration register 2 (RW) */

#define B2_FPR				  0x00922008 /* FBC Parameters Register (R) */
#define B2_FBC_PERF			  0x0092200C /* FBC Performance Register (RW) */
#define B2_FCR				  0x00922010 /* FBC CRC Read Register (R) */
#define B2_FCI				  0x00922014 /* FBC CRC Initiate Register (W) */
#define B2_FCM				  0x00922018 /* FBC CRC Mask Register (W) */
#define B2_FBC_STATUS			  0x0092201C /* FBC Status Register (W) */

/*  FBC Tile Builder Specific Registers  */
#define B2_TB_BMAP_BABoth		  0x0092A000 /* Reads of TB's copy of Shared BMAP_DBA (R) */
#define B2_TB_BMAP_DBA			  0x0092A004 /* Reads of TB's copy of Shared BMAP_DBA (R) */
#define B2_TB_BMAP_SBA			  0x0092A008 /* Reads of TB's copy of Shared BMAP_SBA (R) */
#define B2_FBC_TB_REN			  0x0092A00C /* FBC/TB read address for FBC_SREN (R) */
#define B2_FBC_TB_DBA			  0x0092A010 /* FBC/TB read address for DBA (R) */
#define B2_FBC_TB_SBA			  0x0092A014 /* FBC/TB read address for SBA (R) */
#define B2_FBC_TB_WEN			  0x0092A018 /* FBC/TB read address for FBC_SWEN (R) */

/* 2D Retargeted Registers  */
#define B2_RTG_MEM_USE			  0x009C2000 /* Start Of Retargetter Memory (For Normal Use) (RW) */
#define B2_RTG_MEM_LOAD			  0x009C3000 /* Start Of Retargetter Memory (For Loading Memory) (RW) */

#define UB_UBABoth			  0x00600C04 /* UDBA & USBA (W) */
#define UB_UDBA				  0x00600C08 /* Unbuffered Destination Bitmap Access Register (W) */
#define UB_USBA				  0x00600C0C /* Unbuffered Source Bitmap Access Register (W) */
#define UB_UDSA				  0x00600C10 /* Unbuffered Destination Screen Address Register (W) */
#define UB_USSA				  0x00600C14 /* Unbuffered Source Screen Address Register (W) */
#define UB_UBPDU			  0x00600C20 /* Unbuffered Bitmap Pixel Data Register (Upper Half) (W) */
#define UB_UBPD				  0x00600C24 /* Unbuffered Bitmap Pixel Data Register (W) */
#define UB_UBPMU			  0x00600C28 /* Unbuffered Bitmap Pixel Mask Register (Upper Half) (W) */
#define UB_UBPM				  0x00600C2C /* Unbuffered Bitmap Pixel Mask Register (W) */
#define UB_UBPCU			  0x00600C30 /* Unbuffered Bitmap Pixel Clip Register (Upper Half) (W) */
#define UB_UBPC				  0x00600C34 /* Unbuffered Bitmap Pixel Clip Register (W) */
#define UB_URPH				  0x00600C5C /* Unbuffered Read Prefetch Hint Register (W) */
#define UB_UREN				  0x00600C8C /* Unbuffered Read Enable Register (W) */

/* Host Interface Registers  */
#define UP_CONTROL			  0x00249000 /* Control Register (RW) */
#define UP_FC				  0x00249040 /* Fault Control Register (RW) */
#define UP_TCP				  0x00249080 /* Time-out Circuit Parameter (RW) */
#define UP_DC				  0x002490C0 /* Dribble Count Register (RW) */
#define UP_STI_COUNT			  0x00249140 /* STI Count Register (RW) */
#define UP_PSTI_SCRATCH1		  0x00249180 /* Privileged STI Scratch Register 1 (RW) */
#define UP_PSTI_SCRATCH2		  0x002491A0 /* Privileged STI Scratch Register 2 (RW) */
#define UP_PSTI_SCRATCH3		  0x002491C0 /* Privileged STI Scratch Register 3 (RW) */
#define UP_PSTI_SCRATCH4		  0x002491E0 /* Privileged STI Scratch Register 4 (RW) */
#define UP_INT_ADDR			  0x00249200 /* Interrupt Address Register (RW) */
#define UP_INT_DATA			  0x00249240 /* Interrupt Data Register (RW) */
#define UP_STI_ADDR			  0x00249280 /* STI Write Address Register (W) */
#define UP_STI_DATA			  0x002492C0 /* STI Write Data Register (W) */
#define UP_VBS_CTL			  0x00249300 /* VBS Control Register (-) */

#define UP_CD_STATE1			  0x0024B000 /* CD Buffer State Register One (RW) */
#define UP_CD_STATE2			  0x0024B004 /* CD Buffer State Register Two (RW) */
#define UP_CD_DATA			  0x0024B008 /* CD Buffer Data Register (RW) */
#define UP_CF_STATE			  0x0024B00C /* Command Fifo State Register (RW) */
#define UP_CF_DATA			  0x0024B010 /* Command Fifo Data Register (RW) */
#define UP_CD_HWC			  0x0024B014 /* CD Buffer High Water Count Register (R) */
#define UP_CD_LWC			  0x0024B018 /* CD Buffer Low Water Count Register (R) */
#define UP_CF_HWC			  0x0024B01C /* Command Fifo High Water Count Register (R) */
#define UP_CF_LWC			  0x0024B020 /* Command Fifo Low Water Count Register (R) */
#define UP_CD_DRC			  0x0024B024 /* CD Buffer Dribble Count Register (R) */
#define UP_CF_DRC			  0x0024B028 /* Command FIFO Dribble Count Register (R) */

/* Host Interface Registers  */
#define BP_CONTROL			  0x002C9000 /* Control Register (RW) */
#define BP_FC				  0x002C9040 /* Fault Control Register (RW) */
#define BP_CD_HWC			  0x002CB014 /* CD Buffer High Water Count Register (RW) */
#define BP_CD_LWC			  0x002CB018 /* CD Buffer Low Water Count Register (RW) */
#define BP_CF_HWC			  0x002CB01C /* Command Fifo High Water Count Register (RW) */
#define BP_CF_LWC			  0x002CB020 /* Command Fifo Low Water Count Register (RW) */
#define BP_CD_DRC			  0x002CB024 /* CD Buffer Dribble Count Register (RW) */
#define BP_CF_DRC			  0x002CB028 /* Command FIFO Dribble Count Register (RW) */
#define BP_ICR				  0x002CB800 /* Integrator Counter Register (RW) */

/* Host Interface Registers  */
#define UB_CONTROL			  0x00641000 /* Control Register (RW) */
#define UB_FC				  0x00641040 /* Fault Control Register (R) */
#define UB_TCP				  0x00641080 /* Time-out Circuit Parameter Register (R) */
#define UB_DC				  0x006410C0 /* Dribble Count Register (R) */
#define UB_AGP_FFC			  0x00641100 /* AGP DMA Queue State (R) */
#define UB_STI_COUNT			  0x00641140 /* STI Count Register (R) */
#define UB_PSTI_SCRATCH1		  0x00641180 /* Privileged STI Scratch Register 1 (R) */
#define UB_PSTI_SCRATCH2		  0x006411A0 /* Privileged STI Scratch Register 2 (R) */
#define UB_PSTI_SCRATCH3		  0x006411C0 /* Privileged STI Scratch Register 3 (R) */
#define UB_PSTI_SCRATCH4		  0x006411E0 /* Privileged STI Scratch Register 4 (R) */
#define UB_INT_ADDR			  0x00641200 /* Interrupt Address Register (R) */
#define UB_INT_DATA			  0x00641240 /* Interrupt Data Register (R) */
#define UB_STATUS			  0x00641400 /* Status Register (R) */
#define UB_FFC				  0x00641440 /* Fifo Free Count Register (R) */
#define UB_UREAD_DATA			  0x006414C0 /* Unbuffered Read Data Register (R) */

#define UB_UIRP				  0x00641540 /* Unbuffered Indirect Read Purge Register (W) */

#define UB_CLIP_PLANE_STAMP		  0x00641580 /* STI Scratch 1/Clip Plane Stamp (RW) */
#define UB_STI_SCRATCH1			  0x00641580 /* STI Scratch Register 1 (RW) */
#define UB_STI_SCRATCH2			  0x006415A0 /* STI Scratch Register 2 (RW) */
#define UB_STI_SCRATCH3			  0x006415C0 /* STI Scratch Register 3 (RW) */
#define UB_STI_SCRATCH4			  0x006415E0 /* STI Scratch Register 4 (RW) */
#define UB_SIG_LOWER_LSB		  0x00641700 /* Lower Write Data Signature LSB's (RW) */
#define UB_SIG_LOWER_MSB		  0x00641740 /* Lower Write Data Signature MSB's (RW) */
#define UB_SIG_UPPER_LSB		  0x00641780 /* Upper Write Data Signature LSB's (RW) */
#define UB_SIG_UPPER_MSB		  0x006417C0 /* Upper Write Data Signature MSB's (RW) */
#define UB_TRI_STATUS			  0x00641800 /* Triaqua-specific Status Register (R) */
#define UB_INT_PENDING			  0x00641840 /* Interrupt Pending Register (Diagnostics) (R) */
#define UB_DREAD_ADDR			  0x00641880 /* Direct Read Address Register (Diagnostics) (R) */
#define UB_VREAD_ADDR			  0x006418C0 /* ROM Read Address Register (Diagnostics) (R) */
#define UB_AGP_UDSA			  0x006499C0 /* TM AGP DMA UDSA (-) */
#define UB_AGP_UDMA_UPPER		  0x00649A00 /* TM Reader Address (-) */
#define UB_AGP_UDMA_LOWER		  0x00649A40 /* TM Reader Address (-) */
#define UB_AGP_UDMA_OTC_STRIDE		  0x00649A80 /* AGP DMA Stride (-) */
#define UB_AGP_UDMA_HIGH_WIDE		  0x00649AC0 /* TM AGP DMA Read Hint (-) */
#define UB_AGP_DIAG0			  0x00649B00 /* TBD (-) */
#define UB_AGP_DIAG1			  0x00649B40 /* TBD (-) */
#define UB_AGP_DIAG2			  0x00649B80 /* TBD (-) */
#define UB_AGP_DIAG3			  0x00649BC0 /* TBD (-) */
#define UB_DEVID			  0x00641C00 /* PCI Device ID Register (R) */
#define UB_COM_STAT			  0x00641C04 /* PCI Command And Status Register (R) */
#define UB_REVID			  0x00641C08 /* PCI Revision ID Register (R) */
#define UB_LT				  0x00641C0C /* PCI Latency Timer Register (R) */
#define UB_MBA_LOWER			  0x00641C10 /* PCI Memory Base Address Register [31:0] (R) */
#define UB_MBA_UPPER			  0x00641C14 /* PCI Memory Base Address Register [63:32] (R) */
#define UB_FBMBA_LOWER			  0x00641C18 /* Direct FB Access (R) */
#define UB_FBMBA_UPPER			  0x00641C1C /* Direct FB Access (R) */
#define UB_SUBID			  0x00641C2C /* Subvendor ID (R) */
#define UB_RBA				  0x00641C30 /* PCI Expansion ROM Base Address Register (R) */
#define UB_AGP_CAPPTR			  0x00641C34 /* PCI New Capability (R) */
#define UB_LAT_GNT			  0x00641C3C /* PCI Minimum Grant And Maximum Latency Register (R) */
#define UB_AGP_CAPID			  0x00641C40 /* AGP Capabilities ID (R) */
#define UB_AGP_STATUS			  0x00641C44 /* AGP Capabilities List (R) */
#define UB_AGP_CMD			  0x00641C48 /* AGP Capabilities Enables (R) */
#define UB_PWR_CAPID			  0x00641C4C /* Power Management (R) */
#define UB__PWR_CMD			  0x00641C50 /* Power Management (R) */

#define UB_IRC				  0x00643030 /* Buffered Indirect Read Control Register (R) */
#define UB_UIRC				  0x00643034 /* Unbuffered Indirect Read Control Register (RW) */
#define UB_CD_BUFFER_CTL		  0x00643038 /* CD Buffer Control Register (R) */
#define UB_PDU_UBSCFB			  0x0064303C /* PDU Unbuffered Byte Swap Ctl Reg (frame buffer space) (RW) */
#define UB_PQC				  0x00643040 /* Packet Queue Control Register (R) */
#define UB_PQD				  0x00643100 /* Start Packet Queue Data Register (R) */

#define B2_IRC				  0x00A43030 /* Buffered Indirect Read Control Register (RW) */

#define B2_CD_BUFFER_CTL		  0x00A43038 /* CD Buffer Control Register (RW) */
#define B2_PDU_BSCFB			  0x00A4303C /* PDU Buffered Byte Swap Control Register (frame buffer space) (RW) */
#define B2_PQC				  0x00A43040 /* Packet Queue Control Register (RW) */
#define B2_PQD				  0x00A43100 /* Start Packet Queue Data Register (RW) */

/* Host Interface Address Space  */
#define B3_CONTROL			  0x00E41000 /* Control Register (RW) */
#define B3_FC				  0x00E41040 /* Fault Control Register (R) */
#define B3_STATUS			  0x00E41400 /* Status Register (R) */
#define B3_FFC				  0x00E41440 /* Fifo Free Count Register (R) */
#define B3_READ_DATA			  0x00E41480 /* Buffered Read Data Register (R) */
#define B3_IRP				  0x00E41500 /* Buffered Indirect Read Purge Register (W) */

#define B3_BBC_CONFIG			  0x00EE231C /* Blitzen Bus Controller Config Register (RW) */
#define B3_HUNGRY			  0x00EE2350 /* Hungry Bits From GA's Register (R) */
#define B3_NUM_GA			  0x00EE233C /* Number Of Geometry Accelerators Register (RW) */
#define B3_GA_CONFIG			  0x00CB0038 /* Geometry Accelerator Configuration Register (RW) */
#define B3_GA_NOP_EOC			  0x00CA8000 /* Not Really Used This Way (RW) */
#define B3_GA_IPLL			  0x00CB4030 /* Geometry Accelerator I/O PLL Control Register (RW) */

/* Summit Binc Interface  */
#define UB_BTDstObj_Xi			  0x00660000 /*Address For Direct Binc - dst / src obj inc X (RW) */
#define UB_BTDstObj_Xd			  0x00664000 /*Address For Direct Binc - dst / src obj dec X (W) */
#define B2_BTDstObj_Xi			  0x00A60000 /* Start For Direct Binc - dst / src obj inc X (RW) */
#define B2_BTDstObj_Xd			  0x00A64000 /* Start For Direct Binc - dst / src obj dec X (W) */
#define B2_BTDstObj_Yi			  0x00A68000 /* Start For Direct Binc - dst / src obj inc Y (W) */
#define B2_BTDstObj_Yd			  0x00A6C000 /* Start For Direct Binc - dst / src obj dec Y (W) */
#define B2_BTDstObj_Xi_Yi		  0x00A70000 /* Start For Direct Binc - dst / src obj inc X inc Y  (RW) */
#define B2_BTDstObj_Xi_Yd		  0x00A74000 /* Start For Direct Binc - dst / src obj inc X, dec Y (W) */
#define B2_BTDstObj_Xd_Yi		  0x00A78000 /*For Direct Binc - dst / src obj dec X, inc Y (W) */
#define B2_BTDstObj_Xd_Yd		  0x00A7C000 /* Start For Direct Binc - dst / src obj dec X, dec Y (W) */

#define UB_DMA_OFFSET			  0x006A007C /* Buffered DMA Offset Register (For Diag Only) (R) */
#define UB_DMA_BSCBLK_RO		  0x006A0404 /* Buff. Data Blanket Byte Swap Ctl Read Reg (For Diag) (R) */
#define UB_DMA_DIAG_AFLAG		  0x006A0418 /* DMA Controller Address Flags (Diagnostic) (R) */
#define UB_DMA_DIAG_ADDR		  0x006A041C /* DMA Controller Address (Diagnostic) (R) */
#define UB_DMA_BSCSAV			  0x006A0640 /* Buff. Data Blanket Byte Swap Ctl Save Reg (For Diag) (R) */

#define UB_UDMA_OFFSET			  0x006A087C /* Unbuffered DMA Offset Register (RW) */
#define UB_UDMA_DATA			  0x006A0880 /*Unbuffered DMA Data Register (W) */
#define UB_DMA_UBSCFB			  0x006A0C08 /* Unbuffered DMA Byte Swap Ctl Reg (FBC space) (RW) */
#define UB_UDMA_STATE			  0x006A0C10 /* Unbuffered DMA State Register (R) */
#define UB_UDMA_DIAG_AFLAG		  0x006A0C18 /* Unbuffered DMA Controller Adr Flags (Diagnostic) (R) */
#define UB_UDMA_DIAG_ADDR		  0x006A0C1C /* Unbuffered DMA Controller Adr (Diagnostic) (R) */
#define UB_DMA_UBSCBLK			  0x006A0E00 /* Unbuffered Data Blanket Byte Swap Ctl Reg. (W) */
#define UB_DMA_UBSCSAV			  0x006A0E40 /* Unbuffered Data Blanket Byte Swap Ctl Save Reg (RW) */

#define B2_DMA_OFFSET			  0x00AA007C /* Buffered DMA Offset Register (RW) */
#define B2_DMA_DATA			  0x00AA0080 /*Buffered DMA Data Register (W) */
#define B2_DMA_BSCBLK_RO		  0x00AA0404 /* Address for read of B2_DMA_BSCBLK. (R) */
#define B2_DMA_BSCFB			  0x00AA0408 /* Buffered DMA Byte Swap Ctl Reg (FBC space) (RW) */
#define B2_DMA_DDBSC			  0x00AA040C /*  () */
#define B2_DMA_STATE			  0x00AA0410 /* Buffered DMA State Register (R) */
#define B2_DMA_DIAG_AFLAG		  0x00AA0418 /* Buffered DMA Controller Adr flags (Diagnostic) (R) */
#define B2_DMA_DIAG_ADDR		  0x00AA041C /* Buffered DMA Controller Adr (Diagnostic) (R) */
#define B2_DMA_BSCBLK			  0x00AA0600 /* Buffered Data Blanket Byte Swap Ctl Reg (W) */
#define B2_DMA_BSCSAV			  0x00AA0640 /* Buffered Data Blanket Byte Swap Ctl Save Reg (RW) */

#define B2_DWA				  0x00AC1000 /* Destination Window Address Register (RW) */
#define B2_SWA				  0x00AC1004 /* Source Window Address Register (RW) */
#define B2_DWAX				  0x00AC1010 /* Destination Window X Address Register (18 bits) (RW) */
#define B2_DWAY				  0x00AC1014 /* Destination Window Y Address Register (18 bits) (RW) */
#define B2_SWAX				  0x00AC1018 /* Source Window X Address Register (18 bits) (RW) */
#define B2_SWAY				  0x00AC101C /* Source Window Y Address Register (18 bits) (RW) */
#define B2_VBS				  0x00AC1020 /* Vertical Blank Sync Register (RW) */
#define B2_TCR				  0x00AC1024 /* Throttle Control Register (RW) */
#define B2_YBC				  0x00AC1028 /* YUV to RGB Brightness Control Register (RW) */
#define B2_PSF				  0x00AC1030 /* Pixel Scale Factor Register (RW) */
#define B2_PSS				  0x00AC1034 /* Pixel Scale Setup Register (RW) */
#define B2_PS_RED			  0x00AC1038 /* Pixel Scale Accumulate Red Register (RW) */
#define B2_PS_GRN			  0x00AC1040 /* Pixel Scale Accumulate Green Register (RW) */
#define B2_PS_BLU			  0x00AC1044 /* Pixel Scale Accumulate Blue Register (RW) */
#define B2_WCLIP1UL			  0x00AC1050 /* First Level Window Clip Upper Left Corner Reg (RW) */
#define B2_WCLIP1LR			  0x00AC1054 /* First Level Window Clip Lower Right Corner Reg (RW) */
#define B2_DSAX				  0x00AC2000 /* Destination Screen X Address Reg (19 bits) (RW) */
#define B2_DSAY				  0x00AC2004 /* Destination Screen Y Address Regr (19 bits) (RW) */
#define B2_SSAX				  0x00AC2008 /* Source Screen X Address Register (19 bits) (RW) */
#define B2_SSAY				  0x00AC200C /* Source Screen Y Address Register (19 bits) (RW) */
#define B2_BSE1				  0x00AC2010 /* Binc State Element 1 Register (R) */
#define B2_BSE2				  0x00AC2014 /* Binc State Element 2 Register (R) */

#define B2_MNOOP_R0			  0x00B00000 /* MFU Noop Opcode / Load Master Register 0 (RW) */
#define B2_MNOOP_R1			  0x00B00004 /* MFU Noop Opcode / Load Master Register 1 (RW) */
#define B2_MNOOP_R2			  0x00B00008 /* MFU Noop Opcode / Load Master Register 2 (RW) */
#define B2_MNOOP_R3			  0x00B0000C /* MFU Noop Opcode / Load Master Register 3 (RW) */
#define B2_MNOOP_R4			  0x00B00010 /* MFU Noop Opcode / Load Master Register 4 (RW) */
#define B2_MNOOP_R5			  0x00B00014 /* MFU Noop Opcode / Load Master Register 5 (RW) */
#define B2_MNOOP_R6			  0x00B00018 /* MFU Noop Opcode / Load Master Register 6 (RW) */
#define B2_MNOOP_R7			  0x00B0001C /* MFU Noop Opcode / Load Master Register 7 (RW) */
#define B2_MNOOP_R8			  0x00B00020 /* MFU Noop Opcode / Load Master Register 8 (RW) */
#define B2_MNOOP_R9			  0x00B00024 /* MFU Noop Opcode / Load Master Register 9 (RW) */
#define B2_MNOOP_R10			  0x00B00028 /* MFU Noop Opcode / Load Master Register 10 (RW) */
#define B2_MNOOP_R11			  0x00B0002C /* MFU Noop Opcode / Load Master Register 11 (RW) */
#define B2_MNOOP_R12			  0x00B00030 /* MFU Noop Opcode / Load Master Register 12 (RW) */
#define B2_MNOOP_R13			  0x00B00034 /* MFU Noop Opcode / Load Master Register 13 (RW) */
#define B2_MNOOP_TD			  0x00B00044 /* MFU Noop Opcode / Load Master TD Register (RW) */
#define B2_MSLOAD_R0			  0x00B00100 /* Master To Slave Load Opcode / Load Master Register 0 (W) */
#define B2_MSLOAD_R1			  0x00B00104 /* Master To Slave Load Opcode / Load Master Register 1 (W) */
#define B2_MSLOAD_R2			  0x00B00108 /* Master To Slave Load Opcode / Load Master Register 2 (W) */
#define B2_MSLOAD_R3			  0x00B0010C /* Master To Slave Load Opcode / Load Master Register 3 (W) */
#define B2_MSLOAD_R4			  0x00B00110 /* Master To Slave Load Opcode / Load Master Register 4 (W) */
#define B2_MSLOAD_R5			  0x00B00114 /* Master To Slave Load Opcode / Load Master Register 5 (W) */
#define B2_MSLOAD_R6			  0x00B00118 /* Master To Slave Load Opcode / Load Master Register 6 (W) */
#define B2_MSLOAD_R7			  0x00B0011C /* Master To Slave Load Opcode / Load Master Register 7 (W) */
#define B2_MSLOAD_R8			  0x00B00120 /* Master To Slave Load Opcode / Load Master Register 8 (W) */
#define B2_MSLOAD_R9			  0x00B00124 /* Master To Slave Load Opcode / Load Master Register 9 (W) */
#define B2_MSLOAD_R10			  0x00B00128 /* Master To Slave Load Opcode / Load Master Register 10 (W) */
#define B2_MSLOAD_R11			  0x00B0012C /* Master To Slave Load Opcode / Load Master Register 11 (W) */
#define B2_MSLOAD_R12			  0x00B00130 /* Master To Slave Load Opcode / Load Master Register 12 (W) */
#define B2_MSLOAD_R13			  0x00B00134 /* Master To Slave Load Opcode / Load Master Register 13 (W) */
#define B2_MSLOAD_TD			  0x00B00144 /* Master To Slave Load Opcode / Load Master TD Register (W) */
#define B2_TEXT16_R0			  0x00B00400 /* 16 Bit Text Fill Opcode / Load Master Register 0 (W) */
#define B2_TEXT16_R1			  0x00B00404 /* 16 Bit Text Fill Opcode / Load Master Register 1 (W) */
#define B2_TEXT16_R2			  0x00B00408 /* 16 Bit Text Fill Opcode / Load Master Register 2 (W) */
#define B2_TEXT16_R3			  0x00B0040C /* 16 Bit Text Fill Opcode / Load Master Register 3 (W) */
#define B2_TEXT16_TD			  0x00B00444 /* 16 Bit Text Fill Opcode / Load Master TD Register (W) */
#define B2_TEXT8_R0			  0x00B00480 /* 8 Bit Text Fill Opcode / Load Master Register 0 (W) */
#define B2_TEXT8_R1			  0x00B00484 /* 8 Bit Text Fill Opcode / Load Master Register 1 (W) */
#define B2_TEXT8_R2			  0x00B00488 /* 8 Bit Text Fill Opcode / Load Master Register 2 (W) */
#define B2_TEXT8_R3			  0x00B0048C /* 8 Bit Text Fill Opcode / Load Master Register 3 (W) */
#define B2_TEXT8_TD			  0x00B004C4 /* 8 Bit Text Fill Opcode / Load Master TD Register (W) */
#define B2_TEXT16SETUP_R0		  0x00B00500 /* 16 Bit Text Fill Setup Opcode / Load Master Register 0 (W) */
#define B2_TEXT16SETUP_R1		  0x00B00504 /* 16 Bit Text Fill Setup Opcode / Load Master Register 1 (W) */
#define B2_TEXT16SETUP_R2		  0x00B00508 /* 16 Bit Text Fill Setup Opcode / Load Master Register 2 (W) */
#define B2_TEXT16SETUP_R3		  0x00B0050C /* 16 Bit Text Fill Setup Opcode / Load Master Register 3 (W) */
#define B2_TEXT16SETUP_TD		  0x00B00544 /* 16 Bit Text Fill Setup Opcode / Load Master TD Register (W) */
#define B2_TEXT8SETUP_R0		  0x00B00580 /* 8 Bit Text Fill Setup Opcode / Load Master Register 0 (W) */
#define B2_TEXT8SETUP_R1		  0x00B00584 /* 8 Bit Text Fill Setup Opcode / Load Master Register 1 (W) */
#define B2_TEXT8SETUP_R2		  0x00B00588 /* 8 Bit Text Fill Setup Opcode / Load Master Register 2 (W) */
#define B2_TEXT8SETUP_R3		  0x00B0058C /* 8 Bit Text Fill Setup Opcode / Load Master Register 3 (W) */
#define B2_TEXT8SETUP_TD		  0x00B005C4 /* 8 Bit Text Fill Setup Opcode / Load Master TD Register (W) */
#define B2_SOLIDFILL_R0			  0x00B00800 /* Solid Fill Opcode / Load Master Register 0 (W) */
#define B2_SOLIDFILL_R1			  0x00B00804 /* Solid Fill Opcode / Load Master Register 1 (W) */
#define B2_SOLIDFILL_R2			  0x00B00808 /* Solid Fill Opcode / Load Master Register 2 (W) */
#define B2_SOLIDFILL_R3			  0x00B0080C /* Solid Fill Opcode / Load Master Register 3 (W) */
#define B2_SOLIDFILLT_R0		  0x00B00880 /* Solid Fill Throttled Opcode / Load Master Register 0 (W) */
#define B2_SOLIDFILLT_R1		  0x00B00884 /* Solid Fill Throttled Opcode / Load Master Register 1 (W) */
#define B2_SOLIDFILLT_R2		  0x00B00888 /* Solid Fill Throttled Opcode / Load Master Register 2 (W) */
#define B2_SOLIDFILLT_R3		  0x00B0088C /* Solid Fill Throttled Opcode / Load Master Register 3 (W) */
#define B2_TDFILL_R0			  0x00B00900 /* TD Fill Opcode / Load Master Register 0 (W) */
#define B2_TDFILL_R1			  0x00B00904 /* TD Fill Opcode / Load Master Register 1 (W) */
#define B2_TDFILL_R2			  0x00B00908 /* TD Fill Opcode / Load Master Register 2 (W) */
#define B2_TDFILL_R3			  0x00B0090C /* TD Fill Opcode / Load Master Register 3 (W) */
#define B2_TDFILL_TD			  0x00B00944 /* TD Fill Opcode / Load Master TD Register (W) */
#define B2_TDFILLT_R0			  0x00B00980 /* TD Fill Throttled Opcode / Load Master Register 0 (W) */
#define B2_TDFILLT_R1			  0x00B00984 /* TD Fill Throttled Opcode / Load Master Register 1 (W) */
#define B2_TDFILLT_R2			  0x00B00988 /* TD Fill Throttled Opcode / Load Master Register 2 (W) */
#define B2_TDFILLT_R3			  0x00B0098C /* TD Fill Throttled Opcode / Load Master Register 3 (W) */
#define B2_TDFILLT_TD			  0x00B009C4 /* TD Fill Throttled Opcode / Load Master TD Register (W) */
#define B2_BLT_R0			  0x00B00C00 /* Blt Opcode / Load Master Register 0 (W) */
#define B2_BLT_R1			  0x00B00C04 /* Blt Opcode / Load Master Register 1 (W) */
#define B2_BLT_R2			  0x00B00C08 /* Blt Opcode / Load Master Register 2 (W) */
#define B2_BLT_R3			  0x00B00C0C /* Blt Opcode / Load Master Register 3 (W) */
#define B2_BLT_R4			  0x00B00C10 /* Blt Opcode / Load Master Register 4 (W) */
#define B2_BLT_R5			  0x00B00C14 /* Blt Opcode / Load Master Register 5 (W) */
#define B2_EXBLT_R0			  0x00B00C80 /* Expanding Blt Opcode / Load Master Register 0 (W) */
#define B2_EXBLT_R1			  0x00B00C84 /* Expanding Blt Opcode / Load Master Register 1 (W) */
#define B2_EXBLT_R2			  0x00B00C88 /* Expanding Blt Opcode / Load Master Register 2 (W) */
#define B2_EXBLT_R3			  0x00B00C8C /* Expanding Blt Opcode / Load Master Register 3 (W) */
#define B2_EXBLT_R4			  0x00B00C90 /* Expanding Blt Opcode / Load Master Register 4 (W) */
#define B2_EXBLT_R5			  0x00B00C94 /* Expanding Blt Opcode / Load Master Register 5 (W) */
#define B2_EXBLT_R11			  0x00B00CAC /* Expanding Blt Opcode / Load Master Register 11 (W) */

#define B2_SR0				  0x00B04000 /* MFU Slave Register 0 (RW) */
#define B2_SR1				  0x00B04004 /* MFU Slave Register 1 (RW) */
#define B2_SR2				  0x00B04008 /* MFU Slave Register 2 (RW) */
#define B2_SR3				  0x00B0400C /* MFU Slave Register 3 (RW) */
#define B2_SR4				  0x00B04010 /* MFU Slave Register 4 (RW) */
#define B2_SR5				  0x00B04014 /* MFU Slave Register 5 (RW) */
#define B2_SR6				  0x00B04018 /* MFU Slave Register 6 (RW) */
#define B2_SR7				  0x00B0401C /* MFU Slave Register 7 (RW) */
#define B2_SR8				  0x00B04020 /* MFU Slave Register 8 (RW) */
#define B2_SR9				  0x00B04024 /* MFU Slave Register 9 (RW) */
#define B2_SR10				  0x00B04028 /* MFU Slave Register 10 (RW) */
#define B2_SR11				  0x00B0402C /* MFU Slave Register 11 (RW) */
#define B2_SR12				  0x00B04030 /* MFU Slave Register 12 (RW) */
#define B2_SR13				  0x00B04034 /* MFU Slave Register 13 (RW) */
#define B2_SR14				  0x00B04038 /* MFU Slave Register 14 (RW) */
#define B2_SR15				  0x00B0403C /* MFU Slave Register 15 (RW) */
#define B2_STD				  0x00B04044 /* MFU Slave Transfer Data Register (RW) */
#define B2_SOV				  0x00B08000 /* MFU Scoreboard Override Register (RW) */
#define B2_SB				  0x00B08004 /* MFU Scoreboard Register (R) */
#define B2_MFU_DIAG			  0x00B08040 /* MFU Diagnostic Register (R) */
#define B2_MFU_BSCTD			  0x00B08044 /* MFU Byte Swap Control Register (TD operations) (RW) */
#define B2_MFU_BSCCTL			  0x00B08048 /* MFU Byte Swap Control Register (register pair opcodes) (RW) */
#define B2_MNOOP			  0x00B0C000 /* MFU Noop Opcode / Don't Load Any Master Registers (W) */
#define B2_NOSB				  0x00B0C080 /* MFU Noop Opcode (No Scoreboard) / Don't Load Any Master Registers (W) */
#define B2_MSLOAD			  0x00B0C100 /* Master To Slave Load Opcode / Don't Load Any Master Registers (W) */
#define B2_TEXT16			  0x00B0C400 /* 16 Bit Text Fill Opcode / Don't Load Any Master Registers (W) */
#define B2_TEXT8			  0x00B0C480 /* 8 Bit Text Fill Opcode / Don't Load Any Master Registers (W) */
#define B2_TEXT16SETUP			  0x00B0C500 /* 16 Bit Text Fill Setup Opcode / Don't Load Any Master Registers (W) */
#define B2_TEXT8SETUP			  0x00B0C580 /* 8 Bit Text Fill Setup Opcode / Don't Load Any Master Registers (W) */
#define B2_SOLIDFILL			  0x00B0C800 /* Solid Fill Opcode / Don't Load Any Master Registers (W) */
#define B2_SOLIDFILLT			  0x00B0C880 /* Solid Fill Throttled Opcode / Don't Load Any Master Registers (W) */
#define B2_TDFILL			  0x00B0C900 /* TD Fill Opcode / Don't Load Any Master Registers (W) */
#define B2_TDFILLT			  0x00B0C980 /* TD Fill Throttled Opcode / Don't Load Any Master Registers (W) */
#define B2_BLT				  0x00B0CC00 /* Blt Opcode / Don't Load Any Master Registers (W) */
#define B2_EXBLT			  0x00B0CC80 /* Expanding Blt Opcode / Don't Load Any Master Registers (W) */
#define B2_SS0				  0x00B10000 /* MFU Setup Slave Register 0 (RW) */
#define B2_SS1				  0x00B10004 /* MFU Setup Slave Register 1 (RW) */
#define B2_SS2				  0x00B10008 /* MFU Setup Slave Register 2 (RW) */
#define B2_SS3				  0x00B1000C /* MFU Setup Slave Register 3 (RW) */
#define B2_SS4				  0x00B10010 /* MFU Setup Slave Register 4 (RW) */
#define B2_SS5				  0x00B10014 /* MFU Setup Slave Register 5 (RW) */
#define B2_SS6				  0x00B10018 /* MFU Setup Slave Register 6 (RW) */
#define B2_SSTD				  0x00B10044 /* MFU Setup Slave Transfer Data Register (RW) */

/* Remapped MFU Registers  */
#define B2_MNOOP_R0R1_REMAP		  0x00B20000 /* Start Of MFU Noop Opcode / Load Master Register 0 and 1 (W) */
#define B2_NOSB_R0R1_REMAP		  0x00B20080 /* Start Of MFU Noop Opcode (No Scoreboard) / Load Master Register 0 and 1 (W) */
#define B2_MSLOAD_R0R1_REMAP		  0x00B20100 /* Master To Slave Load Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT16_R0R1_REMAP		  0x00B20400 /* 16 Bit Text Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT8_R0R1_REMAP		  0x00B20480 /* 8 Bit Text Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT16SETUP_R0R1_REMAP	  0x00B20500 /* 16 Bit Text Fill Setup Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT8SETUP_R0R1_REMAP	  0x00B20580 /* 8 Bit Text Fill Setup Opcode / Load Master Register 0 and 1 (W) */
#define B2_SOLIDFILL_R0R1_REMAP		  0x00B20800 /* Solid Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_SOLIDFILLT_R0R1_REMAP	  0x00B20880 /* Solid Fill Throttled Opcode / Load Master Register 0 and 1 (W) */
#define B2_TDFILL_R0R1_REMAP		  0x00B20900 /* TD Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_TDFILLT_R0R1_REMAP		  0x00B20980 /* TD Fill Throttled Opcode / Load Master Register 0 and 1 (W) */
#define B2_BLT_R0R1_REMAP		  0x00B20C00 /* Blt Opcode / Load Master Register 0 and 1 (W) */
#define B2_EXBLT_R0R1_REMAP		  0x00B20C80 /* Expanding Blt Opcode / Load Master Register 0 and 1 (W) */
#define B2_MNOOP_R2R3_REMAP		  0x00B24000 /* Start Of MFU Noop Opcode / Load Master Register 2 and 3 (W) */
#define B2_NOSB_R2R3_REMAP		  0x00B24080 /* Start Of MFU Noop Opcode (No Scoreboard) / Load Master Register 2 and 3 (W) */
#define B2_MSLOAD_R2R3_REMAP		  0x00B24100 /* Master To Slave Load Opcode / Load Master Register 2 and 3 (W) */
#define B2_TEXT16_R2R3_REMAP		  0x00B24400 /* 16 Bit Text Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_TEXT8_R2R3_REMAP		  0x00B24480 /* 8 Bit Text Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_TEXT16SETUP_R2R3_REMAP	  0x00B24500 /* 16 Bit Text Fill Setup Opcode / Load Master Register 2 and 3 (W) */
#define B2_TEXT8SETUP_R2R3_REMAP	  0x00B24580 /* 8 Bit Text Fill Setup Opcode / Load Master Register 2 and 3 (W) */
#define B2_SOLIDFILL_R2R3_REMAP		  0x00B24800 /* Solid Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_SOLIDFILLT_R2R3_REMAP	  0x00B24880 /* Solid Fill Throttled Opcode / Load Master Register 2 and 3 (W) */
#define B2_TDFILL_R2R3_REMAP		  0x00B24900 /* TD Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_TDFILLT_R2R3_REMAP		  0x00B24980 /* TD Fill Throttled Opcode / Load Master Register 2 and 3 (W) */
#define B2_BLT_R2R3_REMAP		  0x00B24C00 /* Blt Opcode / Load Master Register 2 and 3 (W) */
#define B2_EXBLT_R2R3_REMAP		  0x00B24C80 /* Expanding Blt Opcode / Load Master Register 2 and 3 (W) */
#define B2_MNOOP_TD_REMAP		  0x00B28000 /* Start Of MFU Noop Opcode / Load Master Register TD (W) */
#define B2_NOSB_TD_REMAP		  0x00B28080 /* Start Of MFU Noop Opcode (No Scoreboard) / Load Master Register TD (W) */
#define B2_MSLOAD_TD_REMAP		  0x00B28100 /* Master To Slave Load Opcode / Load Master Register TD (W) */
#define B2_TEXT16_TD_REMAP		  0x00B28400 /* 16 Bit Text Fill Opcode / Load Master Register TD (W) */
#define B2_TEXT8_TD_REMAP		  0x00B28480 /* 8 Bit Text Fill Opcode / Load Master Register TD (W) */
#define B2_TEXT16SETUP_TD_REMAP		  0x00B28500 /* 16 Bit Text Fill Setup Opcode / Load Master Register TD (W) */
#define B2_TEXT8SETUP_TD_REMAP		  0x00B28580 /* 8 Bit Text Fill Setup Opcode / Load Master Register TD (W) */
#define B2_SOLIDFILL_TD_REMAP		  0x00B28800 /* Solid Fill Opcode / Load Master Register TD (W) */
#define B2_SOLIDFILLT_TD_REMAP		  0x00B28880 /* Solid Fill Throttled Opcode / Load Master Register TD (W) */
#define B2_TDFILL_TD_REMAP		  0x00B28900 /* TD Fill Opcode / Load Master Register TD (W) */
#define B2_TDFILLT_TD_REMAP		  0x00B28980 /* TD Fill Throttled Opcode / Load Master Register TD (W) */
#define B2_BLT_TD_REMAP			  0x00B28C00 /* Blt Opcode / Load Master Register TD (W) */
#define B2_EXBLT_TD_REMAP		  0x00B28C80 /* Expanding Blt Opcode / Load Master Register TD (W) */
#define B2_MNOOP_ALT			  0x00B2C000 /* Start Of MFU Noop Opcode / Load Master Register Alternating Between MNOOP_R0R1 and MNOOP_R2R3 (W) */
#define B2_NOSB_ALT			  0x00B2C080 /* Start Of MFU Noop Opcode (No Scoreboard) (W) */
#define B2_MSLOAD_ALT			  0x00B2C100 /* Master To Slave Load Opcode (W) */
#define B2_TEXT16_ALT			  0x00B2C400 /* 16 Bit Text Fill Opcode / Load Master Register Alternating Between MNOOP_R0R1 and TEXT16_R2R3 (W) */
#define B2_TEXT8_ALT			  0x00B2C480 /* 8 Bit Text Fill Opcode / Load Master Register Alternating Between MNOOP_R0R1 and TEXT8_R2R3 (W) */
#define B2_TEXT16SETUP_ALT		  0x00B2C500 /* 16 Bit Text Fill Setup Opcode (W) */
#define B2_TEXT8SETUP_ALT		  0x00B2C580 /* 8 Bit Text Fill Setup Opcode (W) */
#define B2_SOLIDFILL_ALT		  0x00B2C800 /* Solid Fill Opcode / Load Master Register Alternating Between MNOOP_R0R1 and SOLIDFILL_R2R3 (W) */
#define B2_SOLIDFILLT_ALT		  0x00B2C880 /* Solid Fill Throttled Opcode / Load Master Register Alternating Between MNOOP_R0R1/SOLIDFILLT_R2R3 (W) */
#define B2_TDFILL_ALT			  0x00B2C900 /* TD Fill Opcode / Load Master Register Alternating Between MNOOP_R0R1 and TDFILL_R2R3 (W) */
#define B2_TDFILLT_ALT			  0x00B2C980 /* TD Fill Throttled Opcode / Load Master Register Alternating Between MNOOP_R0R1 and TDFILLT_R2R3 (W) */
#define B2_MNOOP_R0R1			  0x00B3C000 /* MFU Noop Opcode / Load Master Register 0 and 1 (W) */
#define B2_MNOOP_R2R3			  0x00B3C008 /* MFU Noop Opcode / Load Master Register 2 and 3 (W) */
#define B2_MNOOP_R4R5			  0x00B3C010 /* MFU Noop Opcode / Load Master Register 4 and 5 (W) */
#define B2_MNOOP_R6R7			  0x00B3C018 /* MFU Noop Opcode / Load Master Register 6 and 7 (W) */
#define B2_MNOOP_R8R9			  0x00B3C020 /* MFU Noop Opcode / Load Master Register 8 and 9 (W) */
#define B2_MNOOP_R10R11			  0x00B3C028 /* MFU Noop Opcode / Load Master Register 10 and 11 (W) */
#define B2_MNOOP_R12R13			  0x00B3C030 /* MFU Noop Opcode / Load Master Register 12 and 13 (W) */
#define B2_TEXT16_R0R1			  0x00B3C400 /* 16 Bit Text Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT16_R2R3			  0x00B3C408 /* 16 Bit Text Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_TEXT8_R0R1			  0x00B3C480 /* 8 Bit Text Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT8_R2R3			  0x00B3C488 /* 8 Bit Text Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_TEXT16SETUP_R0R1		  0x00B3C500 /* 16 Bit Text Fill Setup Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT16SETUP_R2R3		  0x00B3C508 /* 16 Bit Text Fill Setup Opcode / Load Master Register 2 and 3 (W) */
#define B2_TEXT8SETUP_R0R1		  0x00B3C580 /* 8 Bit Text Fill Setup Opcode / Load Master Register 0 and 1 (W) */
#define B2_TEXT8SETUP_R2R3		  0x00B3C588 /* 8 Bit Text Fill Setup Opcode / Load Master Register 2 and 3 (W) */
#define B2_SOLIDFILL_R0R1		  0x00B3C800 /* Solid Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_SOLIDFILL_R2R3		  0x00B3C808 /* Solid Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_SOLIDFILLT_R0R1		  0x00B3C880 /* Solid Fill Throttled Opcode / Load Master Register 0 and 1 (W) */
#define B2_SOLIDFILLT_R2R3		  0x00B3C888 /* Solid Fill Throttled Opcode / Load Master Register 2 and 3 (W) */
#define B2_TDFILL_R0R1			  0x00B3C900 /* TD Fill Opcode / Load Master Register 0 and 1 (W) */
#define B2_TDFILL_R2R3			  0x00B3C908 /* TD Fill Opcode / Load Master Register 2 and 3 (W) */
#define B2_TDFILLT_R0R1			  0x00B3C980 /* TD Fill Throttled Opcode / Load Master Register 0 and 1 (W) */
#define B2_TDFILLT_R2R3			  0x00B3C988 /* TD Fill Throttled Opcode / Load Master Register 2 and 3 (W) */
#define B2_BLT_R0R1			  0x00B3CC00 /* Blt Opcode / Load Master Register 0 and 1 (W) */
#define B2_BLT_R2R3			  0x00B3CC08 /* Blt Opcode / Load Master Register 2 and 3 (W) */
#define B2_BLT_R4R5			  0x00B3CC10 /* Blt Opcode / Load Master Register 4 and 5 (W) */
#define B2_EXBLT_R0R1			  0x00B3CC80 /* Expanding Blt Opcode / Load Master Register 0 and 1 (W) */
#define B2_EXBLT_R2R3			  0x00B3CC88 /* Expanding Blt Opcode / Load Master Register 2 and 3 (W) */
#define B2_EXBLT_R4R5			  0x00B3CC90 /* Expanding Blt Opcode / Load Master Register 4 and 5 (W) */
#define B2_EXBLT_R10R11			  0x00B3CCA8 /* Expanding Blt Opcode / Load Master Register 10 and 11 (W) */


#define DSM_NO_BYTE_SWAP			0xe4e4e4e4
#define DSM_RGBA_TO_ARGB_BYTE_SWAP		0x39393939
#define DSM_ARGB_TO_RGBA_BYTE_SWAP		0x93939393
#define DSM_PDU_BYTE_SWAP_DEFAULT		0x1b1b1b1b
#define DSM_DMA_BYTE_SWAP_DEFAULT		0x1b1b1b1b
#define DSM_MFU_BYTE_SWAP_DEFAULT		0x1b1b1b1b
#define DSM_HOST_IS_LITTLE_ENDIAN		0x01010101
#define DSM_HOST_IS_BIG_ENDIAN			0x00000000

#define B2_DBA_BUF_A		(0 << 12)
#define B2_DBA_BUF_AB		(1 << 12)
#define B2_DBA_BUF_ABC		(2 << 12)
#define B2_DBA_BUF_ABCD		(3 << 12)

#define B2_DBA_DIRECT		(0 << 6)
#define B2_DBA_IND_BG		(1 << 6)
#define B2_DBA_IND_FG		(2 << 6)
#define B2_DBA_IND_BG_FG	(3 << 6)
#define B2_DBA_OTC(x)		((x) << 24)
#define B2_DBA_P		(1 << 9)
#define B2_DBA_S		(1 << 10)
#define B2_DBA_D		(1 << 11)
#define B2_DBA_BIN8I		(0 << 16)	/* 8 bit indexed */  /* page 335 */
#define B2_DBA_BIN8F		(7 << 16)	/* ARGB888 truecolor */
#define B2_DBA_OTC04		(2 << 24)	/* 4 bytes per word access */
#define B2_DBA_OTC01		(0 << 24)	/* one ARGB888 per word access */

#define B2_EN2D_BYTE_MODE	0x30 | 0x80	/* mode and disable pixel interpolation */
#define B2_EN2D_WORD_MODE	0x50 | 0x80

#define UB_CP_CURSOR_ENABLE	       0x80000000


static char *mode_option; /* empty means take video mode from ROM */

/* const struct in ROM */
struct visfx_default {
	u8 __pad0[0x48];
	u32 pll_core_0;
	u32 pll_core_1;
	u8 __pad1[0x08];
	u32 pll_ram_0;
	u32 pll_ram_1;
	u8 __pad2[0x18];
	u32 pll_cb4038_0;
	u32 pll_cb4038_1;
	u8 __pad3[0x08];
	u32 pll_ga_ipll_0;
	u32 pll_ga_ipll_1;
	u8 __pad4[0x08];
};

struct visfx_par {
	void __iomem *reg_base;
	unsigned long reg_size;
	u32 abmap, ibmap0, bmap_z, ibmap1, obmap0;
	u32 dba, bmap_dba;
	u32 pseudo_palette[16];
	struct visfx_default *defaults;
	struct device *dev;

	struct fb_info *info;
};

static void visfx_setup(struct fb_info *info);

static u32 visfx_readl(struct fb_info *info, int reg)
{
	struct visfx_par *par = info->par;

	return le32_to_cpu(readl(par->reg_base + reg));
}

static void visfx_writel(struct fb_info *info, int reg, u32 val)
{
	struct visfx_par *par = info->par;

	return writel(cpu_to_le32(val), par->reg_base + reg);
}

static void visfx_write_vram(struct fb_info *info, int reg, u32 val)
{
	struct visfx_par *par = info->par;

	return writel(val, par->reg_base + reg);
}

static int visfx_wait_write_pipe_empty(struct fb_info *info)
{
	u32 status;
	int i;

	for(i = 0; i < 1000000; i++) {
		status = visfx_readl(info, UB_STATUS);
		// this fails on FX-10
		if (WARN_ON_ONCE(status & UB_STATUS_FAULT))
			return -EIO;

		if (!(status & UB_STATUS_WPNE))
			return 0;
		udelay(1);
	}
	WARN_ON_ONCE(1);
	return -ETIMEDOUT;
}

static void visfx_set_vram_addr(struct fb_info *info, int x, int y)
{
	visfx_writel(info, B2_DWA, (y << 16) | x);
}

static u32 visfx_cmap_entry(struct fb_info *info, int color)
{
	struct fb_cmap *cmap;

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
		return color;

	cmap = &info->cmap;
	return (((cmap->blue[color] & 0xff) << 0) |
		((cmap->green[color] & 0xff) << 8) |
		(cmap->red[color] & 0xff) << 16);
}

static void visfx_set_bmove_color(struct fb_info *info, int fg, int bg)
{
	visfx_writel(info, B2_IBC, visfx_cmap_entry(info, bg));
	visfx_writel(info, B2_IFC, visfx_cmap_entry(info, fg));
}

static void visfx_wclip(struct fb_info *info, int x1, int y1, int x2, int y2)
{
	visfx_writel(info, B2_WCLIP1UL, (x1 << 16) | y1);
	visfx_writel(info, B2_WCLIP1LR, (x2 << 16) | y2);
}

#define LINESIZE(x) (((x-1)/8)+1)
static void visfx_copyline(struct fb_info *info, const char *data, int x,
			   int width, int height, int len)
{
	u32 tmp;
	int y;

	for (y = 0; y < height; y++) {
		memcpy(&tmp, &data[y * LINESIZE(width) + x], len);
		visfx_write_vram(info, B2_BTDstObj_Yi, tmp);
	}
}

static void visfx_imageblit_mono(struct fb_info *info, const char *data, int dx, int dy,
				 int width, int height, int fg_color, int bg_color)
{
	int _width, x;

	visfx_writel(info, B2_DBA, B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR) {
		visfx_writel(info, B2_IPM, 0xff);
		visfx_writel(info, B2_BPM, 0xff);
	} else {
		visfx_writel(info, B2_BPM, 0xffffffff);
	}
	visfx_set_bmove_color(info, fg_color, bg_color);

	for (x = 0, _width = width; _width > 0; _width -= 32, x += 4) {
		visfx_set_vram_addr(info, dx + x * 8, dy);
		if (_width >= 32)
			visfx_copyline(info, data, x, width, height, 4);
		else {
			visfx_writel(info, B2_BPM, GENMASK(31, 31 - _width));
			visfx_copyline(info, data, x, width, height, LINESIZE(_width));
		}
	}
	visfx_writel(info, B2_IPM, 0xffffffff);
}

static void visfx_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct visfx_par *par = info->par;
	int x, y;

	visfx_wait_write_pipe_empty(info);

	switch (image->depth) {
	case 1:
		/* visfx_imageblit_mono doesn't work yet for 8bpp */
		if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
			return cfb_imageblit(info, image);

		visfx_imageblit_mono(info, image->data, image->dx, image->dy,
				     image->width, image->height,
				     image->fg_color, image->bg_color);
		break;
	case 8:
		visfx_writel(info, B2_DBA, B2_DBA_OTC01 | B2_DBA_DIRECT);
		visfx_writel(info, B2_BPM, 0xffffffff);

		for (y = 0; y < image->height; y++) {
			visfx_set_vram_addr(info, image->dx, image->dy + y);
			for (x = 0; x < image->width; x++) {
				u32 c = ((unsigned char *)image->data)[y * image->width + x];
				if (info->fix.visual != FB_VISUAL_PSEUDOCOLOR) {
					c = ((u32*)info->pseudo_palette)[c];
					c = le32_to_cpu(c);
				}
				visfx_write_vram(info, B2_BTDstObj_Xi, c);
			}
		}
		break;

	default:
		return cfb_imageblit(info, image);
	}

	visfx_writel(info, B2_DBA, par->dba);
}

static void visfx_fillrect(struct fb_info *info, const struct fb_fillrect *fr)
{
	struct visfx_par *par = info->par;

	visfx_writel(info, B2_DBA, B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	visfx_set_bmove_color(info, fr->color, 0);
	visfx_writel(info, B2_MNOOP_R0R1, (fr->dx << 16) | fr->dy);
	visfx_writel(info, B2_SOLIDFILL_R2R3, (fr->width << 16) | fr->height);

	visfx_writel(info, B2_DBA, par->dba);
}

static void visfx_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	visfx_writel(info, B2_MNOOP_R4R5, (area->sx << 16) | area->sy);
	visfx_writel(info, B2_MNOOP_R2R3, (area->width << 16) | area->height);
	visfx_writel(info, B2_BLT_R0R1, (area->dx << 16) | area->dy);
}

static int visfx_setcolreg(unsigned regno, unsigned red, unsigned green,
			     unsigned blue, unsigned transp,
			     struct fb_info *info)
{
	u32 r, g, b;

	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	switch (info->fix.visual) {
	case FB_VISUAL_PSEUDOCOLOR:
		if (regno >= 256)
			return -EINVAL;
		r = (red >> 8) << 16;
		g = (green >> 8) << 8;
		b = (blue >> 8);
		set_clut:
		visfx_writel(info, B2_LLCA, regno);
		visfx_writel(info, B2_LUTD, r | g | b);
		break;
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_DIRECTCOLOR:
		r = (red >> (16 - info->var.red.length))
			<< info->var.red.offset;
		b = (blue >> (16 - info->var.blue.length))
			<< info->var.blue.offset;
		g = (green >> (16 - info->var.green.length))
			<< info->var.green.offset;
		if (info->fix.visual == FB_VISUAL_DIRECTCOLOR)
			goto set_clut;
		break;
	default:
		return -EINVAL;
	}

	if (regno < 16)
		((u32 *) info->pseudo_palette)[regno] = r | g | b;

	return 0;
}

static int visfx_blank(int blank_mode, struct fb_info *info)
{
	u32 mpc;

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
		mpc = 3;
		break;
	case FB_BLANK_NORMAL:
		mpc = 0;
		break;
	case FB_BLANK_UNBLANK:
		mpc = 3 << 2;
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		mpc = 1 << 1;
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		mpc = 1 << 0;
		break;
	default:
		return 1;
	}
	visfx_writel(info, B2_MPC, mpc);

	return 0;
}

int visfx_sync(struct fb_info *info)
{
	visfx_wait_write_pipe_empty(info);
	return 0;
}

static void visfx_get_video_mode(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	unsigned long n, d;
	u32 tmp;

	tmp = visfx_readl(info, B2_VHAL);
	var->xres = (tmp & 0xffff) + 1;
	var->yres = (tmp >> 16) + 1;

	tmp = visfx_readl(info, B2_PLL_DOT_CTL);
	n = (tmp & 0xff) + 1;
	d = ((tmp >> 8) & 0xff) + 1;
	var->pixclock = (VISFX_SYNC_PLL_BASE / d) * n;

	tmp = visfx_readl(info, B2_HTG);
	var->left_margin = ((tmp >> 20) & 0x1ff) + 1;
	var->hsync_len = (((tmp >> 12) & 0xff) + 1) * 4;
	var->right_margin = (tmp & 0x1ff) + 1;

	tmp = visfx_readl(info, B2_VTG);
	var->upper_margin = ((tmp >> 16) & 0xff) + 1;
	var->vsync_len = ((tmp >> 8) & 0xff) + 1;
	var->lower_margin = (tmp & 0xff) + 1;

	tmp = visfx_readl(info, B2_CFG);
	if (tmp & VISFX_HSYNC_POSITIVE)
		var->sync |= FB_SYNC_HOR_HIGH_ACT;
	if (tmp & VISFX_VSYNC_POSITIVE)
		var->sync |= FB_SYNC_VERT_HIGH_ACT;
}

static int visfx_wait_pll(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	u32 status, pll_stat;
	int i = 0;

	visfx_writel(info, B2_PLL_REF_CNT, BIT(31) | 202500);

	for (i = 0; i < 100000; i++) {
		status = visfx_readl(info, UB_STATUS);
		pll_stat = visfx_readl(info, B2_PLL_REF_CNT) & 0xffffff;
		if (pll_stat == 0 && !(pll_stat & (UB_STATUS_FAULT | UB_STATUS_WPNE)))
			break;
		udelay(1);
	}

	if (status & (UB_STATUS_FAULT | UB_STATUS_WPNE)) {
		dev_err(par->dev, "WPNE error %x\n", status);
		return -EIO;
	}

	if (i == 100000) {
		dev_err(par->dev, "timeout (status %x)\n",
		       visfx_readl(info, B2_PLL_REF_CNT));
		return -ETIMEDOUT;
	}
	return 0;
}

static int visfx_init_pll(struct fb_info *info, u32 reg, u32 value1, u32 value2)
{
	int ret;

	visfx_writel(info, reg, value1);
	ret = visfx_wait_pll(info);
	if (ret)
		return ret;

	visfx_writel(info, reg, value2);
	ret = visfx_wait_pll(info);
	if (ret)
		return ret;

	visfx_writel(info, reg, value2 | 0x10000);
	return visfx_wait_pll(info);
}

static int visfx_set_pll(struct fb_info *info, u32 reg, unsigned long clock)
{
	unsigned long n, d, tmp;

	rational_best_approximation(clock, VISFX_SYNC_PLL_BASE, 0x3f, 0x3f, &n, &d);
	tmp = 0x520000 | ((((d * 4) - 1) << 8) | ((n * 4) - 1));
	return visfx_init_pll(info, reg, tmp, tmp);
}

static int visfx_set_par(struct fb_info *info)
{
	u32 xres, yres, hbp, hsw, hfp, vbp, vsw, vfp, tmp;
	struct fb_var_screeninfo *var = &info->var;
	int ret;

	xres = var->xres;
	yres = var->yres;
	hsw = var->hsync_len / 4 - 1;
	hfp = (var->right_margin - 1) & 0x1ff;
	hbp = var->left_margin - 1;
	vsw = var->vsync_len - 1;
	vfp = var->lower_margin - 1;
	vbp = var->upper_margin - 1;

	ret = visfx_set_pll(info, B2_PLL_DOT_CTL, var->pixclock);
	if (ret) {
		pr_warn("visfx_set_par: failed to set PLL\n");
		return ret;
	}

	visfx_writel(info, B2_HTG, (hbp << 20) | (hsw << 12) | (0xc << 8) | hfp);
	visfx_writel(info, B2_VTG, (vbp << 0) | (vsw << 8) | (vfp << 16));
	visfx_writel(info, B2_VHAL,((yres - 1) << 16) | (xres - 1));
	visfx_writel(info, B2_ETG, 0x12260);
	visfx_writel(info, B2_SCR, 0);
	visfx_wclip(info, 0, 0, xres, yres);

	tmp = VISFX_DFP_ENABLE;
	if (var->sync & FB_SYNC_HOR_HIGH_ACT)
		tmp |= VISFX_HSYNC_POSITIVE;
	if (var->sync & FB_SYNC_VERT_HIGH_ACT)
		tmp |= VISFX_VSYNC_POSITIVE;
	visfx_writel(info, B2_CFG, tmp);
	visfx_writel(info, B2_MPC, 0xc);

	visfx_setup(info);

	return visfx_wait_write_pipe_empty(info);
}

static int visfx_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct fb_var_screeninfo *v = &info->var;

	if (var->pixclock > VISFX_SYNC_PLL_BASE ||
		var->left_margin > 512 ||
		var->right_margin > 512 ||
		var->hsync_len > 512 ||
		var->lower_margin > 256 ||
		var->upper_margin > 256 ||
		var->vsync_len > 256 ||
		var->xres > 2048 ||
		var->yres > 2048)
		return -EINVAL;

	if (var->bits_per_pixel == 24)
		var->bits_per_pixel = 32;
	if (var->bits_per_pixel != 8 && var->bits_per_pixel != 32)
		return -EINVAL;

	if (var->xres & 0x7)
		return -EINVAL;
	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	var->red    = v->red;
	var->green  = v->green;
	var->blue   = v->blue;
	var->transp = v->transp;

	var->nonstd = 0;
	var->height = -1;
	var->width = -1;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->accel_flags = info->flags;

	return 0;
}

static void visfx_update_cursor_image_line(struct fb_info *info,
					   struct fb_cursor *cursor, int y)
{
	unsigned int x, bytecnt;
	u32 data[2] = { 0 };
	u8 d, m;

	bytecnt = ((cursor->image.width - 1) / 8) + 1;

	for (x = 0; x < bytecnt && x < 8; x++) {
		m = cursor->mask[y * bytecnt + x];
		d = cursor->image.data[y * bytecnt + x];

		if (cursor->rop == ROP_XOR)
			((u8 *)data)[x] = d ^ m;
		else
			((u8 *)data)[x] = d & m;
	}

	if (cursor->image.width < 32)
		data[0] &= GENMASK(31, 31 - cursor->image.width + 1);
	visfx_writel(info, UB_CD, data[0]);
	if (cursor->image.width < 64)
		data[0] &= GENMASK(31, 63 - cursor->image.width + 1);
	visfx_writel(info, UB_CD, data[1]);
}

static void visfx_update_cursor_image(struct fb_info *info,
				      struct fb_cursor *cursor)
{
	int y, height = cursor->image.height;

	if (height > 128)
		height = 128;

	visfx_writel(info, UB_CA, 0);
	for (y = 0; y < height; y++)
		visfx_update_cursor_image_line(info, cursor, y);

	for (; y < 256; y++)
		visfx_writel(info, UB_CD, 0);
}

static int visfx_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	u32 cp, color;

	cp = (cursor->image.dx << 16) | (cursor->image.dy & 0xffff);
	visfx_writel(info, UB_CP, cp);

	if (cursor->set & (FB_CUR_SETIMAGE|FB_CUR_SETSHAPE))
		visfx_update_cursor_image(info, cursor);

	if (cursor->set & FB_CUR_SETCMAP) {
		color = cursor->image.fg_color;
		if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
			color = ((u32 *) info->pseudo_palette)[color & 0x0f];
		else
			color = visfx_cmap_entry(info, color);
		visfx_writel(info, UB_CB, color);
		visfx_writel(info, UB_CF, 0);
	}

	if (cursor->enable) {
		cp |= UB_CP_CURSOR_ENABLE;
		visfx_writel(info, UB_CP, cp);
	}
	return 0;
}

static const struct fb_ops visfx_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= visfx_check_var,
	.fb_set_par	= visfx_set_par,
	.fb_setcolreg	= visfx_setcolreg,
	.fb_blank	= visfx_blank,
	.fb_fillrect	= visfx_fillrect,
	.fb_copyarea	= visfx_copyarea,
	.fb_imageblit	= visfx_imageblit,
	.fb_cursor	= visfx_cursor,
	.fb_sync	= visfx_sync,
};

static void visfx_bus_error_timer_enable(struct fb_info *info, bool enable)
{
	u32 tmp = visfx_readl(info, UP_CONTROL);

	if (enable)
		tmp |= 0x1000;
	else
		tmp &= ~0x1000;
	visfx_writel(info, UP_CONTROL, tmp);
}

static int visfx_soft_reset(struct fb_info *info)
{
	int i = 0;
	u32 old;

	old = visfx_readl(info, UP_CONTROL);
	visfx_writel(info, UP_CONTROL, UP_CONTROL_SOFT_RST);
	while (visfx_readl(info, UP_CONTROL) & UP_CONTROL_SOFT_RST) {
		if (i++ > 10000)
			return -ETIMEDOUT;
		udelay(10);
	}
	visfx_writel(info, UP_TCP, 0x00030000);
	visfx_writel(info, UP_CONTROL, old & ~UP_CONTROL_TO);
	visfx_writel(info, B2_WORG, 0);
	if (visfx_readl(info, UB_STATUS) & UB_STATUS_FAULT)
		return -EIO;
	return 0;
}

static int visfx_reset(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	int ret;
	u32 tmp;

	ret = visfx_soft_reset(info);
	if (ret)
		return ret;

	ret = visfx_wait_write_pipe_empty(info);
	if (ret)
		return ret;

	visfx_writel(info, B2_SWEN, 0xffffffff);
	visfx_writel(info, B2_SREN, 0xffffffff);

	ret = visfx_wait_write_pipe_empty(info);
	if (ret)
		return ret;

	visfx_writel(info, B2_MV, 0x2001db0);
	visfx_writel(info, UB_UIRC, 0x04040404);
	visfx_writel(info, B2_IRC, 0x04040404);
	visfx_writel(info, B2_DMA_BSCSAV, 0);
	visfx_writel(info, UB_DMA_UBSCSAV, 0);
	visfx_writel(info, B2_DMA_BSCBLK, 0);
	visfx_writel(info, UB_DMA_UBSCBLK, 0);
	visfx_writel(info, UP_TCP, 0x30000);
	visfx_writel(info, UP_DC, 0x30301);
	visfx_writel(info, BP_CF_DRC, 0x300);
	visfx_writel(info, BP_CF_HWC, 0x104);
	visfx_writel(info, BP_CF_LWC, 0x3e8);
	visfx_writel(info, BP_CD_HWC, 0x4);
	visfx_writel(info, B2_CD_BUFFER_CTL, 0x4040404);
	visfx_writel(info, B2_PDU_BSCFB, 0x1b);
	visfx_writel(info, UB_PDU_UBSCFB, 0x1b);
	visfx_writel(info, B2_DMA_BSCFB, 0x1b);
	visfx_writel(info, UB_DMA_UBSCFB, 0x1b);
	visfx_writel(info, B2_FBC_RBS, 0xe4);
	visfx_writel(info, B2_SOV, 0);
	visfx_writel(info, B2_MFU_BSCCTL, 0x1b);
	visfx_writel(info, B2_MFU_BSCTD, 0x1b);
	visfx_writel(info, BP_ICR, 0);
	visfx_writel(info, B2_SWEN, 0xffffffff);
	visfx_writel(info, B2_SREN, 0x1000001);
	visfx_writel(info, B2_FPR, 0);

	visfx_writel(info, B3_BBC_CONFIG, 0);
	tmp = visfx_readl(info, B3_HUNGRY);

	switch(tmp) {
	case 0x2a:
		visfx_writel(info, B3_NUM_GA, 0x00000003);
		visfx_writel(info, B3_GA_CONFIG, 0x00000002);
		break;
	case 0xaaa:
		visfx_writel(info, B3_NUM_GA, 0x00000006);
		visfx_writel(info, B3_GA_CONFIG, 0x00000005);
		break;
	default:
		dev_err(par->dev, "unknown value %x\n", tmp);
		break;
	}
	visfx_writel(info, B3_BBC_CONFIG, 4);
	return 0;
}

int visfx_set_default_mode(struct fb_info *info)
{
	u32 tmp;
	int ret;

	ret = visfx_init_pll(info, B2_PLL_DOT_CTL,
			     visfx_readl(info, 0x50028),
			     visfx_readl(info, 0x5002c));
	if (ret)
		return ret;

	visfx_writel(info, B2_HTG, visfx_readl(info, 0x50038));
	visfx_writel(info, B2_VTG, visfx_readl(info, 0x5003c));
	visfx_writel(info, B2_CFG, visfx_readl(info, 0x50040));

	tmp = (visfx_readl(info, 0x50024) & 0xffff0000) |
		(visfx_readl(info, 0x50020) & 0xffff);

	visfx_writel(info, B2_VHAL, tmp - 0x10001);

	visfx_writel(info, B2_ETG, visfx_readl(info, 0x50044));
	visfx_writel(info, B2_SCR, visfx_readl(info, 0x5004c));
	visfx_writel(info, B2_IMD, 2);
	return 0;
}

static void visfx_buffer_setup(struct fb_info *info, int num,
			       u32 imc, u32 ibs, u32 iclr)
{
	visfx_writel(info, B2_ICLR0 + (num << 2), iclr);
	visfx_writel(info, B2_IMC0 + (num << 2), imc);
	visfx_writel(info, B2_IBS0 + (num << 2), ibs);
}

static void visfx_setup_and_wait(struct fb_info *info, u32 reg, u32 val)
{
	visfx_writel(info, UP_CF_STATE, (1 << 25));
	visfx_writel(info, B2_VBS, 1);
	visfx_writel(info, reg, val);
	visfx_writel(info, UP_CF_STATE, 0);
	visfx_wait_write_pipe_empty(info);
}

static void visfx_clear_buffer(struct fb_info *info, u32 dba, u32 bmap_dba, u32 mbwb, u32 ifc)
{
	struct fb_var_screeninfo *var = &info->var;

	visfx_writel(info, B2_DBA, dba);
	visfx_writel(info, B2_BMAP_DBA, bmap_dba);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_IPM, 0xffffffff);
	visfx_writel(info, B2_IFC, ifc);
	visfx_writel(info, B2_FOE, 1);
	visfx_writel(info, B2_SOV, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_MBWB, mbwb);
	visfx_writel(info, B2_MNOOP_R0R1, 0);
	visfx_writel(info, B2_SOLIDFILL_R2R3_REMAP, var->xres<<16 | var->yres);
}

static void visfx_setup(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int i;

	visfx_wait_write_pipe_empty(info);

#ifdef __BIG_ENDIAN
	if (var->bits_per_pixel == 32) {
		visfx_writel(info, B2_DMA_BSCFB, DSM_NO_BYTE_SWAP);
		visfx_writel(info, B2_PDU_BSCFB, DSM_NO_BYTE_SWAP);
	} else {
		u32 val = DSM_NO_BYTE_SWAP;
		visfx_writel(info, B2_DMA_BSCFB, val);
		visfx_writel(info, B2_PDU_BSCFB, val);
		visfx_writel(info, B2_FBC_RBS, DSM_NO_BYTE_SWAP);
	}
	visfx_writel(info, B2_MFU_BSCTD, DSM_NO_BYTE_SWAP);
	visfx_writel(info, B2_MFU_BSCCTL, DSM_PDU_BYTE_SWAP_DEFAULT);
#else
	visfx_writel(info, B2_DMA_BSCFB, DSM_PDU_BYTE_SWAP_DEFAULT);
	visfx_writel(info, B2_PDU_BSCFB, DSM_PDU_BYTE_SWAP_DEFAULT);
	visfx_writel(info, B2_MFU_BSCTD, DSM_PDU_BYTE_SWAP_DEFAULT);
	visfx_writel(info, B2_MFU_BSCCTL, DSM_PDU_BYTE_SWAP_DEFAULT);
#endif

	visfx_writel(info, B2_DMA_BSCBLK, 0);
	visfx_writel(info, B2_VTB, 0x35);
	visfx_writel(info, B2_TTB, 0x99);
	visfx_writel(info, B2_EN2D, 0);
	visfx_writel(info, B2_FOE, 1);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_SB, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_ZBO, 0x00080000);
	visfx_writel(info, B2_RTG_MEM_LOAD, 0xc9200);
	visfx_writel(info, B2_TM_TSS, 0);
	visfx_writel(info, B2_FCDA, 0);
	visfx_writel(info, B2_BMAP_Z, 0);
	visfx_writel(info, B2_FSRMWB, 0);
	visfx_readl(info, UB_CONTROL);
	visfx_wait_write_pipe_empty(info);

	visfx_writel(info, UP_CF_STATE, 0x02000000);
	visfx_writel(info, B2_VBS, 1);

	par->abmap  = var->xres / 4;	/* x-resolution encoded in lower bits */
	par->ibmap0 = 0x02681002;
	par->bmap_z = 0x13090006;
	par->obmap0 = 0x23a80000 | var->xres / 2;
	par->ibmap1 = 0x27e00002;

	visfx_setup_and_wait(info, B2_ABMAP, par->abmap);
	visfx_setup_and_wait(info, B2_IBMAP0, par->ibmap0);
	visfx_setup_and_wait(info, B2_BMAP_Z, par->bmap_z);
	visfx_setup_and_wait(info, B2_OBMAP0, par->obmap0);
	visfx_setup_and_wait(info, UP_CF_STATE, 0);
	visfx_setup_and_wait(info, B2_IBMAP1, par->ibmap1);
	visfx_setup_and_wait(info, B2_DUM, 0x81030002);
	visfx_setup_and_wait(info, B2_OXYO, 0);

	visfx_writel(info, B2_PMASK, 0xff);
	visfx_writel(info, B2_MPC, 0x0c);

	for (i = 0; i < 7; i++)
		visfx_buffer_setup(info, i, 0, 0, 0);

	visfx_buffer_setup(info, 0, (var->bits_per_pixel == 32) ? 0x09000004 : 0x2000000, 0, 0);
	visfx_buffer_setup(info, 1, (var->bits_per_pixel == 32) ? 0x09000004 : 0x2000000, 1, 0);
	visfx_setup_and_wait(info, B2_OMC, 0x2000000);
	visfx_writel(info, B2_OTLS, 2);
	visfx_writel(info, B2_OBS, 0);

	visfx_wclip(info, 0, 0, var->xres, var->yres);
	visfx_clear_buffer(info, 0x05000880, par->ibmap0, 0x03f00000, 0);
	visfx_clear_buffer(info, 0x05000880, par->obmap0, 0x00fc0000, (var->bits_per_pixel == 32) ? 0xffffffff : 0);
	visfx_clear_buffer(info, 0x05000880, par->abmap,  0x00900000, 0);

	visfx_writel(info, B2_FATTR, 0);
	visfx_writel(info, B2_OTLS, (var->bits_per_pixel == 8) ? 0 : 0x10002);
	visfx_writel(info, B2_CKEY_HI, 0xffffff);
	visfx_writel(info, B2_CKEY_LO, 0xffffff);

	for (i = 0; i < 256; i++) {
		visfx_writel(info, B2_LLCA, 0x40003000 | i);
		visfx_writel(info, B2_LUTD, 0x010101 * i);
	}

	// visfx_clear_buffer(info, 0x00000a00, par->ibmap0, 0x03f00000, 0);
	visfx_buffer_setup(info, 2, 0x08000084, 0, 0);

	visfx_wait_write_pipe_empty(info);
	visfx_writel(info, B2_BMAP_DBA, par->ibmap0);

	info->fix.accel = FB_ACCEL_NONE;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.line_length = 2048 * var->bits_per_pixel / 8;

	switch (var->bits_per_pixel) {
	default:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		var->red.length = 8;
		var->red.offset = 8;
		var->green.length = 8;
		var->green.offset = 16;
		var->blue.length = 8;
		var->blue.offset = 24;
		var->transp.length = 8;
		var->transp.offset = 0;
		break;
	case 8:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		var->red.length = 8;
		var->red.offset = 0;
		var->green.length = 8;
		var->green.offset = 0;
		var->blue.length = 8;
		var->blue.offset = 0;
		var->transp.length = 0;
		var->transp.offset = 0;
	}

	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	visfx_wclip(info, 0, 0, var->xres, var->yres);

	/* set DBA */
	if (info->var.bits_per_pixel == 32) {
		visfx_writel(info, B2_EN2D, B2_EN2D_WORD_MODE);
		par->dba = B2_DBA_BIN8F | B2_DBA_OTC01 | B2_DBA_DIRECT | B2_DBA_D;
		par->bmap_dba = par->ibmap0;
		visfx_writel(info, B2_BPM, 0xffffffff);
		visfx_writel(info, B2_OTR, 1<<16 | 1<<8 | 0);
		visfx_writel(info, B2_FATTR, 0);
	} else { /* 8-bit indexed: */
		visfx_writel(info, B2_EN2D, B2_EN2D_BYTE_MODE);
		par->dba = B2_DBA_BIN8I | B2_DBA_OTC04 | B2_DBA_DIRECT | B2_DBA_D;
		par->bmap_dba = par->obmap0;
		visfx_writel(info, B2_BPM, 0xff << 24);
		visfx_writel(info, B2_OTR, 2);
		visfx_writel(info, B2_CFS16, 0x40);
		visfx_writel(info, B2_FATTR, 1<<7 | 1<<4); /* CFS16 & force Overlay */
	}

	visfx_writel(info, B2_BMAP_BABoth, par->bmap_dba);
	visfx_writel(info, B2_BABoth, par->dba);
	visfx_writel(info, B2_IPM, 0xffffffff);
}

static int __init visfx_initialize(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	int i, ret;

	pr_info("visfx: PLL: %08x:%08x, RAM: %08x:%08x, CB: %08x:%08x, GA: %08x:%08x\n",
		par->defaults->pll_core_0, par->defaults->pll_core_1,
		par->defaults->pll_ram_0, par->defaults->pll_ram_1,
		par->defaults->pll_cb4038_0, par->defaults->pll_cb4038_1,
		par->defaults->pll_ga_ipll_0, par->defaults->pll_ga_ipll_1);

	visfx_bus_error_timer_enable(info, false);
	ret = visfx_reset(info);
	if (ret)
		return ret;
	visfx_bus_error_timer_enable(info, true);

	ret = visfx_init_pll(info, B2_PLL_CORE_CTL,
			     par->defaults->pll_core_0,
			     par->defaults->pll_core_1);
	if (ret)
		return ret;

	ret = visfx_init_pll(info, B2_PLL_RAM_CTL,
			     par->defaults->pll_ram_0,
			     par->defaults->pll_ram_1);
	if (ret)
		return ret;

	visfx_writel(info, B2_RC_CONFIG1, 0);
	visfx_writel(info, B2_RC_CONFIG0, 0x04444649);
	visfx_writel(info, B2_RC_CONFIG2, 0x0119F051);
	visfx_writel(info, B2_RC_CONFIG1, 0xE3901212);

	ret = visfx_init_pll(info, 0xcb4038,
			     par->defaults->pll_cb4038_0,
			     par->defaults->pll_cb4038_1);
	if (ret)
		return ret;

	for(i = 0; i < 6; i++)
		visfx_writel(info, B3_GA_NOP_EOC, 0);

	ret = visfx_init_pll(info, B3_GA_IPLL,
			     par->defaults->pll_ga_ipll_0,
			     par->defaults->pll_ga_ipll_1);
	if (ret)
		return ret;

	for(i = 0; i < 6; i++)
		visfx_writel(info, B3_GA_NOP_EOC, 0);

	visfx_writel(info, B2_FOE, 0);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_FCDA, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_IPM, 0xffffffff);

	visfx_wclip(info, 0, 0, 2048, 2048);
	visfx_writel(info, B2_EN2D, B2_EN2D_BYTE_MODE);
	visfx_writel(info, B2_BABoth, 0x2000000);
	visfx_writel(info, B2_BMAP_BABoth, 0x200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x80000200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x40000200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x200);
	visfx_writel(info, B2_MV, 0);

	// setup video regs
	ret = visfx_set_default_mode(info);
	if (ret)
		return ret;

	visfx_writel(info, B2_MFU_BSCTD, 0x1b);
	visfx_writel(info, B2_MFU_BSCCTL, 0x1b);
	visfx_writel(info, B2_FBC_RBS, 0xe4);
	visfx_writel(info, B2_FOE, 0);

	visfx_writel(info, B2_EN2D, B2_EN2D_BYTE_MODE);

	return 0;
}

static int __init visfx_get_rom_defaults(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	u32 offset, id;

	if (visfx_readl(info, 0) != 0x55aa0000)
		return -EINVAL;

	offset = le32_to_cpu(visfx_readl(info, 0x08)) + 0x08;
	if (offset > par->reg_size)
		return -EINVAL;

	id = visfx_readl(info, offset);

	if (id != VISFX_CARDTYPE_FX5) {
		dev_err(par->dev, "Unsupported card: ID %08x\n", id);
		return -EINVAL;
	}

	offset = le32_to_cpu(visfx_readl(info, 0x08)) + 0x28;
	if (offset > par->reg_size)
		return -EINVAL;

	offset = visfx_readl(info, offset);
	if (offset > par->reg_size)
		return -EINVAL;

	par->defaults = par->reg_base + offset;
	return 0;
}

static void __init visfx_setup_info(struct pci_dev *pdev, struct fb_info *info)
{
	struct visfx_par *par = info->par;

	info->fbops = &visfx_ops;
	info->flags = FBINFO_HWACCEL_FILLRECT | FBINFO_HWACCEL_IMAGEBLIT |
			FBINFO_HWACCEL_COPYAREA | FBINFO_READS_FAST;
	info->pseudo_palette = par->pseudo_palette;
	info->screen_base = par->reg_base + VISFX_FB_OFFSET;

	strscpy(info->fix.id, "Visualize-FX", sizeof(info->fix.id));
	info->fix.mmio_start = pci_resource_start(pdev, 0);
	info->fix.smem_start = pci_resource_start(pdev, 0) + VISFX_FB_OFFSET;
	info->fix.smem_len = VISFX_FB_LENGTH;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	if (DEFAULT_BPP == 32)
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	else
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	info->fix.line_length = 2048 * DEFAULT_BPP / 8;
	info->fix.accel = FB_ACCEL_NONE;
}

static int __init visfx_init_device(struct pci_dev *pdev, struct sti_struct *sti)
{
	struct visfx_par *par;
	struct fb_info *info;
	char mode[64];
	int ret;

	info = framebuffer_alloc(sizeof(struct visfx_par), &pdev->dev);
	if (!info)
		return -ENOMEM;

	pci_set_drvdata(pdev, info);
	par = info->par;
	par->info = info;
	par->reg_size = pci_resource_len(pdev, 0);
	par->dev = &pdev->dev;

	if (sti) {
		sti->info = info;
		par->reg_base = pci_iomap(pdev, 0, VISFX_FB_OFFSET + VISFX_FB_LENGTH);
	} else
		par->reg_base = pcim_iomap_table(pdev)[0];

	ret = visfx_get_rom_defaults(info);
	if (ret)
		goto err_out_free;

	visfx_setup_info(pdev, info);
	if (sti)
		strscpy(info->fix.id, sti->sti_data->inq_outptr.dev_name,
			sizeof(info->fix.id));

	ret = visfx_initialize(info);
	if (ret)
		goto err_out_free;

	info->var.bits_per_pixel = DEFAULT_BPP;
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto err_out_free;

	visfx_get_video_mode(info);

	if (!mode_option) {
		scnprintf(mode, sizeof(mode), "%dx%d@60",
			info->var.xres, info->var.yres);
		mode_option = mode;
	}

	fb_find_mode(&info->var, info, mode_option, NULL, 0, NULL, DEFAULT_BPP);
	visfx_check_var(&info->var, info);
	visfx_set_par(info);

	ret = register_framebuffer(info);
	if (ret)
		goto err_out_dealloc_cmap;

	fb_info(info, "visfxfb %dx%d-%d frame buffer device, %s, mmio: 0x%04lx\n",
		info->var.xres,
		info->var.yres,
		info->var.bits_per_pixel,
		info->fix.id,
		info->fix.mmio_start);

	return 0;

err_out_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);
err_out_free:
	framebuffer_release(info);
	return ret;
}

static int visfx_probe_pci(struct pci_dev *pdev,
	       const struct pci_device_id *ent)
{
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot enable PCI device: %d\n", ret);
		return ret;
	}

	ret = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
	if (ret) {
		dev_err(&pdev->dev, "Cannot map PCI resources: %d\n", ret);
		return ret;
	}

	return visfx_init_device(pdev, NULL);
}

static void visfx_probe_sti(struct sti_struct *sti, int enable)
{
	if (!sti || !sti->pd)
		return;

	if (sti->graphics_id[0] != VISFX_CARDTYPE_FX5)
		return;

	if (enable)
		visfx_init_device(sti->pd, sti);
	else {
		struct fb_info *info;
		struct visfx_par *par;

		info = pci_get_drvdata(sti->pd);
		par = info->par;
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
		pci_iounmap(sti->pd, par->reg_base);
	}
}

static int __init visfx_options(char *options)
{
	if (!options || !*options)
		return 0;

	mode_option = options;

	return 0;
}

static void __exit visfx_remove(struct pci_dev *pdev)
{
	struct fb_info *info = pci_get_drvdata(pdev);

	unregister_framebuffer(info);
	fb_dealloc_cmap(&info->cmap);
	framebuffer_release(info);
}

static const struct pci_device_id visfx_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_HP, PCI_DEVICE_ID_HP_VISUALIZE_FX4) },
	{ 0 },
};
MODULE_DEVICE_TABLE(pci, visfx_pci_tbl);

static struct pci_driver visfx_driver = {
	.name	   = KBUILD_MODNAME,
	.id_table  = visfx_pci_tbl,
	.probe	   = visfx_probe_pci,
	.remove    = visfx_remove,
};

static int __init visfx_init(void)
{
	char *option = NULL;
	int i;

	if (fb_get_options(KBUILD_MODNAME, &option))
		return -ENODEV;

	visfx_options(option);

	if (IS_ENABLED(CONFIG_STI_CONSOLE) || IS_ENABLED(CONFIG_FB_STI))
		for (i = 1; i <= MAX_STI_ROMS; i++)
			visfx_probe_sti(sti_get_rom(i), 1);

	return pci_register_driver(&visfx_driver);
}

static void __exit visfx_exit(void)
{
	int i;

	if (IS_ENABLED(CONFIG_STI_CONSOLE) || IS_ENABLED(CONFIG_FB_STI))
		for (i = 1; i <= MAX_STI_ROMS; i++)
			visfx_probe_sti(sti_get_rom(i), 0);

	pci_unregister_driver(&visfx_driver);
}

module_init(visfx_init);
module_exit(visfx_exit);

module_param(mode_option, charp, 0);
MODULE_PARM_DESC(mode_option, "Initial video mode or empty for ROM defaults");

MODULE_AUTHOR("Sven Schnelle <svens@stackframe.org>");
MODULE_AUTHOR("Helge Deller <deller@gmx.de>");
MODULE_DESCRIPTION("Framebuffer driver for HP Visualize FX cards");
MODULE_LICENSE("GPL");
