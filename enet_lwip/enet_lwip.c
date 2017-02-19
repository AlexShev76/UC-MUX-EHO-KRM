//*****************************************************************************/
//
// enet_lwip.c - Sample WebServer Application using lwIP.
//
// Copyright (c) 2007-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 6594 of the DK-LM3S9B96 Firmware Package.
//
//*****************************************************************************/
//   MAC IP    00-09-34-XX-8c-1c

#include <string.h>
#include <stdlib.h>


#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_epi.h"
#include "inc/hw_ethernet.h"

#include "driverlib/ethernet.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/locator.h"
#include "utils/lwiplib.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/swupdate.h"

#include <string.h>

//#include <math.h>

//#include "grlib/grlib.h"
//#include "httpserver_raw/httpd.h"
//#include "drivers/kitronix320x240x16_ssd2119_8bit.h"
//#include "drivers/set_pinout.h"
#include "udp_srv.h"

#include "DSP.h"


#define  ADD_EMERSION_STROB


#define ENABLE_START_DETECT_FPGA


#define GET_MAX_ASCAN

#define IP_UPDATER

//#define PASE_ADDR_DEFOULT      0
//#define PASE_ADDR_DEFOULT      8

#define PASE_ADDR_DEFOULT      10                  //  Серийний вариант
//#define PASE_ADDR_DEFOULT      18



//****************************************************************************/*
//
//! \addtogroup example_list
//! <h1>Ethernet with lwIP (enet_lwip)</h1>
//!
//! This example application demonstrates the operation of the Stellaris
//! Ethernet controller using the lwIP TCP/IP Stack.  DHCP is used to obtain
//! an Ethernet address.  If DHCP times out without obtaining an address,
//! AUTOIP will be used to obtain a link-local address.  The address that is
//! selected will be shown on the QVGA display.
//!
//! The file system code will first check to see if an SD card has been plugged
//! into the microSD slot.  If so, all file requests from the web server will
//! be directed to the SD card.  Otherwise, a default set of pages served up
//! by an internal file system will be used.
//!
//! For additional details on lwIP, refer to the lwIP web page at:
//! http://savannah.nongnu.org/projects/lwip/
//
//*****************************************************************************/

//*****************************************************************************/
//
// Defines for setting up the system clock.
//
//*****************************************************************************/
//#define SYSTICKHZ               100
#define TIMER_PERIOD            100
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)
#define SYSTICKUS               (1000000 / SYSTICKHZ)
#define SYSTICKNS               (1000000000 / SYSTICKHZ)

//*****************************************************************************/
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************/
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

//*****************************************************************************/
//
// Position and movement granularity for the status indicator shown while
// the IP address is being determined.
//
//*****************************************************************************/
#define STATUS_X     50
#define STATUS_Y     100
#define MAX_STATUS_X (320 - (2 * STATUS_X))
#define ANIM_STEP_SIZE   8

//*****************************************************************************/
//
// The application's graphics context.
//
//*****************************************************************************/
//tContext g_sContext;

//*****************************************************************************/
//
// External Application references.
//
//*****************************************************************************/
//extern void fs_init(void);
//extern void fs_tick(unsigned long ulTickMS);

//*****************************************************************************/
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************/
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

#ifndef ABS
#define ABS(a)   (((a)<0)?(-1):(a))
#endif


void PinoutSet__(void);



#ifndef ADD_EMERSION_STROB
///#define FPGA_SCALE_PPT_TO_US       80    ///   (1000ns / 12.5)
    #define FPGA_SCALE_PPT_TO_US       8    ///   ((1000ns / 12.5) / 10)
    #define FPGA_ASCALE_PPT_TO_US      8
#else
    #define FPGA_SCALE_PPT_TO_US       2 //2    ///   ((1000ns / 50) / 10)
    #define FPGA_ASCALE_PPT_TO_US      8
#endif





#define FPGA_IP_ADR_REG_r         0x0050   /// r    /// Аппаратный IP

#define FPGA_STROB1_ARU           (1<<4)
#define FPGA_STROB2_ARU           (1<<5)

#define FPGA_ISR_RG_STATE         0x0030   /// r/w   /// 0-IRQEnabled 1,2 - Synhro( off, Internall, External, DP  )
/**
        1111110000000000
        5432109876543210
        ||||||||||||||||
        0000000000000000
        ||        ||||||
        ||        |||||+-         - b0 : Enable Internal Generator IRQ
        ||        |||++--         - b1,b2 : | Select sinc (00-off; 01-Internal Generator;  10-External Generator; 11- DP |
        ||        ||+----         - b3 : IRQ
        ||        |+-----         - Enable Strob1 ARU
        ||        +------         - Enable Strob2 ARU
        ||
        |+---------------         - b14 sync out    0 -in 1-out
        +----------------         - b15 master slave
*/
#define FPGA_ISR_RG_SRC             0x0032   /// r/w  /// 0- Признак прерывания и очистка
#define FPGA_ISR_RG_TIME_lo         0x0034   /// r/w  /// lo
#define FPGA_ISR_RG_TIME_hi         0x0036   /// r/w  /// hi   Период зондирования  12.5 нc
#define FPGA_SINC_EXT_PERIOD        0x0038   /// r/w  /// Период зондирования от внешней синхронизации 100kHz
#define FPGA_SINC_DP_PERIOD         0x003a   /// r/w   /// Период зондирования по ДП
#define FPGA_ASCAN_SWAPBUF          0x003e   ///  w    /// Запрос на построение АСкана  (своп АСкан буфер))

#define FPGA_MODDULE_STATE          0x0002
/**
        0000000000000000
        ||||||||||||||||
        |||||||||||||||+-         - b0 : Enable Immersion Strobe
        ||||||||||||||+--         - b1 : EnableImersion ARU
        |||||||||||||+---         - b2 : Disable GZI OUTs
*/
///     DP1
#define FPGA_SINC_TIMER1_lo          0x0040
#define FPGA_SINC_TIMER1_hi          0x0042
///     DP2
#define FPGA_SINC_REG_0              0x0044
#define FPGA_SINC_REG_1              0x0046
#define FPGA_SINC_REG_2              0x0048


/// GZI
#define FPGA_GZI1_ZI_END        0x0004 	///	w 	 ///  Конец GZI1  и ZI
#define FPGA_GZI2_BEGIN         0x0006   /// w     ///  Начало GZI2
#define FPGA_GZI2_END           0x0008   ///  w     ///  Конец GZI2
/// A-Scan
#define FPGA_ASCAN_BEGIN        0x000A   ///  w     ///  Начало А-Скана
#define FPGA_ASCAN_SCALE        0x000C   ///  w     /// Масштаб для А-Скана
#define FPGA_ASCAN_BEGIN_KU	    0x005C   ///	w    /// Начало А-Скана Усиления


// STROB1_ARU

// STROB2_ARU

//rg_stat(4) = '0' -- включение АРУ Strob 1
//rg_stat(5) = '0' -- включение АРУ Strob 2


/// Strob 1
#define FPGA_STROB1_BEGIN       0x000E   ///  w    /// Строб-1 Начало
#define FPGA_STROB1_END         0x0010   ///  w    /// Строб-1 Конец
#define FPGA_STROB1_LEVEL	    0x0012	///  w	 /// Строб-1 Уровень  0..9b(Уровень) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-время 0)/(0-времяMax)
#define FPGA_STROB1_ARU_MAXKUDAC 0x0056	 /// w		/// Ñòðîá-1 ÀÐÓ Óðîâåíü  ØÀÃ   0..9b    ????
#define FPGA_STROB1_ARU_KU      0x003c   /// w		/// Ñòðîá-1 Óñèëåíèå (÷èòàòü óñèëåíèå ÀÐÓ)   0..9b
#define FPGA_STROB1_ARU_MIN     0x0026   /// w		/// Ñòðîá-1 Óðîâåíü  ÍÈÇ   0..9b
#define FPGA_STROB1_ARU_MAX     0x0024   /// w		/// Ñòðîá-1 Óðîâåíü  ÂÅÐÕ  0..9b
#define FPGA_STROB1_ARU_STEP    0x0028   /// w		/// Ñòðîá-1 Óðîâåíü  ØÀÃ   0..9b
#define FPGA_STROB1_ARU_BEGIN   0x0020   /// w		/// Ñòðîá-1 ÀÐÓ Íà÷àëî
#define FPGA_STROB1_ARU_END     0x0022   /// w		/// Ñòðîá-1 ÀÐÓ Êîíåö
/// Result strobe 1
#define FPGA_STROB_1_MEAS_lo       0x0400     ///    r    /// Строб-1   0..15b= Время   0..10 = Амплитуда
#define FPGA_STROB_1_MEAS_hi       0x0402     ///    r    /// Строб-1   0..15b= Время   0..10 = Амплитуда
#define FPGA_STROB_1_MCOUNT        0x004a     ///    r    /// Количество пиков
#define FPGA_STROB_1_ARURES        0x006a     ///    r    /// значение на ЦАП после АРУ


/// Strob 2
#define FPGA_STROB2_BEGIN	    0x0014 	/// 64????//	w	 /// Строб-2 Начало
#define FPGA_STROB2_END         0x0016 	///	w	 /// Строб-2 Конец
#define FPGA_STROB2_LEVEL	    0x0018 	///	w	 /// Строб-2 Уровень 0..9b(Уровень) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-время 0)/(0-времяMax)
#define FPGA_STROB2_ARU_MAXKUDAC 0x0058 /// w	/// Ñòðîá-2-ÀÐÓ Óðîâåíü  ØÀÃ   0..9b    ????
#define FPGA_STROB2_ARU_KU		0x002E  /// w   /// Ku Ñòðîá-2
#define FPGA_STROB2_ARU_MIN     0x0050  /// w   /// Ñòðîá-2-ÀÐÓ Óðîâåíü  ÍÈÇ   0..9b
#define FPGA_STROB2_ARU_MAX     0x0052  /// w   /// Ñòðîá-2-ÀÐÓ Óðîâåíü  ÂÅÐÕ  0..9b
#define FPGA_STROB2_ARU_STEP    0x0054  /// w   /// Ñòðîá-2-ÀÐÓ Óðîâåíü  ØÀÃ   0..9b
#define FPGA_STROB2_ARU_BEGIN   0x005E  ///	w   /// Ñòðîá-2 Íà÷àëî Óñèëåíèÿ
#define FPGA_STROB2_ARU_END     0x001E 	///	w   /// Ñòðîá-2 Óðîâåíü  0..9b(Óðîâåíü) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-âðåìÿ 0)/(0-âðåìÿMax)
/// Result strobe 2
#define FPGA_STROB_2_MEAS_lo       0x0600     ///    r    /// Строб-1   0..15b= Время   0..10 = Амплитуда
#define FPGA_STROB_2_MEAS_hi       0x0602     ///    r    /// Строб-1   0..15b= Время   0..10 = Амплитуда
#define FPGA_STROB_2_MCOUNT        0x004c     ///    r    /// Количество пиков
#define FPGA_STROB_2_ARURES        0x006c     ///    r    /// значение на ЦАП после АРУ



/// Strob 3
#define FPGA_STROB3_BEGIN       0x001A   ///  w    /// Строб-1 Начало
#define FPGA_STROB3_END         0x001C   ///  w    /// Строб-1 Конец
#define FPGA_STROB3_LEVEL	    0x0060	///  w	 /// Строб-1 Уровень  0..9b(Уровень) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-время 0)/(0-времяMax)
/// Result strobe 3
#define FPGA_STROB_3_MEAS_lo       0x0800     ///    r    /// Строб-1   0..15b= Время   0..10 = Амплитуда
#define FPGA_STROB_3_MEAS_hi       0x0802     ///    r    /// Строб-1   0..15b= Время   0..10 = Амплитуда
#define FPGA_STROB_3_MCOUNT        0x004e     ///    r    /// Количество пиков



//ВРЧ
//addr = x"002C" dac0 -- ЦАП до ВРЧ
//addr = x"005C" as_dac -- начало таблицы ВРЧ
//addr = x"0800" -- запись в ВРЧ таблицу
//addr = x"005А" v_scale  -- шаг таблицы
/// Ku
#define FPGA_AVERAGE                0x002A   ///  w     ///  Усреднение 0=1,1=2, 3=4, 7=8, 15=16
#define FPGA_KU_0                   0x002C   ///  w     /// Ku0
#define FPGA_KU                     0x003c   /// r/w    /// Усиление в пределах Аскана (читать усиление АРУ)   0..9b
#define FPGA_VRCH                  0x0800    ///  w       /// ВРЧ[1024] 0..9b  50нс
//#define FPGA_FILTER                0x1000    ///   w       /// коеффициенты фильтра
#define FPGA_VRCH_SCALE            0x005A    ///  w       /// ВРЧ[1024] 0..9b  50нс


/// Result
#define FPGA_ASCAN                 0x0c00     ///    r    /// A-Scan


#define FPGA_START_DELAY            0x0060     ///  w     Gzi Start Delay
#define FPGA_CUR_SUM_COUNTER        0x0062     ///  r     Cur Sum Scan counter
#define FPGA_PSCAN_DEF_COUNTER      0x0066     ///  r     Count deffect scans in pack scan
#define FPGA_PSCAN_DEF_COUNTERN     0x0068     ///  r     Count deffect scans in pack scan


#define FPGA_GAIN_DP_dp0     0x0070     ///  w
#define FPGA_GAIN_DP_dp1     0x0072     ///  w
#define FPGA_GAIN_DP_adr_ku  0x0074     ///  w
#define FPGA_GAIN_DP_cmd     0x0076     ///  w


/*
таблица 32х42

addr = x"0040" -- младшая часть ДП
addr = x"0042" -- старшая часть ДП
addr = x"0044" -- 6-ти разрядный адрес в таблице + значение ЦАП
addr = x"0046" -- команда по которой будет происходить запись в таблицу


Работа ЦАП
тестовый режим
dac_cv0 <= dac_aru1                  when waru0 = '1' and rg_stat(4) = '0' else -- значение АРУ1
                    dac_aru2                  when waru1 = '1' and rg_stat(5) = '0' else -- значение АРУ2
                    dac_v(10 downto 0)  when run_cv = '1' else                                -- значение ВРЧ
                    dac0(10 downto 0);                                                                      -- значение до ВРЧ

рабочий режим
dac_cv1 <= ext(D_ARU,11)        when waru0 = '1'  else -- значение из таблицы 32х48
                   dac_v(10 downto 0)  when run_cv = '1' else -- значение ВРЧ
                   dac_c(10 downto 0);                                      -- значение до ВРЧ

на ЦАП переключение тестового и рабочего режима.
DAC <= dac_cv0(9 downto 0) when dac_cv0(10) = '0' and rg_stat(3) = '0' else
              dac_cv1(9 downto 0) when dac_cv0(10) = '0' and rg_stat(3) = '1' else
              "0000000000";

Последняя версия snk_b_35_006
*/




#define FPGA_REG( REG )                    (*((volatile unsigned short*)(0xA0000000 + (REG))))
#define FPGA_WRITE( REG,  dat )             *((volatile unsigned short*)(0xA0000000 + (REG))) = (dat)
#define FPGA_READ( dat, REG )             (dat) = *((volatile unsigned short*)(0xA0000000 + (REG)))
#define FPGA_READ32( dat, REG )             (dat) = (*((volatile unsigned short*)( (0xA0000000+(REG)+2)) << 16) | *((volatile unsigned short*)( (0xA0000000+(REG)) ))






#ifdef IP_UPDATER
static volatile tBoolean g_bFirmwareUpdate = false;
#endif


volatile int fpga_BoardMode = (1<<15);   ///  ???? (1<<5);
volatile int chenal_start_state = 0;
volatile unsigned short fpga_REGS[3] = {1, 0, 0};
volatile unsigned short iSateScan;

int SetFPGA_BoardMode( int mode );
int SetFPGA_BoardSyncOut( int mode );

unsigned short FPGA_Read_SYNC_REG( int reg );
void FPGA_Write_SYNC_REG( int reg, unsigned short dat );
void FPGA_Write_SYNC_REGs( unsigned short a0, unsigned short a1, unsigned short a2 );
void FPGA_Read_SYNC_REGs( unsigned short *a0, unsigned short *a1, unsigned short *a2 );



volatile unsigned int FPGA_ISR_RG_STATE_reg = 0;
volatile unsigned int FPGA_STROBE_ARU_reg = 0;


volatile int im_strobe_polarity = 1;

volatile int peaksCount = 0;
volatile int peaksCountI = 0;
volatile int peaksCountSND = 0;
volatile int MaxSpectrPeakAmp = 1;

volatile int TIm_Strobe = 0;

volatile int IRQ_Time = 0;


volatile int PeakDP0_Mode = 0;
volatile int PeakDP1_Mode = 0;
volatile int BoardSyncMode = 0;

volatile int irq_avrage = 1;

volatile int Cur_CountSumScan = 0;
volatile int Set_CountSumScan = 10;


volatile int Cur_SetsID = 0;
volatile int Set_SetsID = 0;
volatile int Cnt_SetsID = 0;


extern char DateBuf[];
tUDPPack_Scans    *ScanPacket = (tUDPPack_Scans*)DateBuf;



int FPGA_Test_Write( volatile unsigned short* REG,  unsigned short dat )
{
/*
#define FPGA_READ_LAST_WRITE_DATE       (*(volatile unsigned short*)(0xA0000000+0x180C))     ///   FACK
#define FPGA_READ_LAST_WRITE_ADR        (*(volatile unsigned short*)(0xA0000000+0x180E))     ///   FACK
int ret;
unsigned int read_d;
unsigned int read_a;
unsigned int write_a;

    *REG = dat;

    ret = 0;
    read_d = FPGA_READ_LAST_WRITE_DATE;
    read_a = FPGA_READ_LAST_WRITE_ADR;
    write_a = (unsigned int)((unsigned int)REG & 0xffff);

    if(  read_d != dat)
    {
        ret |= (1<<0);
        lou_printf( "Write Date ERROR [0x%04X] != [0x%04X]\r\n", dat, read_d );
    }

    if( read_a  !=  write_a )
    {
        ret |= (1<<1);
        lou_printf( "Write Addr ERROR [0x%04X] != [0x%04X]\r\n", write_a, read_a );
    }
    return ret;
*/
    return 0;
}

//void FPGA_REG_Write( volatile unsigned short *dst,  unsigned short dat )
void FPGA_REG_Write( volatile unsigned short* dst,  unsigned short dat )
{
    *((volatile unsigned short*)dst) = dat;
}



void StartInitIRQ( void );
void SetFPGA_Timer( unsigned int Time, int state  );
unsigned int FPGA_Read_DP( );
void FPGA_Write_DP( unsigned int wey );
unsigned  char FPGA_Get_SlotID( void );
void FPGA_Select_Generator(int Generator, unsigned int Period, int IRQMask );
void FPGA_Set_GZI_Shem_EHO( TFPGA_Tact_EHO  *takt );
void FPGA_Set_GZI_Shem(int tact, int state, int freq_Hz, int TactWidth_us, int BAscan_us, int B1_us, int W1_us, int L1_proc10, int St1, int B2_us, int W2_us, int L2_proc10, int St2, int in, int out, int Bi_us, int Wi_us, int Li_proc10, int  iKu );
void FPGA_Set_GZI_Shem_Def( void );
void FPGA_GetAScan( void  );
void FPGA_GetAScan_old( void  );



#define MAX_DAC_POINTS   32   ///64
extern int test_t[];
extern int test_a[];



//*****************************************************************************
//
// Display an lwIP type IP Address.
//
//*****************************************************************************
void DisplayIPAddress(unsigned long ipaddr, unsigned long ulCol,
                 unsigned long ulRow)
{
    unsigned char *pucTemp = (unsigned char *)&ipaddr;
    //
    // Convert the IP Address into a string.
    //
    lou_printf("%d.%d.%d.%d\n\r", pucTemp[0], pucTemp[1], pucTemp[2], pucTemp[3]);
}


#ifdef IP_UPDATER
#include "driverlib/ethernet.h"
void SoftwareUpdateRequestCallback(void)
{
    g_bFirmwareUpdate = true;
}
#endif




void Init_ETH_PHY( void )
{
//   SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);
//    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

//#define PHY_MR0_RESET           0x00008000  // Reset Registers
//#define PHY_MR0_LOOPBK          0x00004000  // Loopback Mode
//#define PHY_MR0_SPEEDSL         0x00002000  // Speed Select
//#define PHY_MR0_ANEGEN          0x00001000  // Auto-Negotiation Enable
//#define PHY_MR0_PWRDN           0x00000800  // Power Down
//#define PHY_MR0_ISO             0x00000400  // Isolate
//#define PHY_MR0_RANEG           0x00000200  // Restart Auto-Negotiation
//#define PHY_MR0_DUPLEX          0x00000100  // Set Duplex Mode
//#define PHY_MR0_COLT            0x00000080  // Collision Test

    EthernetPHYWrite(ETH_BASE, PHY_MR0, PHY_MR0_RESET);
    while( EthernetPHYRead(ETH_BASE, PHY_MR0) & PHY_MR0_RESET );


    EthernetPHYWrite(ETH_BASE, MAC_O_TCTL, (EthernetPHYRead(ETH_BASE, MAC_O_TCTL) | MAC_TCTL_PADEN) );
    EthernetPHYWrite(ETH_BASE, MAC_O_RCTL, (EthernetPHYRead(ETH_BASE, MAC_O_RCTL) | MAC_RCTL_PRMS) );

//    EthernetPHYWrite(ETH_BASE, PHY_MR0, PHY_MR0_DUPLEX | PHY_MR0_SPEEDSL);    // 100 MBit
    EthernetPHYWrite(ETH_BASE, PHY_MR0, PHY_MR0_DUPLEX );                       // 10 MBit






}



//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.
//
//*****************************************************************************
volatile int IPSetOk = 0;
volatile unsigned long ulLastIPAddress = 0;
void lwIPHostTimerHandler(void)
{

    static long lStarPos = 0;
    static tBoolean bIncrementing = true;
    unsigned long ulIPAddress;
//    tRectangle sRect;

    ulIPAddress = lwIPLocalIPAddrGet();

    //
    // If IP Address has not yet been assigned, update the display accordingly
    //
    if(ulIPAddress == 0)
    {
        //
        // Update status bar on the display.  First remove the previous
        // asterisk.
        //
//        GrStringDrawCentered(&g_sContext, "  ", 2, lStarPos + STATUS_X, STATUS_Y, true);

        //
        // Are we currently moving the asterisk right or left?
        //
        if(bIncrementing)
        {
            //
            // Moving right.
            //
            lStarPos += ANIM_STEP_SIZE;
            if(lStarPos >= MAX_STATUS_X)
            {
                //
                // We've reached the right boundary so reverse direction.
                //
                lStarPos = MAX_STATUS_X;
                bIncrementing = false;
            }
        }
        else
        {
            //
            // Moving left.
            //
            lStarPos -= ANIM_STEP_SIZE;
            if(lStarPos < 0)
            {
                //
                // We've reached the left boundary so reverse direction.
                //
                lStarPos = 0;
                bIncrementing = true;
            }
        }

        //
        // Draw the asterisk at the new position.
        //
//        GrStringDrawCentered(&g_sContext, "*", 2, lStarPos + STATUS_X, STATUS_Y, true);
    }

    //
    // Check if IP address has changed, and display if it has.
    //
    else if(ulLastIPAddress != ulIPAddress)
    {

        lou_printf("OLD ADDR : " );
        DisplayIPAddress(ulLastIPAddress, 170, STATUS_Y - 20);
        ulLastIPAddress = ulIPAddress;

        //
        // Clear the status area.
        //
     //   sRect.sXMin = STATUS_X - 10;
    //    sRect.sYMin = STATUS_Y - 30;
    //    sRect.sXMax = MAX_STATUS_X + 10;
    //   sRect.sYMax = STATUS_Y + 10;
    //    GrContextForegroundSet(&g_sContext, ClrBlack);
    //    GrRectFill(&g_sContext, &sRect);

    //    GrContextForegroundSet(&g_sContext, ClrWhite);
    //    GrContextFontSet(&g_sContext, &g_sFontCmss18b);
    //    GrStringDraw(&g_sContext, "IP Address:", -1, 60, STATUS_Y - 20, false);
    //    GrStringDraw(&g_sContext, "Subnet Mask:", -1, 60, STATUS_Y, false);
    //    GrStringDraw(&g_sContext, "Gateway:", -1, 60, STATUS_Y + 20, false);
        lou_printf("NEW ADDR : " );
        DisplayIPAddress(ulIPAddress, 170, STATUS_Y - 20);
        ulIPAddress = lwIPLocalNetMaskGet();
        DisplayIPAddress(ulIPAddress, 170, STATUS_Y);
        ulIPAddress = lwIPLocalGWAddrGet();
        DisplayIPAddress(ulIPAddress, 170, STATUS_Y + 20);
        IPSetOk = 1;
    }
}

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
int enable_timer = 1;
volatile int MyTimerCounter = 0;
void SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
  //
#if NO_SYS
//  if(enable_timer)
    lwIPTimer(SYSTICKMS);
//    lwIPTimer(TIMER_PERIOD);

#endif
    //
    // Run the file system tick handler.
    //
//    fs_tick(SYSTICKMS);
    MyTimerCounter++;
}

//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller.
//
//*****************************************************************************





#define SWAP_BITS( _b_ )     ((~(((((_b_)>> 3)&1)<<0) | ((((_b_)>> 2)&1)<<1) | ((((_b_)>> 1)&1)<<2) | ((((_b_)>> 0)&1)<<3))) & 0x0f)

//#define MAXSENDBUF     (1024*8)
//#define MAXSENDBUF     ( 64*4 )

#define  SIZE_SEND_PACKET           512
#define  COUNT_PACKET_IN_BUFFER     62
#define MAXSENDBUF     ( SIZE_SEND_PACKET * COUNT_PACKET_IN_BUFFER )

int  SendBufOfset  =0;

int MyTimer = 0;
int MyTimerPrintf = 0;
char SendBuf[MAXSENDBUF];

volatile int  *pSendBuf = (int*)SendBuf;
volatile int  *pRSendBuf = (int*)SendBuf;


char PageBuf[SIZE_SEND_PACKET];
char RawBuffer[SIZE_RAW_BUFFER];



int counterLeds[2] = {0.0};
void SetState_Leds( int led, int st )
{
    switch(led){
        case 0:
            if((st&1) == 0)
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2 );
            else
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0 );


        break;
        case 1:
            if((st&1) == 0)
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3 );
            else
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0 );
        break;
    }
}

void PutDump( unsigned char *Buf, int size )
{
int i;
    for(i=0; i< size; i+=4 ){
        if( i % 16 == 0 )
            lou_printf("\n%08X", *((unsigned int*)&Buf[i]));
        else
            lou_printf(" %08X", *((unsigned int*)&Buf[i]));

    }
    lou_printf("\n");
}

void PutDumpBit( unsigned char *Buf, int size )
{
#define BIT( V, i)  (((V)>>i)&1)
int i, j;
    for(i=0; i< size; i+=4 ){
        if( i % 16 == 0 )
            lou_printf("\n");
        for(j=0;j<32; j++)
            lou_printf("%x", BIT( *((uint32_t*)(&Buf[i])), j) );
        lou_printf(" ");
    }
    lou_printf("\n");
}



/*
#define MEAS_STROBE_1  (1<<28)
#define MEAS_STROBE_2  (2<<28)
#define MEAS_STROBE_0  (0<<28)

#define MEAS_TIME_AMP_VAL (0<<26)
#define MEAS_ARU_VALUE (1<<26)

#define PACK_MEAS(_strobe_, _time_, _amp_ )     ((3<<30) | (_strobe_) | (MEAS_TIME_AMP_VAL) | (((_amp_)&0xff) << 18) | (_time_)& 0x3ffff)
#define PACK_ARUVAL(_strobe_, _amp_ )     ((3<<30) | (_strobe_) | (MEAS_ARU_VALUE) | ((_amp_)&0xffff))

*/
void PutDumpBScan( unsigned char *Buf, int size )
{
uint32_t bdat;
int i;
    lou_printf("BUFF\n"  );
    for(i=0; i< size; i+=4 ){
        bdat = *((uint32_t*)(&Buf[i]));
        switch( ((bdat >> 30) & 3) ){
            case 0:
               // lou_printf("    NULL \n"  );
               return;
            break;
            case 1:
                lou_printf("    DP1 %x (%x)\n", ((bdat >> 0) & 0x3FFFFFFF), FPGA_Read_DP( )  );
            break;
            case 2:
               lou_printf("    DP2 %x \n", ((bdat >> 0) & 0x3FFFFFFF)  );
            break;
            case 3:
            break;
                if ( ((bdat >> 26) & 3) == 0){
                    lou_printf("[%08X] S:%x A:%x T:%x\n",bdat, ((bdat >> 28) & 3), ((bdat >> 18) & 0xff), ((bdat >> 0) & 0x3ffff)   );
                } else {
                    lou_printf("[%08X] S:%x :%x\n", bdat, ((bdat >> 28) & 3), ((bdat >> 0) & 0xffff)  );
                }
            break;
        }

    }

}


volatile int SlotID = 0;

extern void SendAScan( void );
extern void SendRAW(void );
extern void SendSetsAnswer( void );


unsigned long ulUser0, ulUser1;

int main(void)
{
#ifdef IP_UPDATER
    unsigned long ulIPAddr=0;
#endif
//  int i;
    unsigned char pucMACArray[8];

    int PrintfCounter;

    int SendTime;

    unsigned int wByte; //, realByte;
    //int waByte;
    volatile char  *pSBuf;
    volatile char  *pRSBuf;


   // volatile unsigned short *FPGA_REG0;
   // volatile unsigned short *FPGA_REG1;
   // volatile unsigned short *FPGA_REG2;
    //volatile unsigned short *FPGA_REG3;
//    volatile unsigned short *FPGA_REG4;
//    volatile unsigned short *FPGA_REG5;
//    tRectangle sRect;


    struct ip_addr IP_ADDRES, IP_MASK, IP_GATE;
    int ai;
//    unsigned short ReadDate;


    fpga_REGS[0] = 1;
    fpga_REGS[1] = 0;
    fpga_REGS[2] = 0;


   // FPGA_REG0 = ((unsigned short*)(0xA0000000 ));
   // FPGA_REG1 = ((unsigned short*)(0xA0000c00 ));

   // FPGA_REG2 = ((unsigned short*)(0xA0000400 ));
   // FPGA_REG3 = ((unsigned short*)(0xA0000800 ));





    //
    // Set the system clock to run at 50MHz from the PLL.
    //
    SysCtlClockSet(    SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |      SYSCTL_XTAL_16MHZ);      ///  ????????????????
//      SysCtlClockSet(    SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |      SYSCTL_XTAL_16MHZ);

//    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |  SYSCTL_XTAL_16MHZ);



    //
    // Set the pinout appropriately for this board.
    //
//    PinoutSet();
    PinoutSet__();

   //
    // Initialize the UART for debug output.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioInit(0);

      lou_printf( "\n\n\nEnet-lwip\n" );

    //
    // Enable and Reset the Ethernet Controller.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);

    lou_printf( "Init  Peripheral :SYSCTL_PERIPH_ETH \n\r");

    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);
    lou_printf( "Init  Peripheral :SYSCTL_PERIPH_ETH \n\r");

    //
    // Enable Port F for Ethernet LEDs.
    //  LED0        Bit 3   Output
    //  LED1        Bit 2   Output
//    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    GPIOPinTypeGPIOOutput( GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3 );
//    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
//    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2 );
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3 );






//    Enable_NO_SYS_LOOP = 0;
    //
    // Configure SysTick for a periodic interrupt.
    //

//    SysTickPeriodSet(SysCtlClockGet() / SYSTICKHZ);

    SysTickPeriodSet(SysCtlClockGet() / TIMER_PERIOD);
    SysTickEnable();
    SysTickIntEnable();

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();


/**      IP Addres   ip addres    */


    SendTime = MyTimerCounter;
    while( (abs(MyTimerCounter-SendTime) < 10) );

    SlotID = 0;

    int HARD_IP_adr = 0;
//#if( PASE_ADDR_DEFOULT != 40)
#if( PASE_ADDR_DEFOULT == 10)
    do{
//            SlotID = FPGA_Get_SlotID( );
            HARD_IP_adr = FPGA_REG(FPGA_IP_ADR_REG_r);
            lou_printf( "Enet-lwip Read ID : %d  [%d]\n\r",  SlotID, HARD_IP_adr);

            counterLeds[0]++; SetState_Leds( 0, counterLeds[0]>>8 );

            if(HARD_IP_adr != 0 ) break;
    }while( SlotID == 0);
#endif
    SlotID += PASE_ADDR_DEFOULT;

//HARD_IP_adr = 78;

    if(HARD_IP_adr != 0 )
    {
        //SlotID = 30 + SWAP_BITS(HARD_IP_adr);
//        SlotID = HARD_IP_adr+30;
        SlotID = HARD_IP_adr;
    }

    //
    // Initialize the file system.
    //
//    fs_init();

    //
    // Configure the hardware MAC address for Ethernet Controller filtering of
    // incoming packets.
    //
    // For the LM3S6965 Evaluation Kit, the MAC address will be stored in the
    // non-volatile USER0 and USER1 registers.  These registers can be read
    // using the FlashUserGet function, as illustrated below.
    //
//    FlashUserGet(&ulUser0, &ulUser1);
    ulUser0 = 0x340900;
//    ulUser1 = 0x1c8c23;
    ulUser1 = 0x1c8c00 | SlotID;

    if((ulUser0 == 0xffffffff) || (ulUser1 == 0xffffffff))
    {
        //
        // We should never get here.  This is an error if the MAC address has
        // not been programmed into the device.  Exit the program.
        //
        lou_printf("MAC Address Not Programmed! \n");
        while(1);
    }



    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
//    pucMACArray[0] = ((ulUser0 >>  0) & 0xff);
//    pucMACArray[1] = ((ulUser0 >>  8) & 0xff);
//    pucMACArray[2] = ((ulUser0 >> 16) & 0xff);
//    pucMACArray[3] = ((ulUser1 >>  0) & 0xff);
//    pucMACArray[4] = ((ulUser1 >>  8) & 0xff);
//    pucMACArray[5] = ((ulUser1 >> 16) & 0xff);



    pucMACArray[0] = ulUser0 & 0xff;
    pucMACArray[1] = (ulUser0 >> 8) & 0xff;
    pucMACArray[2] = (ulUser0 >> 16) & 0xff;
    pucMACArray[3] = ulUser1 & 0xff;
    pucMACArray[4] = (ulUser1 >> 8) & 0xff;
    pucMACArray[5] = (ulUser1 >> 16) & 0xff;



    lou_printf("MAC: %02X-%02X-%02X-%02X-%02X-%02X \n",
              pucMACArray[0], pucMACArray[1], pucMACArray[2],
              pucMACArray[3], pucMACArray[4], pucMACArray[5]);


    //
    // Initialze the lwIP library, using DHCP.
    //
//    lwIPInit(pucMACArray, 0, 0, 0, IPADDR_USE_DHCP);

//    IP_ADDRES.addr = (192<<24) | (168<<16) | (0<<8 )  | 33;
//    IP_ADDRES.addr = (192<<24) | (168<<16) | (0<<8 )  | (30+SlotID);
    IP_ADDRES.addr = (192<<24) | (168<<16) | (0<<8 )  | (SlotID);
    IP_MASK.addr   = (255<<24) | (255<<16) | (255<<8) | 0;
    IP_GATE.addr   = 0; //(192<<24) | (168<<16) | (0<<8)   | 1;
    lwIPInit(pucMACArray, IP_ADDRES.addr, IP_MASK.addr, IP_GATE.addr, IPADDR_USE_STATIC);

//    lou_printf(pcBuffer, "IP: %d.%d.%d.%d", sIPAddr[0] & 0xff, sIPAddr[0] >> 8, sIPAddr[1] & 0xff, sIPAddr[1] >> 8);
//    lou_printf("%s\n", pcBuffer);



#ifdef IP_UPDATER
    //
    // Setup the device locator service.
    //
    lou_printf("Locator Init");
    LocatorInit();
    LocatorMACAddrSet(pucMACArray);
    LocatorAppTitleSet("OKO22 1CH (ntz)");
    lou_printf("......... ok \n\r");
#endif




    //
    // Indicate that DHCP has started.
    //
//    GrStringDrawCentered(&g_sContext, "Waiting for IP", -1, GrContextDpyWidthGet(&g_sContext) / 2,  STATUS_Y - 22, false);
    lou_printf("Waiting for IP \r");
    while( !IPSetOk );  lou_printf("Set IP   --- OK\n");

//    lou_printf("sizeoff(short)=%d\n", sizeof(short));
//    lou_printf("sizeoff(int)=%d\n", sizeof(int));
//    lou_printf("sizeoff(long int)=%d\n", sizeof(long int));
//    lou_printf("sizeoff(unsigned long int)=%d\n", sizeof(unsigned long int));
//    lou_printf("sizeoff(long)=%d\n", sizeof(long));



    /// --------------------------------------------------
//        DSP_InitDevs( 0 );
    /// --------------------------------------------------


    //
    // Initialize the sample httpd server.
    //
//    httpd_init();
//    echo_udp_init();
//    http_init();


#ifdef IP_UPDATER
    SoftwareUpdateInit(SoftwareUpdateRequestCallback);
#endif

///    tcp_echo_init();
///    tcp_SETS_init();

    SETS_udp_init();
    AScan_udp_init();
//    RAW_udp_init();


    //
    // Set the interrupt priorities.  We set the SysTick interrupt to a higher
    // priority than the Ethernet interrupt to ensure that the file system
    // tick is processed if SysTick occurs while the Ethernet handler is being
    // processed.  This is very likely since all the TCP/IP and HTTP work is
    // done in the context of the Ethernet interrupt.
    //
    IntPriorityGroupingSet(4);
    IntPrioritySet(INT_ETH, ETHERNET_INT_PRIORITY);
    IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);






    //
    // Loop forever.  All the work is done in interrupt handlers.
    //
    PrintfCounter = 0;
    SendTime = MyTimerCounter;
    MyTimer = MyTimerCounter;
    MyTimerPrintf = MyTimerCounter;





#ifdef ENABLE_START_DETECT_FPGA
//    do{
//        FPGA_WRITE( FPGA_SINC_DP_PERIOD, 0x1234);
//        FPGA_READ(ReadDate, FPGA_SINC_DP_PERIOD);
//       lou_printf( " Start FPGA [%04X] \r\n", ReadDate );
//    }while( ReadDate != 0x1234 );
#endif

    lou_printf( "Init Ext IRQ " );     StartInitIRQ( );     lou_printf( "-------- ok \r\n" );
    DEFAULT_INIT_FPGA_STRUCT();
    SEND_SETUPS_TO_FPGA();

    ai = 0;

    my_udp_client_init();

//while(1); ///// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//    Init_ETH_PHY();

    while(1)
    {
      counterLeds[1]++; SetState_Leds( 1, counterLeds[1]>>16 );
#ifdef IP_UPDATER
    if(g_bFirmwareUpdate){
        SetState_Leds( 0, 1 );
        SetState_Leds( 1, 1 );
        lou_printf("Updater \n");
        if(ulIPAddr == 0) {
            ulIPAddr = lwIPLocalIPAddrGet();
            if(ulIPAddr){
                lou_printf("Locator IP: %d.%d.%d.%d",
                         ulIPAddr & 0xff, (ulIPAddr >> 8) & 0xff,
                         (ulIPAddr >> 16) & 0xff,  ulIPAddr >> 24);
            }
        }
        SoftwareUpdateBegin();
    }
#endif

    //lou_printf( ".");
    if(pSendBuf == 0)
        {
        pSendBuf = (volatile int *)&SendBuf[0];
        pRSendBuf = (volatile int *)&SendBuf[0];
        }

      SendAScan(  );
      SendSetsAnswer(  );

/* -----------------------------------------------------------------------------------------------------*/
      if( (DateSendState > 0)  )
      {

        pSBuf = (volatile char*)pSendBuf;
        pRSBuf = (volatile char*)pRSendBuf;
        if( ((unsigned int)pSBuf) >= ((unsigned int)pRSendBuf) )
        {
            wByte = (((unsigned int)pSBuf) - ((unsigned int)pRSendBuf));
           // waByte = 1;

        }
        else
        {
            wByte = (((unsigned int)(&(SendBuf[MAXSENDBUF])))- ((unsigned int)pRSendBuf));    ///   ADD For
            //waByte = -1;
        }
        //realByte = wByte;


//#ifdef DEBAG_MESS_SEND_BSKAN
     //   lou_printf( "Write %d pos %d (%d) %x %x\n", wByte, waByte, DateSendState, ((unsigned int)pSBuf) , ((unsigned int)pRSendBuf));
//#endif
        if( ( wByte >= SIZE_SEND_PACKET ) || (abs(MyTimerCounter-SendTime) > 3) )
//        if( wByte >= SIZE_SEND_PACKET  )
        {
            if( wByte >= (SIZE_SEND_PACKET) )  wByte = (SIZE_SEND_PACKET);                 ///   ADD For
            memcpy( (void*)PageBuf, (void*)pRSBuf, wByte );
            if(wByte < SIZE_SEND_PACKET)
                memset( &PageBuf[wByte], 0, (SIZE_SEND_PACKET-wByte));

           // PutDump( PageBuf,  32 );
           // PutDumpBScan( PageBuf,  32 );
            if( my_udp_client_write( PageBuf,  SIZE_SEND_PACKET ) == ERR_OK)
              {
                  DateSendState = 0;
                    pRSendBuf = (volatile int*)(((unsigned  int )pRSendBuf) + wByte);
                    if( ((unsigned int)pRSendBuf) >= ((unsigned int)(&(SendBuf[MAXSENDBUF]))) )
                        pRSendBuf = (volatile int*)SendBuf;
              }
              else
              {
//                lou_printf( "\n BSEND ERROR \r\n");
              }


#ifdef DEBAG_MESS_SEND_BSKAN
            lou_printf( " - ");
#endif
//            lou_printf( "Send :%d/%d  Time: %d ms \r\n",wByte,  realByte, abs(MyTimerCounter-SendTime) );
//   lou_printf( " IRQ Peaks : %d (%d)   %d\r\n", peaksCount, peaksCountI, peaksCountSND);
//   lou_printf( " IM Strobe T : %d \r\n" ,  TIm_Strobe );
//           lou_printf( " HARD IP : %04X \r\n" ,  *(FPGA_IP_ADR_REG_r) );

            SendTime = MyTimerCounter;
        }
#ifdef DEBAG_MESS_SEND_BSKAN
        lou_printf( " \n\r");
#endif

      }
      else
      {
//          SendTime = MyTimerCounter;
      }
/* -----------------------------------------------------------------------------------------------------*/


      MyTimer = MyTimerCounter;

//          *((volatile unsigned short*)(0xA0000000)) = ;
      if( ABS(MyTimerCounter - MyTimerPrintf) > 500 )
      {

          ai++;
          if(ai>512) ai = 0;

          MyTimerPrintf = MyTimerCounter;
          PrintfCounter+=2;


        FPGA_Read_DP( );
        FPGA_Read_SYNC_REG(0);
        FPGA_Read_SYNC_REG(1);
        FPGA_Read_SYNC_REG(2);


      }
    }
}









/*--------------------------------------------------------------------------------------------------------*/
#include "driverlib/epi.h"

#define EPI_PORTA_PINS          0x00
#define EPI_PORTB_PINS          0x30
#define EPI_PORTC_PINS          0xf0
#define EPI_PORTD_PINS          0x0C
#define EPI_PORTE_PINS          0x0F
#define EPI_PORTF_PINS          0x30
#define EPI_PORTG_PINS          0x83
#define EPI_PORTH_PINS          0xFF    //0xBF
#define EPI_PORTJ_PINS          0x7f



void PinoutSet__(void)
{
    //
    // Enable all GPIO banks.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

#if(1)

    HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) =
                                           GPIO_PCTL_PA0_U0RX |
                                           GPIO_PCTL_PA1_U0TX |
                                           GPIO_PCTL_PA2_SSI0CLK |
                                           GPIO_PCTL_PA3_SSI0FSS |
                                           GPIO_PCTL_PA4_SSI0RX |
                                           GPIO_PCTL_PA5_SSI0TX |
                                           GPIO_PCTL_PA6_USB0EPEN |
                                           GPIO_PCTL_PA7_USB0PFLT;

    //
    // GPIO Port B pins
    //
    HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) =
                                           GPIO_PCTL_PB2_I2C0SCL |
                                           GPIO_PCTL_PB3_I2C0SDA |
                                           GPIO_PCTL_PB4_EPI0S23 |
                                           GPIO_PCTL_PB5_EPI0S22 |
                                           GPIO_PCTL_PB6_I2S0TXSCK |
                                           GPIO_PCTL_PB7_NMI;

    //
    // GPIO Port C pins
    //
    HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) = GPIO_PCTL_PC0_TCK |
                                           GPIO_PCTL_PC1_TMS |
                                           GPIO_PCTL_PC2_TDI |
                                           GPIO_PCTL_PC3_TDO |
                                           GPIO_PCTL_PC4_EPI0S2 |
                                           GPIO_PCTL_PC5_EPI0S3 |
                                           GPIO_PCTL_PC6_EPI0S4 |
                                           GPIO_PCTL_PC7_EPI0S5;

    //
    // GPIO Port D pins.
    //
    HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) = GPIO_PCTL_PD0_I2S0RXSCK |
                                           GPIO_PCTL_PD1_I2S0RXWS |
                                           GPIO_PCTL_PD2_EPI0S20 |
                                           GPIO_PCTL_PD3_EPI0S21 |
                                           GPIO_PCTL_PD4_I2S0RXSD |
                                           GPIO_PCTL_PD5_I2S0RXMCLK;

    //
    // GPIO Port E pins
    //
    HWREG(GPIO_PORTE_BASE + GPIO_O_PCTL) = GPIO_PCTL_PE0_EPI0S8 |
                                           GPIO_PCTL_PE1_EPI0S9 |
                                           GPIO_PCTL_PE2_EPI0S24 |
                                           GPIO_PCTL_PE3_EPI0S25 |
                                           GPIO_PCTL_PE4_I2S0TXWS |
                                           GPIO_PCTL_PE5_I2S0TXSD;

    //
    // GPIO Port F pins
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) = GPIO_PCTL_PF1_I2S0TXMCLK |
                                           GPIO_PCTL_PF2_LED1 |
                                           GPIO_PCTL_PF3_LED0 |
                                           GPIO_PCTL_PF4_EPI0S12 |
                                         GPIO_PCTL_PF5_EPI0S15;

    //
    // GPIO Port G pins
    //
    HWREG(GPIO_PORTG_BASE + GPIO_O_PCTL) = GPIO_PCTL_PG0_EPI0S13 |
                                           GPIO_PCTL_PG1_EPI0S14 |
                                           GPIO_PCTL_PG7_EPI0S31;

    //
    // GPIO Port H pins
    //
    HWREG(GPIO_PORTH_BASE + GPIO_O_PCTL) = GPIO_PCTL_PH0_EPI0S6 |
                                           GPIO_PCTL_PH1_EPI0S7 |
                                           GPIO_PCTL_PH2_EPI0S1 |
                                           GPIO_PCTL_PH3_EPI0S0 |
                                           GPIO_PCTL_PH4_EPI0S10 |
                                           GPIO_PCTL_PH5_EPI0S11 |
                                           GPIO_PCTL_PH6_EPI0S26 |
                                           GPIO_PCTL_PH7_EPI0S27;

    //
    // GPIO Port J pins
    //
    HWREG(GPIO_PORTJ_BASE + GPIO_O_PCTL) = GPIO_PCTL_PJ0_EPI0S16 |
                                           GPIO_PCTL_PJ1_EPI0S17 |
                                           GPIO_PCTL_PJ2_EPI0S18 |
                                           GPIO_PCTL_PJ3_EPI0S19 |
                                           GPIO_PCTL_PJ4_EPI0S28 |
                                           GPIO_PCTL_PJ5_EPI0S29 |
                                           GPIO_PCTL_PJ6_EPI0S30 ;



#endif


//  GPIOPinTypeGPIOOutput
//  GPIO_DIR_MODE_IN
    //
    // Enable the EPI peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);


#if(1)
    //
    // Configure the EPI pins that are to be used on this board.
    //
#if (EPI_PORTA_PINS)
    GPIOPadConfigSet(GPIO_PORTA_BASE, EPI_PORTA_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTA_BASE, EPI_PORTA_PINS, GPIO_DIR_MODE_HW);
#endif
#if (EPI_PORTB_PINS)
    GPIOPadConfigSet(GPIO_PORTB_BASE, EPI_PORTB_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTB_BASE, EPI_PORTB_PINS, GPIO_DIR_MODE_HW);
#endif
#if (EPI_PORTD_PINS)
    GPIOPadConfigSet(GPIO_PORTD_BASE, EPI_PORTD_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTD_BASE, EPI_PORTD_PINS, GPIO_DIR_MODE_HW);
#endif
#if (EPI_PORTC_PINS)
    GPIOPadConfigSet(GPIO_PORTC_BASE, EPI_PORTC_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTC_BASE, EPI_PORTC_PINS, GPIO_DIR_MODE_HW);
#endif
#if EPI_PORTE_PINS
    GPIOPadConfigSet(GPIO_PORTE_BASE, EPI_PORTE_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTE_BASE, EPI_PORTE_PINS, GPIO_DIR_MODE_HW);
#endif
#if EPI_PORTF_PINS
    GPIOPadConfigSet(GPIO_PORTF_BASE, EPI_PORTF_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTF_BASE, EPI_PORTF_PINS, GPIO_DIR_MODE_HW);
#endif
#if EPI_PORTG_PINS
    GPIOPadConfigSet(GPIO_PORTG_BASE, EPI_PORTG_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTG_BASE, EPI_PORTG_PINS, GPIO_DIR_MODE_HW);
#endif
#if EPI_PORTJ_PINS
    GPIOPadConfigSet(GPIO_PORTJ_BASE, EPI_PORTJ_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTJ_BASE, EPI_PORTJ_PINS, GPIO_DIR_MODE_HW);
#endif
#if EPI_PORTH_PINS
    GPIOPadConfigSet(GPIO_PORTH_BASE, EPI_PORTH_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTH_BASE, EPI_PORTH_PINS, GPIO_DIR_MODE_HW);
#endif
#endif



#if(1)
    //
    // Set the EPI operating mode for the FPGA/Camera/LCD daughter board.
    // The values used here set the EPI to run at the system clock rate and
    // will allow correct accesses to the FPGA as long as the system clock is
    // 50MHz.
    //
  EPIModeSet(EPI0_BASE, EPI_MODE_GENERAL);
//    EPIModeSet(EPI0_BASE, EPI_MODE_SDRAM);
    EPIDividerSet(EPI0_BASE, 1);        ///  Sys CLK / ( (N/2)+1 )     CLK = 80   N = 1   ~= 40MHz
//    EPIDividerSet(EPI0_BASE, 2);        ///  Sys CLK / ( (N/2)+1 )     CLK = 80   N = 2   ~= 20MHz


/*
    EPIConfigGPModeSet(EPI0_BASE, (EPI_GPMODE_DSIZE_16 | EPI_GPMODE_ASIZE_12 |
                         EPI_GPMODE_WORD_ACCESS |
                         EPI_GPMODE_READWRITE |
//                       EPI_GPMODE_READ2CYCLE |
//                       EPI_GPMODE_WRITE2CYCLE |
                       EPI_GPMODE_CLKPIN |
//                       EPI_GPMODE_RDYEN |
                       EPI_GPMODE_FRAMEPIN
//                         | EPI_GPMODE_FRAME50
                         ), 0, 0);
*/

    EPIConfigGPModeSet(EPI0_BASE, (EPI_GPMODE_DSIZE_16 | EPI_GPMODE_ASIZE_12 |
                         EPI_GPMODE_WORD_ACCESS |
                         EPI_GPMODE_READWRITE |
                         EPI_GPMODE_READ2CYCLE |
//                         EPI_GPMODE_WRITE2CYCLE |
                         EPI_GPMODE_CLKPIN |
//                         EPI_GPMODE_CLKGATE |
//                       EPI_GPMODE_RDYEN |
//                         EPI_GPMODE_FRAME50 |
                         EPI_GPMODE_FRAMEPIN
                         ), 0, 0);




//    EPIAddressMapSet(EPI0_BASE, EPI_ADDR_PER_SIZE_256B | EPI_ADDR_PER_BASE_A);
    EPIAddressMapSet(EPI0_BASE, EPI_ADDR_PER_SIZE_64KB | EPI_ADDR_PER_BASE_A);
//    HWREG(EPI0_BASE + EPI_O_BAUD) = (1<<16) | (1<<0);
//    HWREG(EPI0_BASE + EPI_O_CFG) = EPI_MODE_GENERAL;
#endif



}



/*---------------------------------------------------------------------------------------------------------*/


//*****************************************************************************
//
// Display the interrupt state on the UART.  The currently active and pending
// interrupts are displayed.
//
//*****************************************************************************
void DisplayIntStatus(void)
{
    unsigned long ulTemp;
    //
    // Display the currently active interrupts.
    //
    ulTemp = HWREG(NVIC_ACTIVE0);
    lou_printf("\rActive: %c%c%c ", (ulTemp & 1) ? '1' : ' ', (ulTemp & 2) ? '2' : ' ', (ulTemp & 4) ? '3' : ' ');

    //
    // Display the currently pending interrupts.
    //
    ulTemp = HWREG(NVIC_PEND0);
    lou_printf("Pending: %c%c%c", (ulTemp & 1) ? '1' : ' ', (ulTemp & 2) ? '2' : ' ', (ulTemp & 4) ? '3' : ' ');
}





#define   FPGA_IRQ_bit     (1<<7)
#define   FPGA_IRQ_PORT    GPIO_PORTE_BASE
#define   FPGA_IRQ_INT     INT_GPIOE

void IntFPGA(void);

void StartInitIRQ( void )
{


    SetFPGA_Timer( 0xfffff, 0  );


    GPIOPadConfigSet(FPGA_IRQ_PORT, FPGA_IRQ_bit, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(FPGA_IRQ_PORT, FPGA_IRQ_bit, GPIO_DIR_MODE_IN);


    IntMasterEnable();
    //
    // Enable the interrupts.
    //
    IntEnable(FPGA_IRQ_INT);

    //
    // Set the interrupt priorities so they are all equal.
    //
    IntPrioritySet(FPGA_IRQ_INT, 0x00);


    //
    // Trigger the interrupt for GPIO E.
    //

    HWREG(FPGA_IRQ_PORT + GPIO_O_IS) = FPGA_IRQ_bit;   ///               0x00000404  // GPIO Interrupt Sense
    HWREG(FPGA_IRQ_PORT + GPIO_O_IEV) = FPGA_IRQ_bit;   ///              0x0000040C  // GPIO Interrupt Event
    HWREG(FPGA_IRQ_PORT + GPIO_O_IM) =  FPGA_IRQ_bit;   ///               0x00000410  // GPIO Interrupt Mask

    SetFPGA_Timer( 1000000, 0 );
}


//*****************************************************************************
//
// This is the handler for INT_GPIOA.  It simply saves the interrupt sequence
// number.
//
//*****************************************************************************



int SetFPGA_BoardMode( int mode )
{
    if(mode >= 0){
     fpga_BoardMode &= (1<<14);
     fpga_BoardMode |= (mode&1) << 15;
    }
    return (fpga_BoardMode >> 15) & 1;
}

int SetFPGA_BoardSyncOut( int mode )
{
    if(mode >= 0){
     fpga_BoardMode &= (1<<15);
     fpga_BoardMode |= (mode&1) << 14;
    }
    return (fpga_BoardMode >> 14) & 1;
}




void SetFPGA_Timer( unsigned int Time, int state  )
{




    FPGA_WRITE( FPGA_ISR_RG_TIME_lo, (Time & 0xffff) );
    FPGA_WRITE( FPGA_ISR_RG_TIME_hi, ((Time>>16) & 0xffff) );

    FPGA_WRITE( FPGA_ISR_RG_SRC,  0 );
//    FPGA_WRITE( FPGA_ISR_RG_STATE, state & 1 );
    FPGA_ISR_RG_STATE_reg =  (state & 9) | (fpga_BoardMode) | (1<<13) | FPGA_STROBE_ARU_reg;
    FPGA_WRITE( FPGA_ISR_RG_STATE, FPGA_ISR_RG_STATE_reg );

}

void setStatGer( unsigned int val ){    FPGA_WRITE( FPGA_ISR_RG_STATE, val );  }


/// hi   t:16
/// lo   tact:4    a:12

void write_sBUF(  unsigned int **buf, unsigned int val )
{
    **buf = val;
    (*buf)++;
    //lou_printf( "%p --- ", *buf);
    if( (void*)(*buf) >= (void*)(&SendBuf[MAXSENDBUF] ) )
        (*buf) = (void*)&(SendBuf[0]);

    //lou_printf( "%p \n", *buf);
}

#define WRITE_SENDBUF(  _BUF_, _DAT_ ) write_sBUF( ((unsigned int**)&(_BUF_)), (_DAT_) )

/*
#define WRITE_SENDBUF(  _BUF_, _DAT_ ) { \
    *(_BUF_) = (_DAT_); \
    (_BUF_)++; \
    if( ((unsigned int)(_BUF_) & 0xffff)  >= ((unsigned int)(&SendBuf[MAXSENDBUF]) & 0xffff) )\
        (_BUF_) = (volatile int*)&(SendBuf[0]); \
}while(0)
*/





//#define WRITE_SENDBUF(  _BUF_, _DAT_ )

extern TMUX_SETS *pSETS;
volatile unsigned int GZI_Counter = 0;
volatile unsigned int oldDP0 = 0;
volatile unsigned int oldDP1 = 0;
volatile unsigned int oldDP2 = 0;
volatile unsigned int Send_Counter = 0;

volatile unsigned int  Offset_GZI = 0;
volatile unsigned int  sOffset_GZI = 0;

volatile int AScanPeak_flag = 0;
volatile int AScanCount = 0;
volatile unsigned int *pAScanPeakBuf;

int strobState[3] = {0,0,0};
//#define REG_ns_to_ppt( ns )   (((ns) * 10) >> 7)    //   (* 10 / 125)   ~=  (*10 / 128);
#define REG_ns_to_ppt125( ns )   (((ns) * 10) / 125 / 4 )    //   (* 10 / 125)   ~=  (*10 / 128);
#define REG_ns_to_ppt( ns )   (((ns) * 10) / 500 )
//#define HZ_START_OFFSET         26700 //25700    //= (514*4*12.5)
#define HZ_START_OFFSET         (4560) //+800)
#define HZ_START_IMM_OFFSET     0 //(800)
#define HZ_START_OFFSET_MES     (0) //250
#define GPGA_KU_OFFSET           (32)

#define MEAS_STROBE_1  (2<<28)
#define MEAS_STROBE_2  (3<<28)
#define MEAS_STROBE_0  (1<<28)

#define MEAS_TIME_AMP_VAL (1<<26)
#define MEAS_ARU_VALUE (0<<26)


#define PACK_MEAS(_strobe_, _time_, _amp_ )     ((3<<30) | (_strobe_) | (MEAS_TIME_AMP_VAL) | (((_amp_)&0xff) << 18)    | ((_time_)& 0x3ffff))
#define PACK_ARUVAL(_strobe_, _amp_, _setid_ )  ((3<<30) | (_strobe_) | (MEAS_ARU_VALUE)    | (((_setid_)&0x3ff) << 16) | ((_amp_)&0xffff))

//#define MEAS_TIME_SCALE   (20)
#define MEAS_TIME_SCALE   (1)

int fpga_start_offset_ns = 0;

////   Процедура прерывания (Чтение пиков с плиски )
void IntFPGA(void)    ///  Get Date from FPGA
{

register unsigned int i; //, i0;
///int Date;
register unsigned short pCount;

register unsigned short pCount1;
register unsigned short pCount2;
register unsigned short pCountI;

register unsigned int pTime;
register unsigned short pAmp;
register int sAmp;
register char MUX = 2;
register unsigned int PeakDate;
register int         CurATact;



#if(1)
    MUX = (SlotID -1);
    if(HWREG(FPGA_IRQ_PORT + GPIO_O_RIS) & FPGA_IRQ_bit )
    {
        if( strobState[1] != 0 ) {
            FPGA_READ( pCount1, FPGA_STROB_1_MCOUNT);
            if( pCount1 > 16 )  pCount1 = 16;
        }else pCount1 = 0;

        if( strobState[2] != 0 ) {
            FPGA_READ( pCount2, FPGA_STROB_2_MCOUNT);
            if( pCount2 > 16 )  pCount2 = 16;
        }else pCount2 = 0;

        if( strobState[0] != 0 ) {
            FPGA_READ( pCountI, FPGA_STROB_3_MCOUNT);
            if( pCountI > 16 )  pCountI = 16;
        }else pCountI = 0;

        peaksCount = pCount1 + pCount2 + pCountI;


        pCount = peaksCount;


        peaksCountI = pCountI;


        GZI_Counter += irq_avrage;
        CurATact = AScanPeak_flag;
        if(pCount > 0)
        {
            MUX = MUX & 0x3;
            if(pSendBuf == 0)
            {
                pSendBuf = (volatile int*)&SendBuf[0];
                pRSendBuf = (volatile int*)&SendBuf[0];
                pAScanPeakBuf = (volatile unsigned int*)(&DateBuf[1024]);
            }

           // if( PeakDP0_Mode == 0 ){
                 i = FPGA_Read_DP( )   & 0x3FFFFFFF;
                 WRITE_SENDBUF( pSendBuf, ((1<<30) | i) );
            //}else {
                i = GZI_Counter & 0x3FFFFFFF;
            //}
            WRITE_SENDBUF( pSendBuf, ((2<<30) | i) );  oldDP1 = i;


            if( CurATact  ){
                AScanCount = 0;
                pAScanPeakBuf = (volatile unsigned int*)(&DateBuf[1024]);
            }


            register int mes_ofse = REG_ns_to_ppt(fpga_start_offset_ns-HZ_START_OFFSET_MES);

          //  lou_printf( "%x \n", ((unsigned int)pSendBuf));

         ///   lou_printf("%08X %08X %08X\n", pCount1, pCount2, pCountI );
            if( (strobState[1] != 0) && (pCount1 > 0) ) {
                pCount1 <<= 1;
                i = 0;
                FPGA_READ(pAmp,   (FPGA_STROB_1_ARURES));
                PeakDate = PACK_ARUVAL(MEAS_STROBE_1, pAmp, Cur_SetsID );
                WRITE_SENDBUF( pSendBuf, PeakDate );
                do{
                    FPGA_READ(pTime,  (FPGA_STROB_1_MEAS_lo + i*2));  i++;
                    FPGA_READ(pAmp,   (FPGA_STROB_1_MEAS_lo + i*2));  i++;
                    if(mes_ofse > pTime) continue;
                    pTime = pTime - mes_ofse;
                    sAmp =   pAmp >> 2; ////(signed short)( ((pAmp)&(1<<11)) ? (pAmp | 0xf000) : pAmp) ;
                    //sAmp >>= 1;
                    if(sAmp > 126) sAmp = 126;
                    if(sAmp < -126) sAmp = -126;
                    //sAmp = sAmp * (-1);
                    PeakDate = PACK_MEAS(MEAS_STROBE_1, (pTime*MEAS_TIME_SCALE), sAmp);
                    WRITE_SENDBUF( pSendBuf, PeakDate );
//                    lou_printf("1 %08X \n", pCount1);
//                    if(AScanCount < 63 ) pAScanPeakBuf[AScanCount++]  = PeakDate;
                } while( i < pCount1 );
            //    lou_printf("%08X < %08X < %08X\n", (unsigned int)&(SendBuf[0]), (unsigned int)(pSendBuf) , (unsigned int)&(SendBuf[MAXSENDBUF]) );
            }
            if( (strobState[2] != 0) && (pCount2 > 0) ) {
                pCount2 <<= 1;
                i = 0;
                FPGA_READ(pAmp,   (FPGA_STROB_2_ARURES));
                PeakDate = PACK_ARUVAL(MEAS_STROBE_2, pAmp, Cur_SetsID );
                WRITE_SENDBUF( pSendBuf, PeakDate );
                do{
                    FPGA_READ(pTime,  (FPGA_STROB_2_MEAS_lo + i*2));  i++;
                    FPGA_READ(pAmp,   (FPGA_STROB_2_MEAS_lo + i*2));  i++;
                    if(mes_ofse > pTime) continue;
                    pTime = pTime - mes_ofse;
                    sAmp =   pAmp >> 2; //(signed short)( ((pAmp)&(1<<11)) ? (pAmp | 0xf000) : pAmp) ;
                   // sAmp >>= 1;
                    if(sAmp > 126) sAmp = 126;
                    if(sAmp < -126) sAmp = -126;
                    //sAmp = sAmp * (-1);
                    PeakDate = PACK_MEAS(MEAS_STROBE_2, (pTime*MEAS_TIME_SCALE), sAmp);
                    WRITE_SENDBUF( pSendBuf, PeakDate );
//                    if( AScanCount < 63 ) pAScanPeakBuf[AScanCount++]  = PeakDate;
//                    lou_printf("2 %08X \n", pCount2);
                } while( i < pCount2 );
            }

           // mes_ofse = 0; //REG_ns_to_ppt(HZ_START_OFFSET-HZ_START_OFFSET_MES);
            if( (strobState[0] != 0) && (pCountI > 0) ) {
                //if(pCountI > 1) pCountI = 1;
                pCountI <<= 1;
                i = 0;
                do{

                    FPGA_READ(pTime,  (FPGA_STROB_3_MEAS_lo + i*2));  i++;
                    FPGA_READ(pAmp,   (FPGA_STROB_3_MEAS_lo + i*2));  i++;
//                    lou_printf("Time  %0x \n\r", pTime );

                    if(mes_ofse > pTime) continue;
                    pTime = pTime - mes_ofse;


                    sAmp =   pAmp >> 2; //im_strobe_polarity * 126;
                    if(sAmp > 126) sAmp = 126;
                    if(sAmp < -126) sAmp = -126;
                    //sAmp = sAmp * (-1);
                    PeakDate = PACK_MEAS(MEAS_STROBE_0, (pTime*MEAS_TIME_SCALE), sAmp);
//                    lou_printf("[%08X] \n\r", PeakDate );
                    WRITE_SENDBUF( pSendBuf, PeakDate );
//                    lou_printf("0 %08X \n", pCountI);
                } while( i < pCountI );

            }

            //WRITE_SENDBUF( pSendBuf, ((2<<30) | 0xabcd) );
            peaksCountSND = AScanCount;
            AScanPeak_flag = 0;
            if((AScanCount < 64 )) pAScanPeakBuf[AScanCount++]  = 0;
//            UARTwrite("C", 1);
        }
        else
        {
            //if( PeakDP0_Mode == 0 ){
               i = FPGA_Read_DP( )   & 0x3FFFFFFF;
               WRITE_SENDBUF( pSendBuf, ((1<<30) | i) );
            //}else{
                i = GZI_Counter & 0x3FFFFFFF;
            //}
            WRITE_SENDBUF( pSendBuf, ((2<<30) | i) ); oldDP1 = i;




            if( CurATact  )
               {
                AScanPeak_flag = 0;
                if((AScanCount < 64 )) pAScanPeakBuf[AScanCount++]  = 0;
               }
        }
        DateSendState++;
//        UARTwrite("i", 1);
//        lou_printf("%08X < %08X < %08X\n", (unsigned int)&(SendBuf[0]), (unsigned int)(pSendBuf) , (unsigned int)&(SendBuf[MAXSENDBUF]) );

    }
#else
 //   UARTwrite("a", 1);
#endif
    FPGA_WRITE( FPGA_ISR_RG_SRC, 0);
    HWREG(FPGA_IRQ_PORT + GPIO_O_ICR) = 0xff;
    IntPendClear(FPGA_IRQ_INT);

//    IRQ_Time = MyTimerCounter - IRQ_Time;
}



extern TMUX_DP *pBaseMUX_DP;

unsigned int FPGA_Read_DP(  )
{
    unsigned short res_lo;
    unsigned short res_hi;
    unsigned int res;


         FPGA_READ(res_lo,  FPGA_SINC_TIMER1_lo);
         FPGA_READ(res_hi,  FPGA_SINC_TIMER1_hi);

         res = (unsigned int)( res_lo | (res_hi<<16));
//         lou_printf("READ DP: %08X \n", res) ;

    return res;
}


void FPGA_Write_DP( unsigned int wey )
{
    FPGA_WRITE( FPGA_SINC_TIMER1_lo, (wey & 0xffff));
    FPGA_WRITE( FPGA_SINC_TIMER1_hi, ((wey>>16) & 0xffff));
}



void FPGA_Write_SYNC_REG( int reg, unsigned short dat )
{
    if(reg < 0) reg = 0;
    if(reg > 2) reg = 2;

    fpga_REGS[reg] = dat;

    if( BoardSyncMode != 0 )
    {
        switch(reg){
            case 0 : FPGA_WRITE( FPGA_SINC_REG_0, dat); break;
            case 1 : FPGA_WRITE( FPGA_SINC_REG_1, dat); break;
            case 2 : FPGA_WRITE( FPGA_SINC_REG_2, dat); break;
        }
    }



///    lou_printf("Write SYNC REG[%d] : %04X \n",reg, dat) ;
}

unsigned short FPGA_Read_SYNC_REG( int reg )
{
    unsigned short dat;

    if(reg < 0) reg = 0;
    if(reg > 2) reg = 2;


    if( BoardSyncMode == 0)
    {
        switch(reg){
            case 0 : FPGA_READ(dat, FPGA_SINC_REG_0); fpga_REGS[0] = dat; break;
            case 1 : FPGA_READ(dat, FPGA_SINC_REG_1); fpga_REGS[1] = dat; break;
            case 2 : FPGA_READ(dat, FPGA_SINC_REG_2); fpga_REGS[2] = dat; break;
        }
    //    lou_printf("Read SYNC REG[%d] : %04X \n", reg, dat) ;
    }


    return fpga_REGS[reg];
}



void FPGA_Write_SYNC_REGs( unsigned short a0, unsigned short a1, unsigned short a2 )
{
//    fpga_REGS[0] = a0;
//    fpga_REGS[1] = a1;
//    fpga_REGS[2] = a2;

//    FPGA_WRITE( FPGA_SINC_REG_0, fpga_REGS[0]);
//    FPGA_WRITE( FPGA_SINC_REG_1, fpga_REGS[1]);
//    FPGA_WRITE( FPGA_SINC_REG_2, fpga_REGS[2]);
}


void FPGA_Read_SYNC_REGs( unsigned short *a0, unsigned short *a1, unsigned short *a2 )
{
//    FPGA_READ( *a0, FPGA_SINC_REG_0  );
//    FPGA_READ( *a1, FPGA_SINC_REG_1  );
//    FPGA_READ( *a2, FPGA_SINC_REG_2  );
}





/**
    FPGA_Select_Generator
        Generator
        0 - Off
        1 - Internal generator
        2 - External generator
        3 - DP
        Period
            if( 1 or 2 )  Period int us
            if ( 3 )      Period int ppt
        IRQMask
            Enable Internal generator IRQ
*/
void FPGA_Select_Generator(int Generator, unsigned int Period, int IRQMask )
{

unsigned short reg;

    lou_printf("Generator %d  [%d] \r\n", Generator, Period) ;
    switch(Generator)
    {
        case 0:
//            lou_printf("Generator %d  [%d] \r\n", Generator, Period) ;
            lou_printf("Generator Off \r\n") ;
        break;

        case 1:
            lou_printf("[%d] -> ", Period) ;
            Period = (10000000 / Period / 500)*1000*4;
            lou_printf("Select Internal Generator [%d] %08X \r\n", Period, Period) ;
            FPGA_WRITE( FPGA_ISR_RG_TIME_lo, (Period & 0xffff));
            FPGA_WRITE( FPGA_ISR_RG_TIME_hi, ((Period>>16) & 0xffff) );
        break;
        case 2:
            Period = (1000000 / Period);
            lou_printf("Select External Generator [%d] \r\n", Period) ;
            FPGA_WRITE( FPGA_SINC_EXT_PERIOD, ((Period/10-1) & 0xffff));
        break;
        case 3:
            lou_printf("Select Way Generator [%d] \r\n", Period) ;
            FPGA_WRITE(FPGA_SINC_DP_PERIOD, (Period & 0xffff));
        break;




    }

//    reg = ((Generator & 3) << 1) | (IRQMask&1);


    if( BoardSyncMode != 0){
        fpga_BoardMode |=   (1<<15);
        fpga_BoardMode |=   (1<<14);
    }else{
        fpga_BoardMode &=  ~(1<<15);
        fpga_BoardMode &=  ~(1<<14);
    }


    reg = ((Generator & 3) << 1) | (IRQMask& 9) | fpga_BoardMode | (1<<13);
    FPGA_ISR_RG_STATE_reg = reg | FPGA_STROBE_ARU_reg;
    FPGA_WRITE( FPGA_ISR_RG_STATE, FPGA_ISR_RG_STATE_reg);

}




extern int FPGA_set_AScale( int aWidth_ns );


#define   STROB_LEVEL_FILTER   0x3FF
void FPGA_SetStrobe(TTactStrobe *Strobe, int fpgaStrobe  )
{
int aruMaxKu;
int aruMax;
int aruMin;
int aruStep;
int aruGain;

    aruMaxKu = Strobe->ARUMaxGain_10;
    aruMax = Strobe->ARUTopLevel_10 * 510 / 1000;
    aruMin = Strobe->ARUBottomLevel_10 * 510 / 1000;
    aruStep = (aruMax - aruMin) / 10;
    aruGain = 0;

//    if(Strobe->ARUState == 0 ){
//        aruMaxKu = 1024;
//        aruMax = 510;
//        aruMin = 0;
//        aruStep = 0;
//        aruGain = Strobe->curGain;
 //   }

    if(aruStep < 1) aruStep = 1;
    if(aruMin > 510) aruMin = 510;
    if(aruMax > 510) aruMax = 510;
    if(aruMin < 0)   aruMin = 0;
    if(aruMax < 0)   aruMax = 0;

    lou_printf("--- Strob %d (%d) [%d]------------\r\n", Strobe->Number, Strobe->State, fpga_start_offset_ns );
    lou_printf("     begin  : %d ns %d \r\n", Strobe->Begin_ns, REG_ns_to_ppt(Strobe->Begin_ns) );
    lou_printf("     width  : %d ns %d \r\n", Strobe->Width_ns, REG_ns_to_ppt(Strobe->Begin_ns + Strobe->Width_ns) );
    lou_printf("     Level  : %d %d %d \r\n", Strobe->Level_proc10, Strobe->Mode, Strobe->Type );
    lou_printf("     ARU[%d]: %d [%d %d] \r\n", Strobe->ARUState, Strobe->ARUMaxGain_10, Strobe->ARUBottomLevel_10, Strobe->ARUTopLevel_10 );
    lou_printf("     ARU    : %d [%d %d] \r\n", aruMaxKu, aruMin, aruMax );



    switch(fpgaStrobe) {
        case 0:
            strobState[0] = Strobe->State;
            FPGA_WRITE( FPGA_STROB3_BEGIN, REG_ns_to_ppt(Strobe->Begin_ns + fpga_start_offset_ns));///  w    /// Строб-0 Начало
            FPGA_WRITE( FPGA_STROB3_END,   REG_ns_to_ppt(Strobe->Begin_ns + Strobe->Width_ns + fpga_start_offset_ns)); ///  w    /// Строб-0 Конец
            //FPGA_WRITE( FPGA_STROB3_LEVEL, (((Strobe->Level_proc10 >> 2) & STROB_LEVEL_FILTER) | ((Strobe->Mode&3) << 12) | ((Strobe->Type&1) << 14)) ) ;   ///  w       /// Строб-0 Уровень  0..9b(Уровень) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-время 0)/(0-времяMax)
            FPGA_WRITE( FPGA_STROB3_LEVEL, (((Strobe->Level_proc10>>1) & STROB_LEVEL_FILTER) | ((Strobe->Mode&3) << 12) | ((Strobe->Type&1) << 14)) ) ;   ///  w       /// Строб-0 Уровень  0..9b(Уровень) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-время 0)/(0-времяMax)

        break;
        case 1:

            strobState[1] = Strobe->State;
            FPGA_WRITE( FPGA_STROB1_BEGIN, REG_ns_to_ppt(Strobe->Begin_ns + fpga_start_offset_ns));///  w    /// Строб-1 Начало
            FPGA_WRITE( FPGA_STROB1_END,   REG_ns_to_ppt(Strobe->Begin_ns + Strobe->Width_ns + fpga_start_offset_ns)); ///  w    /// Строб-1 Конец
            //FPGA_WRITE( FPGA_STROB1_LEVEL, (((Strobe->Level_proc10 >> 2) & STROB_LEVEL_FILTER) | ((Strobe->Mode&3) << 12) | ((Strobe->Type&1) << 14)) ) ;  ///  w       /// Строб-1 Уровень  0..9b(Уровень) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-время 0)/(0-времяMax)
            FPGA_WRITE( FPGA_STROB1_LEVEL, (((Strobe->Level_proc10 >>1) & STROB_LEVEL_FILTER) | ((Strobe->Mode&3) << 12) | ((Strobe->Type&1) << 14)) ) ;

            FPGA_WRITE( FPGA_STROB1_ARU_MAXKUDAC, aruMaxKu );
            FPGA_WRITE( FPGA_STROB1_ARU_KU,    aruGain );
            FPGA_WRITE( FPGA_STROB1_ARU_MIN,   aruMin );
            FPGA_WRITE( FPGA_STROB1_ARU_MAX,   aruMax );
            FPGA_WRITE( FPGA_STROB1_ARU_STEP,   aruStep );
            FPGA_WRITE( FPGA_STROB1_ARU_BEGIN,  REG_ns_to_ppt(Strobe->Begin_ns + fpga_start_offset_ns));
            FPGA_WRITE( FPGA_STROB1_ARU_END,    REG_ns_to_ppt(Strobe->Begin_ns + Strobe->Width_ns + fpga_start_offset_ns));


            if(Strobe->ARUState != 0 )
                FPGA_STROBE_ARU_reg &= ~FPGA_STROB1_ARU;
            else
                FPGA_STROBE_ARU_reg |= FPGA_STROB1_ARU;

        break;
        case 2:
            strobState[2] = Strobe->State;
            FPGA_WRITE( FPGA_STROB2_BEGIN, REG_ns_to_ppt(Strobe->Begin_ns + fpga_start_offset_ns));///  w    /// Строб-1 Начало
            FPGA_WRITE( FPGA_STROB2_END,   REG_ns_to_ppt(Strobe->Begin_ns + Strobe->Width_ns + fpga_start_offset_ns)); ///  w    /// Строб-1 Конец
            //FPGA_WRITE( FPGA_STROB2_LEVEL, (((Strobe->Level_proc10 >> 2) & STROB_LEVEL_FILTER) | ((Strobe->Mode&3) << 12) | ((Strobe->Type&1) << 14)) ) ;  ///  w       /// Строб-1 Уровень  0..9b(Уровень) 12b..13b=(00- +pp, 01- -pp, 10- 2pp ), 14b=(1-время 0)/(0-времяMax)
            FPGA_WRITE( FPGA_STROB2_LEVEL, (((Strobe->Level_proc10>>1) & STROB_LEVEL_FILTER) | ((Strobe->Mode&3) << 12) | ((Strobe->Type&1) << 14)) ) ;

            FPGA_WRITE( FPGA_STROB2_ARU_MAXKUDAC, aruMaxKu );
            FPGA_WRITE( FPGA_STROB2_ARU_KU,    aruGain );
            FPGA_WRITE( FPGA_STROB2_ARU_MIN,   aruMin );
            FPGA_WRITE( FPGA_STROB2_ARU_MAX,   aruMax );
            FPGA_WRITE( FPGA_STROB2_ARU_STEP,   aruStep );
            FPGA_WRITE( FPGA_STROB2_ARU_BEGIN,  REG_ns_to_ppt(Strobe->Begin_ns + fpga_start_offset_ns));
            FPGA_WRITE( FPGA_STROB2_ARU_END,    REG_ns_to_ppt(Strobe->Begin_ns + Strobe->Width_ns + fpga_start_offset_ns));

            if(Strobe->ARUState != 0 )
                FPGA_STROBE_ARU_reg &= ~FPGA_STROB2_ARU;
            else
                FPGA_STROBE_ARU_reg |= FPGA_STROB2_ARU;

        break;

    }
}

void FPGA_Write_GAinDP( int Ku )
{
int i;
    for(i=0; i<32; i++){
            if(Ku >= 0){
                FPGA_WRITE(FPGA_GAIN_DP_dp0, 0 );
                FPGA_WRITE(FPGA_GAIN_DP_dp1, 0 );
                FPGA_WRITE(FPGA_GAIN_DP_adr_ku,   ((Ku & 0x3ff) | (i<<11)) );
            }else{
                FPGA_WRITE(FPGA_GAIN_DP_dp0, GainDP[3*i+0] );
                FPGA_WRITE(FPGA_GAIN_DP_dp1, GainDP[3*i+1]);
                FPGA_WRITE(FPGA_GAIN_DP_adr_ku,   ((GainDP[3*i+2] & 0x3ff) | (i<<11)) );
            }
            FPGA_WRITE(FPGA_GAIN_DP_cmd,   1 );
    }

}

void FPGA_SetGain(TFPGA_Tact_EHO  *takt)
{
//unsigned short *vrc_buf = (unsigned short*)( FPGA_REG(FPGA_VRCH) );   // //FPGA_VRCH///  w       /// ВРЧ[1024] 0..9b  50нс
long int vrh_t, vrh_a, vrh_p, vrh_step;
int i;
    lou_printf("Ku : %d db * 10\r\n", takt->Ku10 );
    lou_printf("VRCH_Count :         %d \r\n", takt->VRCH_Count );

    for(i=0; i<takt->VRCH_Count; i++) lou_printf("%6d |", i ); lou_printf("\r\n");
    for(i=0; i<takt->VRCH_Count; i++) lou_printf("%6d |", takt->VRCH_Time_ns[i] );   lou_printf("\r\n");
    for(i=0; i<takt->VRCH_Count; i++) lou_printf("%6d |", takt->VRCH_Amp_db10[i] );   lou_printf("\r\n");

    FPGA_WRITE( FPGA_KU_0,  takt->Ku10);///  w     /// Ku0
    FPGA_WRITE( FPGA_KU,    takt->Ku10);/// r/w    /// Усиление в пределах Аскана (читать усиление АРУ)   0..9b



    FPGA_Write_GAinDP(takt->Ku10);


//    takt->VRCH_Count = 0;
    if( takt->VRCH_Count > 0 )
    {
        vrh_t = takt->ABegin_ns;// + fpga_start_offset_ns >> 2;
        vrh_p = 0;
        vrh_a = takt->VRCH_Amp_db10[vrh_p];

        vrh_step = 125 * 8 / 10 * 4*2;
        //FPGA_WRITE(FPGA_VRCH_SCALE, 0x80 );
        FPGA_WRITE(FPGA_VRCH_SCALE, 0x80/4/4/2 );

        for(i=0;i<1024; i++)
        {
            if( vrh_t > takt->VRCH_Time_ns[vrh_p]  )  vrh_p++;
            if(vrh_p > 0)
            {
                if(vrh_p < takt->VRCH_Count )
                    vrh_a =  ( (takt->VRCH_Amp_db10[vrh_p]) - (takt->VRCH_Amp_db10[vrh_p-1] ) ) * (  vrh_t - (takt->VRCH_Time_ns[vrh_p-1]) ) / ( (takt->VRCH_Time_ns[vrh_p]) - (takt->VRCH_Time_ns[vrh_p-1]) ) + takt->VRCH_Amp_db10[vrh_p-1];
                else
                    vrh_a = takt->VRCH_Amp_db10[takt->VRCH_Count-1];
            }
            else
                vrh_a = takt->VRCH_Amp_db10[vrh_p];

            FPGA_WRITE( (FPGA_VRCH + i*2), (vrh_a & STROB_LEVEL_FILTER));
            vrh_t += vrh_step;
        }
    }
    else
    {
        FPGA_WRITE(FPGA_VRCH_SCALE, 0x80);
        for(i=0;i<1024; i++)
            FPGA_WRITE( (FPGA_VRCH + i*2), 0);
    }


}

void FPGA_Set_GZI_Shem_EHO( TFPGA_Tact_EHO  *takt )
{
int GZ2_w;

    switch( takt->Freq_Hz )   ///   1 - 50 ns
    {
        case   400000: GZ2_w =  6;   break;  // 5
        case  1250000: GZ2_w =  5;   break;  // 5
        case  1800000: GZ2_w =  5;   break;  // 4
        case  2500000: GZ2_w =  3;   break;  // 4
        case  4000000: GZ2_w =  2;   break;  // 3
        case  5000000: GZ2_w =  2;   break;  // 3
        case  6000000: GZ2_w =  2;   break;  // 3
        case  7000000: GZ2_w =  1;   break;  // 3
        case 10000000: GZ2_w =  1;   break;  // 2
        case 15000000: GZ2_w =  1;   break;  // 1

        default :      GZ2_w =  3;   break;
    }


//    if( takt->IM_State != 0 ){
//       fpga_start_offset_ns = HZ_START_IMM_OFFSET; //HZ_START_OFFSET; // + 300; // + (GZ2_w*4 * 100 >> 3);
//    } else {
       fpga_start_offset_ns = HZ_START_OFFSET; // + 300; // + (GZ2_w*4 * 100 >> 3);
//    }
    fpga_start_offset_ns = fpga_start_offset_ns + 25;   // 25 = (12.5 * 2)

    if( BoardSyncMode != 0){
        fpga_BoardMode |=   (1<<15);
        fpga_BoardMode |=   (1<<14);
    }else{
        fpga_BoardMode &=  ~(1<<15);
        fpga_BoardMode &=  ~(1<<14);
    }

    FPGA_ISR_RG_STATE_reg = (0 | fpga_BoardMode | (1<<13) ) | FPGA_STROBE_ARU_reg;
//    FPGA_WRITE( FPGA_ISR_RG_STATE, FPGA_ISR_RG_STATE_reg );


    int GZI_E = 123*4;
    if( (takt->GZI_Energy <= 4) || (takt->GZI_Energy > 123*4 ) ) {
        GZI_E = 123*4;
    }else{
        GZI_E = takt->GZI_Energy;
    }

    fpga_start_offset_ns = (GZI_E+4+GZ2_w*4);
    fpga_start_offset_ns = 0;


/**  Setup the ZI Generator  */
    lou_printf("ZI_E: %d   \r\n", GZI_E);
    lou_printf("GZ2 : %d %d   (%d)  GZI_Energy:%d\r\n", GZI_E+4, GZI_E+4+GZ2_w*4, takt->Freq_Hz, takt->GZI_Energy );
    FPGA_WRITE( FPGA_GZI1_ZI_END,  GZI_E);            ///	w 	 ///  Конец GZI1  и ZI
    FPGA_WRITE( FPGA_GZI2_BEGIN,   (GZI_E+4));            /// w     ///  Начало GZI2
    FPGA_WRITE( FPGA_GZI2_END,     (GZI_E+4) + GZ2_w*4);    ///  w     ///  Конец GZI2




/**  Setup the A-Scan  */
    lou_printf("AScan begin : %d ns  %d\r\n", takt->ABegin_ns, REG_ns_to_ppt(takt->ABegin_ns) );
    lou_printf("AScan width : %d ns \r\n", takt->AWidth_ns );
    FPGA_WRITE( FPGA_ASCAN_BEGIN, REG_ns_to_ppt(takt->ABegin_ns + fpga_start_offset_ns));           ///  w     ///  Начало А-Скана
    int AScan_Ku_begin;
    AScan_Ku_begin = REG_ns_to_ppt(takt->ABegin_ns + fpga_start_offset_ns)- GPGA_KU_OFFSET;
    if(AScan_Ku_begin < 0) AScan_Ku_begin = 0;
    lou_printf("Ku AScan width : %d ns \r\n", AScan_Ku_begin );
    FPGA_WRITE( FPGA_ASCAN_BEGIN_KU, AScan_Ku_begin);           ///  w     ///  Начало А-Скана   KU


    takt->ACount = FPGA_set_AScale( takt->AWidth_ns );        //    *FPGA_ASCAN_SCALE  = Scale;                          ///  w     /// Масштаб для А-Скана
    lou_printf("AScan count : %d ns\r\n", takt->ACount );



    FPGA_SetStrobe( &takt->Strobe[0], 0  );
    FPGA_SetStrobe( &takt->Strobe[1], 1  );
    FPGA_SetStrobe( &takt->Strobe[2], 2  );




    lou_printf("Average :         %d \r\n", takt->Average );
    switch( takt->Average )
    {
        default :
        case 1  : FPGA_WRITE( FPGA_AVERAGE, 0);  irq_avrage = 1;  break;///  w     ///  Усреднение 0=1,1=2, 3=4, 7=8, 15=16
        case 2  : FPGA_WRITE( FPGA_AVERAGE, 1);  irq_avrage = 2;  break;///  w     ///  Усреднение 0=1,1=2, 3=4, 7=8, 15=16
        case 4  : FPGA_WRITE( FPGA_AVERAGE, 3);  irq_avrage = 4;  break;///  w     ///  Усреднение 0=1,1=2, 3=4, 7=8, 15=16
        case 8  : FPGA_WRITE( FPGA_AVERAGE, 7);  irq_avrage = 8;  break;///  w     ///  Усреднение 0=1,1=2, 3=4, 7=8, 15=16
        case 16 : FPGA_WRITE( FPGA_AVERAGE, 15); irq_avrage = 16;  break;///  w     ///  Усреднение 0=1,1=2, 3=4, 7=8, 15=16
    }


    lou_printf("StartDelay :         %d \r\n", takt->StartDelay );
    /*TODO   Лвович поменяет адрес */
   // FPGA_WRITE( FPGA_START_DELAY, takt->StartDelay);

//    FPGA_WRITE( FPGA_SUM_COUNTER, Set_CountSumScan);
    Set_CountSumScan = takt->PackCount;

    Cnt_SetsID = 2;
    Set_SetsID = takt->SetsID;


    FPGA_SetGain( takt );


    FPGA_Write_DP( 0 );
    GZI_Counter = 0;
    Send_Counter = 0;

if( BoardSyncMode != 0){
//    fpga_REGS[1] &= ~(1<<1);
//    if(chenal_start_state) fpga_REGS[1] |= (1<<1);


    FPGA_Write_SYNC_REG(0, 0 );
//    if(chenal_start_state  == 1)  FPGA_Write_SYNC_REG(1, (1<<1));
//    if(chenal_start_state  == 2 ) FPGA_Write_SYNC_REG(1, 0);

//    FPGA_Write_SYNC_REG(2, fpga_REGS[2]);
}


//FPGA_WRITE( FPGA_FILTER///   w       /// коеффициенты фильтра



}


void FPGA_Set_GZI_Shem(int tact, int state, int freq_Hz, int TactWidth_us, int BAscan_us, int B1_us, int W1_us, int L1_proc10, int St1, int B2_us, int W2_us, int L2_proc10, int St2, int in, int out, int Bi_us, int Wi_us, int Li_proc10, int  iKu )
{

}



///FPGA_Set_GZI_Shem( tact, state, freq_Hz, TactWidth_us, BAscan_us, B1_us, W1_us,  L1_proc10, B2_us, W2_us, L2_proc10, in, out  )
void FPGA_Set_GZI_Shem_Def( void )
{

    Tact_EHO.PeriodGZI_Type = 1;
    Tact_EHO.PeriodGZI_us = 1000000;
    Tact_EHO.PeriodGZI_mm = 10;

    Tact_EHO.Freq_Hz = 5000000;

    Tact_EHO.ABegin_ns = 0;
    Tact_EHO.AWidth_ns = 10000;
    Tact_EHO.ACount = 512;

    Tact_EHO.IM_State = 0;
    Tact_EHO.IM_Begin_ns = 0;
    Tact_EHO.IM_Width_ns = 0;
    Tact_EHO.IM_Level_proc10 = 512;
    Tact_EHO.IM_Type = 1; // (1-время 0)/(0-времяMax)
    Tact_EHO.IM_Mode = 1; // (00- +pp, 01- -pp, 10- 2pp )

    Tact_EHO.Strobe[0].Number = 0;
    Tact_EHO.Strobe[0].State = 0;
    Tact_EHO.Strobe[0].Begin_ns = 0;
    Tact_EHO.Strobe[0].Width_ns = 0;
    Tact_EHO.Strobe[0].Level_proc10 = 512;
    Tact_EHO.Strobe[0].Type = 1; // (1-время 0)/(0-времяMax)
    Tact_EHO.Strobe[0].Mode = 1; // (00- +pp, 01- -pp, 10- 2pp )
    Tact_EHO.Strobe[0].ARUState = 1;
    Tact_EHO.Strobe[0].ARUMaxGain_10 = 0;
    Tact_EHO.Strobe[0].ARUTopLevel_10 = 300;
    Tact_EHO.Strobe[0].ARUBottomLevel_10 = 200;

    Tact_EHO.Strobe[1].Number = 1;
    Tact_EHO.Strobe[1].State = 0;
    Tact_EHO.Strobe[1].Begin_ns = 0;
    Tact_EHO.Strobe[1].Width_ns = 0;
    Tact_EHO.Strobe[1].Level_proc10 = 512;
    Tact_EHO.Strobe[1].Type = 1; // (1-время 0)/(0-времяMax)
    Tact_EHO.Strobe[1].Mode = 1; // (00- +pp, 01- -pp, 10- 2pp )
    Tact_EHO.Strobe[1].ARUState = 1;
    Tact_EHO.Strobe[1].ARUMaxGain_10 = 0;
    Tact_EHO.Strobe[1].ARUTopLevel_10 = 300;
    Tact_EHO.Strobe[1].ARUBottomLevel_10 = 200;


    Tact_EHO.Strobe[2].Number = 2;
    Tact_EHO.Strobe[2].State = 0;
    Tact_EHO.Strobe[2].Begin_ns = 0;
    Tact_EHO.Strobe[2].Width_ns = 0;
    Tact_EHO.Strobe[2].Level_proc10 = 512;
    Tact_EHO.Strobe[2].Type = 1; // (1-время 0)/(0-времяMax)
    Tact_EHO.Strobe[2].Mode = 1; // (00- +pp, 01- -pp, 10- 2pp )
    Tact_EHO.Strobe[3].ARUState = 1;
    Tact_EHO.Strobe[3].ARUMaxGain_10 = 0;
    Tact_EHO.Strobe[3].ARUTopLevel_10 = 300;
    Tact_EHO.Strobe[3].ARUBottomLevel_10 = 200;


    Tact_EHO.VRCH_Time_ns[0] = 0;
    Tact_EHO.VRCH_Amp_db10[0] = 0;
    Tact_EHO.VRCH_Count =0;

    Tact_EHO.KuIM10 = 0;
    Tact_EHO.Ku10 = 600;

    Tact_EHO.GZI_Disable = 0;

    FPGA_Set_GZI_Shem_EHO( &Tact_EHO );
}

extern char DateBuf[];
#define FPGA_MAX_ASCAN_MINMAX    512
int DefAScale = 0;
int AScaleLength = 0;
int FPGA_set_AScale( int aWidth_ns )
{
int Scale;
int Length;
    Scale = 0;
    if(pSETS->MaxAscan < 16 ) pSETS->MaxAscan = 512;
    if(pSETS->MaxAscan > 512) pSETS->MaxAscan = 512;
    Scale = REG_ns_to_ppt(aWidth_ns) * 4 / pSETS->MaxAscan;
    Length = REG_ns_to_ppt(aWidth_ns) * 4 / (Scale+1);
    FPGA_WRITE( FPGA_ASCAN_SCALE , Scale );
    DefAScale = Scale;
    AScaleLength = Length;

    lou_printf("AScan scale %d : %d [MAX: %d  RCOUNT : %d]\r\n", DefAScale, AScaleLength, pSETS->MaxAscan, REG_ns_to_ppt(aWidth_ns));
    return (Length);
}

int test_scat_ofset = 0;


void FPGA_GetAScan_old( void  )
{
}



void FPGA_GetAScan( void  )
{
    int i;
    unsigned short Reg;
    unsigned short a,b;
    //unsigned short *pAScanBuf = (unsigned short *)DateBuf;

    FPGA_WRITE( FPGA_ASCAN_SWAPBUF,  1);

    Send_Counter++;
    FPGA_WRITE( FPGA_ASCAN_SCALE , DefAScale );
    ScanPacket->Header.cmd = *((unsigned int*)&"SCAN");
    ScanPacket->Header.Len = sizeof(tUDPPack_ScansDate);//512+8;

    ScanPacket->Date.State = FPGA_Read_SYNC_REG(1) | (iSateScan<<16); //fpga_REGS[0]&1;
    ScanPacket->Date.Counter = Send_Counter;  //FPGA_REG( FPGA_PSCAN_DEF_COUNTERN );

    ScanPacket->Date.ScanCount = Cur_CountSumScan + 1;      ///  FACK +1   ??????????????????
    ScanPacket->Date.RowOverCount = FPGA_REG( FPGA_PSCAN_DEF_COUNTERN );
    ScanPacket->Date.AllOverCount = FPGA_REG( FPGA_PSCAN_DEF_COUNTER );

    ScanPacket->Date.ScanLength = AScaleLength;
    ScanPacket->Date.SetsId = Cur_SetsID;
    ScanPacket->Date.eDP = FPGA_Read_DP();
    ScanPacket->Date.iDP = GZI_Counter;

    for(i=0; i<FPGA_MAX_ASCAN_MINMAX; i++) {
        FPGA_READ( Reg, (FPGA_ASCAN+i*2));
        a = (Reg&0xff);
        b = ((Reg & 0xff00)>>8);
        if(a > 255) a = 255;
        if(b > 255) b = 255;

        //ScanPacket->Date.Date[i] = a | (b << 8);
        ScanPacket->Date.Date[i] = a > b ? a : b;
    }

    test_scat_ofset++;

    AScanCount = 0;
    AScanPeak_flag = (pSETS->AscanTakt+1);


//    lou_printf( "Cur_CountSumScan :%d   IRQ : %d \n", ScanPacket->Date.ScanCount, Cur_CountSumScan);

}













