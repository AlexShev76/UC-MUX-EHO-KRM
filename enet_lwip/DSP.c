#include <stdlib.h>

#include "DSP.h"
#include "udp_srv.h"

TFPGA_Tact_EHO   Tact_EHO;

/// ---------------------------------------------------------------------------------------------------

TMUX_SETS     SETS;
TMUX_SETS     *pSETS = &SETS;
TMUX_PARAM MUX_PARAM;
TMUX_PARAM *pMUX_PARAM = &MUX_PARAM;
TMUX_DP MUX_DP;
TMUX_DP *pMUX_DP = &MUX_DP;
TMUX_DP BaseMUX_DP;
TMUX_DP *pBaseMUX_DP = &BaseMUX_DP;

unsigned short GainDP[32*3];

void InitSETS( void )
{
int TactNum, Strob, Point;
TMUX_TAKT *t;

    pBaseMUX_DP->DP1 = 0;
    pBaseMUX_DP->DP2 = 0;


    pSETS->Version = 4; //CUR_VERSION;

    pSETS->MainBoard_IP = (192<<24) | (168<<16) | (0<<8) | (121<<0);
    pSETS->SyncType     = 1;       ///  0, 1, 2, 3
    pSETS->Fsmp         = 30; //500 * 4; //1000*4;
    pSETS->AscanTakt    = 0;

    for(TactNum = 0; TactNum < MUX_MAX_TAKT_COUNT; TactNum++ )
    {
        t = &(pSETS->TAKT);
        t->State = 0;
        t->PEPFreq_Hz = 5000000;
        t->TaktLen_us = 1000;
        t->AscanDly_us = 50;
        t->AscanLen_us = 1200;

        t->StartDelay = 0;
        t->PackCount =  10;
        t->GZI_Energy = 123*4;


        for( Strob=0; Strob<MUX_MAX_STROB; Strob++ )
        {
            t->Strob[Strob].Dly_us   = 600;
            t->Strob[Strob].Len_us   = 500;
            t->Strob[Strob].Level_10 = 50*10;
        }
        t->Strob[0].Dly_us   = 200;

        t->InChan = 0;
        t->OutChan = 0;

        t->KuTab_count = 3;

        for(Point=0; Point<MUX_MAX_DAC_POINTS; Point++)
        {
            t->KuTab_t_us[Point] = 0;
            t->KuTab_a_10[Point] = 0;
        }

        t->KuTab_t_us[0] = 0;
        t->KuTab_a_10[0] = 1023;

        t->KuTab_t_us[1] = 20;
        t->KuTab_a_10[1] = 1023;

        t->KuTab_t_us[2] = 30;
        t->KuTab_a_10[2] = 1023;
    }
    pSETS->TAKT.State =  1;
    pMUX_PARAM->CLK_Freq_MHz = CLK_FREQ_MHZ_CONST;
    pMUX_PARAM->Ascan_PointsCount = 300;
    pMUX_DP->DP1 = 0;
    pMUX_DP->DP2 = 0;
}

extern void FPGA_Set_GZI_Shem_EHO( TFPGA_Tact_EHO  *takt );
extern void FPGA_Select_Generator(int Generator, unsigned int Period, int IRQMask );

extern volatile int PeakDP0_Mode;
extern volatile int PeakDP1_Mode;
extern volatile int BoardSyncMode;

void WriteSETS_TO_FPGA( void )
{
int i;
        logo_printf("\r\n\r\nVer: %d \r\n", pSETS->Version );
        logo_printf("IP: %08X \r\n", pSETS->MainBoard_IP );
        logo_printf("SyncType: %d \r\n", pSETS->SyncType );
        logo_printf("Fsmp: %d \r\n", pSETS->Fsmp );
//        lou_printf("AscanTakt: %d \r\n", pSETS->AscanTakt );
        logo_printf(" ------------------------------------------------------\r\n" );
#if(0)
        {
            logo_printf("         St: %d \r\n", pSETS->TAKT.State );

            logo_printf("         F: %d ", pSETS->TAKT.PEPFreq_Hz );
            logo_printf(" Tl: %d ", pSETS->TAKT.TaktLen_us );
            logo_printf(" ADel: %d ", pSETS->TAKT.AscanDly_us );
            logo_printf(" ALen %d \r\n", pSETS->TAKT.AscanLen_us );


            for(int j = 0; j < MUX_MAX_STROB; j++ )
            {
                logo_printf("          --Strobe[%d] Sb: %d ",j, pSETS->TAKT.Strob[j].Dly_us );
                logo_printf(" Sw: %d ", pSETS->TAKT.Strob[j].Len_us );
                logo_printf(" Sl: %d  %d %d \r\n", pSETS->TAKT.Strob[j].Level_10, pSETS->TAKT.Strob[j].mState, pSETS->TAKT.Strob[j].mPorog );
            }
            logo_printf("          --Strobe[I] L: %d \r\n",pSETS->TAKT.IStrob.Level_10 );

            logo_printf("         In: %d ", pSETS->TAKT.InChan );
            logo_printf(" Out: %d \r\n", pSETS->TAKT.OutChan );

            logo_printf("         Count: %d \r\n", pSETS->TAKT.KuTab_count );
            logo_printf(" ------------------------------------------------------\r\n" );
        }
#endif
    Tact_EHO.GZI_Disable = (pSETS->TAKT.OutChan >= 0) ? 0 : 1;

    Tact_EHO.PeriodGZI_Type = 1;
    Tact_EHO.PeriodGZI_us = 500000;
    Tact_EHO.PeriodGZI_mm = 10;

    Tact_EHO.Freq_Hz = pSETS->TAKT.PEPFreq_Hz;

    Tact_EHO.ABegin_ns = pSETS->TAKT.AscanDly_us * 100;
    Tact_EHO.AWidth_ns = pSETS->TAKT.AscanLen_us * 100;
    Tact_EHO.ACount = 512;

    for(i = 0; i<MUX_MAX_STROB ; i++){
        Tact_EHO.Strobe[i].Number = i;
        Tact_EHO.Strobe[i].curGain = pSETS->TAKT.KuBase10;

        Tact_EHO.Strobe[i].State = pSETS->TAKT.Strob[i].Len_us == 0 ? 0 : 1;
        Tact_EHO.Strobe[i].Begin_ns = pSETS->TAKT.Strob[i].Dly_us * 100;
        Tact_EHO.Strobe[i].Width_ns = pSETS->TAKT.Strob[i].Len_us * 100;
        Tact_EHO.Strobe[i].Level_proc10 = pSETS->TAKT.Strob[i].Level_10;
        Tact_EHO.Strobe[i].ARUState = pSETS->TAKT.Strob[i].ARUState;           //  0 - Off 1 On
        Tact_EHO.Strobe[i].ARUMaxGain_10 = pSETS->TAKT.Strob[i].ARUMaxGain_10;      //  0-100 * 10
        Tact_EHO.Strobe[i].ARUTopLevel_10 = pSETS->TAKT.Strob[i].ARUTopLevel_10;     //
        Tact_EHO.Strobe[i].ARUBottomLevel_10 = pSETS->TAKT.Strob[i].ARUBottomLevel_10;  //

        switch( pSETS->TAKT.Strob[i].mState ){
            default :
            case 0  : Tact_EHO.Strobe[i].Type = 0;  break; // (1-время 0)/(0-времяMax)
            case 1  : Tact_EHO.Strobe[i].Type = 0;  break; // (1-время 0)/(0-времяMax)
            case 2  : Tact_EHO.Strobe[i].Type = 1;  break; // (1-время 0)/(0-времяMax)
        }
        switch( pSETS->TAKT.Strob[i].mPorog ){
            default:
            case 0: Tact_EHO.Strobe[i].Mode = 0; break;// (00- +pp, 01- -pp, 10- 2pp )
            case 1: Tact_EHO.Strobe[i].Mode = 2; break;// (00- +pp, 01- -pp, 10- 2pp )
            case 2: Tact_EHO.Strobe[i].Mode = 3; break;// (00- +pp, 01- -pp, 10- 2pp )
        }
        Tact_EHO.Strobe[i].Mode = 2;
    }

    if(pSETS->TAKT.KuTab_count > 1) {
        for(i = 0; i < pSETS->TAKT.KuTab_count; i++ ){
            if(i < MUX_MAX_DAC_POINTS){
                 Tact_EHO.VRCH_Time_ns[i] = pSETS->TAKT.KuTab_t_us[i] * 100;
                 Tact_EHO.VRCH_Amp_db10[i] = pSETS->TAKT.KuTab_a_10[i];
            }else{
                Tact_EHO.VRCH_Time_ns[i] = pSETS->TAKT.KuTab_ext_t_us[i-MUX_MAX_DAC_POINTS] * 100;
                Tact_EHO.VRCH_Amp_db10[i] = pSETS->TAKT.KuTab_ext_a_10[i-MUX_MAX_DAC_POINTS];
            }
        }
        Tact_EHO.VRCH_Count =pSETS->TAKT.KuTab_count;
    } else  {
        Tact_EHO.VRCH_Time_ns[0] = 0;
        Tact_EHO.VRCH_Amp_db10[0] = 0;
        Tact_EHO.VRCH_Count =pSETS->TAKT.KuTab_count;
    }

    Tact_EHO.Ku10 =   pSETS->TAKT.KuBase10;

    Tact_EHO.Average = pSETS->TAKT.Average;
    Tact_EHO.StartDelay = pSETS->TAKT.StartDelay;
    Tact_EHO.PackCount = pSETS->TAKT.PackCount;
    Tact_EHO.GZI_Energy = pSETS->TAKT.GZI_Energy;
    Tact_EHO.SetsID = pSETS->SetsID;

    // if(pSETS->Version != CUR_VERSION) return;
    if(pSETS->Version != 4) return;

    FPGA_Set_GZI_Shem_EHO( &Tact_EHO );
    pMUX_PARAM->Ascan_PointsCount = Tact_EHO.ACount;
    SetSyncState( pSETS->SyncType );
}

void SetSyncState( int type )
{
            if(type < 0)  type = pSETS->SyncType;
            PeakDP0_Mode = (pSETS->SyncType >> 8) & 0x1;
            PeakDP1_Mode = (pSETS->SyncType >> 9) & 0x1;
            BoardSyncMode = (pSETS->SyncType >> 10) & 0x1;

            switch( type & 3 ){
            default:
            case 0:
                if(pSETS->Fsmp < 30  )  pSETS->Fsmp = 30;
                FPGA_Select_Generator( 0, pSETS->Fsmp, 0 );
            break;
            case 1:
                BoardSyncMode = 1;
                if(pSETS->Fsmp < 30  )  pSETS->Fsmp = 30;
                FPGA_Select_Generator( 1, pSETS->Fsmp, 1 );
            break;
            case 2:
                if(pSETS->Fsmp < 30  )  pSETS->Fsmp = 30;
                FPGA_Select_Generator( 2, pSETS->Fsmp, 1 );
            break;
            case 3:
                if(pSETS->Fsmp < 1  )  pSETS->Fsmp = 1;
                FPGA_Select_Generator( 3, pSETS->Fsmp, 1 );   /// DP
            break;
        }
}
/// ---------------------------------------------------------------------------------------------------
