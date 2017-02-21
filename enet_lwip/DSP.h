#ifndef DSP_H_INCLUDED
#define DSP_H_INCLUDED




#define  SIZE_RAW_BUFFER     4096+1


/// ----------------------------------------------------------------------------------------------------------------
#define CUR_VERSION              (4)

//#define  CLK_FREQ_MHZ_CONST      (80/4)
#define  CLK_FREQ_MHZ_CONST      (80)

//#if( CUR_VERSION == 3)
//    #define MUX_MAX_TAKT_COUNT  1
//#else
    #define MUX_MAX_TAKT_COUNT  1  //16
//#endif

#define MUX_MAX_DAC_POINTS  (16)
#define MUX_MAX_EXT_DAC_POINTS  (14)
#define MUX_MAX_STROB       3





typedef struct
{
    int Ku;
    int Dly_us;
    int Len_us;
    int Level_10;
    int mState;
    int mPorog;
    int MaxKU10;
    int AruState;
}TMUX_I_STROB;


typedef struct
{
    int State;
    int Dly_us;
    int Len_us;
    int Level_TOP10;
    int Level_BOTTOM10;
    int MaxKU10;
}TMUX_ARU_STROB;


typedef struct
{
    int Dly_us;
    int Len_us;
    int Level_10;
    int mState;
    int mPorog;

    int ARUState;           //  0 - Off 1 On
    int ARUMaxGain_10;      //  0-100 * 10
    int ARUTopLevel_10;     //
    int ARUBottomLevel_10;  //
}TMUX_STROB;

typedef struct
{
/*7*/   int State;                              //   = 1
        int PEPFreq_Hz;                         //   = 5000000
        int TaktLen_us;                         //   = 100000
        int AscanDly_us;                        //   AScan Delay in us   0 ... 2000
        int AscanLen_us;                        //   AScan Range in us   1 ... 2000
/*12*/  TMUX_STROB Strob[MUX_MAX_STROB];        //MUX_MAX_STROB = 3
/*21*/
/*30*/
/*39*/  short InChan;                           // = 0
        short OutChan;                          // = 0


/*40*/  int KuTab_count;                        // Count if VRCH points 0 .. 16
/*41*/  short KuTab_t_us[MUX_MAX_DAC_POINTS];    // MUX_MAX_TAKT_COUNT = 16        VRCH points time in us 0 .. 2000
        short KuTab_a_10[MUX_MAX_DAC_POINTS];    // MUX_MAX_TAKT_COUNT = 16        VRCH points demp in db*10 0 .. 1000

/*56*/  short KuTab_ext_t_us[MUX_MAX_EXT_DAC_POINTS];    // MUX_MAX_EXT_DAC_POINTS = 14        VRCH points time in us 0 .. 2000
        short KuTab_ext_a_10[MUX_MAX_EXT_DAC_POINTS];    // MUX_MAX_EXT_DAC_POINTS = 14        VRCH points demp in db*10 0 .. 1000

/*71*/   int KuBase10;                             //  Ku   in db *10     0 ... 1000
/*72*/   int Average;                              //  Average = 0
/*73*/   int StartDelay;                           //  = 0
/*74*/   int PackCount;                            // PackCount = 1 ... 256   DEF = 10
/*75*/   int GZI_Energy;                           // GZI_Energy = 4 ... 492;
} TMUX_TAKT;


typedef struct
{
        int Version;             // Version   = 3
        int MainBoard_IP;        //  server IP adr     = (192<<24) | (168<<16) | (0<<8) | (99<<0);
        int SendPort;            //  server Port       = 9999
        int SetsID;              //  Sets ID = 0
        int SyncType;
/*       SyncType:
            bits 0..2         0 - off, 1 - intern, 2 - extern , 3 - DP
            bit 8             0/1     Type DP1
            bit 9             0/1     Type DP2
            bit 10            0/1     Board Sync Type 0 - slave    1 - Master
*/
        int Fsmp;                //   30  ... 2000 Hz
        int AscanTakt;           //   = 0
 /*7*/  TMUX_TAKT TAKT;
 /*76*/ int MaxAscan;            //  512
} TMUX_SETS;


typedef struct{
    unsigned int CLK_Freq_MHz;
    unsigned int Ascan_PointsCount;
}TMUX_PARAM;


typedef struct{
    int DP1;
    int DP2;
}TMUX_DP;

typedef struct {
    int Number;

    int curGain;

    int State;
    int Begin_ns;
    int Width_ns;
    int Level_proc10;
    int Type; // (1-время 0)/(0-времяMax)
    int Mode; // (00- +pp, 01- -pp, 10- 2pp )
    int ARUState;           //  0 - Off 1 On
    int ARUMaxGain_10;      //  0-100 * 10
    int ARUTopLevel_10;     //
    int ARUBottomLevel_10;  //
}TTactStrobe;

typedef struct
{
    int SetsID;

    int PeriodGZI_Type;
    int PeriodGZI_us;
    int PeriodGZI_mm;

    int Freq_Hz;

    int ABegin_ns;
    int AWidth_ns;
    int ACount;

    int IM_State;
    int IM_Begin_ns;
    int IM_Width_ns;
    int IM_Level_proc10;
    int IM_Type; // (1-время 0)/(0-времяMax)
    int IM_Mode; // (00- +pp, 01- -pp, 10- 2pp )
    int IM_ARU_MaxKU10;
    int IM_ARU_state;


    TTactStrobe Strobe[MUX_MAX_STROB];




    int VRCH_Time_ns[MUX_MAX_DAC_POINTS+MUX_MAX_EXT_DAC_POINTS];
    int VRCH_Amp_db10[MUX_MAX_DAC_POINTS+MUX_MAX_EXT_DAC_POINTS];
    int VRCH_Count;

    int KuIM10;
    int Ku10;

    int Average;

    int GZI_Disable;

    int StartDelay;
    int PackCount;
    int GZI_Energy;

} TFPGA_Tact_EHO;



/*
typedef struct	{
    int State;
    int Counter;
    int ScanCount;
    int ScanLength;
    int  SetsId;
    int  eDP;
    int  iDP;
    char Date[512];
} tUDPPack_ScansDate;
*/
typedef struct {
    int State;
    int Counter;
    short ScanCount;
    short RowOverCount; // макс. число а-сканов с превышением уровня обнаружения подряд
    short ScanLength;
    short AllOverCount;  // число а-сканов с превышением уровня обнаружения всего
    int  SetsId;
    int  eDP;
    int  iDP;
    char Date[512];
} tUDPPack_ScansDate;

typedef struct	{
    int cmd;
    int Len;
} tUDPPack_ScanHeader;

typedef struct	{
    tUDPPack_ScanHeader    Header;
    tUDPPack_ScansDate Date;
} tUDPPack_Scans;





extern void WriteSETS_TO_FPGA( void );
extern void InitSETS( void );
#define DEFAULT_INIT_FPGA_STRUCT()    InitSETS()
#define SEND_SETUPS_TO_FPGA()    WriteSETS_TO_FPGA()

extern TFPGA_Tact_EHO   Tact_EHO;
extern TMUX_PARAM *pMUX_PARAM;

extern unsigned short GainDP[32*3];

extern void SetSyncState( int type );


#endif // DSP_H_INCLUDED
