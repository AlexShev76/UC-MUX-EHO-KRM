//#include <stdio.h>
#include <stdlib.h>
#include <utils/ustdlib.h>

#include "udp_srv.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "utils/uartstdio.h"


#include <math.h>
#include <string.h>
#include <stdlib.h>
//#include <stdio.h>

#include "fixedptc.h"

#include "std_my.h"
#include "DSP.h"

/// -----------------------------------------------------
int SETSConectTimer = 0;

char LogStr[ 256 ];

volatile unsigned int DateSendState = 0;

volatile int DateConnect = 0;
volatile int SignalKu = 100;

volatile int CurTact = 0;
volatile int CurDacPoint = 1;
volatile int wShemCounter = 0;
/// ------------------------------------------------------------

extern int test_t[];
extern int test_a[];
extern void FPGFA_Set_DACCurve(int tact, short  *t_us, short *a_proc10, int count, int iKu   );

#define MAX_DATA_LENGTH     (540*4)
#define DATA_LENGTH     (512*2)

char DateBuf[ MAX_DATA_LENGTH ];


char CMD[10];
char PARAM[20];
char VALUE[255];
char ANSWER[255];
char *pANSWER;
char wANSWER[255];
///-------------------------------------------------------------
void echo_udp_init(void)
{

}


void ShowIPAddress(unsigned long ipaddr )
{
    unsigned char *pucTemp = (unsigned char *)&ipaddr;
    logo_printf("[%d.%d.%d.%d]\n\r", pucTemp[0], pucTemp[1], pucTemp[2], pucTemp[3]);
}

/*---------------------------------------------------------------------------*/
#include <stdarg.h>
void logo_printf( const char *format, ... )
{
    va_list ap;
    int linelen;
    va_start(ap, format);
    uvsnprintf(LogStr, 0xffff, format, ap);
    va_end(ap);
    linelen = strlen(LogStr);
    UARTwrite( LogStr, linelen );
}

/*-----------------------------------------------------------------------------------------------------------------*/

struct ip_addr udp_server;
struct ip_addr *udp_pserver;


#define UDP_SERVER_PORT         9999
#define UDP_BUFFER_SIZE         540 //512
unsigned int data_send_port  =   (UDP_SERVER_PORT);
struct pbuf *udp_client_pbuf;

struct udp_pcb *udpClient_pcb;

void my_udp_client_init(void)
{
   struct udp_pcb *pcb;
   logo_printf("UDP INIT \n\r");
    pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, UDP_SERVER_PORT);     //client port for outcoming connection
    udpClient_pcb = pcb;

     udp_client_pbuf = pbuf_alloc( PBUF_TRANSPORT, UDP_BUFFER_SIZE,   PBUF_RAM) ;
     udp_pserver = &udp_server;
     IP4_ADDR(udp_pserver, 192, 168, 1, 99);
     ShowIPAddress( *((unsigned long*)(udp_pserver)) );
}


extern TMUX_SETS *pSETS;
int my_udp_client_write( char *src, unsigned int len )
{
err_t ret;
   udp_client_pbuf->payload =  src;
   udp_client_pbuf->len = udp_client_pbuf->tot_len = len;
   ret = udp_sendto(udpClient_pcb, udp_client_pbuf, udp_pserver, data_send_port);
   return ret;
}

/// -------------------------------------------------------------------------------------------------------------
///  UDP SETs Server
static void SETS_udp_recv( void* arg, struct udp_pcb *pcb,  struct pbuf *p, struct ip_addr *addr, u16_t port );
void SETS_udp_init(void)
{
  struct udp_pcb *pcb;
  pcb = udp_new();
  udp_bind( pcb, IP_ADDR_ANY, 8888 );
  udp_recv( pcb, SETS_udp_recv, NULL );
}

#define SETS_udp_recv_DEBUG_PRINTF

extern TMUX_SETS *pSETS;
extern TMUX_PARAM *pMUX_PARAM;
extern TMUX_DP *pMUX_DP;
extern TMUX_DP *pBaseMUX_DP;


volatile int SetsRequest;
struct udp_pcb *UDP_Sets_pcb;
struct pbuf    *UDP_Sets_p;
struct ip_addr *UDP_Sets_addr;
u16_t           UDP_Sets_port;

static void SETS_udp_recv( void* arg, struct udp_pcb *pcb,  struct pbuf *p, struct ip_addr *addr, u16_t port )
{
    if(p != NULL) {
        SetsRequest = 1;
        UDP_Sets_pcb  = pcb;
        UDP_Sets_p    = p;
        UDP_Sets_addr = addr;
        UDP_Sets_port = port;
        SETSConectTimer = 0;
    }else{
        pbuf_free(p);
        udp_remove( pcb );
        SETS_udp_init();
    }
  return ; //ERR_OK;
}

extern volatile unsigned int FPGA_ISR_RG_STATE_reg;
extern volatile unsigned int Strob_AruState;
extern void setStatGer( unsigned int val );
extern volatile unsigned short iSateScan;
extern volatile unsigned short fpga_REGS[3];
extern volatile int BoardSyncMode;
extern volatile int chenal_start_state;
extern unsigned int FPGA_Read_DP( );
extern void FPGA_Write_SYNC_REG( int reg, unsigned short dat );
extern void FPGA_Write_DP( unsigned int wey );

extern void FPGA_Write_GAinDP( int Ku );

void SendSetsAnswer( void )
{
struct udp_pcb *pcb;
struct pbuf    *p;
struct ip_addr *addr;
u16_t           port;

char *rq;
err_t   ret_state;


struct pbuf *udp_pbuf;

int Adr;
int Len = 0;
int GET_CMD; //, i;
char *pBuf;
int CMD = 0;
int addCMD = 0;

        if( SetsRequest == 0 ) return;
        SetsRequest = 0;
        pcb = UDP_Sets_pcb;
        p = UDP_Sets_p;
        addr = UDP_Sets_addr;
        port = UDP_Sets_port;

        rq = p->payload;
        //rqe = &rq[p->len];
#ifdef SETS_udp_recv_DEBUG_PRINTF
        logo_printf( "[%c %c %c %c  %02x %02x %02x %02x] \r\n", rq[0], rq[1], rq[2], rq[3], rq[4], rq[5], rq[6], rq[7]);
#endif
        UARTwrite( rq, 4 );
        UARTwrite( "\r\n", 2 );
///        strcpy(wANSWER, "");
        pANSWER = ANSWER;
        if(p->len > 0)
        {
                addCMD = 0;
                CMD = 0;
                if( (rq[0] == '$') && (rq[1] == 'S') && (rq[2] == 'E') && (rq[3] == 'T') ){  CMD = 1; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'G') && (rq[2] == 'E') && (rq[3] == 'T') ){  CMD = 2; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'W') && (rq[2] == 'R') && (rq[3] == 'T') ){  CMD = 3; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'R') && (rq[2] == 'U') && (rq[3] == 'N') ){  CMD = 4; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'E') && (rq[2] == 'N') && (rq[3] == 'D') ){  CMD = 5; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'S') && (rq[2] == 'E') && (rq[3] == 'E') ){  CMD = 1; addCMD = 1;}
                if( (rq[0] == '$') && (rq[1] == 'S') && (rq[2] == 'T') && (rq[3] == 'R') ){  CMD = 6; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'S') && (rq[2] == 'T') && (rq[3] == 'P') ){  CMD = 7; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'S') && (rq[2] == 'Y') && (rq[3] == 'N') ){  CMD = 8; addCMD = 0;}
                if( (rq[0] == '$') && (rq[1] == 'T') && (rq[2] == 'G') && (rq[3] == 'A') ){  CMD = 9; addCMD = 0;}

                switch( CMD){

                    case 9:
                    // GainDP
                        memcpy( GainDP, &(rq[4]), sizeof(GainDP) );
                        strcpy( pANSWER, "!TGA OK \n\r" );
                        FPGA_Write_GAinDP( -1 );
                    break;

                    case 1:   // $SET

                    Adr = *((int*)(&(rq[4])));
                    Len = *((int*)(&(rq[8])));
                    pBuf = (char*)pSETS;
                    pBuf = &(pBuf[Adr]);

                    memcpy( pBuf, &(rq[12]), Len );
                    strcpy( pANSWER, "!SET OK \n\r" );

#ifdef SETS_udp_recv_DEBUG_PRINTF
                    logo_printf("!!!!!! KU: %d      [%d] %d \r\n", pSETS->TAKT.KuBase10, Len,   pSETS->MaxAscan );
                    logo_printf( "[%c %c %c %c  (%02x %02x %02x %02x) (%02x  %02x  %02x  %02x)] ", rq[0], rq[1], rq[2], rq[3], rq[4], rq[5], rq[6], rq[7], rq[8], rq[9], rq[10], rq[11]);
#endif
                    if( addCMD!=0 ) {
                        WriteSETS_TO_FPGA( );
                        wShemCounter++;
                        IP4_ADDR(udp_pserver, (pSETS->MainBoard_IP>>24)&0xff, (pSETS->MainBoard_IP>>16)&0xff, (pSETS->MainBoard_IP>>8)&0xff, (pSETS->MainBoard_IP>>0)&0xff);
                        data_send_port = pSETS->SendPort;
                        logo_printf("send_port: %d\n", data_send_port);
                        ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                    }
                    Len = strlen(pANSWER); /// sizeof(TMUX_PARAM);
                break;
                case 2:     // $GET
                    GET_CMD = *((int*)(&(rq[4])));
                    switch(GET_CMD)
                    {
                        default :
                                strcpy( pANSWER, "!GET OK \n\r" );
                                Len = strlen(pANSWER);
                        break;

                        case 0:
                                    Len = sizeof(TMUX_PARAM);
                                    memcpy(pANSWER, pMUX_PARAM, Len );
                        break;

                        case 1:
                                pMUX_DP->DP1 = FPGA_Read_DP();
                                pMUX_DP->DP2 = FPGA_Read_DP();
                                Len = sizeof(TMUX_DP);
                                memcpy(pANSWER, pMUX_DP, Len );
                        break;
                    }
                break;
                case 3:    // $WRT

                    memcpy( &GET_CMD, &(rq[4]), 4  );
                    switch(GET_CMD)
                    {
                        case 1:
                            FPGA_Write_DP( 0 );
                        break;
                        default: break;
                    }
                    strcpy( pANSWER, "!WRT OK \n\r" );
                    Len = strlen(pANSWER);

                break;
                case 4:     // $RUN


#ifdef SETS_udp_recv_DEBUG_PRINTF
                    logo_printf( "$RUN \n" );
#endif

                    strcpy( pANSWER, "!RUN OK \n\r" );
                    Len = strlen(pANSWER);


                break;
                case 5:     // $END
                    logo_printf( "$END \n" );
                    WriteSETS_TO_FPGA( );
                    wShemCounter++;
                    strcpy( pANSWER, "!END OK \n\r" );
                    Len = strlen(pANSWER);

                    IP4_ADDR(udp_pserver, (pSETS->MainBoard_IP>>24)&0xff, (pSETS->MainBoard_IP>>16)&0xff, (pSETS->MainBoard_IP>>8)&0xff, (pSETS->MainBoard_IP>>0)&0xff);
                    data_send_port = pSETS->SendPort;
                    ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                break;


                case 6:     // $STR    (start)

                    logo_printf( "$STR (start)\n" );
                    chenal_start_state = 1;
                    setStatGer( FPGA_ISR_RG_STATE_reg );
                    FPGA_Write_SYNC_REG(0, 1);

                     if( BoardSyncMode == 0) SetSyncState( -1 );

                    strcpy( pANSWER, "!STR OK \n\r" );
                    Len = strlen(pANSWER);
                    ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                break;

                case 7:     // $STP    (stop)
                    logo_printf( "$STP (stop)\n" );
                    chenal_start_state = 2;
                    FPGA_Write_SYNC_REG(0, 0);

                    FPGA_ISR_RG_STATE_reg &= ~(1<<14);
                    setStatGer( FPGA_ISR_RG_STATE_reg );

                    if( BoardSyncMode == 0)  SetSyncState( 0 );

                    strcpy( pANSWER, "!STP OK \n\r" );
                    Len = strlen(pANSWER);
                    ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                break;

                case 8:     // $SYN    (stop)
                    logo_printf( "$SYN (sync)\n" );
                    memcpy( &GET_CMD, &(rq[4]), 4  );
                    FPGA_Write_SYNC_REG(1, GET_CMD&0xffff );
                    memcpy( &GET_CMD, &(rq[8]), 4  );
                    iSateScan = GET_CMD & 0xffff;

                    strcpy( pANSWER, "!SYN OK \n\r" );
                    Len = strlen(pANSWER);
                    ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                break;

                default:
                    UARTwrite( "\r\nUNKNOW CMD\r\n", 14);
                    strcpy( pANSWER, "!UNKNOW CMD \n\r" );
                    Len = strlen(pANSWER);
                break;
                }

            udp_pbuf = pbuf_alloc( PBUF_TRANSPORT, UDP_BUFFER_SIZE,   PBUF_RAM) ;
            if(udp_pbuf != NULL)
            {
                udp_pbuf->payload =  pANSWER; ///wANSWER;
                udp_pbuf->len = Len;
                udp_pbuf->tot_len = Len;
                ret_state = udp_sendto(pcb, udp_pbuf, addr, port);

                if(ret_state != ERR_OK )
                    UARTwrite( "!!! UDP WRITE ERROR !!!! \n", 26 );

                pbuf_free(udp_pbuf);
            }
            else
            {
                     UARTwrite( "!!! ERROR pbuf_alloc  !!!! \n" , 26);
            }
        }

    SETSConectTimer = 0;

    pbuf_free(p);
    udp_remove( pcb );
    SETS_udp_init();
}
/// -------------------------------------------------------------------------------------------------------------

int CounterAScan = 0;
volatile int AscanRequest;
struct udp_pcb *UDP_ASCAN_pcb;
struct pbuf    *UDP_ASCAN_p;
struct ip_addr *UDP_ASCAN_addr;
u16_t           UDP_ASCAN_port;

static void AScan_udp_recv( void* arg, struct udp_pcb *pcb,  struct pbuf *p, struct ip_addr *addr, u16_t port );
void AScan_udp_init(void)
{
  struct udp_pcb *pcb;

  pcb = udp_new();
  AscanRequest = 0;
  udp_bind( pcb, IP_ADDR_ANY, 6789 );
  udp_recv( pcb, AScan_udp_recv, NULL );

}


static void AScan_udp_recv( void* arg, struct udp_pcb *pcb,  struct pbuf *p, struct ip_addr *addr, u16_t port )
{
    if(p != NULL) {
        if(p->len > 0)
        {
            AscanRequest = 1;
            UDP_ASCAN_pcb   = pcb;
            UDP_ASCAN_p  = p;
            UDP_ASCAN_addr = addr;
            UDP_ASCAN_port = port;
        }
    }
    else{
        pbuf_free(p);
        udp_remove( pcb );
        AScan_udp_init();
    }
    return;
}


extern void FPGA_GetAScan( void  );
extern char RawBuffer[SIZE_RAW_BUFFER];
#define DEBAG_MESS_SEND_ASKAN
void SendAScan( void )
{
err_t err = 0;
struct udp_pcb *pcb;
struct pbuf *p;
struct pbuf *udp_pbuf;

           if(AscanRequest == 0) return;
           pcb = UDP_ASCAN_pcb;
           p = UDP_ASCAN_p;

           FPGA_GetAScan();

           udp_pbuf = pbuf_alloc( PBUF_TRANSPORT, UDP_BUFFER_SIZE,   PBUF_RAM) ;
           if(udp_pbuf == NULL) {

                logo_printf( "ERROR:  ASCAN udp_send() alloc error: %d\n", err );

                pbuf_free( p );
                udp_remove( pcb );
                AScan_udp_init();

                return;
           }

           udp_pbuf->payload =  DateBuf;
           udp_pbuf->len = udp_pbuf->tot_len = DATA_LENGTH+256;

           err = udp_sendto(pcb, udp_pbuf, UDP_ASCAN_addr, UDP_ASCAN_port);
            if (err != ERR_OK)
            {
                logo_printf( "ERROR:  ASCAN udp_send() error: %d\n", err );
            }
            else
                AscanRequest = 0;

            pbuf_free(udp_pbuf);
            pbuf_free( p );
            udp_remove( pcb );
            AScan_udp_init();
}

extern void FPGA_GetRAW( void  );

volatile int RAW_Request;
struct udp_pcb *UDP_RAW_pcb;
struct pbuf    *UDP_RAW_p;
struct ip_addr *UDP_RAW_addr;
u16_t           UDP_RAW_port;
static void RAW_udp_recv( void* arg, struct udp_pcb *pcb,  struct pbuf *p, struct ip_addr *addr, u16_t port )
{
    if(p != NULL) {

        if(p->len > 0)
        {
            RAW_Request = 1;
            UDP_RAW_pcb   = pcb;
            UDP_RAW_p  = p;
            UDP_RAW_addr = addr;
            UDP_RAW_port = port;
        }
    }
    return;
}

void RAW_udp_init(void)
{
  struct udp_pcb *pcb;

  pcb = udp_new();
  RAW_Request = 0;
  udp_bind( pcb, IP_ADDR_ANY, 6779 );
  udp_recv( pcb, RAW_udp_recv, NULL );
}

void SendRAW( void )
{
err_t err;
struct udp_pcb *pcb;
struct pbuf *p;
struct pbuf *udp_pbuf;

           if(RAW_Request == 0) return;

           pcb = UDP_RAW_pcb;
           p = UDP_RAW_p;

           FPGA_GetRAW();

           udp_pbuf = pbuf_alloc( PBUF_TRANSPORT, UDP_BUFFER_SIZE,   PBUF_RAM) ;

           RawBuffer[0] = 0;
           udp_pbuf->payload =  &RawBuffer[1024*0];
           udp_pbuf->len = udp_pbuf->tot_len = 513; //1025; //SIZE_RAW_BUFFER;
           err = udp_sendto(pcb, udp_pbuf, UDP_RAW_addr, UDP_RAW_port);

           RAW_Request = 0;

            if (err != ERR_OK) {
                logo_printf( "ERROR: RAW  udp_send() error: %d\n", err );
            }

            pbuf_free(udp_pbuf);
            pbuf_free( p );
            udp_remove( pcb );
            RAW_udp_init();
}
/// --------------------------------------------------------------------------------------------------------------
