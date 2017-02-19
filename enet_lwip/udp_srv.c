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
volatile unsigned int DateSendState = 0;

volatile int DateConnect = 0;
volatile int SignalKu = 100;

volatile int CurTact = 0;
volatile int CurDacPoint = 1;
volatile int wShemCounter = 0;
/// ------------------------------------------------------------

void echo_udp_init(void)
{

}





/*------------------------------------------------------------------------------------------------*/

/* This is the data for the actual web page. */
static char indexdata[] =
"HTTP/1.0 200 OK\r\n\
Content-type: text/html\r\n\
\r\n\
<html> \
<head><title>A test page</title></head> \
<body> \
This is a small test page. \
</body> \
</html>";


/* This is the callback function that is called
when a TCP segment has arrived in the connection. */
//static void http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p)
static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
char *rq;
/* If we got a NULL pbuf in p, the remote end has closed
the connection. */
if(p != NULL) {

    /* The payload pointer in the pbuf contains the data
    in the TCP segment. */
    rq = p->payload;
    lou_printf( "HTTP : [%s] \n", rq );
    /* Check if the request was an HTTP "GET /\r\n". */
//    if( rq[0] == 'G' && rq[1] == 'E' &&  rq[2] == 'T' && rq[3] == ' ' &&  rq[4] == '/' && rq[5] == '\r' &&  rq[6] == '\n') {
    if( rq[0] == 'G' && rq[1] == 'E' &&  rq[2] == 'T' && rq[3] == ' ') {
    /* Send the web page to the remote host. A zero
    in the last argument means that the data should
    not be copied into internal buffers. */
      tcp_write(pcb, indexdata, sizeof(indexdata), 0);
    }
/* Free the pbuf. */
pbuf_free(p);
}

/* Close the connection. */
tcp_close(pcb);

  return ERR_OK;

}


/* This is the callback function that is called when
a connection has been accepted. */
//static void http_accept(void *arg, struct tcp_pcb *pcb)
static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
/* Set up the function http_recv() to be called when data
arrives. */
tcp_recv(pcb, http_recv);
  return ERR_OK;
}

/* The initialization function. */
void http_init(void)
{
struct tcp_pcb *pcb;


  pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, 80);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, http_accept);

}




/*-----------------------------------------------------------------------------------------------------------------*/

#define MAX_DATA_LENGTH     (540*4)
#define DATA_LENGTH     (512*2)

//#define MAX_DATA_LENGTH     46000
//#define DATA_LENGTH         20000


char DateBuf[ MAX_DATA_LENGTH ];
int ofset = 0;



int ConectTimer = 0;
extern void FPGA_GetAScan( void  );
extern void FPGA_GetAScan_old( void  );
static err_t tcp_echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
char *rq;
//int i;
err_t   ret_state;
//float val;
//int val;

//fixedpt  fixVal_A, fixVal, fixRes;

//    lou_printf("tcp_echo_recv  [%d]\n", pcb->state);

    if(pcb->state == CLOSE_WAIT)
    {
//        lou_printf("-");
        UARTwrite( "-", 1 );
        tcp_close(pcb);
    }

    if(p != NULL) {



        //lou_printf("*");
///        UARTwrite( "*", 1 );
        rq = p->payload;
    //    rq[ p->len] = 0;
//        lou_printf( "TCP ECHO : [%s] \n", rq );

/**    /// Emulation AScan
          for(i=0; i<DATA_LENGTH; i++)
          {
//             val = (sin( (i+ofset)*0.1 ) * 255 );
//             DateBuf[i] = val;

//          fixVal = fixedpt_fromint(i + ofset);
*
            fixVal = fixedpt_div(fixedpt_fromint(ofset), fixedpt_rconst(2));
            fixVal = fixedpt_add(fixVal, fixedpt_fromint(i));


            fixVal = fixedpt_div(fixVal, fixedpt_rconst(20));
            fixRes = fixedpt_mul(fixedpt_sin( fixVal ), fixedpt_fromint(SignalKu)) ;
            val = fixedpt_toint(fixRes) & 0xff;
//             DateBuf[i] = (((i+ofset) * 4) & 0x1f)+'0';

             //val = (((i+ofset) * 1 * SignalKu / 100 ) & 0xff);
//            val = ((((i+ofset) * 4) & 0x1f) * SignalKu ) & 0xff;
             DateBuf[i] = val;

          }
          ofset++;
//          do{
*/


          FPGA_GetAScan();

          ret_state = tcp_write(pcb, DateBuf, DATA_LENGTH, 0);
          if(ret_state != ERR_OK )
               lou_printf( "!!! TCP WRITE ERROR : %d  !!!! \n", ret_state );
//          }while( ret_state != ERR_OK );
//        tcp_write(pcb, rq, p->len, 0);

        tcp_recved( pcb, p->len);
        ConectTimer = 0;
    }
    pbuf_free(p);


  return ERR_OK;



}


static err_t tcp_echo_poll(void *arg, struct tcp_pcb *pcb)
{
  int *hs;
  hs = arg;

  lou_printf(".");
  if ((hs == NULL) && (pcb->state == ESTABLISHED))
  {
    /*    printf("Null, close\n");*/
    tcp_abort(pcb);
    lou_printf("-\n");
    return ERR_ABRT;
  }
  else {
    ++(*hs);
    if ((*hs) == 10000) {
      lou_printf("-\n");
      tcp_abort(pcb);
      return ERR_ABRT;
    }

    /* If this connection has a file open, try to send some more data. If
     * it has not yet received a GET request, don't do this since it will
     * cause the connection to close immediately. */
//    if(hs && (hs->handle)) {  send_data(pcb, hs); }
  }
  return ERR_OK;
}




static err_t tcp_echo_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  int *hs;

//  lou_printf("Accept\n");
  lou_printf("+");
  ConectTimer = 0;
  hs = &ConectTimer;
//  tcp_arg(pcb, hs);
  tcp_poll(pcb, tcp_echo_poll, 100);
  tcp_recv(pcb, tcp_echo_recv);
  return ERR_OK;
}





void tcp_echo_init(void)
{
struct tcp_pcb *pcb;
  lou_printf("Tcp_echo_init\n");
  pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, ASCAN_PORT);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, tcp_echo_accept);
}




/*-----------------------------------------------------------------------------------------------------------------*/

char *GetParamStr( char *dest, char *src, char *endstr )
{
      while( (src < endstr) )
      {
        if( *src == ' ' ) { src++;  continue; }
        if( *src == ',' ) { src++;  continue; }
        if( *src == '$' ) { src++;  continue; }
        if( *src == '\r' ) { src++;  continue; }
        if( *src == '\n' ) { src++;  continue; }
        break;
      }


      while( (src < endstr) )
      {
          if( *src == ' ' ) break;
          if( *src == ',' ) break;
          if( *src == '$' ) break;
          if( *src == '\r' ) break;
          if( *src == '\n' ) break;
          *dest = *src;
          src++;
          dest++;
      }
    *dest = 0;

    return src;

}




extern int test_t[];
extern int test_a[];
extern void FPGFA_Set_DACCurve(int tact, short  *t_us, short *a_proc10, int count, int iKu   );



char CMD[10];
char PARAM[20];
char VALUE[255];
char ANSWER[255];
char *pANSWER;
char wANSWER[255];
int SETSConectTimer = 0;
static err_t tcp_SETS_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    if(pcb->state == CLOSE_WAIT)
    {
        lou_printf("-");
        tcp_close(pcb);
    }


    pbuf_free(p);


  return ERR_OK;



}


static err_t tcp_SETS_poll(void *arg, struct tcp_pcb *pcb)
{
  int *hs;
  hs = arg;

//  lou_printf("s");
  if ((hs == NULL) && (pcb->state == ESTABLISHED))
  {
    /*    printf("Null, close\n");*/
    tcp_abort(pcb);
    lou_printf("-\n");
    return ERR_ABRT;
  }
  else {
    ++(*hs);
    if ((*hs) == 10000) {
      lou_printf("-\n");
      tcp_abort(pcb);
      return ERR_ABRT;
    }

    /* If this connection has a file open, try to send some more data. If
     * it has not yet received a GET request, don't do this since it will
     * cause the connection to close immediately. */
//    if(hs && (hs->handle)) {  send_data(pcb, hs); }
  }
  return ERR_OK;
}




static err_t tcp_SETS_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  int *hs;

//  lou_printf("Accept\n");
  lou_printf("+");
  SETSConectTimer = 0;
  hs = &SETSConectTimer;
  tcp_arg(pcb, hs);
  tcp_poll(pcb, tcp_SETS_poll, 100);
  tcp_recv(pcb, tcp_SETS_recv);
  return ERR_OK;
}





void tcp_SETS_init(void)
{
struct tcp_pcb *pcb;
  lou_printf("Tcp_SETS_init\n");
  pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, SETS_PORT);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, tcp_SETS_accept);
}




/* -------------------------------   */

void ShowIPAddress(unsigned long ipaddr )
{
    unsigned char *pucTemp = (unsigned char *)&ipaddr;
    lou_printf("[%d.%d.%d.%d]\n\r", pucTemp[0], pucTemp[1], pucTemp[2], pucTemp[3]);
}




/* --------------------------------------------------------------------------------------------------------*/
///  --------------------------  SEND BSCAN --------------------------------------------------------------------------

char BSCANStr[ 256 ];
int BSCANConectTimer;
struct tcp_pcb *BSCAN_pcb = NULL;
volatile int BSCANConnected = 0;

int BSCANConectTimer = 0;
static err_t tcp_BSCAN_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
char *rq;
//int i;
err_t   ret_state;

//    UARTprintf("tcp_echo_recv  [%d]\n", pcb->state);

    if(pcb->state == CLOSE_WAIT)
    {
        UARTprintf("r-");
        BSCANConnected = 0;
        tcp_close(pcb);
    }

    if(p != NULL) {

        rq = p->payload;
    //    rq[ p->len] = 0;
//        UARTprintf( "TCP ECHO : [%s] \n", rq );


          sprintf( BSCANStr, "BScan server OK\n\r");

          ret_state = tcp_write(pcb, BSCANStr, strlen(BSCANStr), 0);
          if(ret_state != ERR_OK )
               UARTprintf( "!!! TCP WRITE ERROR : %d  !!!! \n", ret_state );

        tcp_recved( pcb, p->len);
        BSCANConectTimer = 0;
    }
    pbuf_free(p);


  return ERR_OK;



}


static err_t tcp_BSCAN_poll(void *arg, struct tcp_pcb *pcb)
{
  int *hs;
  hs = arg;

//  UARTprintf(".");
  if ((hs == NULL) && (pcb->state == ESTABLISHED))
  {
    /*    printf("Null, close\n");*/
    tcp_abort(pcb);
    UARTprintf("p-\n");
    BSCANConnected = 0;
    return ERR_ABRT;
  }
  else {
    ++(*hs);
    if ((*hs) == 10000) {
      UARTprintf("-\n");
      BSCANConnected = 0;
      tcp_abort(pcb);
      return ERR_ABRT;
    }

    /* If this connection has a file open, try to send some more data. If
     * it has not yet received a GET request, don't do this since it will
     * cause the connection to close immediately. */
//    if(hs && (hs->handle)) {  send_data(pcb, hs); }
  }
  return ERR_OK;
}




static err_t tcp_BSCAN_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  int *hs;
  UARTprintf("+");
  BSCANConectTimer = 0;
  BSCANConnected = 1;
  hs = &BSCANConectTimer;
  tcp_arg(pcb, hs);
  BSCAN_pcb = pcb;
  tcp_poll(pcb, tcp_BSCAN_poll, 100);
  tcp_recv(pcb, tcp_BSCAN_recv);
  return ERR_OK;
}





void tcp_BSCAN_init(void)
{
struct tcp_pcb *pcb;
  UARTprintf("Tcp_BSCAN_init\n");
  BSCANConnected = 0;
  pcb = tcp_new();
  BSCAN_pcb = pcb;
  tcp_bind(pcb, IP_ADDR_ANY, 3333);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, tcp_BSCAN_accept);
  BSCAN_pcb = pcb;
}



/*---------------------------------------------------------------------------*/
//#include <stdio.h>
//#include <stdlib.h>
int bCounter;
void BSCAN_send( void  )
{
//int linelen;
err_t   ret_state;

///    Тут нужно сделать отправку блока данных пиков
    if( (BSCAN_pcb != NULL) && (BSCANConnected != 0 ) )
    {

//      sprintf(BSCANStr, "[BSCAN %d]\n\r", bCounter++ );
//      strcpy(BSCANStr, "*" );
//      linelen = strlen(BSCANStr);
//      ret_state = tcp_write(BSCAN_pcb, BSCANStr, linelen, 0);
      ret_state = tcp_write(BSCAN_pcb, "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n\r", 42, 0);
      UARTprintf("B");
      BSCANConectTimer = 0;
    }
}










/// ---------------------------------------------------------------------------------------------------------------
/*-----------------------------------------------------------------------------------------------------------------*/

char LogStr[ 256 ];
int LogoConectTimer;
struct tcp_pcb *Logo_pcb = NULL;


int LogoConectTimer = 0;
static err_t tcp_logo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
char *rq;
//int i;
err_t   ret_state;

//    UARTprintf("tcp_echo_recv  [%d]\n", pcb->state);

    if(pcb->state == CLOSE_WAIT)
    {
        UARTprintf("r-");
        tcp_close(pcb);
    }

    if(p != NULL) {

        rq = p->payload;
    //    rq[ p->len] = 0;
//        UARTprintf( "TCP ECHO : [%s] \n", rq );


          sprintf( LogStr, "Logo server OK\n\r");

          ret_state = tcp_write(pcb, LogStr, strlen(LogStr), 0);
          if(ret_state != ERR_OK )
               UARTprintf( "!!! TCP WRITE ERROR : %d  !!!! \n", ret_state );

        tcp_recved( pcb, p->len);
        LogoConectTimer = 0;
    }
    pbuf_free(p);


  return ERR_OK;



}


static err_t tcp_logo_poll(void *arg, struct tcp_pcb *pcb)
{
  int *hs;
  hs = arg;

//  UARTprintf(".");
  if ((hs == NULL) && (pcb->state == ESTABLISHED))
  {
    /*    printf("Null, close\n");*/
    tcp_abort(pcb);
    UARTprintf("p-\n");
    return ERR_ABRT;
  }
  else {
    ++(*hs);
    if ((*hs) == 10000) {
      UARTprintf("-\n");
      tcp_abort(pcb);
      return ERR_ABRT;
    }

    /* If this connection has a file open, try to send some more data. If
     * it has not yet received a GET request, don't do this since it will
     * cause the connection to close immediately. */
//    if(hs && (hs->handle)) {  send_data(pcb, hs); }
  }
  return ERR_OK;
}




static err_t tcp_logo_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  int *hs;
  UARTprintf("+");
  LogoConectTimer = 0;
  hs = &LogoConectTimer;
  tcp_arg(pcb, hs);
  Logo_pcb = pcb;
  tcp_poll(pcb, tcp_logo_poll, 100);
  tcp_recv(pcb, tcp_logo_recv);
  return ERR_OK;
}





void tcp_logo_init(void)
{
struct tcp_pcb *pcb;
  UARTprintf("Tcp_logo_init\n");
  pcb = tcp_new();
  Logo_pcb = pcb;
  tcp_bind(pcb, IP_ADDR_ANY, 123);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, tcp_logo_accept);
}



/*---------------------------------------------------------------------------*/
//#include <stdio.h>
//#include <stdlib.h>
#include <stdarg.h>
void lou_printf( const char *format, ... )
{
    va_list ap;
//    err_t   ret_state;
    int iRet;

    int linelen;


    va_start(ap, format);
//    vsprintf( LogStr, format, ap);
    iRet = uvsnprintf(LogStr, 0xffff, format, ap);
    va_end(ap);

    linelen = strlen(LogStr);

    UARTwrite( LogStr, linelen );


//    if( Logo_pcb != NULL )
//    {
//      ret_state = tcp_write(Logo_pcb, LogStr, linelen, 0);
//      LogoConectTimer = 0;
//    }
///    if(ret_state != ERR_OK )  UARTprintf( "!!! TCP WRITE ERROR : %d  !!!! \n", ret_state );



}






/*-----------------------------------------------------------------------------------------------------------------*/

struct ip_addr udp_server;
struct ip_addr *udp_pserver;



#define UDP_SERVER_PORT         9999
#define UDP_BUFFER_SIZE         540 //512
unsigned int data_send_port  =   (UDP_SERVER_PORT);
struct pbuf *udp_client_pbuf;
///unsigned char udp_buffer[UDP_BUFFER_SIZE];


struct udp_pcb *udpClient_pcb;

void my_udp_client_init(void)
{
   struct udp_pcb *pcb;
//   struct ip_addr dest;
//   struct ip_addr *pdest;
//   err_t ret_val;

   lou_printf("UDP INIT \n\r");



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



//static err_t SETS_UDP___recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
static void SETS_udp_recv( void* arg, struct udp_pcb *pcb,  struct pbuf *p, struct ip_addr *addr, u16_t port )
{
//char *rq, *rqe;
//err_t   ret_state;
//int CmdOK;
//char *end;

//struct pbuf *udp_pbuf;


//int SetValue;

//int Adr;
//int Len;
//int GET_CMD;
//char *pBuf;
//    lou_printf( "$set\r\n", rq[0], rq[1], rq[2], rq[3]);
    if(p != NULL) {

        SetsRequest = 1;

        UDP_Sets_pcb  = pcb;
        UDP_Sets_p    = p;
        UDP_Sets_addr = addr;
        UDP_Sets_port = port;
   //     lou_printf( "SETS_udp_recv \r\n");
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


char *rq, *rqe;
err_t   ret_state;
int CmdOK;
//char *end;

struct pbuf *udp_pbuf;

int k;

//int SetValue;

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
        rqe = &rq[p->len];
#ifdef SETS_udp_recv_DEBUG_PRINTF
        lou_printf( "[%c %c %c %c  %02x %02x %02x %02x] \r\n", rq[0], rq[1], rq[2], rq[3], rq[4], rq[5], rq[6], rq[7]);
#endif
        UARTwrite( rq, 4 );
        UARTwrite( "\r\n", 2 );
///        strcpy(wANSWER, "");
        pANSWER = ANSWER;
        if(p->len > 0)
        {

                CmdOK = 0;
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
                    CmdOK = 1;
                    Adr = *((int*)(&(rq[4])));
                    Len = *((int*)(&(rq[8])));
                    pBuf = (char*)pSETS;
                    pBuf = &(pBuf[Adr]);

//                    if( ((int)pBuf > (int)pSETS) && ((int)pBuf < (int)pSETS + sizeof(TMUX_SETS))  ){
                        memcpy( pBuf, &(rq[12]), Len );
                        strcpy( pANSWER, "!SET OK \n\r" );

#ifdef SETS_udp_recv_DEBUG_PRINTF
                    lou_printf("!!!!!! KU: %d      [%d] %d \r\n", pSETS->TAKT.KuBase10, Len,   pSETS->MaxAscan );
                    lou_printf( "[%c %c %c %c  (%02x %02x %02x %02x) (%02x  %02x  %02x  %02x)] ", rq[0], rq[1], rq[2], rq[3], rq[4], rq[5], rq[6], rq[7], rq[8], rq[9], rq[10], rq[11]);
/*
                    for(k = 0; k < 77; k++ ){
                        if(k%8)
                            lou_printf(" %8d", ((int*)(&rq[12]))[k] );
                        else
                            lou_printf("\r\n%8d", ((int*)(&rq[12]))[k] );
                    }
                    lou_printf( "\n");
*/
#endif
                    if( addCMD!=0 ) {
                        WriteSETS_TO_FPGA( );
                        wShemCounter++;
                        IP4_ADDR(udp_pserver, (pSETS->MainBoard_IP>>24)&0xff, (pSETS->MainBoard_IP>>16)&0xff, (pSETS->MainBoard_IP>>8)&0xff, (pSETS->MainBoard_IP>>0)&0xff);
                        data_send_port = pSETS->SendPort;
                        lou_printf("send_port: %d\n", data_send_port);

                        ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                    }



//                    }else{
//                        strcpy( pANSWER, "!SET ERR \n\r" );
//                    }

                    Len = strlen(pANSWER); /// sizeof(TMUX_PARAM);


                break;
                case 2:     // $GET
                    CmdOK = 1;
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
                    CmdOK = 1;
                    memcpy( &GET_CMD, &(rq[4]), 4  );
                    switch(GET_CMD)
                    {
                        case 1:
//                            memcpy( &pMUX_DP->DP1, &(rq[8]), 4  );
//                            memcpy( &pMUX_DP->DP2, &(rq[12]), 4 );
//                            FPGA_Write_DP( 0, pMUX_DP->DP1 );
//                            FPGA_Write_DP( 1, pMUX_DP->DP2 );
                            FPGA_Write_DP( 0 );
                        break;
                        default: break;
                    }
                    strcpy( pANSWER, "!WRT OK \n\r" );
                    Len = strlen(pANSWER);

                break;
                case 4:     // $RUN
                    CmdOK = 1;

#ifdef SETS_udp_recv_DEBUG_PRINTF
                    lou_printf( "$RUN \n" );
#endif

                    strcpy( pANSWER, "!RUN OK \n\r" );
                    Len = strlen(pANSWER);


                break;
                case 5:     // $END
                    CmdOK = 1;
//#ifdef SETS_udp_recv_DEBUG_PRINTF
                    lou_printf( "$END \n" );
//#endif

                    WriteSETS_TO_FPGA( );
                    wShemCounter++;
                    strcpy( pANSWER, "!END OK \n\r" );
                    Len = strlen(pANSWER);

                    IP4_ADDR(udp_pserver, (pSETS->MainBoard_IP>>24)&0xff, (pSETS->MainBoard_IP>>16)&0xff, (pSETS->MainBoard_IP>>8)&0xff, (pSETS->MainBoard_IP>>0)&0xff);
                    data_send_port = pSETS->SendPort;
                    ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                break;


                case 6:     // $STR    (start)
                    CmdOK = 1;
                    lou_printf( "$STR (start)\n" );
                    chenal_start_state = 1;
                    setStatGer( FPGA_ISR_RG_STATE_reg );
                    FPGA_Write_SYNC_REG(0, 1);

                     if( BoardSyncMode == 0) SetSyncState( -1 );

                    strcpy( pANSWER, "!STR OK \n\r" );
                    Len = strlen(pANSWER);
                    ShowIPAddress( *((unsigned long*)(udp_pserver)) );
                break;

                case 7:     // $STP    (stop)
                    CmdOK = 1;
                    lou_printf( "$STP (stop)\n" );
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
                    CmdOK = 1;
                    lou_printf( "$SYN (sync)\n" );

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

///                lou_printf( "[%s] %08X %08X %08X  \n", pANSWER, GET_CMD,  p->len, Len   );
///                lou_printf( "GET [%02X %02X %02X %02X ] [%02X %02X %02X %02X ]\n", pANSWER[0], pANSWER[1], pANSWER[2], pANSWER[3], pANSWER[4], pANSWER[5], pANSWER[6], pANSWER[7] );
                udp_pbuf->payload =  pANSWER; ///wANSWER;
                udp_pbuf->len = Len;
                udp_pbuf->tot_len = Len;
                ret_state = udp_sendto(pcb, udp_pbuf, addr, port);

                if(ret_state != ERR_OK )
                    UARTwrite( "!!! UDP WRITE ERROR !!!! \n", 26 );
                     ///lou_printf( "!!! UDP WRITE ERROR : %d  !!!! \n", ret_state );

                pbuf_free(udp_pbuf);
            }
            else
            {
                     UARTwrite( "!!! ERROR pbuf_alloc  !!!! \n" , 26);
                     ///lou_printf( "!!! ERROR pbuf_alloc  !!!! \n" );
            }
        }

    SETSConectTimer = 0;

        pbuf_free(p);
        udp_remove( pcb );
        SETS_udp_init();


}


/// -------------------------------------------------------------------------------------------------------------
///  ASCAN UDP



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
//err_t err;
//char *rq;
//struct pbuf *udp_pbuf;
    if(p != NULL) {
//        rq = p->payload;
//        rqe = &rq[p->len];
        if(p->len > 0)
        {
            AscanRequest = 1;
            UDP_ASCAN_pcb   = pcb;
            UDP_ASCAN_p  = p;
            UDP_ASCAN_addr = addr;
            UDP_ASCAN_port = port;
//            lou_printf( "Len %d...\n\r", p->len);

//            lou_printf( "Len %d [%c%c%c%c]...\n\r", p->len, rqe[0], rqe[1], rqe[2], rqe[3]);
///            lou_printf( "%d Len %d ",CounterAScan++, p->len );
///            lou_printf( " HOST %d.%d.%d.%d \n\r",((UDP_ASCAN_addr->addr >> 0) & 0xff),((UDP_ASCAN_addr->addr >> 8) & 0xff), ((UDP_ASCAN_addr->addr >> 16) & 0xff), ((UDP_ASCAN_addr->addr >> 24) & 0xff));
        }
    }
    else{
        pbuf_free(p);
        udp_remove( pcb );
        AScan_udp_init();
    }
    return;
}



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
           //FPGA_GetAScan_old(  );

           udp_pbuf = pbuf_alloc( PBUF_TRANSPORT, UDP_BUFFER_SIZE,   PBUF_RAM) ;
           if(udp_pbuf == NULL) {

                lou_printf( "ERROR:  ASCAN udp_send() alloc error: %d\n", err );

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
                lou_printf( "ERROR:  ASCAN udp_send() error: %d\n", err );
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
//err_t err;
//char *rq;
//struct pbuf *udp_pbuf;


    if(p != NULL) {

///        rq = p->payload;
///        rqe = &rq[p->len];
        if(p->len > 0)
        {
            RAW_Request = 1;
            UDP_RAW_pcb   = pcb;
            UDP_RAW_p  = p;
            UDP_RAW_addr = addr;
            UDP_RAW_port = port;

//             lou_printf( "Len %d [%c%c%c%c]...\n\r", p->len, rqe[0], rqe[1], rqe[2], rqe[3]);
//            lou_printf( "%d Len %d ",CounterAScan++, p->len );
//          lou_printf( " HOST %d.%d.%d.%d: %d \n\r",((UDP_ASCAN_addr->addr >> 0) & 0xff),((UDP_ASCAN_addr->addr >> 8) & 0xff), ((UDP_ASCAN_addr->addr >> 16) & 0xff), ((UDP_ASCAN_addr->addr >> 24) & 0xff), UDP_ASCAN_port);
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

//           UDP_ASCAN_port = 6799;
///           lou_printf( "\nA+B %X \n", UDP_ASCAN_port);
           FPGA_GetRAW();

///           lou_printf( "A+a\n");
           udp_pbuf = pbuf_alloc( PBUF_TRANSPORT, UDP_BUFFER_SIZE,   PBUF_RAM) ;

           RawBuffer[0] = 0;
           udp_pbuf->payload =  &RawBuffer[1024*0];
           udp_pbuf->len = udp_pbuf->tot_len = 513; //1025; //SIZE_RAW_BUFFER;
           err = udp_sendto(pcb, udp_pbuf, UDP_RAW_addr, UDP_RAW_port);


/*          RawBuffer[1024*1-1] = 1;
           udp_pbuf->payload =  &RawBuffer[1024*1-1];
           udp_pbuf->len = udp_pbuf->tot_len = 1025; //SIZE_RAW_BUFFER;
           err = udp_sendto(pcb, udp_pbuf, UDP_RAW_addr, UDP_RAW_port);


           RawBuffer[1024*2-1] = 2;
           udp_pbuf->payload =  &RawBuffer[1024*2-1];
           udp_pbuf->len = udp_pbuf->tot_len = 1025; //SIZE_RAW_BUFFER;
           err = udp_sendto(pcb, udp_pbuf, UDP_RAW_addr, UDP_RAW_port);


           RawBuffer[1024*3-1] = 3;
           udp_pbuf->payload =  &RawBuffer[1024*3-1];
           udp_pbuf->len = udp_pbuf->tot_len = 1025; //SIZE_RAW_BUFFER;
           err = udp_sendto(pcb, udp_pbuf, UDP_RAW_addr, UDP_RAW_port);
*/


//#ifdef DEBAG_MESS_SEND_ASKAN
//           lou_printf( "RAW SEND %d ", err);
//           lou_printf( " HOST %d.%d.%d.%d : %d \n\r",((UDP_ASCAN_addr->addr >> 0) & 0xff),((UDP_ASCAN_addr->addr >> 8) & 0xff), ((UDP_ASCAN_addr->addr >> 16) & 0xff), ((UDP_ASCAN_addr->addr >> 24) & 0xff), UDP_ASCAN_port);
//#endif

           RAW_Request = 0;

            if (err != ERR_OK)
            {
                lou_printf( "ERROR: RAW  udp_send() error: %d\n", err );
            }

            pbuf_free(udp_pbuf);
            pbuf_free( p );
            udp_remove( pcb );
            RAW_udp_init();

///            lou_printf( "A+E\n");
}



/// --------------------------------------------------------------------------------------------------------------




