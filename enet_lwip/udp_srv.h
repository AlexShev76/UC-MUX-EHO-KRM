#ifndef __UDP_SRV_H__
#define __UDP_SRV_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif


#define SETS_PORT   235
#define ASCAN_PORT  234
#define DATA_PORT   233


//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern void echo_udp_init(void);
extern void http_init(void);
extern void tcp_echo_init(void);

extern void tcp_SETS_init(void);




extern volatile unsigned int DateSendState;
extern volatile int DateConnect;

extern struct tcp_pcb *Client_pcb;
extern void my_client_init(void);
extern void client_close(struct tcp_pcb *pcb);
extern int my_client_write( char *src, unsigned int len );
extern void  my_client_write_exit( void );



extern void tcp_logo_init(void);
extern void logo_printf( const char *format, ... );



extern void my_udp_client_init(void);
extern int my_udp_client_write( char *src, unsigned int len );




extern void tcp_BSCAN_init(void);
extern void BSCAN_send( void  );


extern void SETS_udp_init(void);
extern void AScan_udp_init(void);
extern void RAW_udp_init(void );

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __LOCATOR_H__


