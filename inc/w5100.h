/*
 * History:
 *
 * 01.09.2014 - Michal Budzon - Initial
 */

#ifndef	_W5100_H_
#define	_W5100_H_

#include "wnlib.h"

#if (WNLIB_BUS_TYPE == WNLIB_BUS_SPI)
#include "spi_stm32.h" // example
#endif

//-----------------------------------------------------------------------------
//SPI macros
#define WNLIB_InitSPI                    spiInit
#define WNLIB_SpiSendByte                spiTxByte
#define WNLIB_SpiGetByte                 spiRxByte

#define WNLIB_SpiInactive                spiInactive
#define WNLIB_SpiActive                  spiActive
//-----------------------------------------------------------------------------


//------------------------------------------
#define WNLIB_TXRXBUF_SIZE_1KB			 0x00
#define WNLIB_TXRXBUF_SIZE_2KB			 0x01
#define WNLIB_TXRXBUF_SIZE_4KB			 0x10
#define WNLIB_TXRXBUF_SIZE_8KB			 0x11

#define WNLIB_TXFREESIZE_TIMEOUT		 10000

#define WNLIB_DEFAULT_RETR_ATTEMPTS      4
#define WNLIB_DEFAULT_RETR_TIMEOUT       600

//-------------
/**
 @brief Mode Registera address
 */
#define W5100_MR WNLIB_MAP_BASE

/**
 @brief Gateway IP Register address
 */
#define W5100_GAR0				(WNLIB_MAP_BASE + 0x0001)
/**
 @brief Subnet mask Register address
 */
#define W5100_SUBR0			(WNLIB_MAP_BASE + 0x0005)
/**
 @brief Source MAC Register address
 */
#define W5100_SHAR0				(WNLIB_MAP_BASE + 0x0009)
/**
 @brief Source IP Register address
 */
#define W5100_SIPR0				(WNLIB_MAP_BASE + 0x000F)
/**
 @brief Interrupt Register
 */
#define W5100_IR					(WNLIB_MAP_BASE + 0x0015)
/**
 @brief Interrupt mask register
 */
#define W5100_IMR					(WNLIB_MAP_BASE + 0x0016)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define W5100_RTR0				(WNLIB_MAP_BASE + 0x0017)
/**
 @brief Retry count reigster
 */
#define W5100_RCR						(WNLIB_MAP_BASE + 0x0019)
/**
 @brief Receive memory size reigster
 */
#define W5100_RMSR			(WNLIB_MAP_BASE + 0x001A)
/**
 @brief Transmit memory size reigster
 */
#define W5100_TMSR			(WNLIB_MAP_BASE + 0x001B)
/**
 @brief Authentication type register address in PPPoE mode
 */
#define W5100_PATR0					(WNLIB_MAP_BASE + 0x001C)
//#define W5100_PPPALGO (WNLIB_MAP_BASE + 0x001D)
#define W5100_PTIMER (WNLIB_MAP_BASE + 0x0028)
#define W5100_PMAGIC (WNLIB_MAP_BASE + 0x0029)

/**
 @brief Unreachable IP register address in UDP mode
 */
#define W5100_UIPR0				(WNLIB_MAP_BASE + 0x002A)
/**
 @brief Unreachable Port register address in UDP mode
 */
#define W5100_UPORT0			(WNLIB_MAP_BASE + 0x002E)

/**
 @brief socket register
*/
#define CH_BASE (WNLIB_MAP_BASE + 0x0400)
/**
 @brief	size of each channel register map
 */
#define CH_SIZE		0x0100
/**
 @brief socket Mode register
 */
#define Sn_MR(ch)		(CH_BASE + ch * CH_SIZE + 0x0000)
/**
 @brief channel Sn_CR register
 */
#define Sn_CR(ch)       (CH_BASE + ch * CH_SIZE + 0x0001)
/**
 @brief channel interrupt register
 */
#define Sn_IR(ch)       (CH_BASE + ch * CH_SIZE + 0x0002)
/**
 @brief channel status register
 */
#define Sn_SR(ch)       (CH_BASE + ch * CH_SIZE + 0x0003)
/**
 @brief source port register
 */
#define Sn_PORT0(ch)    (CH_BASE + ch * CH_SIZE + 0x0004)
/**
 @brief Peer MAC register address
 */
#define Sn_DHAR0(ch)    (CH_BASE + ch * CH_SIZE + 0x0006)
/**
 @brief Peer IP register address
 */
#define Sn_DIPR0(ch)    (CH_BASE + ch * CH_SIZE + 0x000C)
/**
 @brief Peer port register address
 */
#define Sn_DPORT0(ch)   (CH_BASE + ch * CH_SIZE + 0x0010)
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define Sn_MSSR0(ch)    (CH_BASE + ch * CH_SIZE + 0x0012)
/**
 @brief Protocol of IP Header field register in IP raw mode
 */
#define Sn_PROTO(ch)    (CH_BASE + ch * CH_SIZE + 0x0014)

/** 
 @brief IP Type of Service(TOS) Register 
 */
#define Sn_TOS(ch)      (CH_BASE + ch * CH_SIZE + 0x0015)
/**
 @brief IP Time to live(TTL) Register 
 */
#define Sn_TTL(ch)      (CH_BASE + ch * CH_SIZE + 0x0016)

/**
 @brief Transmit free memory size register
 */
#define Sn_TX_FSR0(ch)	(CH_BASE + ch * CH_SIZE + 0x0020)
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0(ch)   (CH_BASE + ch * CH_SIZE + 0x0022)
/**
 @brief Transmit memory write pointer register address
 */
#define Sn_TX_WR0(ch)   (CH_BASE + ch * CH_SIZE + 0x0024)
/**
 @brief Received data size register
 */
#define Sn_RX_RSR0(ch)	(CH_BASE + ch * CH_SIZE + 0x0026)
/**
 @brief Read point of Receive memory
 */
#define Sn_RX_RD0(ch)   (CH_BASE + ch * CH_SIZE + 0x0028)
/**
 @brief Write point of Receive memory
 */
#define Sn_RX_WR0(ch)   (CH_BASE + ch * CH_SIZE + 0x002A)


/* MODE register values */
#define MR_RST			0x80 /**< reset */
#define MR_PB			0x10 /**< ping block */
#define MR_PPPOE		0x08 /**< enable pppoe */
#define MR_LB  		    0x04 /**< little or big endian selector in indirect mode */
#define MR_AI			0x02 /**< auto-increment in indirect mode */
#define MR_IND			0x01 /**< enable indirect mode */

/* IR register values */
#define IR_CONFLICT	    0x80 /**< check ip confict */
#define IR_UNREACH	    0x40 /**< get the destination unreachable message in UDP sending */
#define IR_PPPoE	    0x20 /**< get the PPPoE close message */
#define IR_SOCK(ch)	   (0x01 << ch) /**< check socket interrupt */

/* Sn_MR values */
#define Sn_MR_CLOSE		0x00		/**< unused socket */
#define Sn_MR_TCP		0x01		/**< TCP */
#define Sn_MR_UDP		0x02		/**< UDP */
#define Sn_MR_IPRAW	    0x03		/**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW	0x04		/**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE		0x05		/**< PPPoE */
#define Sn_MR_ND		0x20		/**< No Delayed Ack(TCP) flag */
#define Sn_MR_MULTI		0x80		/**< support multicating */


/* Sn_CR values */
#define Sn_CR_OPEN		0x01		/**< initialize or open socket */
#define Sn_CR_LISTEN	0x02		/**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT	0x04		/**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON	0x08		/**< send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE		0x10		/**< close socket */
#define Sn_CR_SEND		0x20		/**< updata txbuf pointer, send data */
#define Sn_CR_SEND_MAC	0x21		/**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP	0x22		/**<  send keep alive message */
#define Sn_CR_RECV		0x40		/**< update rxbuf pointer, recv data */

#if (WNLIB_PPP == ENABLED)
  #define Sn_CR_PCON				0x23		 
	#define Sn_CR_PDISCON			0x24		 
	#define Sn_CR_PCR					0x25		 
	#define Sn_CR_PCN					0x26		
	#define Sn_CR_PCJ					0x27		
#endif

/* Sn_IR values */
#if (WNLIB_PPP == ENABLED)
	#define Sn_IR_PRECV			0x80		
	#define Sn_IR_PFAIL			0x40		
	#define Sn_IR_PNEXT			0x20		
#endif
#define Sn_IR_SEND_OK			0x10		/**< complete sending */
#define Sn_IR_TIMEOUT			0x08		/**< assert timeout */
#define Sn_IR_RECV				0x04		/**< receiving data */
#define Sn_IR_DISCON				0x02		/**< closed socket */
#define Sn_IR_CON					0x01		/**< established connection */


/* IP PROTOCOL */
#define IPPROTO_IP              0           /**< Dummy for IP */
#define IPPROTO_ICMP            1           /**< Control message protocol */
#define IPPROTO_IGMP            2           /**< Internet group management protocol */
#define IPPROTO_GGP             3           /**< Gateway^2 (deprecated) */
#define IPPROTO_TCP             6           /**< TCP */
#define IPPROTO_PUP             12          /**< PUP */
#define IPPROTO_UDP             17          /**< UDP */
#define IPPROTO_IDP             22          /**< XNS idp */
#define IPPROTO_ND              77          /**< UNOFFICIAL net disk protocol */
#define IPPROTO_RAW             255         /**< Raw IP packet */

typedef enum{
	SOCKET_TYPE_CLOSED,
	SOCKET_TYPE_TCP,
	SOCKET_TYPE_UDP,
	SOCKET_TYPE_IPRAW,
	SOCKET_TYPE_MACRAW,
	SOCKET_TYPE_PPPOE
}socket_type_t;

typedef enum{
	SOCKET_STATUS_CLOSED	  = 0x00,		/**< closed */
	SOCKET_STATUS_INIT 		  =	0x13,		/**< init state */
	SOCKET_STATUS_LISTEN	  =	0x14,		/**< listen state */
	SOCKET_STATUS_SYNSENT	  = 0x15,		/**< connection state */
	SOCKET_STATUS_SYNRECV     = 0x16,		/**< connection state */
	SOCKET_STATUS_ESTABLISHED =	0x17,		/**< success to connect */
	SOCKET_STATUS_FIN_WAIT	  =	0x18,		/**< closing state */
	SOCKET_STATUS_CLOSING	  = 0x1A,		/**< closing state */
	SOCKET_STATUS_TIME_WAIT	  =	0x1B,		/**< closing state */
	SOCKET_STATUS_CLOSE_WAIT  =	0x1C,		/**< closing state */
	SOCKET_STATUS_LAST_ACK	  =	0x1D,		/**< closing state */
	SOCKET_STATUS_UDP		  = 0x22,		/**< udp SOCKET_STATUSet */
	SOCKET_STATUS_IPRAW		  = 0x32,		/**< ip raw mode SOCKET_STATUSet */
	SOCKET_STATUS_MACRAW	  =	0x42,		/**< mac raw mode SOCKET_STATUSet */
	SOCKET_STATUS_PPPOE		  =	0x5F		/**< pppoe socket */
}socket_status_t;

typedef enum{
	SOCKET_CMD_OPEN      = 0x01,		/**< initialize or open socket */
	SOCKET_CMD_LISTEN    = 0x02,		/**< wait connection request in tcp mode(Server mode) */
	SOCKET_CMD_CONNECT   = 0x04,		/**< send connection request in tcp mode(Client mode) */
	SOCKET_CMD_DISCON    = 0x08,		/**< send closing reqeuset in tcp mode */
	SOCKET_CMD_CLOSE     = 0x10,		/**< close socket */
	SOCKET_CMD_SEND	     = 0x20,		/**< updata txbuf pointer, send data */
	SOCKET_CMD_SEND_MAC	 = 0x21,		/**< send data with MAC address, so without ARP process */
	SOCKET_CMD_SEND_KEEP = 0x22,		/**<  send keep alive message */
	SOCKET_CMD_RECV	     = 0x40		    /**< update rxbuf pointer, recv data */
}socket_cmd_t;

typedef struct{
	uint16_t rx_base;
	uint16_t rx_size;
	uint16_t tx_base;
	uint16_t tx_size;
}socket_buf_t;

typedef enum{
	SOCKET_IR_SEND_OK = 0x10,		/**< complete sending */
	SOCKET_IR_TIMEOUT = 0x08,		/**< assert timeout */
	SOCKET_IR_RECV    = 0x04,		/**< receiving data */
	SOCKET_IR_DISCON  = 0x02,		/**< closed socket */
	SOCKET_IR_CON     = 0x01,
#if (WNLIB_PPP == ENABLED)
	SOCKET_IR_PRECV   = 0x80,
	SOCKET_IR_PFAIL   = 0x40,
	SOCKET_IR_PNEXT   = 0x20,
#endif
	SOCKET_IR_ALL     = 0xFF
}socket_ir_t;

//uint16_t txBufSize (uint8_t soc); // temporary

// PUBLIC FUNCTIONS
void   WNLIB_SetType(uint8_t soc_nr, socket_type_t type);
void   WNLIB_SetPort(uint8_t soc_nr, uint16_t port);
void   WNLIB_OpenSocket(uint8_t soc_nr);
void   WNLIB_InitChip();
void   WNLIB_SetIP(uint8_t* ip);
void   WNLIB_SetMAC(uint8_t* mac);
void   WNLIB_SetGateway(uint8_t* ip);
void   WNLIB_SetSubnetMask(uint8_t* ip);
void   WNLIB_Reset();
void   WNLIB_SetRetransmissionPolicy(uint16_t msec, uint8_t attempts);
socket_status_t WNLIB_SocketStatus(uint8_t soc_nr);
wnlib_error_t   WNLIB_SetBufferSize(uint8 tx_size, uint8 rx_size);
wnlib_error_t   WNLIB_SetCmd(uint8_t soc_nr, socket_cmd_t cmd);
wnlib_error_t   WNLIB_CheckIR(uint8_t soc_nr, socket_ir_t ir);
wnlib_error_t   WNLIB_WaitForReadiness(uint8_t soc_nr, uint32_t ticks);
void            WNLIB_ClearIR(uint8_t s, socket_ir_t ir);
uint16_t        WNLIB_BytesReceived(uint8_t soc_nr);
uint16_t 		WNLIB_GetBufRx(uint8_t soc_nr, uint8_t *data, uint16_t len);
uint16_t 		WNLIB_SetBufTx(uint8_t soc_nr, const uint8_t *data, uint16_t len);
uint16_t        WNLIB_FreeBufTx(uint8_t soc_nr, uint16_t size); // get socket TX free buf size
void 			WNLIB_SetDstIP(uint8_t soc_nr, uint8_t * ip);
void            WNLIB_DstIP(uint8_t soc_nr, uint8_t* ip);
void            WNLIB_SrcIP(uint8_t* ip);
void            WNLIB_DstPort(uint8_t soc_nr, uint8_t* port);
void            WNLIB_SetDstPort(uint8_t soc_nr, uint16_t port);
uint16_t        WINLIB_RxBufSize (uint8_t soc);
uint16_t        WINLIB_TxBufSize (uint8_t soc);


/*********************************************************
* Wiznet chip access function
*********************************************************/

uint8  getISR(uint8 s);
void   putISR(uint8 s, uint8 val);
void setRTR(uint16 timeout); // set retry duration for data transmission, connection, closing ...
void setRCR(uint8 retry); // set retry count (above the value, assert timeout interrupt)
void setIMR(uint8 mask); // set interrupt mask.
void getSIPR(uint8 * addr);
uint8 getIR( void );
void setSn_MSS(SOCKET s, uint16 Sn_MSSR0); // set maximum segment size
void setSn_PROTO(SOCKET s, uint8 proto); // set IP Protocol value using IP-Raw mode
void setSn_DHAR(SOCKET s, uint8 * addr);

void setSn_DPORT(SOCKET s, uint8 * addr);
void getSn_DHAR(SOCKET s, uint8 * addr);

void getSn_DPORT(SOCKET s, uint8 * addr);
void setSn_TTL(SOCKET s, uint8 ttl);
void setMR(uint8 val);

void clearSUBR(void);
void applySUBR(void);

#if (WNLIB_PPP == ENABLED)
uint8 pppinit(uint8 *id, uint8 idlen, uint8 *passwd, uint8 passwdlen);
uint8 pppterm(uint8 *mac,uint8 *sessionid);
#endif

void send_data_processing(SOCKET s, uint8 *data, uint16 len);
//void recv_data_processing(uint8_t socket_nr, uint8_t *data, uint16_t len);

//void read_data(SOCKET s, vuint8 * src, vuint8 * dst, uint16 len);
//void (SOCKET s, vuint8 * src, vuint8 * dst, uint16 len);

#endif
