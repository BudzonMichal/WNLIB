/*
 * History:
 *
 * 01.09.2014 - Michal Budzon - Initial
 */

#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>

#define	WNLIB_ENDIAN_LITTLE   	0	/**<  This must be defined if system is little-endian alignment */
#define	WNLIB_ENDIAN_BIG		1
#define WNLIB_ENDIANESS		    WNLIB_ENDIAN_LITTLE

#define WNLIB_WIZNET_MODULE WIZNET5100 /**< available: WIZNET5100, WIZNET5200, WIZNET 5300, WIZNET5500 */

#define	WNLIB_MAX_SOCKETS		4	/**< Maxmium number of socket  */

#ifndef ENABLED
	#define ENABLED  1
	#define DISABLED !ENABLED
#endif

#define WNLIB_DEBUG      DISABLED
#define WNLIB_INTERRUPTS DISABLED /**< involve interrupt service routine (socket.c) */
#define WNLIB_PPP        DISABLED/* involve pppoe routine (socket.c) */
                            /* If it is defined, the source files(md5.h,md5.c) must be included in your project.
                               Otherwize, the source files must be removed in your project. */

#define WNLIB_BUS_SPI 1
#define WNLIB_BUS_TYPE WNLIB_BUS_SPI /*Enable SPI_mode*/

#define WNLIB_MAP_BASE 0x0000

#define WNLIB_MAP_TXBUF (WNLIB_MAP_BASE + 0x4000) /* Internal Tx buffer address of the iinchip */
#define WNLIB_MAP_RXBUF (WNLIB_MAP_BASE + 0x6000) /* Internal Rx buffer address of the iinchip */

#define WNLIB_WRITE_CMD 0xF0
#define WNLIB_READ_CMD  0x0F

#ifndef LSB
#define LSB(x) ((x) & 0x00FF)
#endif

#ifndef MSB
#define MSB(x) (((x) & 0xFF00) >> 8)
#endif

#define WNLIB_BROADCAST_IP	  {255,255,255,255}
#define WNLIB_SRC_IP		  {192,168,0,69}
#define WNLIB_MAC			  {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03}
#define WNLIB_GATE_IP		  {192,168,0,50}
#define WNLIB_SUBNET_MASK	  {255,255,255,0}

#if (WNLIB_INTERRUPTS == ENABLED)
		// iinchip use external interrupt 4
		#define WNLIB_ISR_DISABLE()	(EIMSK &= ~(0x10))
		#define WNLIB_ISR_ENABLE()	(EIMSK |= 0x10)
		#define WNLIB_ISR_GET(X)		(X = EIMSK)
		#define WNLIB_ISR_SET(X)		(EIMSK = X)
#else
		#define WNLIB_ISR_DISABLE()
		#define WNLIB_ISR_ENABLE()	
		#define WNLIB_ISR_GET(X)
		#define WNLIB_MAP_TXBUFISR_SET(X)
#endif

#ifndef NULL
#define NULL		((void *) 0)
#endif

typedef enum { false, true } bool;

typedef enum{
	WNLIB_ERROR_NONE,
	WNLIB_ERROR_NULL,
	WNLIB_ERROR_NACK,
	WNLIB_ERROR_PARAMS,
	WNLIB_ERROR_TIMEOUT,
	WNLIB_ERROR_UNKNOWN,
	WNLIB_ERROR_UNSUPPORTED
}wnlib_error_t;

#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned int size_t;
#endif

/**
 * The 8-bit signed data type.
 */
typedef uint8_t int8;
/**
 * The volatile 8-bit signed data type.
 */
typedef volatile char vint8;
/**
 * The 8-bit unsigned data type.
 */
typedef unsigned char uint8;
/**
 * The volatile 8-bit unsigned data type.
 */
typedef volatile unsigned char vuint8;

/**
 * The 16-bit signed data type.
 */
typedef int16_t int16;
/**
 * The volatile 16-bit signed data type.
 */
typedef volatile int vint16;
/**
 * The 16-bit unsigned data type.
 */
typedef unsigned int uint16;
/**
 * The volatile 16-bit unsigned data type.
 */
typedef volatile unsigned int vuint16;
/**
 * The 32-bit signed data type.
 */
typedef long int32;
/**
 * The volatile 32-bit signed data type.
 */
typedef volatile long vint32;
/**
 * The 32-bit unsigned data type.
 */
typedef unsigned long uint32;
/**
 * The volatile 32-bit unsigned data type.
 */
typedef volatile unsigned long vuint32;

/* bsd */
typedef uint8			u_char;		/**< 8-bit value */
typedef uint8 			SOCKET;
typedef uint16			wnlib_short;	/**< 16-bit value */
typedef uint16			u_int;		/**< 16-bit value */
typedef uint32			u_long;		/**< 32-bit value */

typedef union _un_l2cval {
	u_long	lVal;
	u_char	cVal[4];
}un_l2cval;

typedef union _un_i2cval {
	u_int	iVal;
	u_char	cVal[2];
}un_i2cval;


/** global define */
#define FW_VERSION		0x01010000	/* System F/W Version : 1.1.0.0	*/
#define HW_VERSION	       0x01000000


#define TX_RX_MAX_BUF_SIZE	2048
#define TX_BUF	0x1100
#define RX_BUF	(TX_BUF+TX_RX_MAX_BUF_SIZE)

#define UART_DEVICE_CNT		1	/**< UART device number */
/* #define SUPPORT_UART_ONE */

#endif		/* _TYPES_H_ */
