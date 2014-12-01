/*
*
* 1.09.2014 - Initial, Michal Budzon
*
*/

#ifndef	SOCKET_H_
#define	SOCKET_H_

#if (WNLIB_WIZNET_MODULE == WIZNET5100)
	#include "w5100.h"
#elif (WNLIB_WIZNET_MODULE == WIZNET5200)
	#include "w5200.h"
#elif (WNLIB_WIZNET_MODULE == WIZNET5300)
	#include "w5300.h"
#elif (WNLIB_WIZNET_MODULE == WIZNET5500)
	#include "w5500.h"
#endif

#define SOCKET_READINESS_TIMEOUT 10000000

uint8_t  InitSockets(void);
uint8_t  CreateSocket(uint8_t soc_nr, socket_type_t type, uint16_t port); // Set and open socket
int32_t  SendDatagramUDP(uint8_t soc_nr, const uint8_t * buf, uint16_t len, uint8_t *addr, uint16_t port); // Send datagram UDP
uint8_t  ListenTCP(uint8_t soc_nr);	// Establish TCP connection (Passive connection)
uint16_t SendPacketTCP(uint8_t s, const uint8_t* data, uint16_t len);
uint8_t  isConnected(uint8_t soc_nr);
int32_t  ReceiveTCP(uint8_t soc_nr, uint8_t * buf, uint16_t len);	// Receive data (TCP)
uint8_t  Close(uint8_t soc_nr); // Close socket
uint8_t  Disconnect(uint8_t soc_nr); // disconnect the connection

uint8    connect(uint8_t soc_nr, uint8 * addr, uint16 port); // Establish TCP connection (Active connection)

int32_t ReceiveDatagramUDP(uint8_t soc_nr, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port);
uint16  igmpsend(uint8_t soc_nr, const uint8 * buf, uint16 len);
#endif // SOCKET_H_
