/*
 * History:
 *
 * 01.09.2014 - Michal Budzon - Initial
 */

#include "socket.h"
#include "string.h"

/**
 @brief  This Socket function initialize the channel in perticular mode, and set the port and wait for W5100 done it.
 @return   1 for sucess else 0.
 */
uint8_t InitSockets() {
	WNLIB_InitSPI();
	WNLIB_InitChip(); // reset chip
	WNLIB_SetBufferSize(WNLIB_TXRXBUF_SIZE_8KB, WNLIB_TXRXBUF_SIZE_8KB);
	WNLIB_SetRetransmissionPolicy(WNLIB_DEFAULT_RETR_TIMEOUT,
			WNLIB_DEFAULT_RETR_ATTEMPTS);

	return WNLIB_ERROR_NONE;
}

uint8_t CreateSocket(uint8_t s, socket_type_t type, uint16_t port) {
	if (port == 0)
		return 1;
	Close(s);

	WNLIB_SetType(s, type);
	WNLIB_SetPort(s, port);
	WNLIB_OpenSocket(s);

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return 1;

	return 0;
}

/**
 @brief  This function established  the connection for the channel in passive (server) mode. This function waits for the request from the peer.
 @param socket_nr socket number.
 @return 0 for success else error_code.
 */
uint8 ListenTCP(uint8_t s) {
	while (WNLIB_SocketStatus(s) != SOCKET_STATUS_INIT) {
		WNLIB_OpenSocket(s);
		if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
				== WNLIB_ERROR_TIMEOUT)
			return 1;
	}

	WNLIB_SetCmd(s, SOCKET_CMD_LISTEN);

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return 3;

	return 0;
}

/**
 @brief  This function close the socket and parameter is "s" which represent the socket number
 */
uint8 Close(uint8_t s) {
	WNLIB_SetCmd(s, SOCKET_CMD_CLOSE);

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return 1;

#if (WNLIB_INTERRUPTS == ENABLED)
	/* m2008.01 [bj] : all clear */
	putISR(s, 0x00);
#endif
	WNLIB_ClearIR(s, SOCKET_IR_ALL);
	return 0;
}

/**
 @brief  This function used for disconnect the socket and parameter is "s" which represent the socket number
 @return 1 for success else 0.
 */
uint8_t Disconnect(uint8_t s) {
	WNLIB_SetCmd(s, SOCKET_CMD_DISCON);

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return 1;
	return 0;
}

uint8_t isConnected(uint8_t s) {
	return (uint8_t)(WNLIB_SocketStatus(s) == SOCKET_STATUS_ESTABLISHED);
}

/**
 @brief  This function used to send the data in TCP mode
 * @param s		the socket index
 * @param buf	a pointer to data
 * @param len	the data size to be send
 @return 1 for success else 0.
 */
uint16_t SendPacketTCP(uint8_t s, const uint8_t* data, uint16_t len) {
	uint16_t free_buf_size;
	uint32_t timeout = WNLIB_TXFREESIZE_TIMEOUT;

	if (WINLIB_TxBufSize(s) < len)
		return -1;

	if (!isConnected(s))
		return -2;

	/* first, check and wait for free TX memory size */
	while (1) {
		free_buf_size = WNLIB_FreeBufTx(s, len);
		if (free_buf_size >= len)
			break;
		if (timeout == 1)
			return -4;
		if (timeout > 1)
			--timeout;
	}

	if (free_buf_size < len)
		return -5;

	WNLIB_SetBufTx(s, data, len);

	WNLIB_SetCmd(s, SOCKET_CMD_SEND);

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return -6;
	/*
	 #if (WNLIB_INTERRUPTS == ENABLED)
	 while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
	 #else
	 while ((ReadReg(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK)
	 #endif
	 {
	 if (ReadReg(Sn_SR(s)) == SOCKET_STATUS_CLOSED) {
	 close(s);
	 return 0;
	 }
	 }

	 #if (WNLIB_INTERRUPTS == ENABLED)
	 putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
	 #else
	 WriteReg(Sn_IR(s), Sn_IR_SEND_OK);
	 #endif
	 return ret;*/

	return 0;
}

/**
 @brief  This function is an application I/F function which is used to send the data for other then TCP mode.
 Unlike TCP transmission, The peer's destination address and the port is needed.
 @param buf    pointer to the data
 @param len    data to send size
 @param addr   destination IP address
 @param port   destination port
 @return This function return send data size for success else -1.
 */
int32_t SendDatagramUDP(uint8_t s, const uint8_t *data, uint16_t len, uint8_t *addr, uint16_t port) {
	uint16_t free_buf_size;
	uint32_t timeout = WNLIB_TXFREESIZE_TIMEOUT;

	WNLIB_OpenSocket(s);

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return -1;

	if (((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00)
			&& (addr[3] == 0x00)) || (port == 0))
		return -2;

	if (WINLIB_TxBufSize(s) < len)
		return -3;

	/* first, check and wait for free TX memory size */
	while (1) {
		free_buf_size = WNLIB_FreeBufTx(s, len);
		if (free_buf_size >= len)
			break;
		if (timeout == 1)
			return -4;
		if (timeout > 1)
			--timeout;
	}

	if (free_buf_size < len)
		return -5;

	WNLIB_SetDstIP(s, addr);
	WNLIB_SetDstPort(s, port);
	WNLIB_SetBufTx(s, data, len);

	clearSUBR();
	WNLIB_SetCmd(s, SOCKET_CMD_SEND);

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return -6;

#if (WNLIB_INTERRUPTS == ENABLED)
	while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
	while (WNLIB_CheckIR(s, SOCKET_IR_SEND_OK) != 1)
#endif
	{
#if (WNLIB_INTERRUPTS == ENABLED)
		if (getISR(s) & Sn_IR_TIMEOUT)
#else
		if (WNLIB_CheckIR(s, SOCKET_IR_TIMEOUT) != 1)
#endif
				{
#if (WNLIB_INTERRUPTS == ENABLED)
			putISR(s, getISR(s) & ~(Sn_IR_SEND_OK | Sn_IR_TIMEOUT)); /* clear SEND_OK & TIMEOUT */
#else
			WNLIB_ClearIR(s, SOCKET_IR_SEND_OK | SOCKET_IR_TIMEOUT);
#endif
			return 0;
		}
	}
#if (WNLIB_INTERRUPTS == ENABLED)
	putISR(s, getISR(socket_nr) & (~Sn_IR_SEND_OK));
#else
	WNLIB_ClearIR(s, SOCKET_IR_SEND_OK);
#endif
	applySUBR();

	return (int32_t) len;
}

/**
 * @brief  This function is an application I/F function which is used to receive the data in TCP mode.
 * It continues to wait for data as much as the application wants to receive.
 * @param socket_nr socket index
 * @param buf       pointer to copy the data to be received
 * @param len       the data size to be read
 * @return 		   received data size
 */
int32_t ReceiveTCP(uint8_t s, uint8_t * buf, uint16_t max)
{
	uint16_t size;

	/* first, get the received size */
	size = WNLIB_BytesReceived(s);

	if (size == 0)	return size;

	if ((max > 0) && (max < size))	size = max;

	WNLIB_GetBufRx(s, buf, size);
	WNLIB_SetCmd(s, SOCKET_CMD_RECV); // confirm data received

	if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT)
			== WNLIB_ERROR_TIMEOUT)
		return -1;

	return size;
}

/**
 @brief  This function is an application I/F function which is used to receive the data in other then
 TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well.
 * @param s	  // the socket number
 * @param buf  // a pointer to copy the data to be received
 * @param max  // the maximum data size to read
 * @param addr // a pointer to store the peer's IP address
 * @param port // a pointer to store the peer's port number.
 @return This function return received data size for success else -1.
 */

int32_t ReceiveDatagramUDP(uint8_t s, uint8_t * buf, uint16_t max, uint8_t * addr, uint16_t *port)
{
	uint8_t  header_len = 8;
	uint8_t  internal_buf[max + header_len];
	uint16_t size = 0;
	uint16_t data_size;

	if (max == 0) return -1;

	/* first, get the received size */
	size = WNLIB_BytesReceived(s);

	if (size < 8) return -2;

	if ((max > 0) && ((max+header_len) < size))	size = max+header_len;

	WNLIB_GetBufRx(s, internal_buf, size); // sth wrong ?

	// read peer's IP address, port number.
	if(addr != 0){
		addr[0]  = internal_buf[0];
		addr[1]  = internal_buf[1];
		addr[2]  = internal_buf[2];
		addr[3]  = internal_buf[3];
	}

	if(port != 0){
		*port    = internal_buf[4];
		*port    = (*port << 8) + internal_buf[5];
	}

	data_size = internal_buf[6];
	data_size = (data_size << 8) + internal_buf[7];

	if(data_size != size-header_len) return -3;

	memcpy(buf, internal_buf+header_len, size-header_len);

	/*read_data(s, (uint8 *) ptr, buf, data_len); // data copy.
	ptr += data_len;

	WriteReg(Sn_RX_RD0(s), (uint8) ((ptr & 0xff00) >> 8));
	WriteReg((Sn_RX_RD0(s) + 1), (uint8) (ptr & 0x00ff));*/
#if 0 == 1
		case Sn_MR_IPRAW:
			read_data(s, (uint8 *) ptr, head, 0x06);
			ptr += 6;

			addr[0] = head[0];
			addr[1] = head[1];
			addr[2] = head[2];
			addr[3] = head[3];
			data_len = head[4];
			data_len = (data_len << 8) + head[5];

			read_data(s, (uint8 *) ptr, buf, data_len); // data copy.
			ptr += data_len;

			WriteReg(Sn_RX_RD0(s), (uint8) ((ptr & 0xff00) >> 8));
			WriteReg((Sn_RX_RD0(s) + 1), (uint8) (ptr & 0x00ff));
			break;
		case Sn_MR_MACRAW:
			read_data(s, (uint8*) ptr, head, 2);
			ptr += 2;
			data_len = head[0];
			data_len = (data_len << 8) + head[1] - 2;

			read_data(s, (uint8*) ptr, buf, data_len);
			ptr += data_len;
			WriteReg(Sn_RX_RD0(s), (uint8) ((ptr & 0xff00) >> 8));
			WriteReg((Sn_RX_RD0(s) + 1), (uint8) (ptr & 0x00ff));

			break;
#endif

	WNLIB_SetCmd(s, SOCKET_CMD_RECV); // confirm data received

	return size-header_len;
}
////////////////////////////////////////////////////////////////////////////////////

/**
 @brief  This function established  the connection for the channel in Active (client) mode.
 This function waits for the untill the connection is established.

 @return 1 for success else 0.
 
uint8 connect(SOCKET s, uint8 * addr, uint16 port) {
#if 1 == 0
	uint8 ret;
	if (((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF)
					&& (addr[3] == 0xFF))
			|| ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00)
					&& (addr[3] == 0x00)) || (port == 0x00)) {
		ret = 0;
#if (WNLIB_DEBUG == ENABLED)
		printf("Fail[invalid ip,port]\r\n");
#endif
	} else {
		ret = 1;
		// set destination IP
		WriteReg(Sn_DIPR0(s), addr[0]);
		WriteReg((Sn_DIPR0(s) + 1), addr[1]);
		WriteReg((Sn_DIPR0(s) + 2), addr[2]);
		WriteReg((Sn_DIPR0(s) + 3), addr[3]);
		WriteReg(Sn_DPORT0(s), (uint8) ((port & 0xff00) >> 8));
		WriteReg((Sn_DPORT0(s) + 1), (uint8) (port & 0x00ff));
		clearSUBR();
		WriteReg(Sn_CR(s), Sn_CR_CONNECT);
		while (ReadReg(Sn_CR(s)))
		;
		/*
		 // check the SYN packet sending...
		 while ( ReadReg(Sn_SR(s)) != SOCK_SYNSENT )
		 {
		 if(ReadReg(Sn_SR(s)) == SOCK_ESTABLISHED)
		 {
		 break;
		 }
		 if (getSn_IR(s) & Sn_IR_TIMEOUT)
		 {
		 setSn_IR(s,(Sn_IR_TIMEOUT ));  // clear TIMEOUT Interrupt
		 ret = 0;
		 break;
		 }
		 }
		 
		applySUBR();

	}

	return ret;
#endif
	return 0;
}

uint16 igmpsend(SOCKET s, const uint8 * buf, uint16 len) {
#if 1 == 0
	uint16 ret = 0;

	if (len > getTxBufferSize(s))
	ret = getTxBufferSize(s); // check size not to exceed MAX size.
	else
	ret = len;

	if (ret != 0){
		// copy data
		send_data_processing(s, (uint8 *) buf, ret);
		WriteReg(Sn_CR(s), Sn_CR_SEND);

		if (WNLIB_WaitForReadiness(s, SOCKET_READINESS_TIMEOUT) == WNLIB_ERROR_TIMEOUT) return;

#if (WNLIB_INTERRUPTS == ENABLED)
		while ( (getISR(s) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
#else
		while ((ReadReg(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK)
#endif
		{
			ReadReg(Sn_SR(s));
#if (WNLIB_INTERRUPTS == ENABLED)
			if (getISR(s) & Sn_IR_TIMEOUT)
#else
			if (ReadReg(Sn_IR(s)) & Sn_IR_TIMEOUT)
#endif
			{
				close(s);

				return 0;
			}
		}

#if (WNLIB_INTERRUPTS == ENABLED)
		putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
#else
		WriteReg(Sn_IR(s), Sn_IR_SEND_OK);
#endif
	}
	return ret;
#endif
	return 0;
}
*/
