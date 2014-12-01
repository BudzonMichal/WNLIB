/*
 * History:
 *
 * 01.09.2014 - Michal Budzon - Initial
 */
 
#include <stdio.h>
#include <string.h>
   
#include "w5100.h"

#if (WNLIB_PPP == ENABLED)
   #include "md5.h"
#endif

socket_buf_t socket_buffer[WNLIB_MAX_SOCKETS];

uint16_t WINLIB_RxBufSize (uint8_t soc) { return socket_buffer[soc].rx_size; }
uint16_t WINLIB_TxBufSize (uint8_t soc) { return socket_buffer[soc].tx_size; }
static uint16_t rxBaseAddr(uint8_t soc) { return socket_buffer[soc].rx_base; }
static uint16_t txBaseAddr(uint8_t soc) { return socket_buffer[soc].tx_base; }
static uint16_t rxBufMask (uint8_t soc) { return socket_buffer[soc].rx_size - 1; }
static uint16_t txBufMask (uint8_t soc) { return socket_buffer[soc].tx_size - 1; }

static uint8  I_STATUS[WNLIB_MAX_SOCKETS];
static uint8  SUBN_VAR[4];
static uint8  IP_VAR[4];

uint8  getISR(uint8 s)           { return I_STATUS[s]; }
void   putISR(uint8 s, uint8 val){ I_STATUS[s] = val; }

// STATIC FUNCTIONS
static uint8_t  Write32(uint16_t addr, uint8_t *data);
static uint16_t Read16(uint16_t addr);
static void     Write16(uint16_t addr, uint16_t data);
static uint8    Write32(uint16_t addr, uint8_t *data);
static uint8    ReadReg(uint16_t addr);
static uint8    WriteReg(uint16_t addr,uint8_t data);
static uint16_t ReadData (uint16_t addr, uint8_t* data, uint16_t len);
static uint16_t WriteData(uint16_t addr, const uint8_t* data, uint16_t len);
static uint16_t rxMemory(uint8_t soc);
static uint16_t txMemory(uint8_t soc);
static void     setTxMemory(uint8_t soc, uint16_t addr);
static void     setRxMemory(uint8_t soc, uint16_t addr);
/**
@brief	It sets up SubnetMask address
@param  a pointer to a 4 -byte array responsible to set the SubnetMask address
*/
void setSUBR(uint8_t * addr)
{
	SUBN_VAR[0] = addr[0];
	SUBN_VAR[1] = addr[1];
	SUBN_VAR[2] = addr[2];
	SUBN_VAR[3] = addr[3];
}

/**
@brief	It sets up SubnetMask address
@param  a pointer to a 4 -byte array responsible to set the SubnetMask addres
*/
void applySUBR(void)
{
	Write32(W5100_SUBR0, SUBN_VAR);
}

void WNLIB_SetType(uint8_t socket_nr, socket_type_t type)
{
	WriteReg(Sn_MR(socket_nr), (uint8_t)type);
}

/**
@brief	This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/
void WNLIB_Reset()
{
	setMR(MR_RST);
}

/**
@brief	This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/
void WNLIB_SetIP(uint8_t* ip)
{
	Write32(W5100_SIPR0, ip);
}

/**
@brief	This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/
void WNLIB_SetMAC(uint8_t* mac)
{
	WriteData(W5100_SHAR0, mac, 6);
}

/**
@brief	This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/
void WNLIB_SetGateway(uint8_t* ip)
{
	Write32(W5100_GAR0, ip);
}

/**
@brief	This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/
void WNLIB_SetSubnetMask(uint8_t* ip)
{
	setSUBR(ip);
	applySUBR();
}

void WNLIB_SetRetransmissionPolicy(uint16_t msec, uint8_t attempts)
{
	uint16_t units = msec * 10;

	WriteReg(W5100_RTR0, MSB(units));
	WriteReg((W5100_RTR0 + 1),LSB(units));

	WriteReg(W5100_RCR, attempts);
}

socket_status_t WNLIB_SocketStatus(uint8_t socket_nr)
{
	return ReadReg(Sn_SR(socket_nr));
}

wnlib_error_t WNLIB_SetCmd(uint8_t socket_nr, socket_cmd_t cmd)
{
	WriteReg(Sn_CR(socket_nr), (uint8_t)cmd);
	return WNLIB_ERROR_NONE;
}

wnlib_error_t WNLIB_WaitForReadiness(uint8_t socket_nr, uint32_t ticks)
{
	while( ReadReg(Sn_CR(socket_nr)) ){ // wait for readiness
		if(ticks == 1) return WNLIB_ERROR_TIMEOUT;
		if(ticks != 0) --ticks;
	}

	return WNLIB_ERROR_NONE;
}

/**
@brief	This function is for initializing the iinchip
*/
void WNLIB_InitChip()
{
	uint8_t ip[4]   = WNLIB_SRC_IP;
	uint8_t mac[6]  = WNLIB_MAC;
	uint8_t gate[4] = WNLIB_GATE_IP;
	uint8_t mask[4] = WNLIB_SUBNET_MASK;

	WNLIB_Reset();
	WNLIB_SetIP(ip);
	WNLIB_SetMAC(mac);
	WNLIB_SetGateway(gate);
	WNLIB_SetSubnetMask(mask);
}

void WNLIB_SetPort(uint8_t socket_nr, uint16_t port)
{
	WriteReg(Sn_PORT0(socket_nr)    , MSB(port));
	WriteReg(Sn_PORT0(socket_nr) + 1, LSB(port));
}

void WNLIB_OpenSocket(uint8_t socket_nr)
{
	WriteReg(Sn_CR(socket_nr) , Sn_CR_OPEN);
}

uint8_t WNLIB_CheckIR(uint8_t s, socket_ir_t ir)
{
	return (ReadReg(Sn_IR(s)) & ir);
}

void WNLIB_ClearIR(uint8_t s, socket_ir_t ir)
{
	WriteReg(Sn_IR(s), ir);
}

/**
@brief	This function is being called by recv() also.

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
/*void recv_data_processing(uint8_t s, uint8_t *data, uint16_t len)
{
	uint16 ptr;
	ptr = ReadReg(Sn_RX_RD0(s));
	ptr = ((ptr & 0x00ff) << 8) + ReadReg(Sn_RX_RD0(s) + 1);
#ifdef __DEF_IINCHIP_DBG__
	printf("ISR_RX: rd_ptr : %.4x\r\n", ptr);
#endif
	read_data(s, (uint8 *)ptr, data, len); // read data
	ptr += len;
	WriteReg(Sn_RX_RD0(s),(uint8)((ptr & 0xff00) >> 8));
	WriteReg((Sn_RX_RD0(s) + 1),(uint8)(ptr & 0x00ff));
}*/

static uint16_t rxMemory(uint8_t soc)
{
	uint16_t addr;

	addr = ReadReg(Sn_RX_RD0(soc)); // Get
	addr = (LSB(addr) << 8);
	addr += ReadReg(Sn_RX_RD0(soc) + 1);

	return addr;
}

static uint16_t txMemory(uint8_t soc)
{
	uint16_t addr;

	addr = ReadReg(Sn_TX_WR0(soc)); // Get
	addr = (LSB(addr) << 8);
	addr += ReadReg(Sn_TX_WR0(soc) + 1);

	return addr;
}

static void setRxMemory(uint8_t soc,uint16_t addr)
{
	WriteReg(Sn_RX_RD0(soc),     MSB(addr));
	WriteReg(Sn_RX_RD0(soc) + 1, LSB(addr));
}

static void setTxMemory(uint8_t soc, uint16_t addr)
{
	WriteReg(Sn_TX_WR0(soc),     MSB(addr));
	WriteReg(Sn_TX_WR0(soc) + 1, LSB(addr));
}

uint16_t WNLIB_GetBufRx(uint8_t s, uint8_t *data, uint16_t len)
{
	uint16_t start_ptr;
	uint16_t read_ptr;
	uint16_t offset;
	uint16_t upper_size;
	uint16_t left_size;

	read_ptr = rxMemory(s);

	offset = read_ptr & rxBufMask(s);        /* calculate offset address */

	start_ptr = rxBaseAddr(s) + offset;   	/* calculate start address(physical address) */

	/* if overflow socket RX memory */
	if(offset + len > WINLIB_RxBufSize(s)){
		upper_size = WINLIB_RxBufSize(s) - offset; /* copy upper_size bytes of get_start_address to destination_addr */
		ReadData(start_ptr, data, upper_size);
		data += upper_size;                        /* update destination_addr*/
		left_size = len - upper_size; /* copy left_size bytes of gSn_RX_BASE to destination_addr */
		ReadData(rxBaseAddr(s), data, left_size);
	}
	else{
		ReadData(start_ptr, data, len);
	}

	read_ptr += len;

	setRxMemory(s, read_ptr);

	return len;
}

uint16_t WNLIB_SetBufTx(uint8_t s, const uint8_t *data, uint16_t len)
{
	uint16_t start_ptr;
	uint16_t write_ptr;
	uint16_t offset;
	uint16_t upper_size;
	uint16_t left_size;

	write_ptr = txMemory(s);

	offset = write_ptr & txBufMask(s); /* calculate offset address */

	start_ptr = txBaseAddr(s) + offset; /* calculate start address(physical address) */

	/* if overflow socket RX memory */
	if(offset + len > WINLIB_TxBufSize(s)){
		upper_size = WINLIB_TxBufSize(s) - offset;   /* copy upper_size bytes of get_start_address to destination_addr */
		WriteData(start_ptr, data, upper_size);
		data += upper_size;                         /* update destination_addr*/
		left_size = len - upper_size;               /* copy left_size bytes of gSn_RX_BASE to destination_addr */
		WriteData(txBaseAddr(s), data, left_size);
	}
	else{
		WriteData(start_ptr, data, len);
	}

	write_ptr += len;
	setTxMemory(s, write_ptr);

	return len;
}
/**
@brief	This function set the transmit & receive buffer size as per the channels is used

Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n\n
Maximum memory size for Tx, Rx in the W5100 is 8K Bytes,\n
In the range of 8KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and receiption from non-allocated channel may cause some problems.\n
If the 8KBytes memory is already  assigned to centain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
tx_size Tx memory size (00 - 1KByte, 01- 2KBtye, 10 - 4KByte, 11 - 8KByte).\n
rx_size Rx memory size (00 - 1KByte, 01- 2KBtye, 10 - 4KByte, 11 - 8KByte). \n
*/
wnlib_error_t WNLIB_SetBufferSize(uint8_t tx_size, uint8_t rx_size)
{
	int16 i;
	int16 ssum, rsum;

	ssum = 0;
	rsum = 0;

	WriteReg(W5100_TMSR, tx_size); /* Set Tx memory size for each channel */
	WriteReg(W5100_RMSR, rx_size);	 /* Set Rx memory size for each channel */

	socket_buffer[0].tx_base = (uint16)(WNLIB_MAP_TXBUF);		/* Set base address of Tx memory for channel #0 */
	socket_buffer[0].rx_base = (uint16)(WNLIB_MAP_RXBUF);		/* Set base address of Rx memory for channel #0 */

    for (i = 0 ; i < WNLIB_MAX_SOCKETS; i++){       // Set the size, masking and base address of Tx & Rx memory by each channel
	    socket_buffer[i].tx_size = (int16)(0);
		socket_buffer[i].rx_size = (int16)(0);
		if (ssum < 8192)
		{
         switch((tx_size >> i*2) & 0x03)  // Set Tx memory size
			{
			case 0:
				socket_buffer[i].tx_size = (int16)(1024);
				break;
			case 1:
				socket_buffer[i].tx_size = (int16)(2048);
				break;
			case 2:
				socket_buffer[i].tx_size = (int16)(4096);
				break;
			case 3:
				socket_buffer[i].tx_size = (int16)(8192);
				break;
			}
		}
		if (rsum < 8192){
         switch((rx_size >> i*2) & 0x03)     // Set Rx memory size
			{
			case 0:
				socket_buffer[i].rx_size = (int16)(1024);
				break;
			case 1:
				socket_buffer[i].rx_size = (int16)(2048);
				break;
			case 2:
				socket_buffer[i].rx_size = (int16)(4096);
				break;
			case 3:
				socket_buffer[i].rx_size = (int16)(8192);
				break;
			}
		}
		ssum += WINLIB_TxBufSize(i);
		rsum += WINLIB_RxBufSize(i);

      if (i != 0){             // Sets base address of Tx and Rx memory for channel #1,#2,#3
    	  socket_buffer[i].tx_base = socket_buffer[i-1].tx_base + WINLIB_TxBufSize(i-1);
    	  socket_buffer[i].rx_base = socket_buffer[i-1].rx_base + WINLIB_RxBufSize(i-1);
		}
	}

   return WNLIB_ERROR_NONE;
}

 /**
@brief	This function writes the data into W5100 registers.
*/
static uint8 WriteReg(uint16_t addr, uint8_t data)
{
#if (WNLIB_BUS_TYPE == WNLIB_BUS_SPI)

	WNLIB_ISR_DISABLE();
	WNLIB_SpiActive();                             // CS=0, SPI start

	WNLIB_SpiSendByte(WNLIB_WRITE_CMD);
	WNLIB_SpiSendByte(MSB(addr));
	WNLIB_SpiSendByte(LSB(addr));
	WNLIB_SpiSendByte(data);

	WNLIB_SpiInactive();    

    WNLIB_ISR_ENABLE();   
#endif
	return 1;
}

/**
@brief	This function writes word into W5100 registers.
*/
static uint8 Write32(uint16_t addr, uint8_t *data)
{
	WriteData(addr, data, 4);
	return 1;
}

static uint16_t Read16(uint16_t addr)
{
	uint16_t ret;

	ret = ReadReg(addr);
	ret = (ret << 8) + ReadReg(addr + 1);

	return ret;
}

static void Write16(uint16_t addr, uint16_t data)
{
	WriteReg(addr,     MSB(data));
	WriteReg(addr + 1, LSB(data));
}


/**
@brief	This function reads the value from W5100 registers.
*/
static uint8 ReadReg(uint16_t addr)
{
#if (WNLIB_BUS_TYPE == WNLIB_BUS_SPI)

	uint8 data;

	WNLIB_ISR_DISABLE();
	WNLIB_SpiActive();                             // CS=0, SPI start

	WNLIB_SpiSendByte(WNLIB_READ_CMD);
	WNLIB_SpiSendByte((addr & 0xFF00) >> 8);
	WNLIB_SpiSendByte(addr & 0x00FF);
	data = WNLIB_SpiGetByte();

	WNLIB_SpiInactive();                          	// SPI end
    WNLIB_ISR_ENABLE();
#endif
	return data;
}


/**
@brief	This function writes into W5100 memory(Buffer)
*/
static uint16_t WriteData(uint16_t addr, const uint8_t* data, uint16_t len)
{
#if (WNLIB_BUS_TYPE == WNLIB_BUS_SPI)
	uint16_t i = 0;

	WNLIB_ISR_DISABLE();

	for(i = 0 ; i<len ; i++)
	   {
		WNLIB_SpiActive();                             // CS=0, SPI start

		WNLIB_SpiSendByte(WNLIB_WRITE_CMD);
		WNLIB_SpiSendByte(MSB(addr + i));
		WNLIB_SpiSendByte(LSB(addr + i));
		WNLIB_SpiSendByte(data[i]);

		WNLIB_SpiInactive();                             // CS=0, SPI end
	   }

	WNLIB_ISR_ENABLE();
#endif
	return len;
}


/**
@brief	This function reads into W5100 memory
*/
static uint16_t ReadData(uint16_t addr, uint8_t* data, uint16_t len)
{
	uint16_t i = 0;
#if (WNLIB_BUS_TYPE == WNLIB_BUS_SPI)
	WNLIB_ISR_DISABLE();

	for (i = 0; i < len; i++){
		WNLIB_SpiActive();                             // CS=0, SPI start

		WNLIB_SpiSendByte(WNLIB_READ_CMD);
		WNLIB_SpiSendByte(MSB(addr + i));
		WNLIB_SpiSendByte(LSB(addr + i));

		data[i] = WNLIB_SpiGetByte();

		WNLIB_SpiInactive();                             // CS=0, SPI end
	}

	WNLIB_ISR_ENABLE();
#endif
	return len;
}

void WNLIB_SetDstIP(uint8_t s, uint8_t * ip)
{
	Write32(Sn_DIPR0(s), ip);
}

void WNLIB_DstIP(uint8_t s, uint8_t* ip)
{
	ReadData(Sn_DIPR0(s), ip, 4);
}

void WNLIB_DstPort(uint8_t s, uint8_t* port)
{
	ReadData(Sn_DPORT0(s), port, 2);
}

void WNLIB_SetDstPort(uint8_t s, uint16_t port)
{
	Write16(Sn_DPORT0(s), port);
}

/**
@brief	get socket TX free buf size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User shuold check this value first and control the size of transmitting data
*/
uint16_t WNLIB_FreeBufTx(uint8_t s, uint16_t size)
{
	return Read16(Sn_TX_FSR0(s));
}

/**
@brief	 get socket RX recv buf size

This gives size of received data in receive buffer.
*/
uint16_t WNLIB_BytesReceived(uint8_t s)
{
   return Read16(Sn_RX_RSR0(s));
}

////////////////////////////////////////////////////////////////////////////////////////////
/**
@brief	Socket interrupt routine
*/ 
#if (WNLIB_INTERRUPTS == ENABLED)
ISR(INT4_vect)
{
	uint8 int_val;
	WNLIB_ISR_DISABLE();
	int_val = ReadReg(IR);
	
	/* +200801[bj] process all of interupt */
   do {
   /*---*/
   
   	if (int_val & IR_CONFLICT)
   	{
   		printf("IP conflict : %.2x\r\n", int_val);
   	}
   	if (int_val & IR_UNREACH)
   	{
   		printf("INT Port Unreachable : %.2x\r\n", int_val);
   		printf("UIPR0 : %d.%d.%d.%d\r\n", ReadReg(UIPR0), ReadReg(UIPR0+1), ReadReg(UIPR0+2), ReadReg(UIPR0+3));
   		printf("UPORT0 : %.2x %.2x\r\n", ReadReg(UPORT0), ReadReg(UPORT0+1));
   	}
   
   	/* +200801[bj] interrupt clear */
   	WriteReg(IR, 0xf0); 
      /*---*/
   
   	if (int_val & IR_SOCK(0))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[0] |= ReadReg(Sn_IR(0)); // can be come to over two times.
   		WriteReg(Sn_IR(0), I_STATUS[0]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(1))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[1] |= ReadReg(Sn_IR(1));
   		WriteReg(Sn_IR(1), I_STATUS[1]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(2))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[2] |= ReadReg(Sn_IR(2));
   		WriteReg(Sn_IR(2), I_STATUS[2]);
      /*---*/
   	}
   	if (int_val & IR_SOCK(3))
   	{
   	/* +-200801[bj] save interrupt value*/
   		I_STATUS[3] |= ReadReg(Sn_IR(3));
   		WriteReg(Sn_IR(3), I_STATUS[3]);
      /*---*/
   	}
   
   	/* +-200801[bj] re-read interrupt value*/
   	int_val = ReadReg(IR);

	/* +200801[bj] if exist, contiue to process */
   } while (int_val != 0x00);
   /*---*/

	WNLIB_ISR_ENABLE();
}
#endif

void setMR(uint8 val)
{

#if (WNLIB_BUS_TYPE == WNLIB_BUS_SPI)
	/* 	DIRECT ACCESS	*/
	WriteReg(W5100_MR,val);
#endif	
}


void getGWIP(uint8 * addr)
{
	addr[0] = ReadReg((W5100_GAR0 + 0));
	addr[1] = ReadReg((W5100_GAR0 + 1));
	addr[2] = ReadReg((W5100_GAR0 + 2));
	addr[3] = ReadReg((W5100_GAR0 + 3));
}

/**
@brief	It sets up SubnetMask address
*/ 
void clearSUBR(
	void	/**< a pointer to a 4 -byte array responsible to set the SubnetMask address */
	)
{
	
	WNLIB_SrcIP((uint8*)&IP_VAR);
	
	if((IP_VAR[0]==0 && IP_VAR[1]==0 &&IP_VAR[2]==0 &&IP_VAR[3]==0))
	{
		WriteReg((W5100_SUBR0 + 0), 0);
		WriteReg((W5100_SUBR0 + 1), 0);
		WriteReg((W5100_SUBR0 + 2), 0);
		WriteReg((W5100_SUBR0 + 3), 0);
	}

}


/**
@brief	This function gets Interrupt register in common register.
 */
uint8 getIR( void )
{
   return ReadReg(W5100_IR);
}



/**
@brief	This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission 
will be there as per RTR (Retry Time-value Register)setting
*/



/**
@brief	This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void setIMR(uint8 mask)
{
	WriteReg(W5100_IMR,mask); // must be setted 0x10.
}


/**
@brief	These below functions are used to get the Gateway, SubnetMask
		and Source Hardware Address (MAC Address) and Source IP address
*/
void WNLIB_SrcIP(uint8_t* ip)
{
	ReadData(W5100_SIPR0, ip, 4);
}


/**
@brief	These below functions are used to get the Destination Hardware Address (MAC Address), Destination IP address and Destination Port.
*/
void getSn_DHAR(SOCKET s, uint8 * addr)
{
	addr[0] = ReadReg(Sn_DHAR0(s));
	addr[1] = ReadReg(Sn_DHAR0(s)+1);
	addr[2] = ReadReg(Sn_DHAR0(s)+2);
	addr[3] = ReadReg(Sn_DHAR0(s)+3);
	addr[4] = ReadReg(Sn_DHAR0(s)+4);
	addr[5] = ReadReg(Sn_DHAR0(s)+5);
}
void setSn_DHAR(SOCKET s, uint8 * addr)
{
	WriteReg((Sn_DHAR0(s) + 0),addr[0]);
	WriteReg((Sn_DHAR0(s) + 1),addr[1]);
	WriteReg((Sn_DHAR0(s) + 2),addr[2]);
	WriteReg((Sn_DHAR0(s) + 3),addr[3]);
	WriteReg((Sn_DHAR0(s) + 4),addr[4]);
	WriteReg((Sn_DHAR0(s) + 5),addr[5]);
}


/**
@brief	This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16 Sn_MSSR0)
{
	WriteReg(Sn_MSSR0(s),(uint8)((Sn_MSSR0 & 0xff00) >> 8));
	WriteReg((Sn_MSSR0(s) + 1),(uint8)(Sn_MSSR0 & 0x00ff));
}

void setSn_TTL(SOCKET s, uint8 ttl)
{
   WriteReg(Sn_TTL(s), ttl);
}


/**
@brief	These below function is used to setup the Protocol Field of IP Header when
		executing the IP Layer RAW mode.
*/
void setSn_PROTO(SOCKET s, uint8 proto)
{
	WriteReg(Sn_PROTO(s),proto);
}

#if (WNLIB_PPP == ENABLED)
#define PPP_OPTION_BUF_LEN 64

uint8 pppinit_in(uint8 * id, uint8 idlen, uint8 * passwd, uint8 passwdlen);


/**
@brief	make PPPoE connection
@return	1 => success to connect, 2 => Auth fail, 3 => timeout, 4 => Auth type not support

*/
uint8 pppinit(uint8 * id, uint8 idlen, uint8 * passwd, uint8 passwdlen)
{
	uint8 ret;
	uint8 isr;
	
	// PHASE0. W5100 PPPoE(ADSL) setup
	// enable pppoe mode
	printf("-- PHASE 0. W5100 PPPoE(ADSL) setup process --\r\n");
	printf("\r\n");
	WriteReg(MR,ReadReg(MR) | MR_PPPOE);

	// open socket in pppoe mode
	isr = ReadReg(Sn_IR(0));// first clear isr(0), W5100 at present time
	WriteReg(Sn_IR(0),isr);
	
	WriteReg(PTIMER,200); // 5sec timeout
	WriteReg(PMAGIC,0x01); // magic number
	WriteReg(Sn_MR(0),Sn_MR_PPPOE);
	WriteReg(Sn_CR(0),Sn_CR_OPEN);
	
	/* +20071122[chungs]:wait to process the command... */
	while( ReadReg(Sn_CR(0)) ) 
		;
	/* ------- */
	
	ret = pppinit_in(id, idlen, passwd, passwdlen);

	// close ppp connection socket
	/* +200801 (hwkim) */
	close(0);
	/* ------- */
	
	return ret;
}


uint8 pppinit_in(uint8 * id, uint8 idlen, uint8 * passwd, uint8 passwdlen)
{
	uint8 i = 0;
	uint8 loop_idx = 0;
	uint8 isr = 0;
	uint8 buf[PPP_OPTION_BUF_LEN];
	uint16 len;
	uint8 str[PPP_OPTION_BUF_LEN];
	uint8 str_idx,dst_idx;

   // PHASE1. PPPoE Discovery
	// start to connect pppoe connection
	printf("-- PHASE 1. PPPoE Discovery process --");
	printf(" ok\r\n");
	printf("\r\n");
	WriteReg(Sn_CR(0),Sn_CR_PCON);
	/* +20071122[chungs]:wait to process the command... */
	while( ReadReg(Sn_CR(0)) ) 
		;
	/* ------- */
	
	wait_10ms(100);

	loop_idx = 0;
	//check whether PPPoE discovery end or not
	while (!(ReadReg(Sn_IR(0)) & Sn_IR_PNEXT))
	{
		printf(".");
		if (loop_idx++ == 10) // timeout
		{
			printf("timeout before LCP\r\n"); 
			return 3;
		}
		wait_10ms(100);
	}

   /* +200801[bj] clear interrupt value*/
   WriteReg(Sn_IR(0), 0xff);
   /*---*/

   // PHASE2. LCP process
	printf("-- PHASE 2. LCP process --");
		
	// send LCP Request
	{
		// Magic number option
		// option format (type value + length value + data)
	   // write magic number value
		buf[0] = 0x05; // type value
		buf[1] = 0x06; // length value
		buf[2] = 0x01; buf[3] = 0x01; buf[4] = 0x01; buf[5]= 0x01; // data
		// for MRU option, 1492 0x05d4  
		// buf[6] = 0x01; buf[7] = 0x04; buf[8] = 0x05; buf[9] = 0xD4;
	}
	send_data_processing(0, buf, 0x06);
	WriteReg(Sn_CR(0),Sn_CR_PCR); // send request 
	/* +20071122[chungs]:wait to process the command... */
	while( ReadReg(Sn_CR(0)) ) 
		;
	/* ------- */
		
	wait_10ms(100);

	while (!((isr = ReadReg(Sn_IR(0))) & Sn_IR_PNEXT))
	{
		if (isr & Sn_IR_PRECV) // Not support option
		{
   /* +200801[bj] clear interrupt value*/
         WriteReg(Sn_IR(0), Sn_IR_PRECV);
   /*---*/
			len = getSn_RX_RSR(0);
			if ( len > 0 )
			{
				recv_data_processing(0, str, len);
				WriteReg(Sn_CR(0),Sn_CR_RECV);
				/* +20071122[chungs]:wait to process the command... */
				while( ReadReg(Sn_CR(0)) ) 
					;
				/* ------- */
				
				// for debug
				//printf("LCP proc len = %d\r\n", len); for (i = 0; i < len; i++) printf ("%02x ", str[i]); printf("\r\n");
				// get option length
				len = str[4]; len = ((len & 0x00ff) << 8) + str[5];
				len += 2;
				str_idx = 6; dst_idx = 0; // ppp header is 6 byte, so starts at 6.
				do 
				{
					if ((str[str_idx] == 0x01) || (str[str_idx] == 0x02) || (str[str_idx] == 0x03) || (str[str_idx] == 0x05))
					{
						// skip as length of support option. str_idx+1 is option's length.
						str_idx += str[str_idx+1];
					}
					else
					{
						// not support option , REJECT
						memcpy((uint8 *)(buf+dst_idx), (uint8 *)(str+str_idx), str[str_idx+1]);
						dst_idx += str[str_idx+1]; str_idx += str[str_idx+1];
					}
				} while (str_idx != len);
	   			// for debug
	   			//printf("LCP dst proc\r\n"); for (i = 0; i < dst_idx; i++) printf ("%02x ", buf[i]); printf("\r\n");
	   
	   			// send LCP REJECT packet
	   			send_data_processing(0, buf, dst_idx);
	   			WriteReg(Sn_CR(0),Sn_CR_PCJ);
				/* +20071122[chungs]:wait to process the command... */
				while( ReadReg(Sn_CR(0)) ) 
					;
				/* ------- */
  			}
		}
		printf(".");
		if (loop_idx++ == 10) // timeout
		{
			printf("timeout after LCP\r\n");
			return 3;
		}
		wait_10ms(100);
	}
	printf(" ok\r\n");
	printf("\r\n");

   /* +200801[bj] clear interrupt value*/
   WriteReg(Sn_IR(0), 0xff);

   /* +200903[bj] clear receive buffer */
	len = getSn_RX_RSR(0);
	if ( len > 0 )
	{
		recv_data_processing(0, str, len);
		//printf("dummy proc len = %d\r\n", len); for (i = 0; i < len; i++) printf ("%02x ", str[i]); printf("\r\n");
		WriteReg(Sn_CR(0),Sn_CR_RECV);
		while( ReadReg(Sn_CR(0)) ) 
			;
	}
   /*---*/

	printf("-- PHASE 3. PPPoE(ADSL) Authentication mode --\r\n");
	printf("Authentication protocol : %.2x %.2x, ", ReadReg(PATR0), ReadReg(PATR0+1));

	loop_idx = 0;
	if (ReadReg(PATR0) == 0xc0 && ReadReg(PATR0+1) == 0x23)
	{
		printf("PAP\r\n"); // in case of adsl normally supports PAP.
		// send authentication data
		// copy (idlen + id + passwdlen + passwd)
		buf[loop_idx] = idlen; loop_idx++;
		memcpy((uint8 *)(buf+loop_idx), (uint8 *)(id), idlen); loop_idx += idlen;
		buf[loop_idx] = passwdlen; loop_idx++;
		memcpy((uint8 *)(buf+loop_idx), (uint8 *)(passwd), passwdlen); loop_idx += passwdlen;
		send_data_processing(0, buf, loop_idx);
		WriteReg(Sn_CR(0),Sn_CR_PCR);
		/* +20071122[chungs]:wait to process the command... */
		while( ReadReg(Sn_CR(0)) ) 
			;
		/* ------- */
		wait_10ms(100);
	}	
	else if (ReadReg(PATR0) == 0xc2 && ReadReg(PATR0+1) == 0x23)
	{
		uint8 chal_len;
		md5_ctx context;
		uint8  digest[16];

		len = getSn_RX_RSR(0);
		if ( len > 0 )
		{
			recv_data_processing(0, str, len);
			WriteReg(Sn_CR(0),Sn_CR_RECV);
			/* +20071122[chungs]:wait to process the command... */
			while( ReadReg(Sn_CR(0)) ) 
				;
			/* ------- */
#if (WNLIB_DEBUG == ENABLED)_
			printf("recv CHAP\r\n");
			{
				int16 i;
				
				for (i = 0; i < 32; i++) 
					printf ("%02x ", str[i]);
			}
			printf("\r\n");
#endif
// str is C2 23 xx CHAL_ID xx xx CHAP_LEN CHAP_DATA
// index  0  1  2  3       4  5  6        7 ...

			memset(buf,0x00,64);
			buf[loop_idx] = str[3]; loop_idx++; // chal_id
			memcpy((uint8 *)(buf+loop_idx), (uint8 *)(passwd), passwdlen); loop_idx += passwdlen; //passwd
			chal_len = str[6]; // chal_id
			memcpy((uint8 *)(buf+loop_idx), (uint8 *)(str+7), chal_len); loop_idx += chal_len; //challenge
			buf[loop_idx] = 0x80;
#if (WNLIB_DEBUG == ENABLED)_
			printf("CHAP proc d1\r\n");
			{
				int16 i;
				for (i = 0; i < 64; i++) 
					printf ("%02x ", buf[i]);
			}
			printf("\r\n");
#endif

			md5_init(&context);
			md5_update(&context, buf, loop_idx);
			md5_final(digest, &context);

#if (WNLIB_DEBUG == ENABLED)_
			printf("CHAP proc d1\r\n");
			{
				int16 i;				
				for (i = 0; i < 16; i++) 
					printf ("%02x", digest[i]);
			}
			printf("\r\n");
#endif
			loop_idx = 0;
			buf[loop_idx] = 16; loop_idx++; // hash_len
			memcpy((uint8 *)(buf+loop_idx), (uint8 *)(digest), 16); loop_idx += 16; // hashed value
			memcpy((uint8 *)(buf+loop_idx), (uint8 *)(id), idlen); loop_idx += idlen; // id
			send_data_processing(0, buf, loop_idx);
			WriteReg(Sn_CR(0),Sn_CR_PCR);
			/* +20071122[chungs]:wait to process the command... */
			while( ReadReg(Sn_CR(0)) ) 
				;
			/* ------- */
			wait_10ms(100);
		}
	}
	else
	{
		printf("Not support\r\n");
#if (WNLIB_DEBUG == ENABLED)_
		printf("Not support PPP Auth type: %.2x%.2x\r\n",ReadReg(PATR0), ReadReg(PATR0+1));
#endif
		return 4;
	}
	printf("\r\n");

	printf("-- Waiting for PPPoE server's admission --");
	loop_idx = 0;
	while (!((isr = ReadReg(Sn_IR(0))) & Sn_IR_PNEXT))
	{
		if (isr & Sn_IR_PFAIL)
		{
   /* +200801[bj] clear interrupt value*/
   WriteReg(Sn_IR(0), 0xff);
   /*---*/
			printf("failed\r\nReinput id, password..\r\n");
			return 2;
		}
		printf(".");
		if (loop_idx++ == 10) // timeout
		{
   /* +200801[bj] clear interrupt value*/
   WriteReg(Sn_IR(0), 0xff);
   /*---*/
			printf("timeout after PAP\r\n");
			return 3;
		}
		wait_10ms(100);
	}
   /* +200801[bj] clear interrupt value*/
   WriteReg(Sn_IR(0), 0xff);

   /* +200903[bj] clear receive buffer */
	len = getSn_RX_RSR(0);
	if ( len > 0 )
	{
		recv_data_processing(0, str, len);
		//printf("dummy proc len = %d\r\n", len); for (i = 0; i < len; i++) printf ("%02x ", str[i]); printf("\r\n");
		WriteReg(Sn_CR(0),Sn_CR_RECV);
		while( ReadReg(Sn_CR(0)) ) 
			;
	}
   /*---*/

	printf("ok\r\n");
	printf("\r\n");
	printf("-- PHASE 4. IPCP process --");
	// IP Address
	buf[0] = 0x03; buf[1] = 0x06; buf[2] = 0x00; buf[3] = 0x00; buf[4] = 0x00; buf[5] = 0x00;
	send_data_processing(0, buf, 6);
	WriteReg(Sn_CR(0),Sn_CR_PCR);
	/* +20071122[chungs]:wait to process the command... */
	while( ReadReg(Sn_CR(0)) ) 
		;
	/* ------- */
	wait_10ms(100);

	loop_idx = 0;
	while (1)
	{
		if (ReadReg(Sn_IR(0)) & Sn_IR_PRECV)
		{
   /* +200801[bj] clear interrupt value*/
   WriteReg(Sn_IR(0), 0xff);
   /*---*/
			len = getSn_RX_RSR(0);
			if ( len > 0 )
			{
				recv_data_processing(0, str, len);
				WriteReg(Sn_CR(0),Sn_CR_RECV);
				/* +20071122[chungs]:wait to process the command... */
				while( ReadReg(Sn_CR(0)) ) 
					;
				/* ------- */
	   			//for debug
	   			//printf("IPCP proc len = %d\r\n", len); for (i = 0; i < len; i++) printf ("%02x ", str[i]); printf("\r\n");
	   			str_idx = 6; dst_idx = 0;
	   			if (str[2] == 0x03) // in case of NAK
	   			{
	   				do 
	   				{
	   					if (str[str_idx] == 0x03) // request only ip information
	   					{
	   						memcpy((uint8 *)(buf+dst_idx), (uint8 *)(str+str_idx), str[str_idx+1]);
	   						dst_idx += str[str_idx+1]; str_idx += str[str_idx+1];
	   					}
	   					else
	   					{
	   						// skip byte
	   						str_idx += str[str_idx+1];
	   					}
	   					// for debug
	   					//printf("s: %d, d: %d, l: %d", str_idx, dst_idx, len);
	   				} while (str_idx != len);
	   				send_data_processing(0, buf, dst_idx);
	   				WriteReg(Sn_CR(0),Sn_CR_PCR); // send ipcp request
	   				/* +20071122[chungs]:wait to process the command... */
					while( ReadReg(Sn_CR(0)) ) 
						;
					/* ------- */
	   				wait_10ms(100);
	   				break;
	   			}
			}
		}
		printf(".");
		if (loop_idx++ == 10) // timeout
		{
			printf("timeout after IPCP\r\n");
			return 3;
		}
		wait_10ms(100);
		send_data_processing(0, buf, 6);
		WriteReg(Sn_CR(0),Sn_CR_PCR); //ipcp re-request
		/* +20071122[chungs]:wait to process the command... */
		while( ReadReg(Sn_CR(0)) ) 
			;
		/* ------- */
	}

	loop_idx = 0;
	while (!(ReadReg(Sn_IR(0)) & Sn_IR_PNEXT))
	{
		printf(".");
		if (loop_idx++ == 10) // timeout
		{
			printf("timeout after IPCP NAK\r\n");
			return 3;
		}
		wait_10ms(100);
		WriteReg(Sn_CR(0),Sn_CR_PCR); // send ipcp request
		/* +20071122[chungs]:wait to process the command... */
		while( ReadReg(Sn_CR(0)) ) 
			;
		/* ------- */
	}
   /* +200801[bj] clear interrupt value*/
   WriteReg(Sn_IR(0), 0xff);
   /*---*/
	printf("ok\r\n");
	printf("\r\n");
	return 1;
	// after this function, User must save the pppoe server's mac address and pppoe session id in current connection
}


/**
@brief	terminate PPPoE connection
*/
uint8 pppterm(uint8 * mac, uint8 * sessionid)
{
	uint16 i;
	uint8 isr;
#if (WNLIB_DEBUG == ENABLED)_
	printf("pppterm()\r\n");
#endif
	/* Set PPPoE bit in MR(Common Mode Register) : enable socket0 pppoe */
	WriteReg(MR,ReadReg(MR) | MR_PPPOE);
	
	// write pppoe server's mac address and session id 
	// must be setted these value.
	for (i = 0; i < 6; i++) WriteReg((Sn_DHAR0(0)+i),mac[i]);
	for (i = 0; i < 2; i++) WriteReg((Sn_DPORT0(0)+i),sessionid[i]);
	isr = ReadReg(Sn_IR(0));
	WriteReg(Sn_IR(0),isr);
	
	//open socket in pppoe mode
	WriteReg(Sn_MR(0),Sn_MR_PPPOE);
	WriteReg(Sn_CR(0),Sn_CR_OPEN);
	/* +20071122[chungs]:wait to process the command... */
	while( ReadReg(Sn_CR(0)) ) 
		;
	/* ------- */
	wait_1us(1);
	// close pppoe connection
	WriteReg(Sn_CR(0),Sn_CR_PDISCON);
	/* +20071122[chungs]:wait to process the command... */
	while( ReadReg(Sn_CR(0)) ) 
		;
	/* ------- */
	wait_10ms(100);
	// close socket
	/* +200801 (hwkim) */
	close(0);
	/* ------- */
	

#if (WNLIB_DEBUG == ENABLED)_
	printf("pppterm() end ..\r\n");
#endif

	return 1;
}
#endif
