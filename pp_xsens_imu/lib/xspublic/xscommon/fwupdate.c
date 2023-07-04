
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  


//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "fwupdate.h"
#include "string.h"
#include "xbus.h"

#ifndef LOG
	#define LOG(...)
#endif

#define XMID_FIRMWARE_UPDATE (0xF2)

#define FWUP_DEFAULT_SLICE_SIZE	(64)


/*!	\brief Firmware updater commands
*/
#define FWUP_READY			(unsigned char)0x52	//'R'
#define FWUP_OK				(unsigned char)0x53	//'S'
#define FWUP_CSERROR		(unsigned char)0x45	//'E'
#define FWUP_CRITICAL		(unsigned char)0x46	//'F'
#define FWUP_FINISHED		(unsigned char)0x46	//'F'
#define FWUP_PAGE			(unsigned char)0x50	//'P'
#define FWUP_HEADER			(unsigned char)0x53	//'S'
#define FWUP_PAGESLICE		(unsigned char)0x54 //'T'
#define FWUP_STARTED		(unsigned char)0x41	//'A'
#define FWUP_OTHER			(unsigned char)0x4F	//'O'


/*!	\brief Helper function for converting a firmware updater command to a string
*/
const char* commandToString(uint8_t command)
{
	switch (command)
	{
		case FWUP_READY:
			return ("FWUP_READY");
		case FWUP_HEADER:
			return ("FWUP_HEADER");
		case FWUP_FINISHED:
			return ("FWUP_FINISHED");
		case FWUP_PAGE:
			return ("FWUP_PAGE");
		case FWUP_PAGESLICE:
			return ("FWUP_PAGESLICE");
		case FWUP_STARTED:
			return ("FWUP_STARTED");
		case FWUP_OTHER:
			return ("FWUP_OTHER");
		default:
			return ("unknown");
	}
}


/*!	\brief Helper function for converting a firmware updater command acknowledge to a string
*/
const char* ackToString(uint8_t command)
{
	switch (command)
	{
		case FWUP_READY:
			return "FWUP_READY";
		case FWUP_OK:
			return "FWUP_OK";
		case FWUP_CSERROR:
			return "FWUP_CSERROR";
		case FWUP_CRITICAL:
			return "FWUP_CRITICAL";
		case FWUP_PAGE:
			return "FWUP_PAGE";
		case FWUP_PAGESLICE:
			return "FWUP_PAGESLICE";
		case FWUP_STARTED:
			return "FWUP_STARTED";
		case FWUP_OTHER:
			return "FWUP_OTHER";
		default:
			return "unknown";
	}
}


/*!	\brief Read a uint32_t from the current position in the xff
*/
uint32_t readUint32(FwUpdate* thisPtr)
{
	uint32_t result;
	uint8_t buffer[4];
	uint32_t n = thisPtr->m_readXffData(buffer, thisPtr->m_readIndex, 4);
	thisPtr->m_readIndex += n;
	if (n == 4)
		result = (uint32_t)buffer[0] << 24 | (uint32_t)buffer[1] << 16 | (uint32_t)buffer[2] << 8 | buffer[3];
	else
	{
		thisPtr->m_endOfFile = 1;
		result = 0;
	}
	return result;
}


/*!	\brief Read a uint16_t from the current position in the xff
*/
uint16_t readUint16(FwUpdate* thisPtr)
{
	uint32_t result;
	uint8_t buffer[2];
	uint32_t n = thisPtr->m_readXffData(buffer, thisPtr->m_readIndex, 2);
	thisPtr->m_readIndex += n;
	if (n == 2)
		result = (uint32_t)buffer[0] << 8 | buffer[1];
	else
	{
		thisPtr->m_endOfFile = 1;
		result = 0;
	}
	return (uint16_t)result;
}


/*!	\brief Read a uint8_t from the current position in the xff
*/
uint8_t readUint8(FwUpdate* thisPtr)
{
	uint8_t result;
	uint8_t buffer[1];
	uint32_t n = thisPtr->m_readXffData(buffer, thisPtr->m_readIndex, 1);
	thisPtr->m_readIndex += n;
	if (n == 1)
		result = buffer[0];
	else
	{
		thisPtr->m_endOfFile = 1;
		result = 0;
	}
	return result;
}


/*!	\brief Read an Xff header from the current position in the xff
*/
static void readXffHeader(FwUpdate* thisPtr)
{
	LOG("Fwu: readXffHeader()\n");

	uint32_t globalIdTemp = readUint32(thisPtr);
	if (globalIdTemp != 0)
	{
		thisPtr->m_xffHeader.m_globalId = globalIdTemp;
		thisPtr->m_xffHeader.m_sectionSize = readUint32(thisPtr);
		thisPtr->m_xffHeader.m_firmwareRevision[0] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_firmwareRevision[1] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_firmwareRevision[2] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_xffVersion[0] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_xffVersion[1] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_chipId = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_numberOfSections = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_addressLength = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_pageSize = readUint16(thisPtr);
		if (thisPtr->m_xffHeader.m_xffVersion[0] >= 2)
			thisPtr->m_xffHeader.m_sliceSize = readUint16(thisPtr);
		else
			thisPtr->m_xffHeader.m_sliceSize = FWUP_DEFAULT_SLICE_SIZE;
	}
	else
	{
		(void) readUint32(thisPtr);	// headerSize
		thisPtr->m_xffHeader.m_globalId = readUint32(thisPtr);
		thisPtr->m_xffHeader.m_sectionSize = readUint32(thisPtr);
		thisPtr->m_xffHeader.m_firmwareRevision[0] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_firmwareRevision[1] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_firmwareRevision[2] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_xffVersion[0] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_xffVersion[1] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_chipId = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_numberOfSections = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_addressLength = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_pageSize = readUint16(thisPtr);
		thisPtr->m_xffHeader.m_sliceSize = readUint16(thisPtr);
		thisPtr->m_xffHeader.m_hardwareVersion[0] = readUint8(thisPtr);
		thisPtr->m_xffHeader.m_hardwareVersion[1] = readUint8(thisPtr);
		if (thisPtr->m_xffHeader.m_xffVersion[0] >= 4)
		{
			for (int n = 0; n < 20; n++)
				thisPtr->m_xffHeader.m_productCode[n] = (char) readUint8(thisPtr);
			thisPtr->m_xffHeader.m_productVariant = readUint32(thisPtr);
		}
	}
}


/*!	\brief Send a FWUP_READY command
*/
static void sendReady(FwUpdate* thisPtr)
{
	Xbus_message(thisPtr->m_txBuffer, 0xFF, XMID_FIRMWARE_UPDATE, 1);
	Xbus_getPointerToPayload(thisPtr->m_txBuffer)[0] = FWUP_READY;
	Xbus_insertChecksum(thisPtr->m_txBuffer);
	thisPtr->m_sendXbusMessage(thisPtr->m_txBuffer);
}


/*!	\brief Send a FWUP_HEADER command
*/
static void sendHeader(FwUpdate* thisPtr)
{
	uint32_t n;
	Xbus_message(thisPtr->m_txBuffer, 0xFF, XMID_FIRMWARE_UPDATE, 1 + thisPtr->m_xffHeader.m_addressLength + 2);
	uint8_t* payload = Xbus_getPointerToPayload(thisPtr->m_txBuffer);
	payload[0] = FWUP_HEADER;
	n = thisPtr->m_readXffData(&payload[1], thisPtr->m_readIndex, thisPtr->m_xffHeader.m_addressLength);
	thisPtr->m_readIndex += n;
	if (n == thisPtr->m_xffHeader.m_addressLength)
	{
		memcpy(&payload[1 + thisPtr->m_xffHeader.m_addressLength], (uint8_t*)&thisPtr->m_nofSlicesPerPage, 2);
		Xbus_insertChecksum(thisPtr->m_txBuffer);
		thisPtr->m_sendXbusMessage(thisPtr->m_txBuffer);
	}
	else
		thisPtr->m_endOfFile = 1;
}


/*!	\brief Send a page slice
*/
static void sendSlice(FwUpdate* thisPtr)
{
	uint32_t n;

	Xbus_message(thisPtr->m_txBuffer, 0xFF, XMID_FIRMWARE_UPDATE, 1 + thisPtr->m_xffHeader.m_sliceSize);
	uint8_t* payload = Xbus_getPointerToPayload(thisPtr->m_txBuffer);

	payload[0] = FWUP_PAGESLICE;
	n = thisPtr->m_readXffData(&payload[1], thisPtr->m_readIndex, thisPtr->m_xffHeader.m_sliceSize);
	thisPtr->m_readIndex += n;
	if (n == thisPtr->m_xffHeader.m_sliceSize)
	{
		Xbus_insertChecksum(thisPtr->m_txBuffer);
		thisPtr->m_sendXbusMessage(thisPtr->m_txBuffer);
	}
	else
		thisPtr->m_endOfFile = 1;
}


/*!	\brief Send a FWUP_OTHER command
*/
static void sendOther(FwUpdate* thisPtr)
{
	Xbus_message(thisPtr->m_txBuffer, 0xFF, XMID_FIRMWARE_UPDATE, 2);
	uint8_t* payload = Xbus_getPointerToPayload(thisPtr->m_txBuffer);
	payload[0] = FWUP_OTHER;
	payload[1] = thisPtr->m_xffHeader.m_chipId;
	LOG("Fwu: Send FWUP_OTHER\n");
	Xbus_insertChecksum(thisPtr->m_txBuffer);
	thisPtr->m_sendXbusMessage(thisPtr->m_txBuffer);
}


/*!	\brief Send a FWUP_FINISHED command
*/
static void sendFinished(FwUpdate* thisPtr)
{
	Xbus_message(thisPtr->m_txBuffer, 0xFF, XMID_FIRMWARE_UPDATE, 1);
	uint8_t* payload = Xbus_getPointerToPayload(thisPtr->m_txBuffer);
	payload[0] = FWUP_FINISHED;
	LOG("Fwu: Send FWUP_FINISHED\n");
	Xbus_insertChecksum(thisPtr->m_txBuffer);
	thisPtr->m_sendXbusMessage(thisPtr->m_txBuffer);
}


/*!	\brief Enter the next section of the xff file
*/
static void enterNewSection(FwUpdate* thisPtr)
{
	LOG("\nFwu: enterNewSection()\n");
	readXffHeader(thisPtr);
	thisPtr->m_nofPages = thisPtr->m_xffHeader.m_sectionSize / ((uint32_t)thisPtr->m_xffHeader.m_pageSize + (uint32_t)thisPtr->m_xffHeader.m_addressLength);
	thisPtr->m_nofSlicesPerPage = thisPtr->m_xffHeader.m_pageSize / thisPtr->m_xffHeader.m_sliceSize;
	thisPtr->m_pageCounter = 0;
	thisPtr->m_sliceCounter = 0;
	sendOther(thisPtr);
}


/*!	\brief Initialize a FwUpdate instance
*/
void FwUpdate_init(FwUpdate* thisPtr)
{
	LOG("Fwu: init()\n");
	thisPtr->m_state = STATE_Idle;
}


/*!	\brief Start a firmware update
*/
void FwUpdate_start(FwUpdate* thisPtr)
{
	if (thisPtr->m_state == STATE_Idle)
	{
		LOG("Fwu: start() --> Send FWUP_READY\n");
		thisPtr->m_readIndex = 0;
		thisPtr->m_endOfFile = 0;
		thisPtr->m_state = STATE_Start;
		sendReady(thisPtr);
	}
	else
	{
		thisPtr->m_readyHandler(FWU_Failed);
		LOG("Fwu: start() failed\n");
	}
}


/*!	\brief Handle xbus message coming from the module
	\param xbusMessage The xbus message from the module to be handled
*/
void FwUpdate_handleXbus(FwUpdate* thisPtr, uint8_t const* xbusMessage)
{
	uint8_t ack;

	if (Xbus_getMessageId(xbusMessage) != XMID_FIRMWARE_UPDATE)
	{
		LOG("Fwu: Got unhandled xbus message 0x%02X (ignored)\n", Xbus_getMessageId(xbusMessage));
		return;
	}

	ack = Xbus_getConstPointerToPayload(xbusMessage)[0];

	switch (thisPtr->m_state)
	{
		case STATE_Idle:
		{
			LOG("Fwu: Got %s in STATE_Idle (ignored)\n", ackToString(ack));
			break;
		}

		case STATE_Start:
		{
			if (ack == FWUP_READY)
			{
				LOG("Fwu: FWUP_READY in STATE_Start --> Enter new section\n");
				enterNewSection(thisPtr);
				thisPtr->m_state = STATE_WaitReady;
			}
			else
				LOG("Fwu: Got %s in STATE_Start (ignored)\n", ackToString(ack));
			break;
		}

		case STATE_WaitReady:
		{
			if (ack == FWUP_READY)
			{
				LOG("Fwu: FWUP_READY in STATE_WaitReady --> Send header\n");
				sendHeader(thisPtr);
				thisPtr->m_state = STATE_WaitHeaderResult;
			}
			else
			{
				LOG("Fwu: Got %s in STATE_WaitReady --> Failed\n", ackToString(ack));
				thisPtr->m_readyHandler(FWU_Failed);
				thisPtr->m_state = STATE_Idle;
			}
			break;
		}

		case STATE_WaitHeaderResult:
		{
			if (ack == FWUP_READY)
			{
				LOG("Fwu: FWUP_READY in STATE_WaitHeaderResult --> Send first slice\n");
				sendSlice(thisPtr);
				thisPtr->m_sliceCounter = 1;
				thisPtr->m_state = STATE_WaitSliceReady;
			}
			else
			{
				LOG("Fwu: Got %s in STATE_WaitHeaderResult --> Failed\n", ackToString(ack));
				thisPtr->m_readyHandler(FWU_Failed);
				thisPtr->m_state = STATE_Idle;
			}
			break;
		}

		case STATE_WaitSliceReady:
		{
			if (ack == FWUP_READY)
			{
				if (thisPtr->m_sliceCounter < thisPtr->m_nofSlicesPerPage)
				{
					LOG("Fwu: FWUP_READY in STATE_WaitSliceReady --> Send slice %d\n", thisPtr->m_sliceCounter);
					sendSlice(thisPtr);
					thisPtr->m_sliceCounter++;
				}
				else
				{
					LOG("Fwu: All slices sent --> STATE_WaitPageOk\n");
					thisPtr->m_state = STATE_WaitPageOk;
				}
			}
			else
			{
				LOG("Fwu: Got %s in STATE_WaitSliceReady --> Failed\n", ackToString(ack));
				thisPtr->m_readyHandler(FWU_Failed);
				thisPtr->m_state = STATE_Idle;
			}
			break;
		}

		case STATE_WaitPageOk:
		{
			if (ack == FWUP_OK)
			{
				LOG("Fwu: FWUP_OK in STATE_WaitPageOk --> STATE_WaitPageReady\n");
				thisPtr->m_state = STATE_WaitPageReady;
			}
			else
			{
				LOG("Fwu: Got %s in STATE_WaitPageOk --> Failed\n", ackToString(ack));
				thisPtr->m_readyHandler(FWU_Failed);
				thisPtr->m_state = STATE_Idle;
			}
			break;
		}

		case STATE_WaitPageReady:
		{
			if (ack == FWUP_READY)
			{
				thisPtr->m_pageCounter++;
				if (thisPtr->m_nofPages != 0 && thisPtr->m_pageCounter == thisPtr->m_nofPages)
				{
					LOG("Fwu: All pages sent --> Enter new section\n");
					enterNewSection(thisPtr);
					thisPtr->m_state = STATE_WaitReady;
				}
				else
				{
					sendHeader(thisPtr);
					if (thisPtr->m_endOfFile)
					{
						LOG("Fwu: End of file --> Firmware update done\n");
						sendFinished(thisPtr);
						thisPtr->m_readyHandler(FWU_Success);
						thisPtr->m_state = STATE_Idle;
					}
					else
					{
						LOG("Fwu: FWUP_READY in STATE_WaitPageReady --> Send header\n");
						thisPtr->m_state = STATE_WaitHeaderResult;
					}
				}
			}
			else
			{
				LOG("Fwu: Got %s in STATE_WaitPageReady --> Failed\n", ackToString(ack));
				thisPtr->m_readyHandler(FWU_Failed);
				thisPtr->m_state = STATE_Idle;
			}
			break;
		}

		default:
			break;
	}
}
