
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

#include "xbusparser.h"
#include <stdio.h>
#include <stdbool.h>
#include "xbus.h"


/*!	\brief Initializes an XbusParser instance
*/
void XbusParser_init(XbusParser* obj, unsigned char* buffer, int bufferSize)
{
	obj->m_rxBufferSize = bufferSize;
	obj->m_rxBuffer = buffer;
	obj->m_idxWrite = 0;
	obj->m_state = haveNothing;
	obj->m_lastError = RES_Ok;
}


/*!	\brief Insert a new byte in the parser
	\param obj The XbusParser instance
	\param byte The byte to insert
	\param messageSize Pointer to an integer in which to return the current message size
*/
enum Result XbusParser_insertByte(XbusParser* obj, unsigned char byte, int* messageSize)
{
	enum Result result = RES_Ok;
	bool incrementIdxWrite = true;

	if (obj->m_state != haveNothing)	// if we have at least a preamble,
		obj->m_rxBuffer[obj->m_idxWrite] = byte;	// store the received byte in the buffer.

	switch (obj->m_state)
	{
		case haveNothing:
			if (byte == XBUS_PREAMBLE)				// if we have a preamble,
			{
				obj->m_idxWrite = 0;						// reset the write index
				obj->m_rxBuffer[obj->m_idxWrite] = byte;		// store the byte
				obj->m_state = havePreamble;				// update our state
				obj->m_totalLength = 1;					// reset m_totalLength
				obj->m_checksum = 0;						// reset m_checksum
				result = RES_FoundPreamble;
			}
			else
			{
				incrementIdxWrite = false;			// don't increment m_idxWrite if we are not in a xbus message
				result = RES_SpuriousByte;
			}
			break;

		case havePreamble:
			obj->m_totalLength++;
			obj->m_checksum += obj->m_rxBuffer[obj->m_idxWrite];
			if (obj->m_totalLength == OFFSET_TO_LEN + 1)
			{
				if (obj->m_rxBuffer[obj->m_idxWrite] != LENGTH_EXTENDER_BYTE)
				{
					obj->m_state = haveLength;
					obj->m_payloadLength = obj->m_rxBuffer[obj->m_idxWrite];
					obj->m_offsetToPayload = OFFSET_TO_PAYLOAD;
				}
				else
				{
					obj->m_state = haveLengthExtenderByte;
					obj->m_offsetToPayload = OFFSET_TO_PAYLOAD_EXT;
				}
			}
			break;

		case haveLengthExtenderByte:
			obj->m_totalLength++;
			obj->m_checksum += obj->m_rxBuffer[obj->m_idxWrite];
			if (obj->m_totalLength == OFFSET_TO_LEN + 2)
				obj->m_payloadLength = 256 * obj->m_rxBuffer[obj->m_idxWrite];
			if (obj->m_totalLength == OFFSET_TO_LEN + 3)
			{
				obj->m_payloadLength += obj->m_rxBuffer[obj->m_idxWrite];
				obj->m_state = haveLength;
			}
			break;

		case haveLength:
			obj->m_totalLength++;
			obj->m_checksum += obj->m_rxBuffer[obj->m_idxWrite];
			if (obj->m_totalLength == (obj->m_offsetToPayload + obj->m_payloadLength + 1)) // if the current byte is the checksum
			{
				if ((obj->m_checksum & 0x00FF) == 0) // if checksum ok
					obj->m_state = haveMessage;
				else // if checksum error,
				{
					obj->m_state = haveNothing;
					return RES_CheckSumError;
				}
			}
			break;

		case haveMessage:
			break;

		case haveError:
			return obj->m_lastError;

		default:
			break;
	}

	if (obj->m_state == haveMessage)
	{
		*messageSize = obj->m_totalLength;
		return RES_MessageReceived;
	}

	if (incrementIdxWrite)
	{
		if (++obj->m_idxWrite >= obj->m_rxBufferSize)
		{
			//Error condition if no packet received yet (report buffer overflow, restart ? Keep parser in error state)
			//User must restart the parser to clear error state
			obj->m_state = haveError;
			obj->m_lastError = RES_BufferOverflow;
			return obj->m_lastError;
		}
	}

	return result;
}


