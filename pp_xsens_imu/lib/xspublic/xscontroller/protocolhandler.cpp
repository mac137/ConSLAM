
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

#include "protocolhandler.h"
#include <xstypes/xsmessage.h>
#include <xstypes/xsresultvalue.h>
#include <iomanip>
#define DUMP_BUFFER_ON_ERROR	512		// this define doubles as the maximum buffer dump size, set to 0 to remove limit
#ifdef DUMP_BUFFER_ON_ERROR
	#include <sstream>
	#include <algorithm>
#endif

/*! \class ProtocolHandler
	\brief Message protocol handling class

	This class' purpose is to get valid messages according to its protocol from the raw
	data that is supplied to it. The default implementation (ProtocolHandler) implements
	the Xsens message protocol. To use a different protocol, overload the findMessage function.

	The class is intended to be state-less with respect to the data it handles.
*/

//! Default constructor
ProtocolHandler::ProtocolHandler()
	: m_ignoreMaxMsgSize(false)
{}

//! Destructor
ProtocolHandler::~ProtocolHandler()
{}

/*! \brief Compute the expected message size given a possibly incomplete message
	\details When analyzing a message with incomplete size information, the function
	will return the minimum size of the message given the information that is available
*/
static int expectedMessageSize(const unsigned char* buffer, int sz)
{
	const XsMessageHeader* hdr = (const XsMessageHeader*) buffer;
	if (sz < 4)
		return XS_LEN_MSGHEADERCS;	// no size information available at all, return a minimum message

	// we have size information
	if (hdr->m_length == XS_EXTLENCODE)
	{
		if (sz < 6)
			return XS_EXTLENCODE + XS_LEN_MSGEXTHEADERCS;	// typical minimum size at which point extended size is needed

		return XS_LEN_MSGEXTHEADERCS + (int)(((uint32_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint32_t) hdr->m_datlen.m_extended.m_length.m_low));
	}
	return XS_LEN_MSGHEADERCS + (int)(hdr->m_length);
}

/*! \brief Write the contents of a uint8 buffer to string as hex characters */
inline std::string dumpBuffer(const uint8_t* buff, XsSize sz)
{
#ifdef DUMP_BUFFER_ON_ERROR
	std::ostringstream ostr;
	ostr << std::hex << std::setfill('0');
#if DUMP_BUFFER_ON_ERROR > 0
	sz = std::min<XsSize>(sz, DUMP_BUFFER_ON_ERROR);
#endif
	for (XsSize i = 0; i < sz; ++i)
		ostr << " " << std::setw(2) << (int) buff[i];
	return ostr.str();
#else
	return std::string();
#endif
}

/*! \copydoc IProtocolHandler::findMessage
*/
MessageLocation ProtocolHandler::findMessage(XsProtocolType& type, const XsByteArray& raw) const
{
	JLTRACEG("Entry");
	type = static_cast<XsProtocolType>(ProtocolHandler::type());
	MessageLocation rv;

	int bufferSize = (int)raw.size();
	if (bufferSize == 0)
		return rv;

	const unsigned char* buffer = raw.data();

	// loop through the buffer to find a preamble
	for (int pre = 0; pre < bufferSize; ++pre)
	{
		if (buffer[pre] == XS_PREAMBLE)
		{
			JLTRACEG("Preamble found at " << pre);
			int remaining = bufferSize - pre;	// remaining bytes in buffer INCLUDING preamble

			if (remaining < XS_LEN_MSGHEADERCS)
			{
				JLTRACEG("Not enough header data read");
				if (rv.m_incompletePos == -1)
				{
					rv.m_incompletePos = pre;
					rv.m_incompleteSize = XS_LEN_MSGHEADERCS;
				}
				break;
			}

			// read header
			const uint8_t* msgStart = &(buffer[pre]);
			const XsMessageHeader* hdr = (const XsMessageHeader*)msgStart;
			if (hdr->m_busId == 0 && hdr->m_messageId == 0)
			{
				// found 'valid' message that isn't actually valid... happens inside GPS raw data
				// skip to next preamble
				// and completely ignore this message, since it cannot be valid
				//JLDEBUGG("Found invalid valid message");
				continue;
			}

			// check the reported size
			int target = expectedMessageSize(&buffer[pre], remaining);

			JLTRACEG("Bytes in buffer=" << remaining << ", full target = " << target);

			if (!m_ignoreMaxMsgSize && target > (XS_LEN_MSGEXTHEADERCS + XS_MAXDATALEN))
			{
				// skip current preamble
				JLALERTG("Invalid message length: " << target);
				continue;
			}

			if (remaining < target)
			{
				// not enough data read, skip current preamble
				JLTRACEG("Not enough data read: " << remaining << " / " << target);
				if (rv.m_incompletePos == -1)
				{
					rv.m_incompletePos = pre;
					rv.m_incompleteSize = target;
				}
				continue;
			}

			XsMessage rcv;
			// we have read enough data to fulfill our target so we'll try to parse the message
			// and check the checksum
			if (rcv.loadFromString(msgStart, (uint16_t)target))
			{
				JLTRACEG("OK, size = " << (int) rcv.getTotalMessageSize() << " buffer: " << dumpBuffer(msgStart, target));
				rv.m_size = (int) rcv.getTotalMessageSize();
				rv.m_startPos = pre;
#if 0
				JLDEBUGG("OK: rv.m_size = " << rv.m_size <<
					" rv.m_startPos = " << rv.m_startPos <<
					" rv.m_incompletePos = " << rv.m_incompletePos <<
					" pre = " << pre << " msg " << dumpBuffer(msgStart, target) <<
					" buffer " << dumpBuffer(buffer, bufferSize));
#endif
				break;
			}

			// Only alert the checksum error if this is not an embedded message
			if (rv.m_incompletePos == -1)
			{
				JLALERTG(
					"Invalid checksum for msg at offset " << pre << " bufferSize = " << bufferSize
					<< " buffer at offset: " << dumpBuffer(raw.data() + pre, raw.size() - pre));
			}
			else
			{
				JLTRACEG("Invalid checksum, size = " << (int)rcv.getTotalMessageSize() << " buffer: " << dumpBuffer(msgStart, target));
			}
		}
	}

	JLTRACEG("Exit");
	return rv;
}

/*! \copydoc IProtocolHandler::convertToMessage
*/
XsMessage ProtocolHandler::convertToMessage(MessageLocation& location, const XsByteArray& raw) const
{
	XsMessage message;

	const unsigned char* buffer = raw.data();
	const uint8_t* msgStart = &(buffer[location.m_startPos]);

	if (message.loadFromString(msgStart, (uint16_t)location.m_size))
	{
		JLTRACEG("OK, size = " << (int)message.getTotalMessageSize() << " buffer: " << dumpBuffer(msgStart, location.m_size));
		location.m_size = (int)message.getTotalMessageSize();

		return message;
	}

	message.clear();
	location.m_startPos = -1;
	location.m_incompletePos = -1;

	return message;
}

/*! \brief Returns the minimum size of a valid message of this protocol including preambles and checksums */
int ProtocolHandler::minimumMessageSize() const
{
	return XS_LEN_MSGHEADERCS;	// minimum size of xsens xbus protocol message
}

/*! \brief Returns the maximum size of a valid message of this protocol including preambles and checksums */
int ProtocolHandler::maximumMessageSize() const
{
	return XS_LEN_MSGEXTHEADERCS + XS_MAXDATALEN;	// maximum size of xsens xbus protocol message
}

/*! \brief Compose a message for transmission
	\param raw The raw byte array to be constructed from the message
	\param msg The message to translate into a raw byte array
	\returns The size of the generated byte array
*/
int ProtocolHandler::composeMessage(XsByteArray& raw, const XsMessage& msg)
{
	if (msg.getTotalMessageSize() < 5)	// minimum size of an xsens message including envelope is 5 bytes
		return -1;

	raw.assign(msg.getTotalMessageSize(), msg.getMessageStart());
	return (int) raw.size();
}

int ProtocolHandler::type() const
{
	return XPT_Xbus;
}

void ProtocolHandler::ignoreMaximumMessageSize(bool ignore)
{
	m_ignoreMaxMsgSize = ignore;
}
