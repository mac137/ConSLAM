
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

#ifndef FWUPDATE_H
#define FWUPDATE_H

#include "xbus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FWU_REQUIRED_TXBUFFER_SIZE 300

/*!	\brief Result value
*/
typedef enum
{
	FWU_Success,
	FWU_Failed
} FWU_Result;


/*!	\brief Definition of a xff section header
*/
typedef struct
{
	uint32_t m_globalId;			/*!< globalId*/
	uint32_t m_sectionSize;			/*!< Section size*/
	uint8_t m_firmwareRevision[3];	/*!< Firmware revision*/
	uint8_t m_xffVersion[2];		/*!< xff version*/
	uint8_t m_chipId;				/*!< ChipId*/
	uint8_t m_numberOfSections;		/*!< Number of sections in the xff file*/
	uint8_t m_addressLength;		/*!< Address length*/
	uint16_t m_pageSize;			/*!< Page size*/
	uint16_t m_sliceSize;			/*!< Slice size*/
	uint8_t m_hardwareVersion[2];	/*!< Hardware version (only in xff version > 2)*/
	char m_productCode[20];			/*!< Product code (only in xff version > 3)*/
	uint32_t m_productVariant;		/*!< Product variant (only in xff version > 3)*/
} XffHeader;


/*!	\brief Internal state
*/
typedef enum
{
	STATE_Idle,
	STATE_Start,
	STATE_WaitReady,
	STATE_WaitHeaderResult,
	STATE_WaitSliceReady,
	STATE_WaitPageOk,
	STATE_WaitPageReady
} FWU_State;



/*!	\brief FwUpdate object definition
*/
typedef struct
{
	/*	External dependencies. Host should fill in these members */

	/*!	\brief Callback function by which FwUpdate requests for xff data
		\param buffer Target buffer in which the xff data should be written by the host
		\param offset Offset in the xff file where reading should start
		\param length Number of bytes which is requested
		\returns Number of bytes which is actually written to the buffer
	*/
	uint32_t (*m_readXffData)(uint8_t* buffer, uint32_t offset, uint32_t length);

	/*!	\brief Callback function via which FwUpdate can send xbus messages to the module
		\param xbusMessage Xbus message that should be send to the module
	*/
	void (*m_sendXbusMessage)(uint8_t const* xbusMessage);

	/*!	\brief Callback function by which FwUpdate notifies the host that a firmware update has finished
		\param result FWU_Success or FWU_Failed
	*/
	void (*m_readyHandler)(FWU_Result result);

	/*!	\brief Memory needed by the FwUpdate. Host must allocate a block of memory
		of size FWU_REQUIRED_TXBUFFER_SIZE
	*/
	uint8_t* m_txBuffer;

	/*	State variables for internal use (the user must not touch these) */
	FWU_State m_state;				/*!< Internal state member of FwUpdate*/
	XffHeader m_xffHeader;			/*!< Internal state member of FwUpdate*/
	uint32_t m_nofPages;			/*!< Internal state member of FwUpdate*/
	uint32_t m_nofSlicesPerPage;	/*!< Internal state member of FwUpdate*/
	uint32_t m_pageCounter;			/*!< Internal state member of FwUpdate*/
	uint32_t m_sliceCounter;		/*!< Internal state member of FwUpdate*/
	uint32_t m_readIndex;			/*!< Internal state member of FwUpdate*/
	uint8_t m_endOfFile;			/*!< Internal state member of FwUpdate*/
} FwUpdate;


void FwUpdate_init(FwUpdate* thisPtr);

void FwUpdate_start(FwUpdate* thisPtr);

void FwUpdate_handleXbus(FwUpdate* thisPtr, uint8_t const* xbusMessage);


#ifdef __cplusplus
}
#endif /* extern "C" */


#endif
