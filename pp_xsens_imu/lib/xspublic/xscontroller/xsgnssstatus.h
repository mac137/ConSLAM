
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

#ifndef XSGNSSSTATUS_H
#define XSGNSSSTATUS_H

#include <xstypes/pstdint.h>

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief GNSS status flags
	\details The Flags contain bits the indicate the operational status of the GNSS module
*/
enum XsGnssStatusFlag
{
	XGSF_GnssReceiverInitialized	= 0x01	//! GNSS receiver has been initialized succesfully
	, XGSF_GnssReceiverDataOk		= 0x02	//! GNSS receiver has correctly received Data
	, XGSF_AntennaDetectionMask		= 0x70	//! 3-Bit field indicating the status of the GNSS antenna
};

/*! \brief GNSS status flags offsets
	\details Sometimes (rarely) it is necessary to know the bit offset instead of the bit mask (ie when
	shifting to only keep a subset of flags) for the status flags. This enumeration provides these
	offsets.
	\sa XsGnssStatusFlag
*/
enum XsGnssStatusFlagOffset
{
	XGSFO_OffsetGnssReceiverInitialized	= 0
	, XGSFO_OffsetGnssReceiverDataOk			= 1
	, XGSFO_OffsetAntennaDetection			= 4
};

/*! \brief Contains flags indicating the status of the GNSS antenna
*/
enum XsGnssAntennaStatus
{
	XGAS_OpenCircuit = 0,	//! No antenna is connected to the antenne port
	XGAS_ShortCircuit = 1,	//! The antennaconnector seems to be shorted with Ground
	XGAS_OK = 2,			//! Everything seems ok, antenna connected
	XGAS_DontKnow = 3,		//! Unknown Status
	XGAS_Requesting = 4,	//! Busy requesting the current antenna status from the GNSS receiver
	XGAS_Init = 5			//! GNSS receiver is initalizing, requesting antenna status not available yet
};
/*! @} */
typedef enum XsGnssStatusFlag XsGnssStatusFlag;
typedef enum XsGnssStatusFlagOffset XsGnssStatusFlagOffset;
typedef enum XsGnssAntennaStatus XsGnssAntennaStatus;

/*! \brief Contains flags indicating the opperational status of the GNSS module
*/
struct XsGnssStatus
{
	uint8_t m_flags;		//!< Flags that specify which tests have passed

#ifdef __cplusplus
	//! \brief Create a new %XsGnssStatus from supplied flags
	static inline XsGnssStatus create(uint8_t resultFlags)
	{
		XsGnssStatus tmp = { resultFlags };
		return tmp;
	}

	//! \brief Returns whether the gnss Receiver is initialized correctly
	inline bool gnssReceiverInitialized() const
	{
		return (m_flags & XGSF_GnssReceiverInitialized) != 0;
	}

	//! \brief Returns whether the the data from the GNSS receiver is OK
	inline bool gnssReceiverDataOk() const
	{
		return (m_flags & XGSF_GnssReceiverDataOk) != 0;
	}

	//! \brief Returns the antenna status
	XsGnssAntennaStatus gnssAntennaStatus() const
	{
		return ((XsGnssAntennaStatus)((m_flags & XGSF_AntennaDetectionMask) >> XGSFO_OffsetAntennaDetection));
	}

#endif
};

typedef struct XsGnssStatus XsGnssStatus;

#endif
