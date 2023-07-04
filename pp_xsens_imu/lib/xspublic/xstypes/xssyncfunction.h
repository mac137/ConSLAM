
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

#ifndef XSSYNCFUNCTION_H
#define XSSYNCFUNCTION_H

/*!	\addtogroup enums Global enumerations
	@{
*/

//AUTO namespace xstypes {
/*! \brief Actions to be taken on input triggers */
enum XsSyncFunction
{
	XSF_StartRecordingIn,					/*!< \brief Start recording on trigger or emit trigger when first recording frame is started (In). \remark Applies to Awinda Station. */
	XSF_StopRecordingIn,					/*!< \brief Stop recording on trigger or emit trigger when recording is stopped (In). \remark Applies to Awinda Station. */
	XSF_ResetTimer,							/*!< \brief On input trigger, the outgoing timer of the station will be set to 0. \remark Applies to Awinda Station. */
	XSF_TriggerIndication,					/*!< \brief An indication is sent to the driver when trigger is detected. \remark Applies to Awinda Station. */

	XSF_IntervalTransitionMeasurement,		/*!< \brief Emit trigger on an interval transition during measurement and recording. */
	XSF_IntervalTransitionRecording,		/*!< \brief Emit trigger on an interval transition during recording. */
	XSF_GotoOperational,					/*!< \brief Emit trigger when going to Operational mode \remark Applies to Awinda Station. */

	XSF_SampleAndSend,						/*!< \brief Sample a sample and send the MT Data message. \remark Applies to Mt. */
	XSF_SendLatest,							/*!< \brief Send the latest sample. \remark Applies to Mt. */
	XSF_ClockBiasEstimation,				/*!< \brief Do a clock bias estimation on trigger. \remark Applies to Mti-G. */

	XSF_PulseWithModulation,				/*!< \brief Do interval transition measurement with pulse width modulation. */

	XSF_StartSampling,						/*!< \brief Start sampling. Data will only be transmitted after this trigger has been received. \remark Applies only to Mk4. */

	XSF_StartRecordingOut,					/*!< \brief Start recording on trigger or emit trigger when first recording frame is started (Out). \remark Applies to Awinda Station. */
	XSF_StopRecordingOut,					/*!< \brief Stop recording on trigger or emit trigger when recording is stopped (Out). \remark Applies to Awinda Station. */

	XSF_Gnss1Pps,							/*!< \brief Emit trigger on the start of each second, generated by the GNSS receiver. */

	XSF_BusSync,							/*!< \brief Supply periodic trigger to synchronize multiple MTi devices on a single bus. */

	XSF_Invalid,							/*!< \brief Invalid action \details This indicates the trigger action is not usable. */
	XSF_Count = XSF_Invalid					/*!< \brief Amount of trigger actions */
};
/*! @} */
typedef enum XsSyncFunction XsSyncFunction;
//AUTO }

#endif
