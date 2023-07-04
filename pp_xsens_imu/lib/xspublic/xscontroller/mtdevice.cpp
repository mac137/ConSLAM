
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

#include "mtdevice.h"
#include "xsdef.h"
#include <xstypes/xssensorranges.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsvector3.h>
#include "xsselftestresult.h"
#include <xstypes/xsstatusflag.h>
#include <algorithm>

// Undef the windows min macro that conflict with e.g. std::numeric_limits<..>::min
#ifdef min
	#undef min
#endif

using namespace xsens;

/*! \brief Constructs a standalone MtDevice with device Id \a id
*/
MtDevice::MtDevice(XsDeviceId const& id)
	: XsDeviceEx(id)
{
}

/*! \brief Constructs a standalone MtDevice based on \a comm
*/
MtDevice::MtDevice(Communicator* comm)
	: XsDeviceEx(comm)
{
}

/*! \brief Constructs a standalone MtDevice based on \a master and \a childDeviceId
*/
MtDevice::MtDevice(XsDevice* master, const XsDeviceId& childDeviceId)
	: XsDeviceEx(master, childDeviceId)
{
}

/*! \brief Destroys the MtDevice
*/
MtDevice::~MtDevice()
{
}

/*! \brief Checks for the sanity of a message
	\param msg A message to check
	\returns True if successful
*/
bool MtDevice::messageLooksSane(const XsMessage& msg) const
{
	return msg.getBusId() == 1 || XsDevice::messageLooksSane(msg);
}

/*! \brief Initialize the Mt device using the supplied filter profiles
	\returns True if successful
*/
bool MtDevice::initialize()
{
	if (!XsDeviceEx::initialize())
		return false;

	// we must create the data caches first so they are available even if the rest of the init fails
	// when reading from file almost all init can fail but we can still read data into the caches
	if (!readDeviceConfiguration())
	{
		setInitialized(false);
		return false;
	}

	fetchAvailableHardwareScenarios();
	MtDevice::updateFilterProfiles();

	return true;
}

/*! \brief Updates the scenarios
*/
void MtDevice::updateFilterProfiles()
{
	const XsMtDeviceConfiguration& info = deviceConfigurationConst().deviceInfo(deviceId());
	if (info.m_filterProfile != 0)
	{
		m_hardwareFilterProfile = XsFilterProfile(info.m_filterProfile & 0xFF
				, info.m_filterProfile >> 8
				, m_hardwareFilterProfile.kind()
				, m_hardwareFilterProfile.label()
				, info.m_filterType
				, info.m_filterMajor
				, info.m_filterMinor);
	}

	for (auto i = m_hardwareFilterProfiles.begin(); i != m_hardwareFilterProfiles.end(); ++i)
	{
		if (i->type() == m_hardwareFilterProfile.type() || i->label() == m_hardwareFilterProfile.label())
		{
			m_hardwareFilterProfile.setLabel(i->label());
			m_hardwareFilterProfile.setKind(i->kind());
			m_hardwareFilterProfile.setVersion(i->version());
			break;
		}
	}
}

/*! \returns True if this is a motion tracker
*/
bool MtDevice::isMotionTracker() const
{
	return true;
}

/*! \copybrief XsDevice::updateRateForDataIdentifier
*/
int MtDevice::updateRateForDataIdentifier(XsDataIdentifier dataType) const
{
	return XsDevice::updateRateForDataIdentifier(dataType);
}

/*! \copybrief XsDevice::stringOutputType
*/
uint16_t MtDevice::stringOutputType() const
{
	XsMessage snd(XMID_ReqStringOutputType), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::stringSamplePeriod
*/
uint16_t MtDevice::stringSamplePeriod() const
{
	XsMessage snd(XMID_ReqPeriod), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::stringSkipFactor
*/
uint16_t MtDevice::stringSkipFactor() const
{
	XsMessage snd(XMID_ReqOutputSkipFactor), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::deviceOptionFlags
*/
XsDeviceOptionFlag MtDevice::deviceOptionFlags() const
{
	XsMessage snd(XMID_ReqOptionFlags), rcv;
	if (doTransaction(snd, rcv))
		return (XsDeviceOptionFlag)rcv.getDataLong();
	return XDOF_None;
}

/*! \copybrief XsDevice::ubloxGnssPlatform
*/
XsUbloxGnssPlatform MtDevice::ubloxGnssPlatform() const
{
	XsMessage snd(XMID_ReqGnssPlatform), rcv;
	if (doTransaction(snd, rcv))
		return static_cast<XsUbloxGnssPlatform>(rcv.getDataShort());
	return XGP_Portable;
}

/*! \copybrief XsDevice::setUbloxGnssPlatform
*/
bool MtDevice::setUbloxGnssPlatform(XsUbloxGnssPlatform ubloxGnssPlatform)
{
	XsMessage snd(XMID_SetGnssPlatform, 1);
	snd.setBusId(busId());
	snd.setDataShort((uint16_t)ubloxGnssPlatform);
	if (!doTransaction(snd))
		return false;
	return true;
}

/*! \copydoc XsDevice::gnssReceiverSettings
*/
XsIntArray MtDevice::gnssReceiverSettings() const
{
	XsMessage snd(XMID_ReqGnssReceiverSettings), rcv;
	snd.setBusId(busId());
	if (doTransaction(snd, rcv))
	{
		XsIntArray gnssReceiverSettings = XsIntArray(4);
		gnssReceiverSettings[0] = (int)rcv.getDataShort(0);//receiver type
		gnssReceiverSettings[1] = (int)rcv.getDataShort(2);//receiver baud rate
		gnssReceiverSettings[2] = (int)rcv.getDataShort(4);//receiver input rate
		gnssReceiverSettings[3] = (int)rcv.getDataLong(6);//receiver options
		return gnssReceiverSettings;
	}
	return XsIntArray();
}

/*! \copydoc XsDevice::setGnssReceiverSettings
*/
bool MtDevice::setGnssReceiverSettings(const XsIntArray& gnssReceiverSettings)
{
	XsMessage snd(XMID_SetGnssReceiverSettings, 10);
	snd.setBusId(busId());

	if (gnssReceiverSettings.size() > 0)
		snd.setDataShort((uint16_t)gnssReceiverSettings[0], 0);//receiver type
	if (gnssReceiverSettings.size() > 1)
		snd.setDataShort((uint16_t)gnssReceiverSettings[1], 2);//receiver baud rate
	if (gnssReceiverSettings.size() > 2)
		snd.setDataShort((uint16_t)gnssReceiverSettings[2], 4);//receiver input rate
	if (gnssReceiverSettings.size() > 3)
		snd.setDataLong((uint32_t)gnssReceiverSettings[3], 6);//receiver options

	XsMessage ack;
	if (!doTransaction(snd, ack))
		return false;

	return true;
}

/*! \copybrief XsDevice::outputConfiguration
*/
XsOutputConfigurationArray MtDevice::outputConfiguration() const
{
	return XsOutputConfigurationArray();
}

/*! \brief Checks if this device can do orientation reset in firmware
	\param method The reset method
	\returns True if successful
*/
bool MtDevice::canDoOrientationResetInFirmware(XsResetMethod method)
{
	switch (method)
	{
		case XRM_DefaultAlignment:
		case XRM_DefaultHeading:
		case XRM_DefaultInclination:
			return true;

		case XRM_None:
			return false;

		default:
			break;
	}

	return updateRateForDataIdentifier(XDI_OrientationGroup) > 0;
}

/*! \copybrief XsDevice::scheduleOrientationReset */
bool MtDevice::scheduleOrientationReset(XsResetMethod method)
{
	switch (deviceState())
	{
		case XDS_Measurement:
		case XDS_Recording:
		case XDS_WaitingForRecordingStart:
		case XDS_FlushingData:
			if (method == XRM_StoreAlignmentMatrix)
				return false;

			if (canDoOrientationResetInFirmware(method))
				if (!XsDevice::scheduleOrientationReset(method))
					return false;

			break;

		case XDS_Config:
			if (method != XRM_StoreAlignmentMatrix)
				return false;

			if (canDoOrientationResetInFirmware(method))
			{
				if (!storeAlignmentMatrix())
					return false;
				// read stored value from emts by reinitializing
				return reinitialize();
			}
			return true;

		case XDS_Initial:
		case XDS_Destructing:
		default:
			return false;
	}
	return true;
}

/*! \brief Store the current alignment matrix in the device.
	\details The alignment matrix is computed when doing an orientation reset and needs to be stored
	explicitly in the device or it will be forgotten when the device restarts. This function will tell
	the device to store its alignment matrix or to write the locally computed alignment matrix to the
	device when filtering is done on the PC side.
	\returns true if the alignment matrix was successfully written to the non-volatile memory of the
	device
*/
bool MtDevice::storeAlignmentMatrix()
{
	if (!XsDevice::scheduleOrientationReset(XRM_StoreAlignmentMatrix))
		return false;

	return true;
}

/*! \brief The heading offset set for this device
*/
double MtDevice::headingOffset() const
{
	XsMessage snd(XMID_ReqHeading), rcv;
	snd.setBusId(busId());

	if (!doTransaction(snd, rcv))
		return 0;

	return (double)rcv.getDataFloat();
}

/*! \copybrief XsDevice::setLocationId
*/
bool MtDevice::setLocationId(int id)
{
	XsMessage snd(XMID_SetLocationId, XS_LEN_LOCATIONID);
	snd.setBusId(busId());
	snd.setDataShort((uint16_t)id);

	return doTransaction(snd);
}

/*! \copybrief XsDevice::locationId
*/
int MtDevice::locationId() const
{
	XsMessage snd(XMID_ReqLocationId), rcv;
	snd.setBusId(busId());

	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::serialBaudRate
*/
XsBaudRate MtDevice::serialBaudRate() const
{
	XsMessage snd(XMID_ReqBaudrate), rcv;
	snd.setBusId(busId());

	if (!doTransaction(snd, rcv))
		return XBR_Invalid;

	return (XsBaudRate)rcv.getDataByte();
}

/*! \copybrief XsDevice::hardwareVersion
*/
XsVersion MtDevice::hardwareVersion() const
{
	XsMessage snd(XMID_ReqHardwareVersion), rcv;
	if (!doTransaction(snd, rcv))
		return XsVersion();
	uint16_t hwv = rcv.getDataShort();
	return XsVersion(hwv >> 8, hwv & 0xff);
}

/*! \copybrief XsDevice::availableOnboardFilterProfiles
*/
XsFilterProfileArray MtDevice::availableOnboardFilterProfiles() const
{
	return m_hardwareFilterProfiles;
}

/*!	\brief Request the filter profiles headers from the hardware device and returns a vector with the found profiles.
	the order in the output vector is the same as the order in the hardware device.
*/
XsFilterProfileArray MtDevice::readFilterProfilesFromDevice() const
{
	XsFilterProfileArray result;

	XsMessage snd(XMID_ReqAvailableFilterProfiles);
	snd.setBusId(busId());

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return result;

	const char filterType = deviceConfigurationConst().deviceInfo(deviceId()).m_filterType;

	XsSize nofScenarios = rcv.getDataSize() / (1 + 1 + XS_LEN_FILTERPROFILELABEL);

	result.resize(nofScenarios);
	for (XsSize i = 0; i < nofScenarios; ++i)
	{
		uint8_t type = rcv.getDataByte(i * (1 + 1 + XS_LEN_FILTERPROFILELABEL));
		result[i].setType(type);
		result[i].setVersion(rcv.getDataByte(1 + i * (1 + 1 + XS_LEN_FILTERPROFILELABEL)));
		result[i].setLabel((const char*) rcv.getDataBuffer(2 + i * (1 + 1 + XS_LEN_FILTERPROFILELABEL)));
		result[i].setFilterType(filterType);
		XsString kind;
		if (type == XFPK_Base)
			kind = "base";
		else if (type == XFPK_Additional)
			kind = "additional";
		else if (type == XFPK_Heading)
			kind = "heading";
		result[i].setKind(kind.c_str());
	}
	return result;
}

/*! \brief Fetches available hardware scenarios
*/
void MtDevice::fetchAvailableHardwareScenarios()
{
	m_hardwareFilterProfiles.clear();
	m_hardwareFilterProfiles = readFilterProfilesFromDevice();
	std::sort(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(),
		[](XsFilterProfile const & left, XsFilterProfile const & right)
	{
		if (left.type() == right.type())
			return strcmp(left.label(), right.label()) < 0;
		else
			return left.type() < right.type();
	}
	);
}

/*! \copybrief XsDevice::productCode
*/
XsString MtDevice::productCode() const
{
	XsMessage snd(XMID_ReqProductCode), rcv;
	if (!doTransaction(snd, rcv))
		return XsString();

	const char* pc = (const char*) rcv.getDataBuffer();
	assert(pc);
	std::string result(pc ? pc : "                    ", 20);
	std::string::difference_type thingy = (std::string::difference_type) result.find(" ");
	if (thingy < 20)
		result.erase(result.begin() + thingy, result.end());
	return XsString(result);
}

/*! \copybrief XsDevice::reinitialize
*/
bool MtDevice::reinitialize()
{
	if (!readDeviceConfiguration())
		return false;

	clearDataCache();
	fetchAvailableHardwareScenarios();
	return true;
}

/*! \brief Restore to factory default settings
*/
bool MtDevice::restoreFactoryDefaults()
{
	if (!XsDevice::restoreFactoryDefaults())
		return false;

	return reinitialize();
}

/*! \copybrief XsDevice::onboardFilterProfile
*/
XsFilterProfile MtDevice::onboardFilterProfile() const
{
	XsMessage snd(XMID_ReqFilterProfile);
	snd.setBusId(busId());
	XsMessage rcv;

	//tries to get the current filter profile from the device
	if (!doTransaction(snd, rcv))
		return m_hardwareFilterProfile;//in case the transaction with the device fails or if the dev is in measurement

	auto profilesFromDevice = readFilterProfilesFromDevice();
	std::string fullMessageData((const char*)rcv.getDataBuffer());
	if (fullMessageData.empty())//for older devices where the profiles are numbers
	{
		auto numericFilter = rcv.getDataShort();
		//looks-up the full profile data among all the profiles available for the device
		for (auto currentProfile : profilesFromDevice)
		{
			if (currentProfile.type() == numericFilter)
				return currentProfile;
		}
	}
	else//for newer devices where the profiles are strings
	{
		//removes the checksum at the end of the message and keeps only the name of the profile
		// Stop when we see the first space or at message size, whichever is the lowest.
		fullMessageData = fullMessageData.substr(0, std::min(rcv.getDataSize(), fullMessageData.find(' ')));

		//looks up the full profile structure from the ones available on the device, by using the label
		for (auto currentProfile : profilesFromDevice)
		{
			if (currentProfile.label() == fullMessageData)
				return currentProfile;
		}

		//some strings have the names of two profiles separated by a '/', so the above look-up will not work
		//thus two separate profiles need to be found and "combined"
		auto splitterPosition = fullMessageData.find('/');
		if (splitterPosition != std::string::npos)
		{
			std::string combinedName;
			auto firstProfileName = fullMessageData.substr(0, splitterPosition);
			auto secondProfileName = fullMessageData.substr(splitterPosition + 1, fullMessageData.size());//the '/' is not taken into consideration
			XsFilterProfile firstProfile, secondProfile, combinedProfile;
			for (auto currentProfile : profilesFromDevice)
			{
				if (currentProfile.label() == firstProfileName)
					firstProfile = currentProfile;
				else if (currentProfile.label() == secondProfileName)
					secondProfile = currentProfile;
			}
			//sets all the data from the first profile
			combinedProfile = firstProfile;
			//and changes the name to a combination from the first and second profile
			combinedName.append(firstProfile.label()).append("/").append(secondProfile.label());
			combinedProfile.setLabel(combinedName.c_str());
			return combinedProfile;
		}
	}
	return m_hardwareFilterProfile;//in case everything above fails
}

/*! \copybrief XsDevice::setOnboardFilterProfile
*/
bool MtDevice::setOnboardFilterProfile(int profileType)
{
	if (deviceState() != XDS_Config)
		return false;

	XsFilterProfileArray::iterator item = std::find_if(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(),
			[profileType](XsFilterProfile const & p)
	{
		return p.type() == profileType;
	});
	if (item == m_hardwareFilterProfiles.end())
		return false;

	XsMessage snd(XMID_SetFilterProfile, XS_LEN_SETFILTERPROFILE);
	snd.setBusId(busId());
	snd.setDataShort((uint16_t)profileType);

	if (!doTransaction(snd))
		return false;

	m_hardwareFilterProfile = *item;
	return true;
}

/*! \copybrief XsDevice::setOnboardFilterProfile
*/
bool MtDevice::setOnboardFilterProfile(XsString const& profile)
{
	if (deviceState() != XDS_Config)
		return false;

	XsStringArray profileList(profile, "/");

	XsFilterProfileArray::iterator item[2];
	int i = 0;
	for (auto currentProfile : profileList)
	{
		item[i++] = std::find_if(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(),
				[currentProfile](XsFilterProfile const & p)
		{
			return currentProfile == p.label();
		});
		if (i == 2)
			break;
	}
	if (i == 0 || item[0] == m_hardwareFilterProfiles.end())
		return false;

	XsMessage snd(XMID_SetFilterProfile, profile.size());
	snd.setBusId(busId());
	snd.setDataBuffer((const uint8_t*)profile.c_str(), profile.size());

	if (!doTransaction(snd))
		return false;

	if (item[1] != m_hardwareFilterProfiles.end())
	{
		m_hardwareFilterProfile = *item[0];
		m_hardwareFilterProfile.setLabel(profile.c_str());	// Use info from first item, but label can be 2 profiles
	}
	else
		m_hardwareFilterProfile = *item[0];
	return true;
}

/*! \returns The accelerometer range for this device
	\details The range is an absolute maximum number. This means that if 100 is returned, the sensor range is (100, -100)
*/
double MtDevice::accelerometerRange() const
{
	return ::accelerometerRange(productCode(), hardwareVersion().major());
}

/*! \returns The accelerometer range for this device
	\details The range is an absolute maximum number. This means that if 300 is returned, the sensor range is (300, -300)
*/
double MtDevice::gyroscopeRange() const
{
	return ::gyroscopeRange(productCode());
}

/*! \brief Write the emts of the device to the open logfile
	\note The default implementation of MtDevice does not include any children
*/
void MtDevice::writeDeviceSettingsToFile()
{
	writeMessageToLogFile(m_emtsBlob);
}

/*! \copybrief XsDevice::setNoRotation
*/
bool MtDevice::setNoRotation(uint16_t duration)
{
	XsMessage snd(XMID_SetNoRotation, 2);
	snd.setBusId(busId());
	snd.setDataShort(duration);

	return doTransaction(snd);
}

/*!	\brief Set the current sensor position.
	\details Use this function to set the current position in latitude, longitude, altitude.
	\param lla The LLA vector
	\returns True if successful
	\sa initialPositionLLA
*/
bool MtDevice::setInitialPositionLLA(const XsVector& lla)
{
	uint8_t bid = (uint8_t)busId();
	if (bid == XS_BID_INVALID || bid == XS_BID_BROADCAST || lla.size() != 3)
		return false;

	XsMessage snd(XMID_SetLatLonAlt, XS_LEN_LATLONALT);
	snd.setDataFloat((float) lla[0], 0);
	snd.setDataFloat((float) lla[1], 4);
	snd.setDataFloat((float) lla[2], 8);
	snd.setBusId(bid);

	return doTransaction(snd);
}

/*! \returns the current sensor position
	\sa setInitialPositionLLA
*/
XsVector MtDevice::initialPositionLLA() const
{
	XsVector vec;
	XsMessage snd(XMID_ReqLatLonAlt), rcv;
	if (doTransaction(snd, rcv))
	{
		vec.setSize(3);
		for (XsSize i = 0; i < 3; i++)
			vec[i] = rcv.getDataDouble(i * 8);
	}
	return vec;
}

/*! \brief Convert mt sync ticks to microseconds
*/
uint32_t MtDevice::syncTicksToUs(uint32_t ticks) const
{
	return ((uint32_t)(((double) ticks) * XS_SYNC_CLOCK_TICKS_TO_US + 0.5));
}

/*! \brief Convert microseconds to mt sync ticks
*/
uint32_t MtDevice::usToSyncTicks(uint32_t us) const
{
	return ((uint32_t)(((double) us) * XS_SYNC_CLOCK_US_TO_TICKS + 0.5));
}

/*! \returns the error mode of the device.
	\see setErrorMode
*/
XsErrorMode MtDevice::errorMode() const
{
	XsMessage snd(XMID_ReqErrorMode), rcv;
	if (!doTransaction(snd, rcv))
		return XEM_Ignore;
	return static_cast<XsErrorMode>(rcv.getDataShort());
}

/*! \brief Set the error mode of the device
	\details The error mode determines how the device handles errors. See
	the low-level communication documentation for more details.
	\param em The error mode
	\returns True if successful
	\see errorMode
*/
bool MtDevice::setErrorMode(XsErrorMode em)
{
	XsMessage snd(XMID_SetErrorMode, 2);
	snd.setBusId(busId());
	snd.setDataShort(em);
	return doTransaction(snd);
}

/*! \brief Return the RS485 acknowledge transmission delay of the device
	\details The RS485 acknowledge transmission delay determines the minimal
	delay to response on request messages. See the low-level communication
	documentation for more details.
	\returns The delay value
	\see setRs485TransmissionDelay()
*/
uint16_t MtDevice::rs485TransmissionDelay() const
{
	XsMessage snd(XMID_ReqTransmitDelay), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \brief Set the RS485 acknowledge transmission delay of the device
	\param delay The delay value
	\returns True if successful
	\see rs485TransmissionDelay()
*/
bool MtDevice::setRs485TransmissionDelay(uint16_t delay)
{
	XsMessage snd(XMID_SetTransmitDelay, 2);
	snd.setBusId(busId());
	snd.setDataShort(delay);

	return doTransaction(snd);
}

/*! \brief Request data from the motion tracker
	\details The reply is handled by the live data path.
	This functionality is only available if outputSkipFactor() is set to 0xffff.
	\returns True if successful
*/
bool MtDevice::requestData()
{
	XsMessage snd(XMID_ReqData);
	snd.setBusId(busId());

	return sendRawMessage(snd);
}

/*! \brief Run a self test
	\returns The self test result
*/
XsSelfTestResult MtDevice::runSelfTest()
{
	XsMessage snd(XMID_RunSelfTest, 0);
	snd.setBusId(busId());
	XsMessage rcv;
	if (!doTransaction(snd, rcv, 3000))
		return XsSelfTestResult();

	return XsSelfTestResult::create(rcv.getDataShort());
}

/*! \copybrief XsDevice::storeFilterState
*/
bool MtDevice::storeFilterState()
{
	if (deviceState() == XDS_Config)
	{
		XsMessage snd(XMID_StoreFilterState);
		snd.setBusId(busId());

		if (doTransaction(snd))
			return true;
	}
	return false;
}

/*! \brief Calculates the frequency
	\param baseFrequency The base frequency to calculate with
	\param skipFactor The skip factor to calculate with
	\returns The calculated frequency
*/
int MtDevice::calcFrequency(int baseFrequency, uint16_t skipFactor)
{
	int result = baseFrequency / (skipFactor + 1);
	return result;
}

///@{ \name Log files
/*!
	\brief Set the read position of the open log file to the start of the file.

	If software filtering is enabled, the appropriate filters will be restarted
	as if the file was just opened. This function sets the lastResult value
	to indicate success (XRV_OK) or failure.

	\return True if successful
	\sa lastResult()
	\note This is a low-level file operation.
	\internal
*/
bool MtDevice::resetLogFileReadPosition()
{
	JLDEBUGG("");

	if (!XsDevice::resetLogFileReadPosition())
		return false;

	return true;
}

///@} end Log files

uint32_t MtDevice::supportedStatusFlags() const
{
	// essentially an unknown device, assume everything is supported
	return ~(uint32_t)0;
}

/*!	\brief Helper function to strip the hardware type from the product code
	\param code A productcode to be stripped
	\returns Code without the hardware type postfix
*/
XsString MtDevice::stripProductCode(const XsString& code)
{
	XsString hwtype = findHardwareType(code);
	if (hwtype.empty())
		return code;

	ptrdiff_t offset = code.findSubStr(hwtype);
	while (offset >= 0 && code[(unsigned int)offset] != '-')
		--offset;

	if (offset < 0)
		return code;

	return code.mid(0, (unsigned int)offset);
}

/*! \brief Returns the base update rate (Hz) corresponding to the \a dataType. Returns 0 if no update rate is available
*/
int MtDevice::getBaseFrequency(XsDataIdentifier dataType) const
{
	(void) dataType;
	return 0;
}
