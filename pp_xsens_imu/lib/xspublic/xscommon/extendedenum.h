
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

#ifndef EXTENDEDENUM_H
#define EXTENDEDENUM_H

#ifdef __cplusplus

/*! \brief This template can be used to extend an existing enum with new values.

	Example, extend QNetworkReply::NetworkError with 2 new values:
	enum ICloudServiceError : int
	{
		CSE_GenericParsingError = QNetworkReply::NetworkError::UnknownServerError+1,
		CSE_FileCreationError,
	};

	typedef ExtendedEnum<QNetworkReply::NetworkError, ICloudServiceError> CloudServiceError;


	It can now be used as such:
	void func(QNetworkReply::NetworkError err)
	void func(CloudServiceError err)
	void func(ICloudServiceError err)

	switch(err.value())
	{
	case QNetworkReply::NoError:
	case CSE_GenericParsingError:
	}

	if (err == QNetworkReply::NoError)
	if (err == CSE_GenericParsingError)

	etc...

	\note There is no way for the compiler to ensure that the enum values do not overlap!
*/
template <typename BaseEnumT, typename ExtensionEnumT, BaseEnumT defValue = BaseEnumT(0)>
class ExtendedEnum
{
public:
	//! \brief default constructor, initializes to defValue template parameter
	inline ExtendedEnum()
		: m_value(defValue)
	{
	}

	//! \brief copy constructor
	inline ExtendedEnum(ExtendedEnum const& e)
		: m_value(e.m_value)
	{
	}

	//! \brief Initializing constructor, initializes to \a e
	inline ExtendedEnum(ExtensionEnumT e)
		: m_value(static_cast<int>(e))
	{
	}

	//! \brief Initializing constructor, initializes to \a e
	inline ExtendedEnum(BaseEnumT e)
		: m_value(static_cast<int>(e))
	{
	}

	//! \brief Initializing constructor, initializes to equivalent of integer value \a val
	explicit inline ExtendedEnum(int val)
		: m_value(val)
	{
	}

	//! \brief Implicit conversion operator to ExtensionEnumT
	inline operator ExtensionEnumT() const
	{
		return static_cast<ExtensionEnumT>(m_value);
	}

	//! \brief Implicit conversion operator to BaseEnumT
	inline operator BaseEnumT() const
	{
		return static_cast<BaseEnumT>(m_value);
	}

	//! \brief Assignment operator, copies from \a e
	inline ExtendedEnum& operator = (ExtendedEnum const& e) = default;

	//! \brief Assignment operator, copies from \a e
	inline ExtendedEnum& operator = (ExtensionEnumT e)
	{
		m_value = static_cast<int>(e);
		return *this;
	}

	//! \brief Assignment operator, copies from \a e
	inline ExtendedEnum& operator = (BaseEnumT e)
	{
		m_value = static_cast<int>(e);
		return *this;
	}

	//! \brief Returns the integer value, useful in a switch
	inline int value() const
	{
		return m_value;
	}

private:
	int m_value;	//!< The contained value, stored as int
};

#endif

#endif
