
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

#ifndef XSQUATERNIONARRAY_H
#define XSQUATERNIONARRAY_H

#include "xsarray.h"

#ifdef __cplusplus
#include "xsquaternion.h"
extern "C" {
#endif

extern XsArrayDescriptor const XSTYPES_DLL_API g_xsQuaternionArrayDescriptor;

#ifndef __cplusplus
#define XSQUATERNIONARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsQuaternionArrayDescriptor)
struct XsQuaternion;

XSARRAY_STRUCT(XsQuaternionArray, struct XsQuaternion);
typedef struct XsQuaternionArray XsQuaternionArray;

XSTYPES_DLL_API void XsQuaternionArray_construct(XsQuaternionArray* thisPtr, XsSize count, struct XsQuaternion const* src);
#define XsQuaternionArray_destruct(thisPtr)		XsArray_destruct(thisPtr)
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
struct XsQuaternionArray : public XsArrayImpl<XsQuaternion, g_xsQuaternionArrayDescriptor, XsQuaternionArray>
{
	//! \brief Constructs an XsQuaternionArray
	inline explicit XsQuaternionArray(XsSize sz = 0, XsQuaternion const* src = 0)
		: ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsQuaternionArray as a copy of \a other
	inline XsQuaternionArray(XsQuaternionArray const& other)
		: ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsQuaternionArray that references the data supplied in \a ref
	inline explicit XsQuaternionArray(XsQuaternion* ref, XsSize sz, XsDataFlags flags /* = XSDF_None */)
		: ArrayImpl(ref, sz, flags)
	{
	}

#ifndef SWIG
	/*! \brief Swap the contents the \a first and \a second array */
	friend void swap(XsQuaternionArray& first, XsQuaternionArray& second)
	{
		first.swap(second);
	}
#endif

#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsQuaternionArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsQuaternionArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif

};
#endif

#endif
