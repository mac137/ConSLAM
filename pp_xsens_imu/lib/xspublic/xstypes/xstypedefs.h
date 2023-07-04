
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

#ifndef XSTYPEDEFS_H
#define XSTYPEDEFS_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifndef XSENS_SINGLE_PRECISION
	#include <stddef.h>
	typedef double XsReal;	//!< Defines the floating point type used by the Xsens libraries
	typedef size_t XsSize;	//!< XsSize must be unsigned number!
	typedef ptrdiff_t XsSizeSigned;	//!< signed variant of XsSize
	#define XSREAL_ALLOWS_MEMCPY	1
	#ifndef PRINTF_SIZET_MODIFIER
		#if defined(XSENS_64BIT)
			#if defined(__APPLE__)
				#define PRINTF_SIZET_MODIFIER "l"
			#else
				#define PRINTF_SIZET_MODIFIER PRINTF_INT64_MODIFIER
			#endif
		#else
			#if defined(__APPLE__)
				#define PRINTF_SIZET_MODIFIER "l"
			#else
				#define PRINTF_SIZET_MODIFIER PRINTF_INT32_MODIFIER
			#endif
		#endif
	#endif // PRINTF_SIZET_MODIFIER
#else
	typedef float XsReal;			//!< Defines the floating point type used by the Xsens libraries
	typedef unsigned int XsSize;	//!< XsSize must be unsigned number!
	typedef int XsSizeSigned;		//!< signed variant of XsSize
	#define XSREAL_ALLOWS_MEMCPY	1
	#define PRINTF_SIZET_MODIFIER ""
#endif // XSENS_SINGLE_PRECISION


/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief These flags define the behaviour of data contained by Xsens data structures
	\details Normally, the user should never need to use these directly.
*/
enum XsDataFlags
{
	XSDF_None		= 0,	//!< No flag set
	XSDF_Managed	= 1,	//!< The contained data should be managed (freed) by the object, when false, the object assumes the memory is freed by some other process after its destruction
	XSDF_FixedSize	= 2,	//!< The contained data points to a fixed-size buffer, this allows creation of dynamic objects on the stack without malloc/free overhead.
	XSDF_Empty		= 4		//!< The object contains undefined data / should be considered empty. Usually only relevant when XSDF_FixedSize is also set, as otherwise the data pointer will be NULL and empty-ness is implicit.
};
/*! @} */
typedef enum XsDataFlags XsDataFlags;

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API const char* XsDataFlags_toString(XsDataFlags f);

#ifdef __cplusplus
} // extern "C"
/*! \brief \copybrief XsDataFlags_toString \sa XsDataFlags_toString */
inline const char* toString(XsDataFlags s)
{
	return XsDataFlags_toString(s);
}
#endif
#ifdef SWIG
	// note that there is no XSDEPRECATED_END, deprecated definitions are expected to be the last definitions in a list
	#define XSDEPRECATED_START	private: /* deprecated start */
#else
	// note that there is no XSDEPRECATED_END, deprecated definitions are expected to be the last definitions in a list
	#define XSDEPRECATED_START	/* deprecated start */
#endif
#ifndef __cplusplus
	// define BOOL, TRUE and FALSE
	#ifndef BOOL
		typedef int BOOL;
	#endif

	#ifndef TRUE
		#define TRUE (1)
	#endif

	#ifndef FALSE
		#define FALSE (0)
	#endif
	#define XSFALLTHROUGH	/* fallthrough */
	#define XSNORETURN		/* noreturn */
	#define XSDEPRECATED	/* deprecated */
#else
	#if __cplusplus >= 201703L
		#define XSFALLTHROUGH	[[fallthrough]] /* FALLTHRU fallthrough */
		#define XSNORETURN		[[noreturn]] /* noreturn */
		#define XSDEPRECATED	[[deprecated]] /* deprecated */
	#else
		#if defined(__GNUC__)
			#define XSFALLTHROUGH	[[fallthrough]] /* FALLTHRU fallthrough */
			#define XSNORETURN		[[noreturn]] /* noreturn */
		#else
			#define XSFALLTHROUGH	/* FALLTHRU fallthrough */
			#define XSNORETURN		/* noreturn */
		#endif
		#if __cplusplus >= 201402L
			#define XSDEPRECATED	[[deprecated]] /* deprecated */
		#else
			#define XSDEPRECATED	/* deprecated */
		#endif
	#endif
#endif // __cplusplus

#define XS_ENUM_TO_STR_CASE(value) case value: return #value

// different alignment commands for gcc / MSVS for structures that need to be 1-byte aligned.
#if defined (SWIG)
	#define XS_PACKED_STRUCT_START
	#define XS_PACKED_STRUCT_END
#elif defined (_MSC_VER)
	#define XS_PACKED_STRUCT_START __pragma(pack(push, 1));
	#define XS_PACKED_STRUCT_END __pragma(pack(pop));
#elif defined (__ICCARM__)
	#define XS_PACKED_STRUCT_START _Pragma("pack(push, 1)");
	#define XS_PACKED_STRUCT_END _Pragma("pack(pop)");
#elif defined (__ICC8051__)
	#define XS_PACKED_STRUCT_START
	#define XS_PACKED_STRUCT_END
#elif defined (__GNUC__)
	#define XS_PACKED_STRUCT_START
	#define XS_PACKED_STRUCT_END
#elif (defined(__arm__) && defined(__ARMCC_VERSION))
	#define XS_PACKED_STRUCT_START
	#define XS_PACKED_STRUCT_END
#elif (defined(__ADSP21000__))
	#define XS_PACKED_STRUCT_START
	#define XS_PACKED_STRUCT_END
#else
	#error "Structure packing macros not defined for this compiler"
	#define XS_PACKED_STRUCT_START
	#define XS_PACKED_STRUCT_END
#endif

#ifdef __GNUC__
	#define XS_PACKED_STRUCT __attribute__((__packed__))
#else
	#define XS_PACKED_STRUCT
#endif

#if defined (__ICCARM__)
	#define XS_PACKED_POINTER __packed
#else
	#define XS_PACKED_POINTER
#endif

#endif
