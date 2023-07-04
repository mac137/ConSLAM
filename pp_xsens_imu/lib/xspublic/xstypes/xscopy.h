
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

#ifndef XSCOPY_H
#define XSCOPY_H

#define XSLISTCOPY(C)	\
	if (copy == thisPtr)\
	{\
		return;\
	}\
	C##_assign(copy, thisPtr->m_size, thisPtr->m_data)

#define XSLISTSWAP3(C, B, S)	\
	if ((!a->m_data || (a->m_flags & XSDF_Managed)) && (!b->m_data || (b->m_flags & XSDF_Managed))) {\
		B tmp;\
		*((C**) &tmp.m_data) = a->m_data;\
		*((XsSize*) &tmp.m_size) = a->m_size;\
		*((XsSize*) &tmp.m_flags) = a->m_flags;\
		*((C**) &a->m_data) = b->m_data;\
		*((XsSize*) &a->m_size) = b->m_size;\
		*((XsSize*) &a->m_flags) = b->m_flags;\
		*((C**) &b->m_data) = tmp.m_data;\
		*((XsSize*) &b->m_size) = tmp.m_size;\
		*((XsSize*) &b->m_flags) = tmp.m_flags;\
	} else {	/* elementwise swap */ \
		XsSize i;\
		assert(a->m_size == b->m_size);\
		for (i = 0; i < a->m_size; ++i) S(&a->m_data[i], &b->m_data[i]);\
	}

#define XSLISTSWAP2(C, B)	XSLISTSWAP3(C, B, C##_swap)

#define XSLISTSWAP(C)	XSLISTSWAP2(C, C##Array)

#ifdef __cplusplus
/*! \brief Byte-wise copy for data (de-)serialization
	\details The function essentially performs a memcpy from \a src to \a tgt using the size of \a tgt
	\param tgt The destination of the copy (reference)
	\param src The source of the copy (pointer)
*/
template <typename T>
inline static void xsByteCopy(T& tgt, void const* src)
{
	memcpy(&tgt, src, sizeof(T));
}

/*! \brief Byte-wise copy for data (de-)serialization followed by a multiplication
	\details The function essentially performs a memcpy from \a src to \a tgt using the size of \a tgt
	\param tgt The destination of the copy (reference)
	\param src The source of the copy (pointer)
	\param mul The multiplication value
*/
template <typename T>
inline static void xsByteCopyMultiply(T& tgt, void const* src, T mul)
{
	memcpy(&tgt, src, sizeof(T));
	tgt *= mul;
}
#endif

#endif
