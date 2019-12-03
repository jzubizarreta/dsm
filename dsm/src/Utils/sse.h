/**
* This file is part of DSM.
*
* Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza
* Developed by Jon Zubizarreta,
* for more information see <https://github.com/jzubizarreta/dsm>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSM. If not, see <http://www.gnu.org/licenses/>.
*/

// Useful mathematical function to work with SSE instructions

#pragma once

#include <cstdint>
#include <limits>

#include "dsm/BuildFlags.h"

namespace dsm
{

#if defined(ENABLE_SSE)

#include <xmmintrin.h>
#include <emmintrin.h>

	// log function approximation with sse
	inline __m128 logapprox_ps(__m128 x)
	{
		// 1) transform the problem into delimited log2
		// 2) obtain exponential and mantissa coefficients
		// 3) log2(x) polynomial approximation
		// 4) log(x) = log2(x) * log(2)

		// floating-point numbers are represented by a mantissa and an exponent
		// we do this to restrict the values of the polynomial approximation
		// in this case, we will approximate log2(x) in the range [1,2]
		// http://cstl-csm.semo.edu/xzhang/Class%20Folder/CS280/Workbook_HTML/FLOATING_tut.htm
		// the exponent value is stored with a bias of 127. we need to subtract it.
		// f = mant * (2^exp)

		// bits as int
		const __m128i xi = _mm_castps_si128(x);
		const __m128i c1 = _mm_set1_epi32(0x7FFFFF);
		const __m128i c2 = _mm_set1_epi32(0x3F800000);

		// 23 bits for the mantissa
		const __m128 exp = _mm_cvtepi32_ps(_mm_sub_epi32(_mm_srli_epi32(xi, 23), _mm_set1_epi32(127)));		

		// mantissa is always 1.xxxxxxxxx, thus we are in the range [1,2]
		const __m128 mant = _mm_castsi128_ps(_mm_or_si128(_mm_and_si128(xi, c1), c2));

		// polynomial approximation of log2(x) for the range [1,2]
		// obtained from Matlab using the function polyfit()
		// log2(x) = k0 + k1*x + k2*x^2 + k3*x^3 + k4*x^4 + k5*x^5
		const __m128 k0 = _mm_set_ps1(-2.786804907259363f);
		const __m128 k1 = _mm_set_ps1(5.046850880751210f);
		const __m128 k2 = _mm_set_ps1(-3.492463531811749f);
		const __m128 k3 = _mm_set_ps1(1.593883053111344f);
		const __m128 k4 = _mm_set_ps1(-0.404861876828451f);
		const __m128 k5 = _mm_set_ps1(0.043428314915453f);

		const __m128 poly = _mm_mul_ps(mant, _mm_add_ps(k1,
								_mm_mul_ps(mant, _mm_add_ps(k2,
									_mm_mul_ps(mant, _mm_add_ps(k3,
										_mm_mul_ps(mant, _mm_add_ps(k4,
											_mm_mul_ps(mant, k5)))))))));

		// log2(f) = log2(mant) + exp
		// log(f) = log2(f) * log(2)
		const __m128 log2 = _mm_set_ps1(0.693147180559945f);
		__m128 logApprox = _mm_mul_ps(_mm_add_ps(_mm_add_ps(poly, k0), exp), log2);

		// finally set special cases	
		const __m128 pinf = _mm_set_ps1(std::numeric_limits<float>::infinity());
		const __m128 minf = _mm_set_ps1(-std::numeric_limits<float>::infinity());
		const __m128 nan = _mm_set_ps1(std::numeric_limits<float>::quiet_NaN());

		const __m128 isInf = _mm_cmpeq_ps(x, pinf);
		const __m128 isZero = _mm_cmpeq_ps(x, _mm_setzero_ps());
		const __m128 isLessZero = _mm_cmplt_ps(x, _mm_setzero_ps());
		const __m128 isNan = _mm_cmpunord_ps(x, x);

		logApprox = _mm_or_ps(_mm_and_ps(isInf, pinf), _mm_andnot_ps(isInf, logApprox));			// log(Inf) = Inf
		logApprox = _mm_or_ps(_mm_and_ps(isZero, minf), _mm_andnot_ps(isZero, logApprox));			// log(0) = -Inf
		logApprox = _mm_or_ps(_mm_and_ps(isLessZero, nan), _mm_andnot_ps(isLessZero, logApprox));	// log(<0) = nan
		logApprox = _mm_or_ps(_mm_and_ps(isNan, nan), _mm_andnot_ps(isNan, logApprox));				// log(nan) = nan

		return logApprox;
	}

#endif

}