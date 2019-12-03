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

#pragma once

#include "dsm/BuildFlags.h"

#if defined(ENABLE_SSE)
	#include <smmintrin.h>
#endif

namespace dsm
{
	/* reads bilinear interpolated element from a T* array
	*/
	template<typename T, typename U = float>
	inline U bilinearInterpolation(const T* const mat, const U &x, const U &y, const int &width)
	{
		const int int_x = static_cast<const int>(x);
		const int int_y = static_cast<const int>(y);

		const U dx = x - int_x;
		const U dy = y - int_y;
		const U dxdy = dx*dy;

		const T* const source = mat + int_x + int_y*width;

		return (dxdy * source[1 + width] + (dx - dxdy) * source[1] 
			+ (dy - dxdy) * source[width] + (1.f - dx - dy + dxdy) * source[0]);
	}

#if defined(ENABLE_SSE)

	/* reads bilinear interpolated element from a float* array using sse instructions
	*/
	inline __m128 bilinearInterpolationFloatSSE(const float* const mat, const __m128 &x, const __m128 &y, const int &width)
	{
		const __m128 w = _mm_set_ps1((float)width);
		const __m128 ones = _mm_set_ps1(1.f);

		const __m128 int_x = _mm_floor_ps(x);
		const __m128 int_y = _mm_floor_ps(y);

		const __m128 dx = _mm_sub_ps(x, int_x);
		const __m128 dy = _mm_sub_ps(y, int_y);
		const __m128 dxdy = _mm_mul_ps(dx, dy);

		alignas(16) float index[4];		// 16-byte aligned
		_mm_store_ps(index, _mm_add_ps(_mm_mul_ps(int_y, w), int_x));

		__m128 val1 = _mm_loadu_ps(mat + (int)index[0]);				// source[0], source[1], 0, 0
		__m128 val2 = _mm_loadu_ps(mat + (int)index[0] + width);		// source[width], source[1 + width], 0, 0

		val1 = _mm_movelh_ps(val1, _mm_loadu_ps(mat + (int)index[1]));			// source[0], source[1], source2[0], source2[1]
		val2 = _mm_movelh_ps(val2, _mm_loadu_ps(mat + (int)index[1] + width));	// source[width], source[1 + width], source2[width], source2[1 + width]

		__m128 val3 = _mm_loadu_ps(mat + (int)index[2]);				// source3[0], source3[1], 0, 0
		__m128 val4 = _mm_loadu_ps(mat + (int)index[2] + width);		// source3[width], source3[1 + width], 0, 0

		val3 = _mm_movelh_ps(val3, _mm_loadu_ps(mat + (int)index[3]));			// source3[0], source3[1], source4[0], source4[1]
		val4 = _mm_movelh_ps(val4, _mm_loadu_ps(mat + (int)index[3] + width));	// source3[width], source3[1 + width], source4[width], source4[1 + width]

		// reorder
		const __m128 sourceAt0 = _mm_shuffle_ps(val1, val3, _MM_SHUFFLE(2, 0, 2, 0));		// source1[0], source2[0], source3[0], source4[0]
		const __m128 sourceAt1 = _mm_shuffle_ps(val1, val3, _MM_SHUFFLE(3, 1, 3, 1));		// source1[1], source2[1], source3[1], source4[1]
		const __m128 sourceAtW0 = _mm_shuffle_ps(val2, val4, _MM_SHUFFLE(2, 0, 2, 0));		// source1[width], source2[width], source3[width], source4[width]
		const __m128 sourceAtW1 = _mm_shuffle_ps(val2, val4, _MM_SHUFFLE(3, 1, 3, 1));		// source1[1 + width], source2[1 + width], source3[1 + width], source4[1 + width]

		// (1.f - dx - dy + dxdy) * source[0]
		val1 = _mm_mul_ps(_mm_sub_ps(_mm_add_ps(ones, dxdy), _mm_add_ps(dx, dy)), sourceAt0);

		// (dx - dxdy) * source[1]
		val2 = _mm_mul_ps(_mm_sub_ps(dx, dxdy), sourceAt1);

		// (dy - dxdy) * source[width]
		val3 = _mm_mul_ps(_mm_sub_ps(dy, dxdy), sourceAtW0);

		// dxdy * source[1 + width]
		val4 = _mm_mul_ps(dxdy, sourceAtW1);

		return _mm_add_ps(_mm_add_ps(_mm_add_ps(val1, val2), val3), val4);
	}

	/* reads bilinear interpolated element from an unsigned char* array using sse instructions
	*/
	inline __m128 bilinearInterpolationUcharSSE(const unsigned char* const mat, const __m128 &x, const __m128 &y, const int &width)
	{
		const __m128 w = _mm_set_ps1((float)width);
		const __m128 ones = _mm_set_ps1(1.f);

		const __m128 int_x = _mm_floor_ps(x);
		const __m128 int_y = _mm_floor_ps(y);

		const __m128 dx = _mm_sub_ps(x, int_x);
		const __m128 dy = _mm_sub_ps(y, int_y);
		const __m128 dxdy = _mm_mul_ps(dx, dy);

		alignas(16) float index[4];		// 16-byte aligned
		_mm_store_ps(index, _mm_add_ps(_mm_mul_ps(int_y, w), int_x));

		// read and convert from uchar to float
		__m128 val1 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[0]))));			
		__m128 val2 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[0] + width))));	

		__m128 val3 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[1]))));			
		__m128 val4 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[1] + width))));	

		val1 = _mm_movelh_ps(val1, val3);	
		val2 = _mm_movelh_ps(val2, val4);	

		// read and convert from uchar to float
		val3 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[2]))));					
		val4 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[2] + width))));			

		__m128 val5 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[3]))));			
		__m128 val6 = _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)(mat + (int)index[3] + width))));	

		val3 = _mm_movelh_ps(val3, val5);	
		val4 = _mm_movelh_ps(val4, val6);	

		// reorder
		const __m128 sourceAt0 = _mm_shuffle_ps(val1, val3, _MM_SHUFFLE(2, 0, 2, 0));
		const __m128 sourceAt1 = _mm_shuffle_ps(val1, val3, _MM_SHUFFLE(3, 1, 3, 1));
		const __m128 sourceAtW0 = _mm_shuffle_ps(val2, val4, _MM_SHUFFLE(2, 0, 2, 0));
		const __m128 sourceAtW1 = _mm_shuffle_ps(val2, val4, _MM_SHUFFLE(3, 1, 3, 1));

		//(1.f - dx - dy + dxdy) * bp[0]
		val1 = _mm_mul_ps(_mm_sub_ps(_mm_add_ps(ones, dxdy), _mm_add_ps(dx, dy)), sourceAt0);

		//(dx - dxdy) * bp[1]
		val2 = _mm_mul_ps(_mm_sub_ps(dx, dxdy), sourceAt1);

		//(dy - dxdy) * bp[width]
		val3 = _mm_mul_ps(_mm_sub_ps(dy, dxdy), sourceAtW0);

		//dxdy * bp[1 + width]
		val4 = _mm_mul_ps(dxdy, sourceAtW1);

		return _mm_add_ps(_mm_add_ps(_mm_add_ps(val1, val2), val3), val4);
	}

#endif
}
