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

#include <math.h>

#include "dsm/BuildFlags.h"

#if defined(ENABLE_SSE)
	#include <xmmintrin.h>
#endif

/* Robust estimators are resistant to outliers 
* M-estimators are a maximum likelihood type estimator
* If the rho function can be differentiated, the M-estimator is said to be a psi-type
*/
namespace LossFunction
{
	namespace Huber
	{
		// The value c = 1.345 provides an asymptotic efficiency 
		// of 95% for the normal distribuion
		const float SCALE = 1.345f;

		// Loss Function Norm calculation
		// input: least squares residual
		// output: huber norm residual
		template<typename T>
		T rho(T constant, T residual)
		{
			if (fabs(residual) < constant)
			{
				return (T)0.5*residual*residual;
			}
			else
			{
				return constant * (fabs(residual) - (T)0.5*constant);
			}
		}

		// Loss Function Influence calculation
		// If the rho function can be differentiated, the M-estimator is said to be a psi-type
		// Instead of minimising the function directly, it may be simpler
		// to use the function’s first order conditions set to zero
		// input: least squares residual
		// output: psi value for the input residual
		template<typename T>
		T psi(T constant, T residual)
		{
			if (fabs(residual) < constant)
			{
				return residual;
			}
			else
			{
				return constant * residual / fabs(residual);
			}
		}

		// Loss Function Weight calculation
		// input: least squares residual
		// output: weight value for the input residual 
		template<typename T>
		T weight(T constant, T residual)
		{
			if (fabs(residual) < constant)
			{
				return (T)1;
			}
			else
			{
				return constant / fabs(residual);
			}
		}

#if defined(ENABLE_SSE)
		inline __m128 weightSSE(__m128 constant, __m128 residual)
		{
			// absolute value
			residual = _mm_max_ps(residual, _mm_sub_ps(_mm_setzero_ps(), residual));

			__m128 w_loss = _mm_cmplt_ps(residual, constant);

			w_loss = _mm_or_ps(_mm_and_ps(w_loss, _mm_set_ps1(1.f)),
				_mm_andnot_ps(w_loss, _mm_mul_ps(constant, _mm_rcp_ps(residual))));

			return w_loss;
		}
#endif
	}

	namespace Tukey
	{
		// The value c = 4.6851 provides an asymptotic efficiency 
		// of 95% for the normal distribuion
		const float SCALE = 4.6851f;

		// Loss Function Norm calculation
		// input: least squares residual
		// output: tukey norm residual
		template<typename T>
		T rho(T constant, T residual)
		{
			if (fabs(residual) <= constant)
			{
				return (T)1 - pow((T)1 - pow(residual / constant, (T)2), (T)3);
			}
			else
			{
				return (T)1;
			}
		}

		// Loss Function Influence calculation
		// If the rho function can be differentiated, the M-estimator is said to be a psi-type
		// Instead of minimising the function directly, it may be simpler
		// to use the function’s first order conditions set to zero
		// input: least squares residual
		// output: psi value for the input residual
		template<typename T>
		T psi(T constant, T residual)
		{
			if (fabs(residual) < constant)
			{
				return residual * pow((T)1 - pow(residual / constant, (T)2), (T)2);
			}
			else
			{
				return (T)0;
			}
		}

		// Loss Function Weight calculation
		// input: least squares residual
		// output: weight value for the input residual
		template<typename T>
		T weight(T constant, T residual)
		{
			if (fabs(residual) < constant)
			{
				return pow((T)1 - pow(residual / constant, (T)2), (T)2);
			}
			else
			{
				return (T)0;
			}
		}

#if defined(ENABLE_SSE)
		inline __m128 weightSSE(__m128 constant, __m128 residual)
		{
			// absolute value
			residual = _mm_max_ps(residual, _mm_sub_ps(_mm_setzero_ps(), residual));

			__m128 w_loss = _mm_cmplt_ps(residual, constant);

			__m128 trueValue = _mm_mul_ps(residual, _mm_rcp_ps(constant));
			trueValue = _mm_sub_ps(_mm_set_ps1(1.f), _mm_mul_ps(trueValue, trueValue));
			trueValue = _mm_mul_ps(trueValue, trueValue);

			w_loss = _mm_and_ps(w_loss, trueValue);

			return w_loss;
		}
#endif
	}
}