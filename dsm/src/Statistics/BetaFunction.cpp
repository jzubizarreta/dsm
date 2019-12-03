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

#include "BetaFunction.h"

#include <iostream>
#include <algorithm>	// for min, max
#include <cmath>		// for lgamma
#include <limits>		// for numer_limits

namespace dsm
{
	float beta(float a, float b)
	{
		return exp(lgamma(a) + lgamma(b) - lgamma(a + b));
	}

	float betaln(float a, float b)
	{
		return lgamma(a) + lgamma(b) - lgamma(a + b);
	}

	float beta(float x, float a, float b, Tail tail)
	{
		// a, b >= 0
		// 0 <= x <= 1
		if (x < 0.f || x > 1.f || a <= 0.f || b <= 0.f)
		{
			std::cout << "beta() wrong input arguments!\n";
			return std::numeric_limits<float>::quiet_NaN();
		}

		// fast return values
		if (x == 0.f)
		{
			if (tail == lower) return 0.f;
			else return beta(a, b);
		}
		else if (x == 1.f)
		{
			if (tail == lower) return beta(a, b);
			else return 0.f;
		}

		// check tail
		// the implementation computes the lower tail
		if (tail == upper)
		{
			// use symmetric relation
			return beta(a, b) - beta(x, a, b, lower);
		}

		// the continued fraction converges rapidly for x < (a+1)/(a+b+2)
		// we can use the symmetry relation Eq. 26.5.2
		if (x > (a + 1.f) / (a + b + 2.f))
		{
			return beta(a, b) - beta(1.f - x, b, a, lower);
		}

		// We follow the Eq. 26.5.8 to compute the value

		// factors that multiply the continued fraction
		const float front = exp(a*log(x) + b * log(1.f - x)) / a;

		// continued fraction 
		// Use Lentz's algorithm to evaluate it
		const float eps = 1e-30f;

		float cf = 1.f;
		float c = 1.f;
		float e = 0.f;

		int i = 0;
		while (true)
		{
			const int m = i / 2;

			// alternate d
			float d;
			if (i == 0)
			{
				d = 1.f;
			}
			else if (i % 2 == 0)
			{
				d = (m*(b - m)*x) / ((a + 2.f*m - 1.f)*(a + 2.f*m));
			}
			else
			{
				d = -((a + m)*(a + b + m)*x) / ((a + 2.f*m)*(a + 2.f*m + 1.f));
			}

			// do Lentz step
			e = 1.f + d * e;
			if (fabs(e) < eps) e = eps;
			e = 1.f / e;

			c = 1.f + d / c;
			if (fabs(c) < eps) c = eps;

			const float ce = c * e;
			cf *= ce;

			// convergence
			if (fabs(ce - 1.f) < 1e-8f) 
			{
				break;
			}

			i++;
		}

		// correct the continued fraction from Lentz's algorithm
		// multiply by the front factor
		return front * (cf - 1.f);
	}

	float betainc(float x, float a, float b, Tail tail)
	{
		// regularized incomplete beta function
		return beta(x, a, b, tail) / beta(a, b);
	}

	float betaincinv(float p, float a, float b, Tail tail)
	{
		// solve the inverse incomplete beta function
		// using Newton's method and betainc function
		// f  = betainc(x,a,b) - p,
		// f' = x^(a-1)*(1-x)^(b-1)/beta(a,b),

		// a, b >= 0
		// 0 <= p <= 1
		if (p < 0.f || p > 1.f || a <= 0.f || b <= 0.f)
		{
			std::cout << "betaincinv() wrong input arguments!\n";
			return std::numeric_limits<float>::quiet_NaN();
		}

		// easy return values
		if (p == 0.f)
		{
			if (tail == lower) return 0.f;
			else return 1.f;
		}
		else if (p == 1.f)
		{
			if (tail == lower) return 1.f;
			else return 0.f;
		}

		// check tail
		// the implementation computes the lower tail
		if (tail == upper)
		{
			return 1.f - betaincinv(p, b, a, lower);
		}

		// change tail if required
		// we will work with the smallest tail
		if (p > 0.5f)
		{
			return 1.f - betaincinv(1.f - p, b, a, lower);
		}

		// log of beta
		const float lbeta = betaln(a, b);

		// Starting point, we will follow:
		// Cran, G. W., et al. “Remark AS R19 and Algorithm AS 109: A Remark on Algorithms: AS 63: 
		// The Incomplete Beta Integral AS 64: Inverse of the Incomplete Beta Function Ratio.” 
		// Journal of the Royal Statistical Society. Series C (Applied Statistics), 
		// vol. 26, no. 1, 1977, pp. 111–114

		float x;		// the result

		const float r = sqrt(-2.f*log(p));
		const float y = r - (2.30753f + 0.27061f*r) / (1.f + (0.99229f + 0.04481f*r)*r);

		if (a > 1.f && b > 1.f)
		{
			const float z = (y*y - 3.f) / 6.f;
			const float s = 1.f / (2.f * a - 1.f);
			const float t = 1.f / (2.f * b - 1.f);
			const float h = 2.f / (s + t);
			const float w = y*sqrt(h + z) / h - (t - s)*(z + 5.f / 6.f - 2.f / (3.f * h));
			x = a / (a + b*exp(2 * w));
		}
		else
		{
			const float z = 2.f * b;
			float t = 1.f / (9.f * b); 
			t = (1.f - t + y*sqrt(t));
			t = z*t*t*t;

			if (t > 0.f)
			{
				t = (4.f * a + z - 2.f) / t;

				if (t > 1.f)
				{
					x = 1.f - 2.f / (t + 1.f);
				}
				else
				{
					x = exp((log(p*a) + lbeta) / a);
				}
			}
			else
			{
				x = 1.f - exp((log((1.f - p)*b) + lbeta) / b);
			}
		}

		// solve for x by a Newton's method
		// Newton's method: new x = x - f/f'
		// https://en.wikipedia.org/wiki/Newton%27s_method

		x = std::min(std::max(x, 0.0001f), 0.9999f);
		const float am1 = a - 1.f;
		const float bm1 = b - 1.f;

		// Convergence tolerances
		const int maxIter = 1000;
		const float xtol = 1e-05f;
		const float ftol = 1e-06f;

		float oldf = std::numeric_limits<float>::infinity();
		float oldx = std::numeric_limits<float>::infinity();

		// control variables
		float xlo = 0.f;		// boundary
		float xhi = 1.f;
		int it = 0;				// number of iterations

		while (it < maxIter)
		{
			float f = betainc(x, a, b, lower) - p;

			// Check failures to converge to the root
			// Halve the step if it overshot the root
			if (f*oldf < 0.f && fabs(oldf) <= fabs(f))
			{
				x = 0.5f*(x + oldx);
				f = betainc(x, a, b, tail) - p;
			}

			// Update boundaries, shrink
			if (f > 0) xhi = x;
			else xlo = x;

			// Convergence criteria
			if ((fabs(f) < ftol) || 
				(fabs(x - oldx) < xtol*x + std::numeric_limits<float>::epsilon()))
			{
				break;
			}

			// Newton's step
			oldx = x;
			oldf = f;
			const float logx = log(x);
			const float log1mx = log(1.f - x);

			const float df = exp(am1*logx + bm1*log1mx - lbeta);
			x = x - f / df;

			// Check boundaries
			if (x <= xlo)
			{
				x = 0.99f*xlo + 0.01f*xhi;
			}
			else if (x >= xhi)
			{
				x = 0.01f*xlo + 0.99f*xhi;
			}

			// Increase counter
			it++;
		}

		return x;
	}
}