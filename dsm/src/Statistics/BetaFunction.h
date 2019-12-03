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

#include "Tail.h"
#include "FullSystem/DSMLib.h"

namespace dsm
{
	// Beta Functions based on:
	// Abramowitz et al. 'Handbook of Mathematical Functions' 1964, 26.5
	// https://en.wikipedia.org/wiki/Beta_function

	// Beta function (x=1)
	DSM_EXPORTS_DLL float beta(float a, float b);

	// Log of the Beta function
	DSM_EXPORTS_DLL float betaln(float a, float b);

	// Incomplete Beta function
	DSM_EXPORTS_DLL float beta(float x, float a, float b, Tail tail = lower);

	// Regularized Incomplete Beta function
	// Ix = beta(x,a,b)/beta(a,b)
	DSM_EXPORTS_DLL float betainc(float x, float a, float b, Tail tail = lower);

	// Inverse Beta function
	// Ix^-1 = beta(x,a,b)/beta(a,b) = p
	DSM_EXPORTS_DLL float betaincinv(float p, float a, float b, Tail tail = lower);
}