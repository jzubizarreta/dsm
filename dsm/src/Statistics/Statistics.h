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

#include <vector>
#include <numeric>

namespace dsm
{
	// mean of the input vector
	// https://en.wikipedia.org/wiki/Mean
	template<typename T>
	T mean(const std::vector<T>& v)
	{
		T sum = 0;
		for (auto it = v.begin(); it != v.end(); ++it)
		{
			sum += *it;
		}
		return sum / v.size();
	}

	// median calculator
	// https://en.wikipedia.org/wiki/Median
	template<typename T>
	T median(const std::vector<T>& v)
	{
		auto d = v;		// copy required!
		std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
		return d[d.size() / 2];
	}

	// median absolute deviation
	// https://en.wikipedia.org/wiki/Median_absolute_deviation
	template<typename T>
	T mad(const std::vector<T>& v)
	{
		auto d = v;		// copy required!

		std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
		const T m = d[d.size() / 2];

		for (auto it = d.begin(); it != d.end(); ++it)
		{
			*it = abs(*it - m);
		}

		std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
		return d[d.size() / 2];
	}

	// kurtosis
	// https://en.wikipedia.org/wiki/Kurtosis
	template<typename T>
	T kurtosis(const std::vector<T>& v)
	{
		std::vector<T> s2v(v.size());
		std::vector<T> m4v(v.size());

		const T m = mean(v);

		for (size_t i = 0; i < v.size(); ++i)
		{
			T n = v[i] - m;
			s2v[i] = n*n;
			m4v[i] = s2v[i] * s2v[i];
		}

		const T s2 = mean(s2v);
		const T m4 = mean(m4v);

		return m4 / (s2*s2);
	}

}