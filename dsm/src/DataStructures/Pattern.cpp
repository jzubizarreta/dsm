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

#include "Pattern.h"

namespace dsm
{
	// implementation
	Pattern::Type Pattern::type = Pattern::Type::DSO;

	Pattern::Pattern()
	{
	}

	Pattern::~Pattern()
	{
	}

	Pattern& Pattern::getInstance()
	{
		static Pattern thePattern;
		return thePattern;
	}

	void Pattern::initialize()
	{
		auto &pattern = Pattern::getInstance();
		pattern.init();
	}

	void Pattern::setType(Type newType)
	{
		Pattern::type = newType;
	}

	int32_t Pattern::at(int32_t idx, int32_t dim)
	{
		auto &pattern = Pattern::getInstance();

		assert(idx < pattern.sizeTypes[Pattern::type]);
				
		return pattern.patternTypes[Pattern::type](dim, idx);
	}

	const Eigen::Ref<Eigen::Vector2i> Pattern::at(int32_t idx)
	{
		auto &pattern = Pattern::getInstance();

		assert(idx < pattern.sizeTypes[Pattern::type]);

		return pattern.patternTypes[Pattern::type].col(idx);
	}

	int32_t Pattern::width()
	{
		auto &pattern = Pattern::getInstance();
		return pattern.widthTypes[Pattern::type];
	}

	int32_t Pattern::padding()
	{
		auto &pattern = Pattern::getInstance();
		return pattern.paddingTypes[Pattern::type];
	}

	int32_t Pattern::size()
	{
		auto &pattern = Pattern::getInstance();
		return pattern.sizeTypes[Pattern::type];
	}

	void Pattern::init()
	{
		// generate all patterns by hand

		// 0) DSO 8 point spread pattern
		//
		//		0	0	x	0	0
		//		0	x	0	x	0
		//		x	0	x	0	x
		//		0	x	0	0	0
		//		0	0	x	0	0

		sizeTypes.push_back(8);
		widthTypes.push_back(5);
		paddingTypes.push_back(2);

		Eigen::Matrix<int, 2, 8> pattern0;
		pattern0 << 0, -1, 1, -2, 0, 2, -1, 0,
					-2, -1, -1, 0, 0, 0, 1, 2;

		patternTypes.push_back(pattern0);

		// 1) 5 x 5 full pattern
		int32_t width = 5;
		int32_t padding = width / 2;

		sizeTypes.push_back(width*width);
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		Eigen::Matrix2Xi pattern;
		pattern.resize(2, width*width);
		for (int32_t i = -padding; i <= padding; ++i)
		{
			for (int32_t j = -padding; j <= padding; ++j)
			{
				int32_t index = (i + padding) * width + (j + padding);
				pattern(0, index) = j;
				pattern(1, index) = i;
			}
		}
		patternTypes.push_back(pattern);

		// 2) 11 x 11 full pattern
		width = 11;
		padding = width / 2;

		sizeTypes.push_back(width*width);
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, width*width);
		for (int32_t i = -padding; i <= padding; ++i)
		{
			for (int32_t j = -padding; j <= padding; ++j)
			{
				int32_t index = (i + padding) * width + (j + padding);
				pattern(0, index) = j;
				pattern(1, index) = i;
			}
		}
		patternTypes.push_back(pattern);

		// 3) 21 x 21 full pattern
		width = 21;
		padding = width / 2;

		sizeTypes.push_back(width*width);
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, width*width);
		for (int32_t i = -padding; i <= padding; ++i)
		{
			for (int32_t j = -padding; j <= padding; ++j)
			{
				int32_t index = (i + padding) * width + (j + padding);
				pattern(0, index) = j;
				pattern(1, index) = i;
			}
		}
		patternTypes.push_back(pattern);

		// 4) 31 x 31 full pattern
		width = 31;
		padding = width / 2;

		sizeTypes.push_back(width*width);
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, width*width);
		for (int32_t i = -padding; i <= padding; ++i)
		{
			for (int32_t j = -padding; j <= padding; ++j)
			{
				int32_t index = (i + padding) * width + (j + padding);
				pattern(0, index) = j;
				pattern(1, index) = i;
			}
		}
		patternTypes.push_back(pattern);

		// 5) 41 x 41 full pattern
		width = 41;
		padding = width / 2;

		sizeTypes.push_back(width*width);
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, width*width);
		for (int32_t i = -padding; i <= padding; ++i)
		{
			for (int32_t j = -padding; j <= padding; ++j)
			{
				int32_t index = (i + padding) * width + (j + padding);
				pattern(0, index) = j;
				pattern(1, index) = i;
			}
		}
		patternTypes.push_back(pattern);

		// 6) 51 x 51 full pattern
		width = 51;
		padding = width / 2;

		sizeTypes.push_back(width*width);
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, width*width);
		for (int32_t i = -padding; i <= padding; ++i)
		{
			for (int32_t j = -padding; j <= padding; ++j)
			{
				int32_t index = (i + padding) * width + (j + padding);
				pattern(0, index) = j;
				pattern(1, index) = i;
			}
		}
		patternTypes.push_back(pattern);

		// 7) 5 x 5 spread pattern
		width = 5;
		padding = width / 2;

		sizeTypes.push_back(padding*padding + (padding + 1)*(padding + 1));
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, sizeTypes.back());
		int32_t index = 0;
		for (int32_t i = -padding; i <= padding; i += 2)
		{
			for (int32_t j = -padding; j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		for (int32_t i = (-padding + 1); i <= padding; i += 2)
		{
			for (int32_t j = (-padding + 1); j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		patternTypes.push_back(pattern);

		// 8) 11 x 11 spread pattern
		width = 11;
		padding = width / 2;

		sizeTypes.push_back(padding*padding + (padding+1)*(padding+1));
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, sizeTypes.back());
		index = 0;
		for (int32_t i = -padding; i <= padding; i += 2)
		{
			for (int32_t j = -padding; j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		for (int32_t i = (-padding + 1); i <= padding; i += 2)
		{
			for (int32_t j = (-padding + 1); j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		patternTypes.push_back(pattern);

		// 9) 21 x 21 spread pattern
		width = 21;
		padding = width / 2;

		sizeTypes.push_back(padding*padding + (padding + 1)*(padding + 1));
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, sizeTypes.back());
		index = 0;
		for (int32_t i = -padding; i <= padding; i += 2)
		{
			for (int32_t j = -padding; j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		for (int32_t i = (-padding + 1); i <= padding; i += 2)
		{
			for (int32_t j = (-padding + 1); j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		patternTypes.push_back(pattern);

		// 10) 31 x 31 spread pattern
		width = 31;
		padding = width / 2;

		sizeTypes.push_back(padding*padding + (padding + 1)*(padding + 1));
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, sizeTypes.back());
		index = 0;
		for (int32_t i = -padding; i <= padding; i += 2)
		{
			for (int32_t j = -padding; j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		for (int32_t i = (-padding + 1); i <= padding; i += 2)
		{
			for (int32_t j = (-padding + 1); j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		patternTypes.push_back(pattern);

		// 11) 41 x 41 spread pattern
		width = 41;
		padding = width / 2;

		sizeTypes.push_back(padding*padding + (padding + 1)*(padding + 1));
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, sizeTypes.back());
		index = 0;
		for (int32_t i = -padding; i <= padding; i += 2)
		{
			for (int32_t j = -padding; j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		for (int32_t i = (-padding + 1); i <= padding; i += 2)
		{
			for (int32_t j = (-padding + 1); j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		patternTypes.push_back(pattern);

		// 12) 51 x 51 spread pattern
		width = 51;
		padding = width / 2;

		sizeTypes.push_back(padding*padding + (padding + 1)*(padding + 1));
		widthTypes.push_back(width);
		paddingTypes.push_back(padding);

		pattern.resize(2, sizeTypes.back());
		index = 0;
		for (int32_t i = -padding; i <= padding; i += 2)
		{
			for (int32_t j = -padding; j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		for (int32_t i = (-padding + 1); i <= padding; i += 2)
		{
			for (int32_t j = (-padding + 1); j <= padding; j += 2)
			{
				pattern(0, index) = j;
				pattern(1, index) = i;
				++index;
			}
		}
		patternTypes.push_back(pattern);

		// 13) 11 x 11 circle pattern
		sizeTypes.push_back(45);
		widthTypes.push_back(11);
		paddingTypes.push_back(5);

		Eigen::Matrix<int, 2, 45> pattern13;
		pattern13 << -2, -1, 0, 1, 2, -3, 3, -4, -1, 0, 1, 4, -5, -2, 2, 5, -5, -3, 3, 5, -5, -3, 0, 3, 5, -5, -3, 3, 5, -5, -2, 2, 5, -4, -1, 0, 1, 4, -3, 3, -2, -1, 0, 1, 2,
					 -5, -5, -5, -5, -5, -4, -4, -3, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 5, 5, 5, 5, 5;

		patternTypes.push_back(pattern13);
	}
}