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

#include <Eigen/Core>

#include "dsm/BuildFlags.h"

#include "FullSystem/DSMLib.h"

namespace dsm
{
	// Pattern type class
	// It allows creating and accessing multiple type of patterns
	class DSM_EXPORTS_DLL Pattern
	{
		enum Type {DSO=0, FULL_5=1, FULL_11=2, FULL_21=3, FULL_31=4, FULL_41=5, FULL_51=6, 
						  SPREAD_5=7, SPREAD_11=8, SPREAD_21=9, SPREAD_31=10, SPREAD_41=11, SPREAD_51=12,
						  CIRCLE_11=13};

	public:

		// this must be called once at startup
		static void initialize();

		// sets pattern type to be used
		static void setType(Type newType);

		// returns pattern look up table index at the point "idx" with dimension (x/y) "dim"
		static int32_t at(int32_t idx, int32_t dim);

		// returns pattern look up table index at the point "idx"
		static const Eigen::Ref<Eigen::Vector2i> at(int32_t idx);

		// returns pattern width / 2
		static int32_t padding();

		// returns pattern width
		static int32_t width();

		// returns pattern number of points
		static int32_t size();

		// Dont forget to declare these two. You want to make sure they
		// are unacceptable otherwise you may accidentally get copies of
		// your singleton appearing.
		Pattern(Pattern const&) = delete;
		void operator=(Pattern const&) = delete;

	private:
		Pattern();
		~Pattern();

		// main function that creates a single instance of Pattern
		static Pattern& getInstance();

		// generates all pattern types
		void init();

	private:

		std::vector<int32_t> sizeTypes;				// width x width
		std::vector<int32_t> widthTypes;			// width
		std::vector<int32_t> paddingTypes;			// width / 2
		std::vector<Eigen::Matrix2Xi> patternTypes;

		static Type type;
	};
}