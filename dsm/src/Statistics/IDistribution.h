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
#include "Utils/EigenTypes.h"
#include "FullSystem/DSMLib.h"

#include "dsm/BuildFlags.h"

#include <vector>
#include <memory>

namespace dsm
{
	// Distribution interface
	class DSM_EXPORTS_DLL IDistribution
	{

	public:

		enum Type {NORMAL = 0, TSTUDENT = 1};

		IDistribution();
		virtual ~IDistribution() = 0;

		// check if is empty
		virtual bool empty() const = 0;

		// type of the distribution
		virtual Type type() const = 0;

		// Probability Distribution Function
		virtual float pdf(float x) const = 0;

		// Cumulative Distribution Function
		virtual float cdf(float x, Tail tail = lower) const = 0;

		// Inverse Cumulative Distribution Function
		virtual float icdf(float p, Tail tail = lower) const = 0;

		// Weight based on the distribution
		virtual float weight(float energy) const = 0;

#if defined(ENABLE_SSE)

		virtual __m128 weightSSE(__m128 energy) const = 0;

#endif

		// Distribution fit
		static std::shared_ptr<IDistribution> fitdist(const std::vector<float> &dist, IDistribution::Type type,
													  const Eigen::VecXf& fixed = Eigen::VecXf());
	};
}