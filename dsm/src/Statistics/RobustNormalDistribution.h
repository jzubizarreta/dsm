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
#include "IDistribution.h"

namespace dsm
{
	// Normal Distribution + Huber weight
	// https://en.wikipedia.org/wiki/Normal_distribution
	// https://en.wikipedia.org/wiki/Huber_loss

	class RobustNormalDistribution : public IDistribution
	{
	public:

		// Constructor & Destructor
		RobustNormalDistribution(float mu = 0.f, float sigma = 1.f);
		RobustNormalDistribution(const RobustNormalDistribution& nDist);		
		~RobustNormalDistribution();

		// check if is empty
		bool empty() const;

		// type of the distribution
		IDistribution::Type type() const;

		// Accessors
		float mu() const;
		float sigma() const;

		// Probability Distribution Function
		float pdf(float x) const;

		// Cumulative Distribution Function
		float cdf(float x, Tail tail = lower) const;

		// Inverse Cumulative Distribution Function
		float icdf(float p, Tail tail = lower) const;

		// Weight based on Huber
		float weight(float energy) const;

#if defined(ENABLE_SSE)

		__m128 weightSSE(__m128 energy) const;

#endif

		// Fit Normal Distribution using n parameters: location (0), scale (1) 
		// Select fixed parameters giving valid values
		static std::shared_ptr<RobustNormalDistribution> fitdist(const std::vector<float> &dist, const Eigen::VecXf& fixed = Eigen::VecXf());

	private:

		const float mu_;
		const float sigma_;

#if defined(ENABLE_SSE)

		const __m128 mu_sse;
		const __m128 sigma_sse;

#endif

	};
}