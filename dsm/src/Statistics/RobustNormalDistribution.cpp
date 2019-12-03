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

#include "RobustNormalDistribution.h"
#include "Statistics.h"
#include "Utils/LossFunction.h"
#include "Utils/Settings.h"

#include <iostream>

namespace dsm
{
	RobustNormalDistribution::RobustNormalDistribution(float mu, float sigma) :
#if defined(ENABLE_SSE)
		mu_sse(_mm_set_ps1(mu)),
		sigma_sse(_mm_set_ps1(sigma)),
#endif
		mu_(mu),
		sigma_(sigma)
	{}

	RobustNormalDistribution::RobustNormalDistribution(const RobustNormalDistribution& nDist) :
#if defined(ENABLE_SSE)
		mu_sse(_mm_set_ps1(nDist.mu_)),
		sigma_sse(_mm_set_ps1(nDist.sigma_)),
#endif
		mu_(nDist.mu_),
		sigma_(nDist.sigma_)
	{}

	RobustNormalDistribution::~RobustNormalDistribution()
	{}

	bool RobustNormalDistribution::empty() const
	{
		return this->sigma_ <= 0.f;
	}

	IDistribution::Type RobustNormalDistribution::type() const
	{
		return IDistribution::Type::NORMAL;
	}

	float RobustNormalDistribution::mu() const
	{
		return this->mu_;
	}

	float RobustNormalDistribution::sigma() const
	{
		return this->sigma_;
	}

	float RobustNormalDistribution::pdf(float x) const
	{	
		const float t = (x - this->mu_) / this->sigma_;

		const float a = 1.f / (this->sigma_*sqrt(2.f*Settings::PI));
		const float b = -0.5f*t*t;

		return a*exp(b);
	}

	float RobustNormalDistribution::cdf(float x, Tail tail) const
	{
		// TODO
		return 0.f;
	}

	float RobustNormalDistribution::icdf(float p, Tail tail) const
	{
		if (this->empty()) return std::numeric_limits<float>::max();

		// p has the range [0,1]
		if (p < 0.f || p > 1.f)
		{
			std::cout << "RobustNormalDistribution::icdf() wrong input arguments!\n";
			return std::numeric_limits<float>::quiet_NaN();
		}

		// fast return values
		if (p == 0.f)
		{
			if (tail == lower) return -std::numeric_limits<float>::infinity();
			else return std::numeric_limits<float>::infinity();
		}
		else if (p == 1.f)
		{
			if (tail == lower) return std::numeric_limits<float>::infinity();
			else return -std::numeric_limits<float>::infinity();
		}

		// 99.7% -> mu + 3*sigma 
		// 95% -> mu + 2*sigma 
		// 68% -> mu + 1*sigma 
		if (p == 0.997f)
		{
			return this->mu_ + 3.f * this->sigma_;
		}
		else if (p == 0.95f)
		{
			return this->mu_ + 2.f * this->sigma_;
		}
		else if (p == 0.68f)
		{
			return this->mu_ + 1.f * this->sigma_;
		}
		else
		{
			// TODO
			std::cout << "RobustNormalDistribution::icdf() only valid for 99.7% 95% 68%!\n";
			return std::numeric_limits<float>::quiet_NaN();
		}
	}

	float RobustNormalDistribution::weight(float energy) const
	{
		const auto& settings = Settings::getInstance();

		// check if we have a valid distribution
		if (this->empty()) return 1.f;

		// robust normal with huber
		if (settings.useFixedHuberScale)
		{
			return LossFunction::Huber::weight(settings.huberScale, energy);
		}
		else
		{
			// normalize with scale and location
			const float energyNorm = (energy - this->mu_) / this->sigma_;

			return LossFunction::Huber::weight(LossFunction::Huber::SCALE, energyNorm);
		}	
	}

#if defined(ENABLE_SSE)

	__m128 RobustNormalDistribution::weightSSE(__m128 energy) const
	{
		const auto& settings = Settings::getInstance();

		// check if we have a valid distribution
		if (this->empty()) return _mm_set_ps1(1.f);

		// robust normal with huber
		if (settings.useFixedHuberScale)
		{
			return LossFunction::Huber::weightSSE(_mm_set_ps1(settings.huberScale), energy);
		}
		else
		{
			// normalize with scale and location
			const __m128 energyNorm = _mm_mul_ps(_mm_sub_ps(energy, this->mu_sse), _mm_rcp_ps(this->sigma_sse));

			return LossFunction::Huber::weightSSE(_mm_set_ps1(LossFunction::Huber::SCALE), energyNorm);
		}
	}

#endif

	std::shared_ptr<RobustNormalDistribution> RobustNormalDistribution::fitdist(const std::vector<float> &dist, const Eigen::VecXf& fixed)
	{
		if (!(fixed.size() == 0 || fixed.size() == 2))
		{
			std::cout << "RobustNormalDistribution::fitdist() requires a valid fixed vector \n";
			return std::make_shared<RobustNormalDistribution>(0.f, 0.f);
		}

		bool fixMu = false, fixSigma = false;

		if (fixed.size() == 2)
		{
			if (fixed[0] >= 0.f) fixMu = true;
			if (fixed[1] > 0.f) fixSigma = true;
		}

		if (!fixMu && !fixSigma)
		{
			// general case
			const float mu = dsm::median(dist);					// mu: median
			const float sigma = 1.4826f*dsm::mad(dist);			// sigma: aproximate with normal 1.4826*mad
			return std::make_shared<RobustNormalDistribution>(mu, sigma);
		}
		else if (fixMu && !fixSigma)
		{
			// fix mu
			const float sigma = 1.4826f*dsm::mad(dist);			// sigma: aproximate with normal 1.4826*mad
			return std::make_shared<RobustNormalDistribution>(fixed[0], sigma);
		}
		else
		{
			std::cout << "RobustNormalDistribution::fitdist() requires a valid parameter composition \n";
			return std::make_shared<RobustNormalDistribution>(0.f, 0.f);
		}
	}
}