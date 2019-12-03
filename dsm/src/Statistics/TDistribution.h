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

#include "IDistribution.h"
#include "Utils/EigenTypes.h"
#include "FullSystem/DSMLib.h"

#include <vector>
#include <memory>

namespace dsm
{
	// Generalized Student's t-distribution
	// https://en.wikipedia.org/wiki/Student's_t-distribution

	class DSM_EXPORTS_DLL TDistribution : public IDistribution
	{
	public:		

		// Constructor & Destructor
		TDistribution(float nu, float mu = 0.f, float sigma = 1.f);
		TDistribution(const TDistribution& tDist);		
		~TDistribution();

		// check if is empty
		bool empty() const;

		// type of the distribution
		IDistribution::Type type() const;

		// Accessors
		float nu() const;
		float mu() const;
		float sigma() const;		

		// Probability Distribution Function
		float pdf(float x) const;

		// Cumulative Distribution Function
		float cdf(float x, Tail tail = lower) const;

		// Inverse Cumulative Distribution Function
		float icdf(float p, Tail tail = lower) const;

		// Weight based on the distribution
		float weight(float energy) const;

#if defined(ENABLE_SSE)

		__m128 weightSSE(__m128 energy) const;

#endif

		// Fit T-Distribution using n parameters: degrees of freedom (0), location (1), scale (2) 
		// Select fixed parameters giving valid values
		static std::shared_ptr<TDistribution> fitdist(const std::vector<float> &dist, const Eigen::VecXf& fixed = Eigen::VecXf());
		
	private:

		// Initialize parameters
		static Eigen::VecXf initialize(const std::vector<float> &dist, const Eigen::VecXf& fixed);
		static float initNu(const std::vector<float> &dist);
		static float initMu(const std::vector<float> &dist);
		static float initSigma(const std::vector<float> &dist);

		// Negative Log Likelihood calculator
		static float negLogLikelihood(float nu, float mu, float sigma, const std::vector<float> &data);

	private:

		// General T-Distribution parameterization
		const float nu_;		// degrees of freedom
		const float mu_;		// location
		const float sigma_;		// scale	

#if defined(ENABLE_SSE)

		const __m128 nu_sse;
		const __m128 mu_sse;
		const __m128 sigma_sse;
		const __m128 nup1_sse;

#endif

		// Negative Log Likelihood struct
		struct NLL
		{
			float operator()(const Eigen::Vec3f& params,
							 const std::vector<float>& data) const;
		};

		// Negative Log Likelihood struct with fixed mu
		struct NLLmu
		{
			NLLmu(float mu);

			float operator()(const Eigen::Vec2f& params,
							 const std::vector<float>& data) const;

		private:

			const float mu_;		// fixed location
		};

		// Negative Log Likelihood struct with fixed mu
		struct NLLmunu
		{
			NLLmunu(float nu, float mu);

			float operator()(const Eigen::VecXf& params, const std::vector<float>& data) const;

		private:

			const float nu_;		// fixed dof
			const float mu_;		// fixed location			
		};

	};
}