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

#include "TDistribution.h"
#include "NelderMead.h"
#include "BetaFunction.h"
#include "Statistics.h"
#include "Utils/Settings.h"
#include "Utils/UtilFunctions.h"
#include "Utils/sse.h"

#include "dsm/BuildFlags.h"

#include <iostream>
#include <cmath>

namespace dsm
{
	TDistribution::TDistribution(float nu, float mu, float sigma) :
#if defined(ENABLE_SSE)
		nu_sse(_mm_set_ps1(nu)),
		mu_sse(_mm_set_ps1(mu)),
		sigma_sse(_mm_set_ps1(sigma)),
		nup1_sse(_mm_set_ps1(nu + 1.f)),
#endif
		nu_(nu),
		mu_(mu),
		sigma_(sigma)
	{}

	TDistribution::TDistribution(const TDistribution& tDist) :
#if defined(ENABLE_SSE)
		nu_sse(_mm_set_ps1(tDist.nu_)),
		mu_sse(_mm_set_ps1(tDist.mu_)),
		sigma_sse(_mm_set_ps1(tDist.sigma_)),
		nup1_sse(_mm_set_ps1(tDist.nu_ + 1.f)),
#endif
		nu_(tDist.nu_),
		mu_(tDist.mu_),
		sigma_(tDist.sigma_)
	{}

	TDistribution::~TDistribution()
	{}

	bool TDistribution::empty() const
	{
		return (this->nu_ <= 0.f ||
				this->sigma_ <= 0.f);
	}

	IDistribution::Type TDistribution::type() const
	{
		return IDistribution::Type::TSTUDENT;
	}

	float TDistribution::nu() const
	{
		return this->nu_;
	}

	float TDistribution::mu() const
	{
		return this->mu_;
	}

	float TDistribution::sigma() const
	{
		return this->sigma_;
	}

	float TDistribution::pdf(float x) const
	{
		const float t = (x - this->mu_) / this->sigma_;

		// use lgamma for numerical stability
		const float a = exp(lgamma(0.5f*(this->nu_ + 1.f)) - lgamma(0.5f*this->nu_));
		const float b = pow(1.f + (t*t) / this->nu_, 0.5f*(this->nu_ + 1.f));
		const float y = a / (this->sigma_*sqrt(this->nu_*Settings::PI)*b);

		return y;
	}

	float TDistribution::cdf(float x, Tail tail) const
	{
		// See 'Handbook of Mathematical Functions' Abramowitz et al. 1964
		// Eq. 26.7.1, 26.5.27, 26.5.2

		// Work with t as an standard t-distribution
		const float t = (x - this->mu_) / this->sigma_;

		// standard t-distribution cdf
		const float s = this->nu_ / (this->nu_ + t*t);						// Eq. 26.7.1 & 26.5.27
		float cdf = 0.5f*betainc(s, 0.5f*this->nu_, 0.5f, lower);			// Eq. 26.7.1 & 26.5.27

		// tail selection
		if ((t > 0 && tail == lower) ||
			(t < 0 && tail == upper))
		{
			cdf = 1.f - cdf;			// 26.5.2
		}

		return cdf;
	}

	float TDistribution::icdf(float p, Tail tail) const
	{
		if (this->empty()) return std::numeric_limits<float>::max();

		// See 'Handbook of Mathematical Functions' Abramowitz et al. 1964
		// Eq. 26.7.1, 26.5.27, 26.5.2

		// standard inverse t-distribution cdf
		// use betaincinv which uses the Newton's method
		// we will make the inverse operations of cdf()

		// p has the range [0,1]
		if (p < 0.f || p > 1.f)
		{
			std::cout << "TDistribution::icdf() wrong input arguments!\n";
			return std::numeric_limits<float>::quiet_NaN();
		}

		// fast return values
		if (p == 0.f)
		{
			if(tail == lower) return -std::numeric_limits<float>::infinity();
			else return std::numeric_limits<float>::infinity();
		}
		else if (p == 1.f)
		{
			if (tail == lower) return std::numeric_limits<float>::infinity();
			else return -std::numeric_limits<float>::infinity();
		}

		// tail selection
		// assume always the smallest tail
		bool tailChanged = false;
		if (p > 0.5f)
		{
			p = 1.f - p;
			tailChanged = true;
		}

		const float s = betaincinv(2.f*p, 0.5f*this->nu_, 0.5f, lower);	// s >= 0

		// Eq. 26.7.1 & 26.5.27
		const float onems = 1.f - s;
		float t = sqrt(this->nu_ * onems / s);		// t >= 0

		if ((!tailChanged && tail == lower) ||
			(tailChanged && tail == upper))
		{
			// t <= 0
			t = -t;
		}

		// return generalized value
		// with scale and location
		return t*this->sigma_ + this->mu_;
	}

	float TDistribution::weight(float energy) const
	{
		// check if we have a valid distribution
		if (this->empty()) return 1.f;

		// normalize with scale and location
		const float energyNorm = (energy - this->mu_) / this->sigma_;
		
		// weight calculation
		const float weight = (this->nu_ + 1.f) / (this->nu_ + energyNorm*energyNorm);

		return weight;
	}

#if defined(ENABLE_SSE)

	__m128 TDistribution::weightSSE(__m128 energy) const
	{
		// check if we have a valid distribution
		if (this->empty()) return _mm_set_ps1(1.f);

		// normalize with scale and location
		const __m128 energyNorm = _mm_mul_ps(_mm_sub_ps(energy, this->mu_sse), _mm_rcp_ps(this->sigma_sse));

		// weight calculation
		const __m128 den = _mm_add_ps(this->nu_sse, _mm_mul_ps(energyNorm, energyNorm));
		const __m128 weight = _mm_mul_ps(this->nup1_sse, _mm_rcp_ps(den));
		
		return weight;
	}

#endif

	std::shared_ptr<TDistribution> TDistribution::fitdist(const std::vector<float> &dist, const Eigen::VecXf& fixed)
	{
		if (!(fixed.size() == 0 || fixed.size() == 3))
		{
			std::cout << "TDistribution::fitdist() requires a valid fixed vector \n";
			return std::make_shared<TDistribution>(0.f, 0.f, 0.f);
		}

		// Initialize nu, mu and sigma
		Eigen::VecXf start = TDistribution::initialize(dist, fixed);	

		// Maximize the log - likelihood with respect to nu, mu and sigma
		NelderMead nm;
		NelderMead::Options options;

		// create the right minimizer
		bool fixNu = false, fixMu = false, fixSigma = false;

		if (fixed.size() == 3)
		{
			if (fixed[0] > 0.f) fixNu = true;
			if (fixed[1] >= 0.f) fixMu = true;
			if (fixed[2] > 0.f) fixSigma = true;
		}

		if (!fixNu && !fixMu && !fixSigma)
		{
			// general case
			const Eigen::Vec3f params = nm.minimize(NLL(), start, dist, options);
			return std::make_shared<TDistribution>(params[0], params[1], params[2]);
		}
		else if (!fixNu && fixMu && !fixSigma)
		{
			// fix mu
			const Eigen::Vec2f params = nm.minimize(NLLmu(fixed[1]), start, dist, options);
			return std::make_shared<TDistribution>(params[0], fixed[1], params[1]);
		}
		else if(fixNu && fixMu && !fixSigma)
		{
			// fix nu & mu
			const Eigen::VecXf params = nm.minimize(NLLmunu(fixed[0], fixed[1]), start, dist, options);
			return std::make_shared<TDistribution>(fixed[0], fixed[1], params[0]);
		}
		else
		{
			std::cout << "TDistribution::fitdist() requires a valid parameter composition \n";
			return std::make_shared<TDistribution>(0.f, 0.f, 0.f);
		}
	}

	Eigen::VecXf TDistribution::initialize(const std::vector<float> &dist, const Eigen::VecXf& fixed)
	{
		// Initialization of non fixed parameters

		int numFixed = 0;
		if (fixed[0] > 0.f) numFixed++;			// nu > 0
		if (fixed[1] >= 0.f) numFixed++;		// mu >= 0
		if (fixed[2] > 0.f) numFixed++;			// sigma > 0

		Eigen::VecXf start(3 - numFixed);
		int num = 0;

		// nu
		if (!(fixed[0] > 0.f))
		{ 
			start[num] = TDistribution::initNu(dist);
			num++;
		}

		// mu
		if (!(fixed[1] >= 0.f))
		{
			start[num] = TDistribution::initMu(dist);
			num++;
		}

		// sigma
		if (!(fixed[2] > 0.f))
		{
			start[num] = TDistribution::initSigma(dist);
			num++;
		}

		assert(num == (3 - numFixed));

		return start;
	}

	float TDistribution::initNu(const std::vector<float> &dist)
	{
		// nu: using excess kurtosis
		// kurt - 3 = 6 / (nu - 4), for nu > 4
		const float kurt = std::max(dsm::kurtosis(dist), 4.f);
		return (4.f*kurt - 6.f) / (kurt - 3.f);
	}

	float TDistribution::initMu(const std::vector<float> &dist)
	{
		// mu: median
		return dsm::median(dist);
	}

	float TDistribution::initSigma(const std::vector<float> &dist)
	{
		// sigma: aproximate with normal 1.4826*mad
		return 1.4826f*dsm::mad(dist);
	}

	float TDistribution::negLogLikelihood(float nu, float mu, float sigma, 
										  const std::vector<float> &data)
	{
		// precompute constant part
		const float k = 0.5f*(nu + 1.f);
		const float isigma = 1.f / sigma;

		const float c = lgamma(k) - lgamma(0.5f*nu) +
			0.5f*nu * log(nu) - log(sigma) - 0.5f*log(Settings::PI);

		const int s = (int)data.size();

		// calculate the negative log likelihood
		// this is the slowest part

#if defined(ENABLE_SSE)

		const int gap = s % 4;
		const int numSSE = s - gap;

		const __m128 mmu = _mm_set_ps1(mu);
		const __m128 msigma = _mm_set_ps1(isigma);
		const __m128 mnu = _mm_set_ps1(nu);
		const __m128 mk = _mm_set_ps1(k);

		const float* dataPtr = &data[0];

		__m128 sum = _mm_setzero_ps();

		// sse to make it faster
		for (int i = 0; i < numSSE; i += 4)
		{
			const __m128 t = _mm_mul_ps(_mm_sub_ps(_mm_load_ps(dataPtr + i), mmu), msigma);
			const __m128 w = _mm_add_ps(mnu, _mm_mul_ps(t, t));
			const __m128 logw = dsm::logapprox_ps(w);

			sum = _mm_add_ps(sum, _mm_mul_ps(mk, logw));
		}

		// sum horizontally
		sum = _mm_hadd_ps(sum, sum);
		float nll = _mm_cvtss_f32(_mm_hadd_ps(sum, sum));

		// the rest normally
		for (int i = numSSE; i < s; ++i)
		{
			const float t = (data[i] - mu) * isigma;
			const float w = nu + t * t;
			const float logw = log(w);

			// negative log likelihod
			nll += k*logw;
		}
#else

		float nll = 0.f;
		for (int i = 0; i < s; ++i)
		{
			const float t = (data[i] - mu) * isigma;
			const float w = nu + t * t;
			const float logw = log(w);

			// negative log likelihod
			nll += k*logw;
		}

#endif

		// constant part
		nll -= c*s;

		return nll;
	}

	float TDistribution::NLL::operator()(const Eigen::Vec3f& params,
										 const std::vector<float>& data) const
	{
		// do not let nu and sigma become zero or negative
		if (params[0] <= 0.f || params[2] <= 0.f)
		{
			return std::numeric_limits<float>::max();
		}

		// calculate the negative log likelihood
		return TDistribution::negLogLikelihood(params[0], params[1], params[2], data);
	}

	TDistribution::NLLmu::NLLmu(float mu) :
		mu_(mu)
	{}

	float TDistribution::NLLmu::operator()(const Eigen::Vec2f& params,
										   const std::vector<float>& data) const
	{
		// do not let nu and sigma become zero or negative
		if (params[0] <= 0. || params[1] <= 0.f)
		{
			return std::numeric_limits<float>::max();
		}

		// calculate the negative log likelihood
		return TDistribution::negLogLikelihood(params[0], this->mu_, params[1], data);
	}

	TDistribution::NLLmunu::NLLmunu(float nu, float mu) :
		nu_(nu),
		mu_(mu)		
	{}

	float TDistribution::NLLmunu::operator()(const Eigen::VecXf& params, const std::vector<float>& data) const
	{
		// do not let sigma become zero or negative
		if (params[0] <= 0.f)
		{
			return std::numeric_limits<float>::max();
		}

		// calculate the negative log likelihood
		return TDistribution::negLogLikelihood(this->nu_, this->mu_, params[0], data);
	}
}