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

#include "FrameTrackerProblem.h"
#include "FrameTrackerReference.h"
#include "Utils/GlobalCalibration.h"
#include "dsm/BuildFlags.h"

#include <Eigen/Cholesky>

namespace dsm
{
	FrameTrackerProblem::FrameTrackerProblem(const int levels) :
		jacobians(levels)
	{
	}

	FrameTrackerProblem::~FrameTrackerProblem()
	{
	}

	void FrameTrackerProblem::computeJacobAtIdentity(const std::shared_ptr<FrameTrackerReference>& trackingReference, int lvl)
	{
		const auto& settings = Settings::getInstance();
		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Mat33f& K = calib.matrix3f(lvl);

		// constant values
		const float* pointX = trackingReference->x(lvl);
		const float* pointY = trackingReference->y(lvl);
		const float* pointIDepth = trackingReference->iDepth(lvl);
		const float* pointGx = trackingReference->gx(lvl);
		const float* pointGy = trackingReference->gy(lvl);
		const float* pointColor = trackingReference->color(lvl);
		const float* pointWeight = trackingReference->weight(lvl);
		const int num = trackingReference->numPoints(lvl);

		// resize jacobian vector
		this->jacobians[lvl].resize(num);

#if defined(ENABLE_SSE)

		const int gap = num % 4;
		const int numSSE = num - gap;

		const __m128 zeros = _mm_setzero_ps();
		const __m128 ones = _mm_set_ps1(1.f);
		const __m128 minusOnes = _mm_set_ps1(-1.f);

		const __m128 mfx = _mm_set_ps1(K(0, 0));
		const __m128 mfy = _mm_set_ps1(K(1, 1));

		const __m128 varScaleTrans = _mm_set_ps1(settings.varScaleTrans);
		const __m128 varScaleRot = _mm_set_ps1(settings.varScaleRot);
		const __m128 varScaleAlpha = _mm_set_ps1(settings.varScaleAlpha);
		const __m128 varScaleBeta = _mm_set_ps1(settings.varScaleBeta);

		for (int i = 0; i < numSSE; i += 4)
		{
			const __m128 x = _mm_load_ps(pointX + i);
			const __m128 y = _mm_load_ps(pointY + i);
			const __m128 iDepth = _mm_load_ps(pointIDepth + i);

			const __m128 fxgx = _mm_mul_ps(_mm_load_ps(pointGx + i), mfx);
			const __m128 fygy = _mm_mul_ps(_mm_load_ps(pointGy + i), mfy);

			const __m128 xfxgx = _mm_mul_ps(x, fxgx);
			const __m128 yfygy = _mm_mul_ps(y, fygy);

			const __m128 weightsse = _mm_load_ps(pointWeight + i);

			__m128 J0 = _mm_mul_ps(_mm_mul_ps(weightsse, _mm_mul_ps(fxgx, iDepth)), varScaleTrans);
			__m128 J1 = _mm_mul_ps(_mm_mul_ps(weightsse, _mm_mul_ps(fygy, iDepth)), varScaleTrans);
			__m128 J2 = _mm_mul_ps(_mm_mul_ps(weightsse, _mm_sub_ps(zeros, _mm_mul_ps(_mm_add_ps(xfxgx, yfygy), iDepth))), varScaleTrans);
			__m128 J3 = _mm_mul_ps(_mm_mul_ps(weightsse, _mm_sub_ps(zeros, _mm_add_ps(_mm_add_ps(_mm_mul_ps(yfygy, y), _mm_mul_ps(xfxgx, y)), fygy))), varScaleRot);
			__m128 J4 = _mm_mul_ps(_mm_mul_ps(weightsse, _mm_add_ps(_mm_add_ps(_mm_mul_ps(yfygy, x), _mm_mul_ps(xfxgx, x)), fxgx)), varScaleRot);
			__m128 J5 = _mm_mul_ps(_mm_mul_ps(weightsse, _mm_sub_ps(_mm_mul_ps(fygy, x), _mm_mul_ps(fxgx, y))), varScaleRot);
			__m128 J6 = _mm_mul_ps(_mm_mul_ps(weightsse, _mm_sub_ps(zeros, _mm_load_ps(pointColor + i))), varScaleAlpha);
			__m128 J7 = _mm_mul_ps(_mm_mul_ps(weightsse, minusOnes), varScaleBeta);

			// reorder and save
			auto& Jpt0 = this->jacobians[lvl][i];
			auto& Jpt1 = this->jacobians[lvl][i + 1];
			auto& Jpt2 = this->jacobians[lvl][i + 2];
			auto& Jpt3 = this->jacobians[lvl][i + 3];

			__m128 val1 = _mm_unpacklo_ps(J0, J1);				// J00, J10, J01, J11
			__m128 val2 = _mm_unpackhi_ps(J0, J1);				// J02, J12, J03, J13
			__m128 val3 = _mm_unpacklo_ps(J2, J3);				// J20, J30, J21, J31
			__m128 val4 = _mm_unpackhi_ps(J2, J3);				// J22, J32, J23, J33

			_mm_store_ps(Jpt0.data(), _mm_movelh_ps(val1, val3));			// J00, J10, J20, J30
			_mm_store_ps(Jpt1.data(), _mm_movehl_ps(val3, val1));			// J01, J11, J21, J31
			_mm_store_ps(Jpt2.data(), _mm_movelh_ps(val2, val4));			// J02, J12, J22, J32
			_mm_store_ps(Jpt3.data(), _mm_movehl_ps(val4, val2));			// J03, J13, J23, J33

			val1 = _mm_unpacklo_ps(J4, J5);						// J40, J50, J41, J51
			val2 = _mm_unpackhi_ps(J4, J5);						// J42, J52, J43, J53
			val3 = _mm_unpacklo_ps(J6, J7);						// J60, J70, J61, J71
			val4 = _mm_unpackhi_ps(J6, J7);						// J62, J72, J63, J73

			_mm_store_ps(Jpt0.data() + 4, _mm_movelh_ps(val1, val3));		// J40, J50, J60, J70
			_mm_store_ps(Jpt1.data() + 4, _mm_movehl_ps(val3, val1));		// J41, J51, J61, J71
			_mm_store_ps(Jpt2.data() + 4, _mm_movelh_ps(val2, val4));		// J42, J52, J62, J72
			_mm_store_ps(Jpt3.data() + 4, _mm_movehl_ps(val4, val2));		// J43, J53, J63, J73
		}

		// update the rest of the points with normal instructions
		for (int i = numSSE; i < num; ++i)
		{
			const float x = pointX[i];
			const float y = pointY[i];
			const float iDepth = pointIDepth[i];
			const float weight = pointWeight[i];

			const float fxgx = K(0, 0)*pointGx[i];
			const float fygy = K(1, 1)*pointGy[i];

			const float xfxgx = x * fxgx;
			const float yfygy = y * fygy;

			auto& J = this->jacobians[lvl][i];

			// respect to SE3 params
			J[0] = fxgx * iDepth;						// (fx*gx)/z
			J[1] = fygy * iDepth;						// (fy*gy)/z
			J[2] = -(xfxgx + yfygy)*iDepth;				// -(fx*gx*x + fy*gy*y)/z^2
			J[3] = -(yfygy*y + xfxgx*y + fygy);			// -(fy*gy*y^2 + fx*gx*x*y + fy*gy*z^2)/z^2
			J[4] = (xfxgx*x + yfygy*x + fxgx);			// (fx*gx*x^2 + fy*gy*y*x + fx*gx*z^2)/z^2
			J[5] = (fygy*x - fxgx*y);					// (fy*gy*x - fx*gx*y)/z

			// respect to affine light
			J[6] = -pointColor[i];
			J[7] = -1.f;

			// weight by gradient
			// use this to transform to adimensional values
			J *= weight;

			// scale variables
			// "Numerical Optimization" Nocedal et al. 2006, page 95
			J.segment<3>(0) *= settings.varScaleTrans;
			J.segment<3>(3) *= settings.varScaleRot;
			J[6] *= settings.varScaleAlpha;
			J[7] *= settings.varScaleBeta;
		}

#else

		for (int i = 0; i < num; ++i)
		{
			const float x = pointX[i];
			const float y = pointY[i];
			const float iDepth = pointIDepth[i];
			const float weight = pointWeight[i];

			const float fxgx = K(0, 0)*pointGx[i];
			const float fygy = K(1, 1)*pointGy[i];

			const float xfxgx = x * fxgx;
			const float yfygy = y * fygy;

			auto& J = this->jacobians[lvl][i];

			// respect to SE3 params
			J[0] = fxgx * iDepth;						// (fx*gx)/z
			J[1] = fygy * iDepth;						// (fy*gy)/z
			J[2] = -(xfxgx + yfygy)*iDepth;				// -(fx*gx*x + fy*gy*y)/z^2
			J[3] = -(yfygy*y + xfxgx*y + fygy);			// -(fy*gy*y^2 + fx*gx*x*y + fy*gy*z^2)/z^2
			J[4] = (xfxgx*x + yfygy*x + fxgx);			// (fx*gx*x^2 + fy*gy*y*x + fx*gx*z^2)/z^2
			J[5] = (fygy*x - fxgx*y);					// (fy*gy*x - fx*gx*y)/z

			// respect to affine light
			J[6] = -pointColor[i];
			J[7] = -1.f;

			// weight by gradient
			// use this to transform to adimensional values
			J *= weight;

			// scale variables
			// "Numerical Optimization" Nocedal et al. 2006, page 95
			J.segment<3>(0) *= settings.varScaleTrans;
			J.segment<3>(3) *= settings.varScaleRot;
			J[6] *= settings.varScaleAlpha;
			J[7] *= settings.varScaleBeta;
		}
#endif

	}

	bool FrameTrackerProblem::solve(float* residuals, float* weights, unsigned int* valid, int lvl,
									const float lambda, const float convEps, Eigen::Vec8f& delta)
	{
		const auto& settings = Settings::getInstance();

		// reset linear system
		this->H_.reset();
		this->g_.reset();

		float num = 0.f;

		const auto& Jv = this->jacobians[lvl];
		const int numberOfPoints = (int)Jv.size();

		// make the summation of all valid points to create the linear system
		for (int i = 0; i < numberOfPoints; ++i)
		{
			if (valid[i] == 0xffffffff)		// true
			{
				const auto& J = Jv[i];
				this->H_.add(J, J, weights[i]);
				this->g_.add(J, residuals[i], weights[i]);
				num++;
			}
		}

		// fill hessian symmetrically
		this->H_.fillSymmetric();

		// normalize with the number of added measurements
		const double normalize = 1.0 / num;
		Eigen::Mat88d H = this->H_.block() * normalize;
		Eigen::Vec8d b = this->g_.block() * normalize;

		// compute levenberg-marquardt hessian
		for (int i = 0; i < 8; i++)
		{
			H(i, i) *= (1.0 + lambda);
		}

		// preconditioning: scaling with the square root of the diag(hessian)
		// "Numerical Optimization" Nocedal et al. 2006, page 119
		Eigen::Vec8d scaleInv = H.diagonal().cwiseSqrt().cwiseInverse();
		H = scaleInv.asDiagonal() * H * scaleInv.asDiagonal();
		b = scaleInv.asDiagonal() * b;

		// solve the linear system of equations by cholesky
		Eigen::Vec8d scaledInc = H.ldlt().solve(-b);

		// obtain steps from scaled ones
		delta = (scaleInv.asDiagonal() * scaledInc).cast<float>();
		delta.segment<3>(0) *= settings.varScaleTrans;
		delta.segment<3>(3) *= settings.varScaleRot;
		delta[6] *= settings.varScaleAlpha;
		delta[7] *= settings.varScaleBeta;

		return !(scaledInc.norm() > convEps);
	}
}