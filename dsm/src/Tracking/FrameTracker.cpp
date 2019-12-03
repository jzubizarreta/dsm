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

#include "FrameTracker.h"
#include "FrameTrackerReference.h"
#include "FrameTrackerProblem.h"
#include "DataStructures/Frame.h"
#include "Statistics/TDistribution.h"
#include "Statistics/RobustNormalDistribution.h"
#include "Utils/GlobalCalibration.h"
#include "Utils/Interpolation.h"
#include "Utils/UtilFunctions.h"
#include "Utils/LossFunction.h"
#include "Utils/Projection.h"

#include "Visualizer/IVisualizer.h"

#include "opencv2/core.hpp"

#include <iostream>

namespace dsm
{
	// Tracker Settings
	FrameTrackerSettings::FrameTrackerSettings()
	{
		this->levels = Settings::getInstance().trackingMaxLevel + 1;

		// Set default settings
		if (this->levels > 6)
		{
			throw std::runtime_error("Error: SE3TrackerSettings() default settings are intended for a maximum of 6 levels!");
		}

		// levmar lambda
		this->lambdaInitial = 0.01f;
		this->lambdaSuccessFac = 0.5f;
		this->lambdaFailFac = 5.0f;
		this->lambdaLimit = 0.001f;

		// convergence managements
		this->convergenceEps = 1.f;

		// number of iteration
		this->maxItsPerLvl = { 10, 20, 50, 75, 100, 100 };

		// tracking lost management
		this->minimumPointUsage = 0.1f;
	}

	FrameTrackerSettings::~FrameTrackerSettings()
	{}

	// Tracker
	FrameTracker::FrameTracker(const int width, const int height, const FrameTrackerSettings &settings)
	{
		// first of all copy the tracker settings
		this->config = std::make_unique<FrameTrackerSettings>(settings);

		// optimization problem
		this->problem = std::make_unique<FrameTrackerProblem>(this->config->levels);

		// buffers
		this->residuals = (float*)Eigen::internal::aligned_malloc(width*height * sizeof(float));
		this->weights = (float*)Eigen::internal::aligned_malloc(width*height * sizeof(float));
		this->validityMask = (unsigned int*)Eigen::internal::aligned_malloc(width*height * sizeof(unsigned int));
		
		this->tempResiduals = (float*)Eigen::internal::aligned_malloc(width*height * sizeof(float));
		this->tempWeights = (float*)Eigen::internal::aligned_malloc(width*height * sizeof(float));
		this->tempValidityMask = (unsigned int*)Eigen::internal::aligned_malloc(width*height * sizeof(unsigned int));

		this->reset();
	}

	FrameTracker::~FrameTracker()
	{
		Eigen::internal::aligned_free((void*)this->residuals);
		Eigen::internal::aligned_free((void*)this->weights);
		Eigen::internal::aligned_free((void*)this->validityMask);

		Eigen::internal::aligned_free((void*)this->tempResiduals);
		Eigen::internal::aligned_free((void*)this->tempWeights);
		Eigen::internal::aligned_free((void*)this->tempValidityMask);
	}

	void FrameTracker::reset()
	{
		this->pointUsage_ = 0.f;
		this->finalResidual_ = 0.f;
	}

	float FrameTracker::pointUsage() const
	{
		return this->pointUsage_;
	}

	float FrameTracker::totalResidual() const
	{
		return this->finalResidual_;
	}
	
	bool FrameTracker::trackFrame(const std::shared_ptr<FrameTrackerReference>& reference, const std::shared_ptr<Frame>& newFrame,
								  Sophus::SE3f& inOutFrameToRef, AffineLight& inOutLight, std::shared_ptr<IDistribution>& errorDistribution,
								  IVisualizer* outputWrapper)
	{
		assert(reference != nullptr);
		assert(newFrame != nullptr);

		const auto& calib = GlobalCalibration::getInstance();
		const auto& settings = Settings::getInstance();

		// initialize all control variables
		this->reset();

		// light prior
		AffineLight affineLight(inOutLight);

		// pose prior
		Sophus::SE3f referenceToFrame = inOutFrameToRef.inverse();

		// distribution
		std::shared_ptr<IDistribution> distribution;

		// do tracking
		int goodIterations = 0;
		int badIterations = 0;

		float pcTime = 0.f;
		float solveTime = 0.f;

		// for each pyramid level
		for (int lvl = settings.trackingMaxLevel; lvl >= 0; --lvl)
		{
			Utils::Time t1 = std::chrono::steady_clock::now();

			// create the reference pointcloud
			const bool pcUpdated = reference->generateReferencePointCloud(lvl);

			// precompute jacobians for reference
			if (pcUpdated)
			{			
				this->problem->computeJacobAtIdentity(reference, lvl);
			}	

			Utils::Time t2 = std::chrono::steady_clock::now();
			pcTime += Utils::elapsedTime(t1, t2);

			// compute the warped point residuals
			float bestPointUsage = this->computeResiduals(reference, newFrame, lvl,
														  referenceToFrame, affineLight, 
														  this->residuals, this->validityMask);

			if (bestPointUsage < this->config->minimumPointUsage)
			{
				return false;
			}

			// fit distribution error for this level
			// keep it fixed during all optimization, it changes the objective function
			distribution = this->computeErrorDistribution(reference, lvl, this->residuals, this->validityMask);

			// compute weights & check outliers
			float bestResidualLvl = this->computeWeightAndOutliers(reference, lvl, distribution,
																   this->residuals, this->validityMask, 
																   this->weights);

			// iterate
			int iteration;
			float lambda = this->config->lambdaInitial;
			for (iteration = 0; iteration < this->config->maxItsPerLvl[lvl]; ++iteration)
			{
				Utils::Time t3 = std::chrono::steady_clock::now();

				// solve
				Eigen::Vec8f inc;
				const bool hasConverged = this->problem->solve(this->residuals, this->weights, this->validityMask,
															   lvl, lambda, this->config->convergenceEps, inc);

				Utils::Time t4 = std::chrono::steady_clock::now();
				solveTime += Utils::elapsedTime(t3, t4);

				// update parameters
				const Sophus::SE3f newReferenceToFrame = referenceToFrame * Sophus::SE3f::exp(-inc.head<6>());				// pose
				const AffineLight newAffineLight = AffineLight::calcRelative(AffineLight(inc[6], inc[7]), affineLight);		// affine light

				// evaluate the residual again with the updated pose
				const float currentPointUsage = this->computeResiduals(reference, newFrame, lvl,
																	   newReferenceToFrame, newAffineLight,
																	   this->tempResiduals, this->tempValidityMask);

				if (currentPointUsage < this->config->minimumPointUsage)
				{
					return false;
				}

				// compute weights & check outliers
				const float currentResidual = this->computeWeightAndOutliers(reference, lvl, distribution,
																			 this->tempResiduals, this->tempValidityMask, 
																			 this->tempWeights);

				// accept the update?
				if (currentResidual < bestResidualLvl)
				{
					referenceToFrame = newReferenceToFrame;
					affineLight = newAffineLight;

					// update lambda
					lambda *= this->config->lambdaSuccessFac;

					// save best values
					bestResidualLvl = currentResidual;
					bestPointUsage = currentPointUsage;

					// take the re-evaluated values
					std::swap(this->tempResiduals, this->residuals);
					std::swap(this->tempValidityMask, this->validityMask);
					std::swap(this->tempWeights, this->weights);

					goodIterations++;
				}
				else
				{
					// update lambda
					lambda *= this->config->lambdaFailFac;

					if (lambda < this->config->lambdaLimit)
					{
						lambda = this->config->lambdaLimit;
					}

					badIterations++;
				}

				// check if the optimization has converged
				if (hasConverged)
				{
					break;
				}
			}

			// compute distribution again for lowest level
			if (lvl == 0)
			{
				distribution = this->computeErrorDistribution(reference, lvl, this->residuals, this->validityMask);

				this->finalResidual_ = sqrt(bestResidualLvl);
				this->pointUsage_ = bestPointUsage;
			}
		}

		//std::cout << "Ref time: " << pcTime << ", solver: " << solveTime << "\n";

		//std::cout << "Tracking good iterations: " << goodIterations;
		//std::cout << ", bad iteartions: " << badIterations << std::endl;

		//std::cout << "Final energy: " << finalResidual_ << std::endl;
			
		// calculate the pose from parent (reference) to this (frame)
		inOutFrameToRef = referenceToFrame.inverse();

		// relative affine light	
		inOutLight = affineLight;

		// error distribution
		errorDistribution = distribution;

		// visualization
		const auto& systemSettings = Settings::getInstance();

		if (outputWrapper && systemSettings.debugShowTrackResult)
		{
			//draw - debug
			const float* u = reference->u(0);
			const float* v = reference->v(0);
			const float* x = reference->x(0);
			const float* y = reference->y(0);
			const float* iDepth = reference->iDepth(0);

			const Eigen::Mat33f& K = calib.matrix3f(0);
			const int width = calib.width(0);
			const int height = calib.height(0);

			const Eigen::Mat33f R = referenceToFrame.rotationMatrix();
			const Eigen::Vec3f& t = referenceToFrame.translation();

			const Eigen::Mat33f KR = K*R;
			const Eigen::Vec3f Kt = K*t;

			cv::Mat_<uchar> refImg(height, width);
			cv::Mat_<uchar> newImg(height, width);

			const float* refImage = reference->reference()->image(0);
			const float* newImage = newFrame->image(0);

			for (int row = 0; row < height; row++)
			{
				for (int col = 0; col < width; col++)
				{
					int idx = row*width + col;
					refImg.data[idx] = (unsigned char)refImage[idx];
					newImg.data[idx] = (unsigned char)newImage[idx];
				}
			}

			cv::Mat output;
			cv::hconcat(refImg, newImg, output);
			cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);

			for (int i = 0; i < reference->numPoints(0); ++i)
			{
				Eigen::Vec3f ptWarp = KR * Eigen::Vec3f(x[i], y[i], 1.f) + Kt*iDepth[i];

				float unew = (ptWarp[0] / ptWarp[2]);
				float vnew = (ptWarp[1] / ptWarp[2]);

				if (unew < 1 || vnew < 1 || unew > width - 1 || vnew > height - 1) continue;

				output.at<cv::Vec3b>(static_cast<int>(v[i] + 0.5f), static_cast<int>(u[i] + 0.5f)) = cv::Vec3b(0, 255, 0);
				output.at<cv::Vec3b>(static_cast<int>(vnew + 0.5f), static_cast<int>(unew + 0.5f) + width) = cv::Vec3b(0, 255, 0);
			}

			outputWrapper->publishTrackingResult(output);
		}

		if (outputWrapper && systemSettings.debugShowTrackError)
		{
			int width = calib.width(0);
			int height = calib.height(0);

			cv::Mat cvError(height, width, CV_8UC3);
			cvError.setTo(cv::Scalar(0, 0, 0));

			const float* u = reference->u(0);
			const float* v = reference->v(0);
			const int numPoints = reference->numPoints(0);

			for (int i = 0; i < numPoints; ++i)
			{
				if (this->validityMask[i] == 0x0) continue;

				Eigen::Vec3f rgb = Utils::colorMap(0.f, 10.f, fabs(this->residuals[i]));

				cvError.at<cv::Vec3b>(int(v[i] + 0.5f), int(u[i] + 0.5f)) = cv::Vec3b((unsigned char)(rgb[2] * 255), 
																					  (unsigned char)(rgb[1] * 255), 
																					  (unsigned char)(rgb[0] * 255));
			}

			outputWrapper->publishTrackingError(cvError);
		}

		if (outputWrapper && systemSettings.debugShowTrackWeights)
		{
			int width = calib.width(0);
			int height = calib.height(0);

			cv::Mat cvWeight(height, width, CV_8UC3);
			cvWeight.setTo(cv::Scalar(255, 178, 102));

			const float* u = reference->u(0);
			const float* v = reference->v(0);
			const int numPoints = reference->numPoints(0);

			float max = -1;
			for (int i = 0; i < numPoints; ++i)
			{
				if (this->validityMask[i] == 0x0) continue;

				if (this->weights[i] > max)
					max = this->weights[i];
			}

			for (int i = 0; i < numPoints; ++i)
			{
				if (this->validityMask[i] == 0x0) continue;

				// the higher weight, the brighter
				unsigned char value = static_cast<unsigned char>(255 * (this->weights[i] / max));
				cvWeight.at<cv::Vec3b>(int(v[i] + 0.5f), int(u[i] + 0.5f)) = cv::Vec3b(value, value, value);
			}

			outputWrapper->publishTrackingWeight(cvWeight);
		}

		if (outputWrapper && systemSettings.debugShowTrackLight)
		{
			cv::Mat cvAffineLight(255, 255, CV_8UC3);
			cvAffineLight.setTo(cv::Scalar(255, 255, 255));

			const Eigen::Mat33f& K = calib.matrix3f(0);
			const int width = calib.width(0);
			const int height = calib.height(0);

			const Eigen::Mat33f R = referenceToFrame.rotationMatrix();
			const Eigen::Vec3f& t = referenceToFrame.translation();

			const Eigen::Mat33f KR = K*R;
			const Eigen::Vec3f Kt = K*t;

			const float* color = reference->color(0);
			const float* x = reference->x(0);
			const float* y = reference->y(0);
			const float* iDepth = reference->iDepth(0);
			const int numPoints = reference->numPoints(0);

			const float* newImage = newFrame->image(0);

			for (int i = 0; i < numPoints; ++i)
			{
				if (this->validityMask[i] == 0x0) continue;

				// transform point
				Eigen::Vec3f ptWarp = KR * Eigen::Vec3f(x[i], y[i], 1.f) + Kt*iDepth[i];

				float unew = (ptWarp[0] / ptWarp[2]);
				float vnew = (ptWarp[1] / ptWarp[2]);

				const float newImgColor = bilinearInterpolation(newImage, unew, vnew, width);

				// image coordiantes
				int row = static_cast<int>(color[i] + 0.5f);
				int col = static_cast<int>(newImgColor + 0.5f);

				col = std::min(254, col);
				col = std::max(0, col);

				row = std::min(254, row);
				row = std::max(0, row);

				// draw
				cvAffineLight.at<cv::Vec3b>(254 - row, col) = cv::Vec3b(0, 0, 0);
			}

			// draw affine light line
			cv::Point p1, p2;

			const float light_a = affineLight.a();
			const float light_b = affineLight.b();

			int pt1x = (int)((-light_b / light_a) + 0.5f);
			if (pt1x >= 0)
			{
				// cross with y = 0
				pt1x = std::min(254, pt1x);
				p1 = cv::Point(pt1x, 0);
			}
			else
			{
				// cross with x = 0
				int pt1y = (int)(light_b + 0.5f);
				pt1y = std::min(254, pt1y);
				p1 = cv::Point(0, pt1y);
			}

			int pt2x = (int)(((254.f - light_b) / light_a) + 0.5f);
			if (pt2x <= 254)
			{
				// cross with y = 254
				pt2x = std::max(0, pt2x);
				p2 = cv::Point(pt2x, 254);
			}
			else
			{
				// cross with x = 254
				int pt2y = (int)(light_a*254.f + light_b + 0.5f);
				pt2y = std::max(0, pt2y);
				p2 = cv::Point(254, pt2y);
			}

			cv::line(cvAffineLight, cv::Point(p1.x, 254 - p1.y), cv::Point(p2.x, 254 - p2.y), cv::Scalar(0, 0, 255), 1);

			outputWrapper->publishTrackingLight(cvAffineLight);
		}

		if (outputWrapper && systemSettings.debugShowTrackDistribution)
		{
			const int numPoints = reference->numPoints(0);

			std::vector<float> dist;
			dist.reserve(numPoints);

			for (int i = 0; i < numPoints; ++i)
			{
				// skip only oob pixels
				// keep outliers also
				if (this->validityMask[i] == 0x0 && 
					this->residuals[i] == 0.f) continue;

				dist.push_back(this->residuals[i]);
			}

			// draw histogram
			cv::Mat hist = Utils::drawDistribution(dist, distribution, 
												   Eigen::Vector2f(-255.f, 255.f),
												   Eigen::Vector2f(0, 0.1f),
												   Eigen::Vector2i(510, 510),
												   510);

			outputWrapper->publishTrackingDistribution(hist);
		}

		return true;
	}

	float FrameTracker::computeResiduals(const std::shared_ptr<FrameTrackerReference>& reference,
										 const std::shared_ptr<Frame>& newFrame, const int lvl,
										 const Sophus::SE3f &refToFrame, const AffineLight& light,
										 float* res, unsigned int* validMask) const
	{
		const auto& calib = GlobalCalibration::getInstance();
		const auto& settings = Settings::getInstance();

		// constant values
		const Eigen::Mat33f& K = calib.matrix3f(lvl);
		const int width = calib.width(lvl);
		const int height = calib.height(lvl);

		const float* const newImage = newFrame->image(lvl);

		const float* const pointU = reference->u(lvl);
		const float* const pointV = reference->v(lvl);
		const float* const pointX = reference->x(lvl);
		const float* const pointY = reference->y(lvl);
		const float* const pointIDepth = reference->iDepth(lvl);
		const float* const pointColor = reference->color(lvl);
		const float* const pointWeight = reference->weight(lvl);
		const int numPoints = reference->numPoints(lvl);

		const Eigen::Mat34f Rt = refToFrame.matrix3x4();
		const Eigen::Vec3f& t = refToFrame.translation();

		const float light_a = light.a();
		const float light_b = light.b();

		float usageCount = 0;

#if defined(ENABLE_SSE)

		int gap = numPoints % 4;
		int numSSE = numPoints - gap;

		const __m128 zeros = _mm_setzero_ps();
		const __m128 ones = _mm_set_ps1(1.f);
		const __m128 one = _mm_setr_ps(0.f, 0.f, 0.f, 1.f);
		const __m128 min = _mm_set_ps1(1.1f);
		const __m128 max_w = _mm_set_ps1((float)width - 2.1f);
		const __m128 max_h = _mm_set_ps1((float)height - 2.1f);

		// calib
		const __m128 mfx = _mm_set_ps1(K(0, 0));
		const __m128 mfy = _mm_set_ps1(K(1, 1));
		const __m128 mcx = _mm_set_ps1(K(0, 2));
		const __m128 mcy = _mm_set_ps1(K(1, 2));

		// do not read using _mm_load_ps. Rt is colMajor
		// Using Eigen::RowMajor is giving weird solutions
		const __m128 r0 = _mm_setr_ps(Rt(0, 0), Rt(0, 1), Rt(0, 2), Rt(0, 3));
		const __m128 r1 = _mm_setr_ps(Rt(1, 0), Rt(1, 1), Rt(1, 2), Rt(1, 3));
		const __m128 r2 = _mm_setr_ps(Rt(2, 0), Rt(2, 1), Rt(2, 2), Rt(2, 3));

		const __m128 t0 = _mm_set_ps1(t[0]);
		const __m128 t1 = _mm_set_ps1(t[1]);
		const __m128 t2 = _mm_set_ps1(t[2]);

		// light
		const __m128 ma = _mm_set_ps1(light_a);
		const __m128 mb = _mm_set_ps1(light_b);

		__m128 usageCountSSE = _mm_setzero_ps();

		__m128 val1, val2, val3, val4;

		for (int i = 0; i < numSSE; i += 4)
		{
			__m128 xxxx = _mm_load_ps(pointX + i);			// x1, x2, x3, x4
			__m128 yyyy = _mm_load_ps(pointY + i);			// y1, y2, y3, y4
			__m128 zzzz = _mm_load_ps(pointIDepth + i);		// z1, z2, z3, z4

			const __m128 iDepth = zzzz;

			val1 = _mm_unpacklo_ps(xxxx, yyyy);				// x1, y1, x2, y2
			val2 = _mm_unpackhi_ps(xxxx, yyyy);				// x3, y3, x4, y4
			val3 = _mm_unpacklo_ps(ones, zzzz);				// 1, z1, 1, z2
			val4 = _mm_unpackhi_ps(ones, zzzz);				// 1, z3, 1, z4

			__m128 pt1 = _mm_movelh_ps(val1, val3);			// x1, y1, 1, z1
			__m128 pt2 = _mm_movehl_ps(val3, val1);			// x2, y2, 1, z2
			__m128 pt3 = _mm_movelh_ps(val2, val4);			// x3, y3, 1, z3
			__m128 pt4 = _mm_movehl_ps(val4, val2);			// x4, y4, 1, z4

			// Perform matrix multiplication
			val1 = _mm_mul_ps(r0, pt1);
			val2 = _mm_mul_ps(r1, pt1);
			val3 = _mm_mul_ps(r2, pt1);
			pt1 = _mm_hadd_ps(_mm_hadd_ps(val1, val2), _mm_hadd_ps(val3, one));		// transformed point 1

			val1 = _mm_mul_ps(r0, pt2);
			val2 = _mm_mul_ps(r1, pt2);
			val3 = _mm_mul_ps(r2, pt2);
			pt2 = _mm_hadd_ps(_mm_hadd_ps(val1, val2), _mm_hadd_ps(val3, one));		// transformed point 2

			val1 = _mm_mul_ps(r0, pt3);
			val2 = _mm_mul_ps(r1, pt3);
			val3 = _mm_mul_ps(r2, pt3);
			pt3 = _mm_hadd_ps(_mm_hadd_ps(val1, val2), _mm_hadd_ps(val3, one));		// transformed point 3

			val1 = _mm_mul_ps(r0, pt4);
			val2 = _mm_mul_ps(r1, pt4);
			val3 = _mm_mul_ps(r2, pt4);
			pt4 = _mm_hadd_ps(_mm_hadd_ps(val1, val2), _mm_hadd_ps(val3, one));		// transformed point 4

			// pack values again
			val1 = _mm_unpacklo_ps(pt1, pt2);	// x1, x2, y1, y2
			val2 = _mm_unpackhi_ps(pt1, pt2);	// z1, z2, 1, 1
			val3 = _mm_unpacklo_ps(pt3, pt4);	// x3, x4, y3, y4
			val4 = _mm_unpackhi_ps(pt3, pt4);	// z3, z4, 1, 1

			xxxx = _mm_movelh_ps(val1, val3);	// x1, x2, x3, x4
			yyyy = _mm_movehl_ps(val3, val1);	// y1, y2, y3, y4
			zzzz = _mm_movelh_ps(val2, val4);	// z1, z2, z3, z4

			// rescale factor
			const __m128 rescale = _mm_rcp_ps(zzzz);

			// calculate projections
			__m128 u = _mm_add_ps(_mm_mul_ps(_mm_mul_ps(xxxx, rescale), mfx), mcx);
			__m128 v = _mm_add_ps(_mm_mul_ps(_mm_mul_ps(yyyy, rescale), mfy), mcy);

			// check inverse depth consistency
			__m128 mask = _mm_cmpgt_ps(rescale, zeros);

			// check that the pixel is valid
			mask = _mm_and_ps(_mm_cmpgt_ps(u, min), mask);		// u > 1.1
			mask = _mm_and_ps(_mm_cmpgt_ps(v, min), mask);		// v > 1.1
			mask = _mm_and_ps(_mm_cmplt_ps(u, max_w), mask);	// u < w-2.1
			mask = _mm_and_ps(_mm_cmplt_ps(v, max_h), mask);	// v < h-2.1

			// set zero to bad points
			u = _mm_and_ps(u, mask);
			v = _mm_and_ps(v, mask);

			// interpolation using sse
			val1 = bilinearInterpolationFloatSSE(newImage, u, v, width);

			// residual
			const __m128 photoResidual = _mm_sub_ps(_mm_sub_ps(_mm_load_ps(pointColor + i), _mm_mul_ps(ma, val1)), mb);

			// if depth becomes larger: pixel becomes "smaller", hence count it less.
			val4 = _mm_cmplt_ps(rescale, ones);
			const __m128 depthChange = _mm_or_ps(_mm_and_ps(val4, rescale), _mm_andnot_ps(val4, ones));

			// transform from photometric residual to geometric residual
			const __m128 residual = _mm_mul_ps(_mm_load_ps(pointWeight + i), photoResidual);

			// usage
			usageCountSSE = _mm_add_ps(usageCountSSE, _mm_and_ps(mask, depthChange));

			// store
			_mm_store_ps(res + i, _mm_and_ps(residual, mask));
			_mm_store_ps((float*)validMask + i, mask);
		}

		// sum horizontally
		usageCountSSE = _mm_hadd_ps(usageCountSSE, usageCountSSE);
		usageCount = _mm_cvtss_f32(_mm_hadd_ps(usageCountSSE, usageCountSSE));

		for (int i = numSSE; i < numPoints; ++i)
		{
			const Eigen::Vec3f ptRT = Rt * Eigen::Vec4f(pointX[i], pointY[i], 1.f, pointIDepth[i]);
			const float rescale = 1.f / ptRT[2];
			const Eigen::Vec2f ptRTn = ptRT.head<2>() * rescale;
			const float uRT = ptRTn[0] * K(0, 0) + K(0, 2);
			const float vRT = ptRTn[1] * K(1, 1) + K(1, 2);

			// check that the pixel is valid and the point is in from of the camera
			if (!(uRT > 2.1f && vRT > 2.1f && uRT < width - 2.1f && vRT < height - 2.1f && rescale > 0.f))
			{
				res[i] = 0.f;
				validMask[i] = 0x0;		//false
				continue;
			}

			const float newImgColor = bilinearInterpolation(newImage, uRT, vRT, width);

			const float photoResidual = pointColor[i] - light_a*newImgColor - light_b;

			// if depth becomes larger: pixel becomes "smaller", hence count it less.
			usageCount += rescale < 1.f ? rescale : 1.f;

			// transform from photometric residual to geometric residual
			const float residual = pointWeight[i] * photoResidual;

			// store
			res[i] = residual;
			validMask[i] = 0xffffffff;		//true
		}

#else

		for (int i = 0; i < numPoints; ++i)
		{
			const Eigen::Vec3f ptRT = Rt * Eigen::Vec4f(pointX[i], pointY[i], 1.f, pointIDepth[i]);
			const float rescale = 1.f / ptRT[2];
			const Eigen::Vec2f ptRTn = ptRT.head<2>() * rescale;
			const float uRT = ptRTn[0] * K(0, 0) + K(0, 2);
			const float vRT = ptRTn[1] * K(1, 1) + K(1, 2);

			// check that the pixel is valid and the point is in from of the camera
			if (!(uRT > 2.1f && vRT > 2.1f && uRT < width - 2.1f && vRT < height - 2.1f && rescale > 0.f))
			{
				res[i] = 0.f;
				validMask[i] = 0x0;		//false				
				continue;
			}

			const float newImgColor = bilinearInterpolation(newImage, uRT, vRT, width);

			const float photoResidual = pointColor[i] - light_a*newImgColor - light_b;

			// if depth becomes larger: pixel becomes "smaller", hence count it less.
			usageCount += rescale < 1.f ? rescale : 1.f;

			// transform from photometric residual to geometric residual
			const float residual = pointWeight[i] * photoResidual;

			// store
			res[i] = residual;
			validMask[i] = 0xffffffff;		//true
		}

#endif

		return (usageCount / numPoints);	// point usage
	}

	std::shared_ptr<IDistribution> FrameTracker::computeErrorDistribution(const std::shared_ptr<FrameTrackerReference>& reference, const int lvl,
																		  const float* const res, const unsigned int* const validMask) const
	{
		const auto& settings = Settings::getInstance();

		const int numPoints = reference->numPoints(lvl);

		std::vector<float> dist;
		dist.reserve(numPoints);

		for (int i = 0; i < numPoints; ++i)
		{
			// skip only oob residuals
			// keep outliers also
			if (validMask[i] == 0x0 &&
				res[i] == 0.f) continue;

			dist.push_back(res[i]);
		}

		// fit distribution parameters
		if (settings.useTDistribution)
		{
			return IDistribution::fitdist(dist, IDistribution::Type::TSTUDENT,
				Eigen::Vec3f(settings.nuFixed, settings.muFixed, settings.sigmaFixed));
		}
		else
		{
			return IDistribution::fitdist(dist, IDistribution::Type::NORMAL,
				Eigen::Vec2f(settings.muFixed, settings.sigmaFixed));
		}
	}

	float FrameTracker::computeWeightAndOutliers(const std::shared_ptr<FrameTrackerReference>& reference, const int lvl,
												 std::shared_ptr<IDistribution>& errorDistribution,
												 const float* const res, unsigned int* const validMask,
												 float* const w) const
	{
		const auto& settings = Settings::getInstance();	

		// outliers threshold percentile
		const float maxEnergy = errorDistribution->icdf(settings.inlierPercentile);

		// number of points
		const int numPoints = reference->numPoints(lvl);

		float energy = 0.f;
		int numEnergy = 0;

#if defined(ENABLE_SSE)

		int gap = numPoints % 4;
		int numSSE = numPoints - gap;

		const __m128 zeroSSE = _mm_setzero_ps();
		__m128 energySSE = _mm_setzero_ps();
		__m128 numEnergySSE = _mm_setzero_ps();

		const __m128 ones = _mm_set_ps1(1.f);
		const __m128 maxEnergySSE = _mm_set_ps1(maxEnergy);

		for (int i = 0; i < numSSE; i += 4)
		{
			// current values
			__m128 mask = _mm_load_ps((float*)validMask + i);
			const __m128 residual = _mm_load_ps(res + i);

			// absolute value
			const __m128 absResidual = _mm_max_ps(residual, _mm_sub_ps(zeroSSE, residual));

			// outlier
			mask = _mm_and_ps(_mm_cmple_ps(absResidual, maxEnergySSE), mask);

			// weight
			const __m128 weight = _mm_and_ps(errorDistribution->weightSSE(residual), mask);

			// store
			_mm_store_ps(w + i, weight);
			_mm_store_ps((float*)validMask + i, mask);

			// total energy
			energySSE = _mm_add_ps(energySSE, _mm_mul_ps(weight, _mm_mul_ps(residual, residual)));

			// counter
			numEnergySSE = _mm_add_ps(numEnergySSE, _mm_and_ps(mask, ones));
		}

		// sum horizontally
		energySSE = _mm_hadd_ps(energySSE, energySSE);
		energy = _mm_cvtss_f32(_mm_hadd_ps(energySSE, energySSE));

		numEnergySSE = _mm_hadd_ps(numEnergySSE, numEnergySSE);
		numEnergy = (int)_mm_cvtss_f32(_mm_hadd_ps(numEnergySSE, numEnergySSE));

		for (int i = numSSE; i < numPoints; ++i)
		{
			// skip if already marked as not valid
			if (validMask[i] == 0x0) continue;
			
			// outlier
			if (fabs(res[i]) <= maxEnergy)	// good point
			{
				w[i] = errorDistribution->weight(res[i]);
				validMask[i] = 0xffffffff;		
			}
			else							// bad point
			{
				w[i] = 0.f;
				validMask[i] = 0x0;				
			}

			// total energy
			energy += w[i] * res[i] * res[i];

			// counter
			numEnergy++;
		}
#else

		for (int i = 0; i < numPoints; ++i)
		{
			// skip if already marked as not valid
			if (validMask[i] == 0x0) continue;

			// outlier
			if (fabs(res[i]) <= maxEnergy)	// good point
			{
				w[i] = errorDistribution->weight(res[i]);
				validMask[i] = 0xffffffff;
			}
			else							// bad point
			{
				w[i] = 0.f;
				validMask[i] = 0x0;
			}

			// total energy
			energy += w[i] * res[i] * res[i];

			// counter
			numEnergy++;
		}

#endif

		return energy / (numEnergy + Settings::DIVISION_EPS);
	}
}
