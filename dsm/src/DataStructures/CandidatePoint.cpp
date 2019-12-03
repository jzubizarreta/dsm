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

#include <iostream>
#include <algorithm>

#include "CandidatePoint.h"
#include "Frame.h"
#include "Utils/UtilFunctions.h"
#include "Utils/Interpolation.h"
#include "Utils/Settings.h"
#include "Utils/Projection.h"
#include "Utils/GlobalCalibration.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace dsm
{
	CandidatePoint::CandidatePoint(float x, float y, int32_t lvl, const std::shared_ptr<Frame>& frame) :
		u0_(x, y), refFrame_(frame.get()), detectedLevel_(lvl), status_(UNINITIALIZED), lastObservation_(GOOD)
	{
		const auto& settings = Settings::getInstance();
		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& invK = calib.invMatrix3f(this->detectedLevel_);

		// position in the detected level
		this->u_ = ((this->u0_ + Eigen::Vector2f(0.5f, 0.5f)) / (float)((int)1 << this->detectedLevel_)) - Eigen::Vector2f(0.5f, 0.5f);

		this->ray_ = Eigen::Vector3f(this->u_[0]*invK(0, 0) + invK(0, 2),
									 this->u_[1]*invK(1, 1) + invK(1, 2),
									 1.f);

		// initialize geometric parameters to invalid values
		this->iDepth_ = 0.f;
		this->iDepthSigma_ = std::numeric_limits<float>::max();

		// observation evaluation
		this->matchQuality_ = 0.f;
		this->matchUncertainty_ = std::numeric_limits<float>::max();
		this->matchBaseline_ = 0.f;

		// initialize values in the detected lvl
		const float* const image = this->refFrame_->image(this->detectedLevel_);
		const float* const grad = this->refFrame_->gradient(this->detectedLevel_);		// squared norm magnitude
		const float* const gx = this->refFrame_->gx(this->detectedLevel_);
		const float* const gy = this->refFrame_->gy(this->detectedLevel_);
		const int width = calib.width(this->detectedLevel_);
		const int height = calib.height(this->detectedLevel_);
		const float weightConstant = settings.weightConstant;

		this->color_.resize(Pattern::size());
		this->weights_.resize(Pattern::size());
		this->gx2 = this->gy2 = this->gxy = 0.f;

		for (int idx = 0; idx < Pattern::size(); ++idx)
		{
			const Eigen::Vector2f uj = this->u_ + Pattern::at(idx).cast<float>();

			if (!Utils::checkImageBoundaries(uj, width, height))
			{
				// mark as outlier from start
				// we can even take the reference pattern in this level
				this->status_ = PointStatus::OUTLIER;
				this->lastObservation_ = ObserveStatus::OOB;
				return;
			}
			
			this->color_[idx] = bilinearInterpolation(image, uj[0], uj[1], width);
			const float gxj = bilinearInterpolation(gx, uj[0], uj[1], width);
			const float gyj = bilinearInterpolation(gy, uj[0], uj[1], width);

			// hessian approximation
			const float gxgx = gxj*gxj;
			const float gygy = gyj*gyj;
			const float mag2 = gxgx + gygy;

			this->gx2 += gxgx;
			this->gy2 += gygy;
			this->gxy += gxj*gyj;

			// weight = sqrt( c^2 / (c^2 + grad^2) )
			this->weights_[idx] = sqrtf(weightConstant / (weightConstant + mag2));
		}

		// normalize gradients
		const float norm = this->gx2 + this->gy2 + std::numeric_limits<float>::epsilon();
		this->gx2 /= norm;
		this->gy2 /= norm;
		this->gxy /= norm;
	}

	CandidatePoint::~CandidatePoint()
	{}

	float CandidatePoint::u0() const
	{
		return this->u0_[0];
	}

	float CandidatePoint::v0() const
	{
		return this->u0_[1];
	}

	int32_t CandidatePoint::detectedLevel() const
	{
		return this->detectedLevel_;
	}

	Frame* CandidatePoint::reference() const
	{
		return this->refFrame_;
	}

	float CandidatePoint::iDepth() const
	{
		return this->iDepth_;
	}

	CandidatePoint::PointStatus CandidatePoint::status() const
	{
		return this->status_;
	}

	CandidatePoint::ObserveStatus CandidatePoint::lastObservation() const
	{
		return this->lastObservation_;
	}

	float CandidatePoint::matchQuality() const
	{
		return this->matchQuality_;
	}

	float CandidatePoint::matchUncertainty() const
	{
		return this->matchUncertainty_;
	}

	float CandidatePoint::matchBaseline() const
	{
		return this->matchBaseline_;
	}

	Visibility CandidatePoint::visibility(int activeID) const
	{
		return this->visibility_[activeID];
	}

	bool CandidatePoint::isVisible(int activeID) const
	{
		return this->visibility_[activeID] == Visibility::VISIBLE;
	}

	CandidatePoint::ObserveStatus CandidatePoint::observe(const std::shared_ptr<Frame>& other)
	{
		if (this->status_ == PointStatus::OUTLIER)
		{
			return ObserveStatus::SKIPPED;
		}

		const auto& settings = Settings::getInstance();
		const auto& calib = GlobalCalibration::getInstance();

		// image size
		const Eigen::Matrix3f& K = calib.matrix3f(this->detectedLevel_);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(this->detectedLevel_);
		const int32_t width = calib.width(this->detectedLevel_);
		const int32_t height = calib.height(this->detectedLevel_);

		// frame error distribution
		const std::shared_ptr<IDistribution> errorDistribution = other->errorDistribution();

		// current two-view geometry
		// other: frame -> use relative values with tracking parent
		// reference: keyframe -> use global values
		const Sophus::SE3f otherPose = other->parent()->camToWorld() * other->thisToParentPose();
		const Sophus::SE3f& refPose = this->refFrame_->camToWorld();

		const Sophus::SE3f refToOther = otherPose.inverse() * refPose;
		const Eigen::Matrix3f rot = refToOther.rotationMatrix();
		const Eigen::Vector3f& trans = refToOther.translation();
		const Eigen::Matrix3f KR = K * rot;
		const Eigen::Vector3f Kt = K * trans;

		const Eigen::Vector3f KRray = KR*this->ray_;

		// 1) epipolar geometry
		float epiLineLength;
		Eigen::Vector2f epiLineDir, pxStart, pxEnd;
		const ObserveStatus epiLineStatus = this->calcEpipolarGeometry(KRray, Kt, epiLineDir, epiLineLength, pxStart, pxEnd);

		if (epiLineStatus != ObserveStatus::GOOD)
		{
			if (epiLineStatus == ObserveStatus::BAD_EPILINE)
			{
				this->status_ = PointStatus::OUTLIER;

				if (this->status_ == PointStatus::UNINITIALIZED)
				{
					// if this happens, check iDepthMax value when UNINITIALIZED
					std::cout << "Bad epipolar geometry during initialization... This should not happen!\n";
				}
			}
			return this->lastObservation_ = epiLineStatus;
		}

		// 2) point uncertainty for this geometric configuration
		// use Eq. 6 from "Semi-Dense Visual Odometry for a Monocular Camera" Engel et al. 2013
		const float dirx2 = epiLineDir[0] * epiLineDir[0];
		const float diry2 = epiLineDir[1] * epiLineDir[1];
		const float dirxy = epiLineDir[0] * epiLineDir[1];

		// cosine between gradient and epipolar line
		const float gradDotLine2 = (dirx2*this->gx2 + 2.f*dirxy*this->gxy + diry2*this->gy2);

		// matching uncertainty
		const float disparitySigma = sqrt(settings.epiLineSigma / (gradDotLine2 + std::numeric_limits<float>::epsilon()));

		// check bad conditioned configuration if [-2*disparitySigma, 2*disparitySigma] is bigger than the epipolar line
		if (disparitySigma*4.f > epiLineLength)
		{
			return this->lastObservation_ = ObserveStatus::BAD_CONDITIONED;
		}

		const Eigen::Matrix3f KRKinv = KR * Kinv;

		// 3) Do epipolar line search
		const int numEvaluations = (int)(epiLineLength + 0.5f);

		const float* const newImage = other->image(this->detectedLevel_);
		const AffineLight newLight = AffineLight::calcGlobal(other->parent()->affineLight(),
															 other->thisToParentLight());
		const AffineLight refToOtherLight = AffineLight::calcRelative(this->refFrame_->affineLight(),
																	  newLight);

		const float light_a = refToOtherLight.a();
		const float light_b = refToOtherLight.b();

		Eigen::Vector2f px = pxStart;
		Eigen::Vector2f bestMatch;
		float bestMatchIDepth;
		float bestMatchNumBad;
		float bestMatchError = std::numeric_limits<float>::max();
		int bestMatchIndex = -1;
		std::vector<float> errors(numEvaluations);

		for (int n = 0; n < numEvaluations; ++n)
		{
			// match_n inverse depth
			float iDepthN;
			if (epiLineDir[0] * epiLineDir[0] > epiLineDir[1] * epiLineDir[1])
			{
				iDepthN = (KRray[2] * px[0] - KRray[0]) / (Kt[0] - Kt[2] * px[0]);
			}
			else
			{
				iDepthN = (KRray[2] * px[1] - KRray[1]) / (Kt[1] - Kt[2] * px[1]);
			}

			// project all points to other image with the inverse depth
			float error2 = 0.f;
			float numBad = 0.f;
			for (int32_t idx = 0; idx < Pattern::size(); ++idx)
			{
				const Eigen::Vector2f uj = this->u_ + Pattern::at(idx).cast<float>();

				Eigen::Vector2f pt2d;
				if (!Utils::project(uj[0], uj[1], iDepthN, width, height, KRKinv, Kt, pt2d))
				{
					error2 = std::numeric_limits<float>::max();
					break;
				}

				const float newColor = bilinearInterpolation(newImage, pt2d[0], pt2d[1], width);
				const float errorj = this->weights_[idx] * (this->color_[idx] - light_a*newColor - light_b);

				error2 += errorj*errorj;

				// bad pixels
				if (fabs(errorj) > settings.stereoMaxEnergy)
				{
					numBad++;
				}
			}

			if (error2 < bestMatchError)
			{
				// set best.
				bestMatchError = error2;
				bestMatchIDepth = iDepthN;
				bestMatchNumBad = numBad;
				bestMatchIndex = n;					

				bestMatch = px;
			}

			errors[n] = error2;
			px += epiLineDir;
		}

		// 4) Check if many bad pixels
		if (bestMatchNumBad / Pattern::size() > settings.maxPixelOutlier)
		{
			return this->setBadObservation();
		}

		// 5) obtain the second best match
		const auto start = bestMatchIndex > settings.secondBestRadius ? std::begin(errors) + bestMatchIndex - settings.secondBestRadius : std::begin(errors);
		const auto end = errors.size() - settings.secondBestRadius > bestMatchIndex ? std::begin(errors) + bestMatchIndex + settings.secondBestRadius + 1 : std::end(errors);

		std::fill(start, end, std::numeric_limits<float>::max());
		const float secondBestMatchError = *std::min_element(std::begin(errors), std::end(errors));

		// quality with the second best match
		const float newQuality = secondBestMatchError / bestMatchError;
			
		// 6) inverse depth uncertainty
		// Eq. 8 and 9 from "Probabilistic Semi-Dense Mapping from Highly Accurate Feature-Based Monocular SLAM" Mur-Artal et al. 2015
		float iDepthSigmaHypo;
		if (epiLineDir[0] * epiLineDir[0] > epiLineDir[1] * epiLineDir[1])
		{
			// iDepthSigma Hypothesis
			const float disparitySigma_x = disparitySigma*epiLineDir[0];
			const float bestMatch_x_minus = bestMatch[0] - disparitySigma_x;
			const float bestMatch_x_plus = bestMatch[0] + disparitySigma_x;

			const float iDepthMinus = (KRray[2] * bestMatch_x_minus - KRray[0]) / (Kt[0] - Kt[2] * bestMatch_x_minus);
			const float iDepthPlus = (KRray[2] * bestMatch_x_plus - KRray[0]) / (Kt[0] - Kt[2] * bestMatch_x_plus);

			iDepthSigmaHypo = std::max(abs(iDepthPlus - bestMatchIDepth), abs(iDepthMinus - bestMatchIDepth));
		}
		else
		{
			// iDepthSigma Hypothesis
			const float disparitySigma_y = disparitySigma*epiLineDir[1];
			const float bestMatch_y_minus = bestMatch[1] - disparitySigma_y;
			const float bestMatch_y_plus = bestMatch[1] + disparitySigma_y;

			const float iDepthMinus = (KRray[2] * bestMatch_y_minus - KRray[1]) / (Kt[1] - Kt[2] * bestMatch_y_minus);
			const float iDepthPlus = (KRray[2] * bestMatch_y_plus - KRray[1]) / (Kt[1] - Kt[2] * bestMatch_y_plus);

			iDepthSigmaHypo = std::max(abs(iDepthPlus - bestMatchIDepth), abs(iDepthMinus - bestMatchIDepth));
		}

		// 7) matched point baseline in pixels
		// we will keep the match with highest baseline
		// since it should have been estimated more accurately

		// point at infinity
		const Eigen::Vector2f pxInf = KRray.head<2>() / KRray[2];

		// baseline in pixels
		const float baseline = (bestMatch - pxInf).norm();

		if (baseline > this->matchBaseline_)
		{
			this->iDepth_ = bestMatchIDepth;
			this->iDepthSigma_ = iDepthSigmaHypo;
			this->matchUncertainty_ = disparitySigma;
			this->matchBaseline_ = baseline;

			// store always the worst quality
			if (this->status_ == PointStatus::UNINITIALIZED ||
				newQuality < this->matchQuality_)
			{
				this->matchQuality_ = newQuality;
			}
		}

		this->status_ = PointStatus::INITIALIZED;
		return this->lastObservation_ = ObserveStatus::GOOD;
	}

	CandidatePoint::ObserveStatus CandidatePoint::calcEpipolarGeometry(const Eigen::Vector3f& KRray, const Eigen::Vector3f& Kt, Eigen::Vector2f& epiLineDir,
																	   float& epiLineLength, Eigen::Vector2f& pxStart, Eigen::Vector2f& pxEnd) const
	{
		const auto& settings = Settings::getInstance();
		const auto& calib = GlobalCalibration::getInstance();
		const int width = calib.width(this->detectedLevel_);
		const int height = calib.height(this->detectedLevel_);

		// If initialized, use the uncertainty to reduce the search range
		float iDepthMin, iDepthMax;
		if (this->status_ == PointStatus::INITIALIZED)
		{
			// obtain inverse depth search range from uncertainty
			// range = +- 2*sigma
			iDepthMin = this->iDepth_ - 2.f*this->iDepthSigma_;
			iDepthMax = this->iDepth_ + 2.f*this->iDepthSigma_;
		}
		// else, do full search
		else
		{
			// minIDepth at infinity
			iDepthMin = 0.f;

			// maxIDepth at random value in front of both cameras
			// it does not matter which value, it is only to obtain epiLineDir
			// TODO: we should select a value that gives positive iDepth in both cameras
			iDepthMax = 1e-02f;
		}

		// check iDepthMax, it should not be negative
		// point at iDepthMax must be in front of both cameras
		if (iDepthMax < 0.f)
		{
			return ObserveStatus::BAD_EPILINE;
		}
		
		// project to new image
		if (!Utils::project(KRray, Kt, iDepthMin, width, height, pxStart))
		{
			return ObserveStatus::OOB;
		}

		// do not check image boundaries here
		if (!Utils::project<float, false>(KRray, Kt, iDepthMax, width, height, pxEnd))
		{
			return ObserveStatus::BAD_EPILINE;
		}

		// epipolar line
		epiLineDir = pxEnd - pxStart;
		epiLineLength = epiLineDir.norm();
		epiLineDir /= (epiLineLength + std::numeric_limits<float>::epsilon());

		// limit search range
		const float maxSearchRange = sqrtf((float)width*width + (float)height*height)*settings.maxEplLengthFactor;
		if (this->status_ == PointStatus::INITIALIZED)
		{
			// check if search distance is too short
			if (epiLineLength < settings.minEplLengthSkip)
			{
				return ObserveStatus::SKIPPED;
			}			

			// check if search distance is too long
			// do not discard, keep searching
			if (epiLineLength > maxSearchRange)
			{
				pxEnd = pxStart + maxSearchRange*epiLineDir;
				epiLineLength = maxSearchRange;
			}
		}
		else
		{
			// set maximum search range
			pxEnd = pxStart + maxSearchRange*epiLineDir;
			epiLineLength = maxSearchRange;
		}

		// check image boundaries at iDepthMax here!
		if (!Utils::checkImageBoundaries(pxEnd, width, height))
		{
			return ObserveStatus::OOB;
		}

		return ObserveStatus::GOOD;
	}

	CandidatePoint::ObserveStatus CandidatePoint::setBadObservation()
	{
		// outlier!
		if (this->lastObservation_ == ObserveStatus::BAD_ERROR)
		{
			// two consecutive times... enough
			// do not search anymore
			this->status_ = PointStatus::OUTLIER;
		}
		else
		{
			this->lastObservation_ = ObserveStatus::BAD_ERROR;
		}

		return this->lastObservation_;
	}

	void CandidatePoint::optimize(const std::vector<std::shared_ptr<Frame>>& activeKeyframes)
	{
		if (this->status_ == PointStatus::UNINITIALIZED ||
			this->status_ == PointStatus::OUTLIER)
		{
			return;
		}

		const auto& calib = GlobalCalibration::getInstance();
		const auto& settings = Settings::getInstance();

		const Eigen::Matrix3f& K = calib.matrix3f(this->detectedLevel_);
		const int width = (int)calib.width(this->detectedLevel_);
		const int height = (int)calib.height(this->detectedLevel_);

		// do non-linear optimization to minimize the photometric error
		// of the point in all the active keyframes

		float lambda = 0.1f;
		const float lambdaSuccessFac = 0.5f;
		const float lambdaFailFac = 5.0f;
		const float convergenceEps = 1e-03f;

		float currentIDepth = this->iDepth_;

		const int numActiveKeyframes = (int)activeKeyframes.size();

		// reset visibility
		this->visibility_.resize(numActiveKeyframes, Visibility::UNINITIALIZED);
		std::fill(this->visibility_.begin(), this->visibility_.end(), Visibility::UNINITIALIZED);

		// compute first step
		float H = 0.f;
		float b = 0.f;
		float energy = 0.f;
		for (int i = 0; i < numActiveKeyframes; ++i)
		{
			const std::shared_ptr<Frame>& targetKeyframe = activeKeyframes[i];

			// do not evaluate in the reference frame
			// changing inverse depth does not change residual
			if (targetKeyframe.get() == this->refFrame_) continue;

			// big outlier factor to give a chance
			this->visibility_[targetKeyframe->activeID()] = this->computeJacobianAndResidual(targetKeyframe, K, 
																							 width, height, currentIDepth, 100.f, 
																							 H, b, energy);
		}

		// check that there are good values for optimization H != 0
		if (fabs(H) < 1e-03f)
		{
			this->status_ = PointStatus::INITIALIZED;
			return;
		}

		// generate the residual and jacobian for each keyframe if visible
		for (int it = 0; it < settings.candidateOptIterations; ++it)
		{
			// step
			H *= 1.f + lambda;			// levmar
			const float step = -b / H;

			const float newIDepth = currentIDepth + step;

			// calculate current energy with the new inverse depth
			// and estimate if we improve or not
			float newH = 0.f;
			float newb = 0.f;
			float newEnergy = 0.f;
			for (int i = 0; i < numActiveKeyframes; ++i)
			{
				const std::shared_ptr<Frame>& targetKeyframe = activeKeyframes[i];

				// do not evaluate in the reference frame
				// changing inverse depth does not change residual
				if (targetKeyframe.get() == this->refFrame_) continue;

				// restrict outlier images with low factor
				this->visibility_[targetKeyframe->activeID()] = this->computeJacobianAndResidual(targetKeyframe, K, 
																								 width, height, newIDepth, 1.f, 
																								 newH, newb, newEnergy);
			}

			if (fabs(H) < 1e-03f)
			{
				this->status_ = PointStatus::INITIALIZED;
				return;
			}

			if (newEnergy < energy)
			{
				// save new values
				currentIDepth = newIDepth;
				H = newH;
				b = newb;
				energy = newEnergy;

				// update lambda
				lambda *= lambdaSuccessFac;

				// should we limit lambda value?
			}
			else
			{
				// update lambda
				lambda *= lambdaFailFac;
			}

			// check if the optimization has converged
			if (fabs(step) < convergenceEps)
			{
				break;
			}
		}

		// check if inverse depth has real values
		if (!std::isfinite(currentIDepth))
		{
			this->status_ = PointStatus::OUTLIER;
			return;
		}

		// store new values
		this->iDepth_ = currentIDepth;
		this->status_ = PointStatus::OPTIMIZED;
	}

	Visibility CandidatePoint::computeJacobianAndResidual(const std::shared_ptr<Frame>& frame, const Eigen::Matrix3f& K,
														  int width, int height, float inverseDepth, float outlierIncFactor,
														  float &H, float &b, float &energy) const
	{
		const auto& settings = Settings::getInstance();

		// relative data
		// all are keyframes
		const Sophus::SE3f refToFramePose = frame->camToWorld().inverse() * this->refFrame_->camToWorld();
		const Eigen::Matrix3f rot = refToFramePose.rotationMatrix();
		const Eigen::Vector3f& trans = refToFramePose.translation();

		const AffineLight refToFrameLight = AffineLight::calcRelative(this->refFrame_->affineLight(), frame->affineLight());
		const float light_a = refToFrameLight.a();
		const float light_b = refToFrameLight.b();

		const float* newImage = frame->image(this->detectedLevel_);
		const float* gx = frame->gx(this->detectedLevel_);
		const float* gy = frame->gy(this->detectedLevel_);

		const auto& dist = frame->errorDistribution();
		const float energyThreshold = frame->energyThreshold()*outlierIncFactor;

		// compute the residual and jacobian in the frame
		// for inverse depth optimization
		float frameH = 0.f;
		float frameB = 0.f;
		float frameEnergy = 0.f;

		float numBad = 0.f;

		Eigen::Vector2f Xpn, pt2d;
		float newIDepth, rescale;

		for (int idx = 0; idx < Pattern::size(); ++idx)
		{
			const Eigen::Vector2f uj = this->u_ + Pattern::at(idx).cast<float>();

			if (!Utils::project(uj[0], uj[1], inverseDepth, K, width, height, rot, trans,
								Xpn, pt2d, newIDepth, rescale))
			{
				return Visibility::OOB;
			}
		
			// residual
			const float newImgColor = bilinearInterpolation(newImage, pt2d[0], pt2d[1], width);

			// weight by gradient
			// this is similar to estimate displacement from intensity residual
			// we want to work with adimensional values
			const float error = this->weights_[idx] * (this->color_[idx] - light_a*newImgColor - light_b);

			// weight function
			const float weight = dist->weight(error);
			frameEnergy += weight * error*error;

			// bad pixels
			if (fabs(error) > energyThreshold)
			{
				numBad++;				
			}

			const float newGx = bilinearInterpolation(gx, pt2d[0], pt2d[1], width);
			const float newGy = bilinearInterpolation(gy, pt2d[0], pt2d[1], width);
			float jacob = -this->weights_[idx] * light_a * (newGx*K(0, 0)*rescale*(trans[0] - trans[2] * Xpn[0]) +
															newGy*K(1, 1)*rescale*(trans[1] - trans[2] * Xpn[1]));
		
			// linear system
			frameH += weight * jacob*jacob;
			frameB += weight * error*jacob;			
		}

		Visibility vis = Visibility::VISIBLE;

		// check energy for this frame
		if (numBad / Pattern::size() > settings.maxPixelOutlier)
		{
			vis = Visibility::OUTLIER;
		}

		H += frameH;
		b += frameB;
		energy += frameEnergy;		

		return vis;
	}
}
