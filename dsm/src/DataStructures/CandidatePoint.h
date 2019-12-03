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

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "Visibility.h"
#include "Pattern.h"
#include "AffineLight.h"
#include "Utils/LossFunction.h"
#include "FullSystem/DSMLib.h"

namespace dsm
{
	class Frame;
	class ActivePoint;

	// Candidate point class used for point initialization
	// before they are inserted into the optimization
	// This class tries to obtaint a good inverse depth initialization
	class CandidatePoint
	{

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		friend class ActivePoint;

		// control flag
		enum PointStatus { UNINITIALIZED = 0, INITIALIZED = 1, OPTIMIZED = 2, OUTLIER = 3 };

		// GOOD: succesfuly matched
		// BAD: error too big, outlier
		// OOB: the point went out of image boundary
		// SKIPPED: estimation good enough or need to skip
		// BAD_CONDITIONED: geometry is not appropriate enough to estimate a good inverse depth
		enum ObserveStatus { GOOD = 0, BAD_ERROR = 1, BAD_EPILINE = 2, OOB = 3, SKIPPED = 4, BAD_CONDITIONED = 5 };

		CandidatePoint(float x, float y, int32_t lvl, const std::shared_ptr<Frame>& refFrame);

		~CandidatePoint();

		// point in reference frame
		float u0() const;
		float v0() const;

		int32_t detectedLevel() const;

		// point reference image
		// the first image in which the point was observed
		Frame* reference() const;

		// inverse depth in reference frame coordinates
		float iDepth() const;

		// point status flag
		PointStatus status() const;
		
		// observation status flag
		ObserveStatus lastObservation() const;

		// last observation
		float matchQuality() const;
		float matchUncertainty() const;
		float matchBaseline() const;

		// visibility
		Visibility visibility(int activeID) const;
		bool isVisible(int activeID) const;

		// initialize & update inverse depth and orientation using one frame
		ObserveStatus observe(const std::shared_ptr<Frame>& other);

		// optimize point inverse depth using all active keyframes
		void optimize(const std::vector<std::shared_ptr<Frame>>& activeKeyframes);

	private:

		// epipolar line search range computation
		ObserveStatus calcEpipolarGeometry(const Eigen::Vector3f& KRray, const Eigen::Vector3f& Kt, Eigen::Vector2f& epiLineDir,
										   float& epiLineLength, Eigen::Vector2f& pxStart, Eigen::Vector2f& pxEnd) const;

		// set bad observation
		ObserveStatus setBadObservation();

		// inverse depth optimization evaluation
		Visibility computeJacobianAndResidual(const std::shared_ptr<Frame>& frame, const Eigen::Matrix3f& K, 
											  int width, int height, float inverseDepth, float outlierIncFactor, 
											  float &H, float &b, float &energy) const;

	private:

		// pixel location at zero level
		const Eigen::Vector2f u0_;

		// image pyramid level in which was detected
		const int32_t detectedLevel_;

		// pixel localtion at detected level
		Eigen::Vector2f u_;

		// reference frame
		Frame* const refFrame_;

		// pattern values
		std::vector<float> color_;					// color of each point in the pattern in reference image
		std::vector<float> weights_;				// weight for each point in the pattern in reference image

		// ray from camera optic center to the central pixel in the detected level
		// we will assume the normal has the ray direction pointing to the camera center
		Eigen::Vector3f ray_;						

		// pattern normalized gradients
		float gx2, gxy, gy2;

		// geometric parameterization
		float iDepth_;
		float iDepthSigma_;

		// status of the point
		PointStatus status_;
		ObserveStatus lastObservation_;

		// point tracking evaluation
		float matchQuality_;
		float matchUncertainty_;				// match uncertainty
		float matchBaseline_;					// match baseline in pixels

		// visibility of the point in each active keyframe
		std::vector<Visibility> visibility_;
	};
}