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

#include "Visibility.h"
#include "FullSystem/DSMLib.h"
#include "Utils/EigenTypes.h"

#include <vector>
#include <array>
#include <unordered_map>
#include <memory>

#include <Eigen/Core>

namespace dsm
{
	class Frame;
	class CandidatePoint;

	class PointParameterBlock;
	class PhotometricResidual;

	// Active point in the sliding window
	// It can only be activated from a initialized candidate
	class DSM_EXPORTS_DLL ActivePoint
	{

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		// The constructor will move all the required information
		// The candidate point must be deleted after ActivePoint constructor
		ActivePoint(int32_t creationID, const std::unique_ptr<CandidatePoint>& candidate);

		// Destructor
		~ActivePoint();

		// born identifier (temporal)
		int32_t currentID() const;

		// point center
		float u(int32_t lvl) const;
		float v(int32_t lvl) const;

		// point reference image
		// the first image in which the point was observed
		Frame* reference() const;

		// 3D central point
		Eigen::Vector3f pt3d() const;

		// vectors
		const Eigen::VecXf& colors(int32_t lvl) const;
		const Eigen::VecXf& weights(int32_t lvl) const;
		const bool valid(int32_t lvl) const;

		// inverse depth in reference frame coordinates
		float iDepth() const;	
		void setIDepth(float value);

		// visibility
		Visibility visibility(int keyframeID) const;
		void setVisibility(int idx, Visibility vis);

		// center projection in last keyframe if active
		const Eigen::Vector2f& centerProjection() const;
		void setCenterProjection(const Eigen::Vector2f& center);		

		// observations
		const std::unordered_map<Frame*, std::unique_ptr<PhotometricResidual>>& observations() const;
		void addObservation(Frame* const kf, std::unique_ptr<PhotometricResidual>& obs);
		void eraseObservation(Frame* const kf);
		bool observationExists(Frame* const kf);
		int32_t numObservations() const;

		// point age based on creationID: 0, 1, 2, ...
		int32_t age(int newKeyframeID) const;

		// optimization functions
		void mergeOptimizationResult();

		// get optimization parameter block
		const std::unique_ptr<PointParameterBlock>& pointBlock() const;

		// optimization hessian
		float iDepthHessian() const;
		void setIDepthHessian(float H);

		// observations maximum parallax
		float parallax() const;
		void setParallax(float p);

	private:

		// keyframeID at the moment the point was created
		// It is not the refFrame identifier
		// It is used for point's age management
		int32_t currentID_;

		// reference frame
		Frame * refFrame_;

		// values for multiple pyramid levels
		std::vector<float> u_;						// center location at each level
		std::vector<float> v_;
		std::vector<Eigen::VecXf> color_;			// color of each point in the pattern in reference image
		std::vector<Eigen::VecXf> weights_;			// weight for each point in the pattern in reference image
		std::vector<bool> validity_;				// if the point is valid in the level

		Eigen::Vector3f ray_;						// ray from camera optic center to the central pixel in the detected level

		// geometric parameterization
		float iDepth_;								// point inverse depth

		// pattern center position in reference keyframe at zero level
		Eigen::Vector2f centerProjection_;

		// all valid observations for this point: <targetFrame, residual>
		std::unordered_map<Frame*, std::unique_ptr<PhotometricResidual>> observations_;

		// visibility of the point in each keyframe (historical)
		std::vector<Visibility> visibility_;

		// ceres optimization data
		std::unique_ptr<PointParameterBlock> pointBlock_;

		// optimization statistics
		float iDepthHessian_;				// sum of all the inverse depth hessians
		float parallax_;					// maximum parallax within the observations
	};
}