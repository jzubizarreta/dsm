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
#include <mutex>

#include <Eigen/Core>

namespace dsm
{
	class Frame;

	// This class is a point cloud representation of active point
	// It is referenced to last keyframe coordinate system
	// It is used during image alignment for camera tracking 
	class FrameTrackerReference
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		// constructor: width and height are the dimensions of the reference keyframe in the lower level
		FrameTrackerReference(const int32_t width, const int32_t height);

		// Destructor
		~FrameTrackerReference();

		// resets all the internal data but does not release memory
		void reset();

		// sets a new reference Keyframe and creates inverse depth pyramids
		void setNewReference(const std::vector<std::shared_ptr<Frame>>& activeKeyframes);

		// computes the reference point cloud from the inverse depth maps and reference keyframe
		bool generateReferencePointCloud(const int32_t level);

		// reference
		Frame* reference() const;

		// depth map
		const float* iDepthMap(const int32_t level) const;

		// mean inverse depth
		float meanIDepth() const;

		// point cloud
		const float* u(const int32_t level) const;
		const float* v(const int32_t level) const;
		const float* x(const int32_t level) const;
		const float* y(const int32_t level) const;
		const float* iDepth(const int32_t level) const;
		const float* gx(const int32_t level) const;
		const float* gy(const int32_t level) const;
		const float* color(const int32_t level) const;
		const float* weight(const int32_t level) const;
		int numPoints(const int32_t level) const;

		// first tracking residual
		void setFirstFrameResidual(float val);
		float firstFrameResidual() const;

	private:

		// reference in which all points are generated
		Frame* reference_;

		// depth map for point cloud generation
		std::vector<float*> iDepthMap_;
		std::vector<bool*> validMap_;

		// iDepth map duplication for smoothing
		float* iDepthMapNonSmooth_;

		// mean inverse depth for keyframe selection
		float meanIDepth_;

		// point cloud data
		// aligned to work with SSE
		std::vector<float*> u_;				// pixel u
		std::vector<float*> v_;				// pixel v
		std::vector<float*> x_;				// unprojected x
		std::vector<float*> y_;				// unprojected y
		std::vector<float*> iDepth_;		// inverse depth
		std::vector<float*> gx_;			// image gx		
		std::vector<float*> gy_;			// image gy
		std::vector<float*> color_;			// point image color
		std::vector<float*> weight_;		// point weight based on gradient magnitude
		std::vector<int> numPoints_;		// number of points per level		

		// first tracking residual towards this reference
		float firstFrameResidual_;

		// mutex
		std::mutex accessMutex_;
	};
}