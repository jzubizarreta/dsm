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

#include <Eigen/Core>

#include <memory>
#include <mutex>

#include "Visualizer/IVisualizer.h"

namespace dsm
{
	class Frame;

	struct KeyframePoint
	{
		float u, v;								// pixel location
		float iDepth;							// inverse depth
		float iDepthHessian;					// opt hessian
		float parallax;							// triangulation parallax
		std::vector<unsigned char> color;		// pattern color
	};

	// Keyframe visualizer class. It can render keyframe pose and its
	// associated point cloud
	// It is not multi-thread safe
	class KeyframeVisualizer
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		// constructor from slam frame
		KeyframeVisualizer();

		// disable copy constuctors
		KeyframeVisualizer(const KeyframeVisualizer&) = delete;
		KeyframeVisualizer& operator=(const KeyframeVisualizer&) = delete;

		// destructor
		~KeyframeVisualizer();

		// builds keyframe local pointcloud and copies pose and type
		void compute(const std::shared_ptr<dsm::Frame>& keyframe, KeyframeType keyframeType);
		void computeType(KeyframeType keyframeType);

		// updates rendering buffers
		bool update(float threshold, float parallaxTreshold);

		// display functions
		void drawCamera(float size, bool useType);
		void drawPointCloud(float size, bool useType);		// call update() first!

		// accessors
		const Eigen::Matrix4f& getPose() const;
		const std::vector<Eigen::Vector3f>& getPointCloud() const;
		const std::vector<Eigen::Matrix<unsigned char, 3, 1>>& getColors() const;
		KeyframeType getType() const;
		int getNumPoints() const;

	private:
		 
		// copied data set by compute()
		std::vector<KeyframePoint> pointCloud;

		// rendering data set by compute() and update()
		KeyframeType type;
		Eigen::Matrix4f camToWorld;
		std::vector<Eigen::Vector3f> vertexBuffer;
		std::vector<Eigen::Matrix<unsigned char, 3, 1>> colorBuffer;
		int numBufferPoints;
		float varThr;
		float parThr;

		// flag to control if requires to be updated
		bool needUpdate;
	};
}