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

#include <vector>
#include <memory>

#include "opencv2/core.hpp"

namespace dsm
{
	class Frame;
	class ActivePoint;

	// Distance Map of pixels in an image of
	// active points projected to the image
	// It uses the squared Euclidean distance L2
	class DistanceTransform
	{
	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		DistanceTransform(int width, int height);
		~DistanceTransform();

		// squared distance of the pixel u, v in the map
		float dist(int x, int y) const;

		int getNumObstacles() const;

		// distance map computation for ActivePoints
		// compute distance until maxDist is reached
		void compute(const std::vector<std::shared_ptr<Frame>>& activeKeyframes,
					 const std::shared_ptr<Frame>& frame);

		// adds new pixel to map and updates distance map until maxDist is reached
		void add(const std::vector<std::shared_ptr<Frame>>& newAddedKeyframes, const std::shared_ptr<Frame>& frame);
		void add(const std::shared_ptr<Frame>& newAddedKeyframe, const std::shared_ptr<Frame>& frame);
		void add(const std::unique_ptr<ActivePoint>& activePoint, const std::shared_ptr<Frame>& frame);

		// draws the map
		cv::Mat drawDistanceTransform(bool normalize);

	private:

		// calculate distance for all pixel in the radius
		bool radiusDist(int x, int y, int radius);

	private:

		const int w, h;					// image size

		float* distMap;					// distance map
		Eigen::Vector2i* obstacles;		// obstacle in the map with distance = 0
		bool* requireUpdate;			// flag to reduce computation
		int numObstacles;				// number of obstacles in the map
	};
}