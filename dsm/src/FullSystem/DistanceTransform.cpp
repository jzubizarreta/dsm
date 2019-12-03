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

#include "DistanceTransform.h"
#include "DataStructures/Frame.h"
#include "DataStructures/ActivePoint.h"
#include "Utils/GlobalCalibration.h"

#include "Utils/UtilFunctions.h"

namespace dsm
{
	DistanceTransform::DistanceTransform(int width, int height) : w(width), h(height), numObstacles(0)
	{
		// init
		this->requireUpdate = (bool*)Eigen::internal::aligned_malloc(this->w * this->h * sizeof(bool));
		this->obstacles = (Eigen::Vector2i*)Eigen::internal::aligned_malloc(this->w * this->h * sizeof(Eigen::Vector2i));
		this->distMap = (float*)Eigen::internal::aligned_malloc(this->w * this->h * sizeof(float));
	}

	DistanceTransform::~DistanceTransform()
	{
		Eigen::internal::aligned_free(this->requireUpdate);
		Eigen::internal::aligned_free(this->obstacles);
		Eigen::internal::aligned_free(this->distMap);
	}

	cv::Mat DistanceTransform::drawDistanceTransform(bool normalize)
	{
		cv::Mat distTransform(this->h, this->w, CV_32FC1);
		std::copy(this->distMap, this->distMap + this->w*this->h, (float*)distTransform.data);

		if (normalize)
		{
			distTransform = distTransform + 1;		// set range to [1, Inf] to have all positive values
			cv::log(distTransform, distTransform);	// log scale
			cv::normalize(distTransform, distTransform, 0, 1.0, cv::NORM_MINMAX);		// normalize gray scale
		}

		return distTransform;
	}

	float DistanceTransform::dist(int x, int y) const
	{
		return this->distMap[x + (y * this->w)];
	}

	int DistanceTransform::getNumObstacles() const
	{
		return this->numObstacles;
	}

	void DistanceTransform::compute(const std::vector<std::shared_ptr<Frame>>& activeKeyframes,
									const std::shared_ptr<Frame>& frame)
	{
		// reset distance map
		std::fill(this->distMap, this->distMap + this->w * this->h, std::numeric_limits<float>::max());
		this->numObstacles = 0;

		const int keyframeID = frame->keyframeID();

		// generate point in frame and put dist = 0 in distance map
		for (int i = 0; i < activeKeyframes.size(); ++i)
		{
			if (activeKeyframes[i] == frame) continue;

			// go through all active points
			const auto& activePoints = activeKeyframes[i]->activePoints();
			for (const auto& point : activePoints)
			{
				// check visibility
				if (point->visibility(keyframeID) != Visibility::VISIBLE) continue;

				const Eigen::Vector2f pointInFrame = point->centerProjection();

				int x = static_cast<int>(pointInFrame[0] + 0.5f);
				int y = static_cast<int>(pointInFrame[1] + 0.5f);

				this->obstacles[this->numObstacles] = Eigen::Vector2i(x, y);
				this->requireUpdate[this->numObstacles] = true;
				this->distMap[x + (y * this->w)] = 0;
				++this->numObstacles;
			}
		}

		// go throught all obstacles propagating the distance outwards
		int radius = 1;
		bool updated = true;

		while (updated)
		{
			updated = false;

			for (int i = 0; i < this->numObstacles; ++i)
			{
				if (!this->requireUpdate[i]) continue;

				int x = this->obstacles[i].x();
				int y = this->obstacles[i].y();

				if (this->radiusDist(x, y, radius))
				{
					updated = true;
				}
				else
				{
					this->requireUpdate[i] = false;
				}
			}

			radius++;
		}
	}

	void DistanceTransform::add(const std::vector<std::shared_ptr<Frame>>& newAddedKeyframes, const std::shared_ptr<Frame>& frame)
	{
		const int prevNumObstacles = this->numObstacles;

		const int keyframeID = frame->keyframeID();

		// generate point in frame and put dist = 0 in distance map
		for (int i = 0; i < newAddedKeyframes.size(); ++i)
		{
			if (newAddedKeyframes[i] == frame) continue;

			// go through all active points
			const auto& activePoints = newAddedKeyframes[i]->activePoints();
			for (const auto& point : activePoints)
			{
				// check visibility
				if (point->visibility(keyframeID) != Visibility::VISIBLE) continue;

				const Eigen::Vector2f pointInFrame = point->centerProjection();

				int x = static_cast<int>(pointInFrame[0] + 0.5f);
				int y = static_cast<int>(pointInFrame[1] + 0.5f);

				this->obstacles[this->numObstacles] = Eigen::Vector2i(x, y);
				this->requireUpdate[this->numObstacles] = true;
				this->distMap[x + (y * this->w)] = 0;
				++this->numObstacles;
			}
		}

		// go throught all obstacles propagating the distance outwards
		int radius = 1;
		bool updated = true;

		while (updated)
		{
			updated = false;

			for (int i = prevNumObstacles; i < this->numObstacles; ++i)
			{
				if (!this->requireUpdate[i]) continue;

				int x = this->obstacles[i].x();
				int y = this->obstacles[i].y();

				if (this->radiusDist(x, y, radius))
				{
					updated = true;
				}
				else
				{
					this->requireUpdate[i] = false;
				}
			}

			radius++;
		}
	}

	void DistanceTransform::add(const std::shared_ptr<Frame>& newAddedKeyframe, const std::shared_ptr<Frame>& frame)
	{
		const int prevNumObstacles = this->numObstacles;

		const int keyframeID = frame->keyframeID();

		// generate point in frame and put dist = 0 in distance map
		if (newAddedKeyframe == frame) return;

		// go through all active points
		const auto& activePoints = newAddedKeyframe->activePoints();
		for (const auto& point : activePoints)
		{
			// check visibility
			if (point->visibility(keyframeID) != Visibility::VISIBLE) continue;

			const Eigen::Vector2f pointInFrame = point->centerProjection();

			int x = static_cast<int>(pointInFrame[0] + 0.5f);
			int y = static_cast<int>(pointInFrame[1] + 0.5f);

			this->obstacles[this->numObstacles] = Eigen::Vector2i(x, y);
			this->requireUpdate[this->numObstacles] = true;
			this->distMap[x + (y * this->w)] = 0;
			++this->numObstacles;
		}

		// go throught all obstacles propagating the distance outwards
		int radius = 1;
		bool updated = true;

		while (updated)
		{
			updated = false;

			for (int i = prevNumObstacles; i < this->numObstacles; ++i)
			{
				if (!this->requireUpdate[i]) continue;

				int x = this->obstacles[i].x();
				int y = this->obstacles[i].y();

				if (this->radiusDist(x, y, radius))
				{
					updated = true;
				}
				else
				{
					this->requireUpdate[i] = false;
				}
			}

			radius++;
		}
	}

	void DistanceTransform::add(const std::unique_ptr<ActivePoint>& activePoint, const std::shared_ptr<Frame>& frame)
	{
		if (activePoint->visibility(frame->keyframeID()) != Visibility::VISIBLE) return;

		// set obstacle
		Eigen::Vector2f pointInFrame = activePoint->centerProjection();

		int x = static_cast<int>(pointInFrame[0] + 0.5f);
		int y = static_cast<int>(pointInFrame[1] + 0.5f);

		int idx = x + (y * this->w);

		this->obstacles[this->numObstacles] = Eigen::Vector2i(x, y);
		this->requireUpdate[this->numObstacles] = true;
		this->distMap[idx] = 0;
		++this->numObstacles;

		// iterate in the new obstacle only
		int radius = 1;
		bool updated = true;

		while (updated)
		{
			updated = this->radiusDist(x, y, radius);
			this->requireUpdate[this->numObstacles - 1] = updated;
			
			radius++;
		}
	}

	bool DistanceTransform::radiusDist(int x, int y, int radius)
	{
		bool updated = false;

		int u, v;

		// row -radius
		v = y - radius;
		if (v >= 0 && v < this->h)		// check that it is a valid row
		{
			// compute ranges
			int min_u = std::max(x - radius, 0);
			int max_u = std::min(x + radius, this->w - 1);	

			for (u = min_u; u <= max_u; ++u)
			{
				float distX = static_cast<float>(u - x);
				float distY = static_cast<float>(v - y);

				//float dist = std::sqrtf(distX * distX + distY * distY);
				float dist = distX * distX + distY * distY;

				int idx = u + (v * this->w);

				if (this->distMap[idx] > dist)
				{
					this->distMap[idx] = dist;
					updated = true;
				}
			}
		}

		// row +radius
		v = y + radius;
		if (v >= 0 && v < this->h)		// check that it is a valid row
		{
			// compute ranges
			int min_u = std::max(x - radius, 0);
			int max_u = std::min(x + radius, this->w - 1);

			for (u = min_u; u <= max_u; ++u)
			{
				float distX = static_cast<float>(u - x);
				float distY = static_cast<float>(v - y);

				//float dist = std::sqrtf(distX * distX + distY * distY);
				float dist = distX * distX + distY * distY;

				int idx = u + (v * this->w);

				if (this->distMap[idx] > dist)
				{
					this->distMap[idx] = dist;
					updated = true;
				}
			}
		}

		// col -radius
		u = x - radius;
		if (u >= 0 && u < this->w)		// check that it is a valid col
		{
			// compute ranges
			int min_v = std::max(y - radius + 1, 0);
			int max_v = std::min(y + radius - 1, this->h - 1);

			for (v = min_v; v <= max_v; ++v)
			{
				float distX = static_cast<float>(u - x);
				float distY = static_cast<float>(v - y);

				//float dist = std::sqrtf(distX * distX + distY * distY);
				float dist = distX * distX + distY * distY;

				int idx = u + (v * this->w);

				if (this->distMap[idx] > dist)
				{
					this->distMap[idx] = dist;
					updated = true;
				}
			}
		}

		// col +radius
		u = x + radius;
		if (u >= 0 && u < this->w)		// check that it is a valid col
		{
			// compute ranges
			int min_v = std::max(y - radius + 1, 0);
			int max_v = std::min(y + radius - 1, this->h - 1);

			for (v = min_v; v <= max_v; ++v)
			{
				float distX = static_cast<float>(u - x);
				float distY = static_cast<float>(v - y);

				//float dist = std::sqrtf(distX * distX + distY * distY);
				float dist = distX * distX + distY * distY;

				int idx = u + (v * this->w);

				if (this->distMap[idx] > dist)
				{
					this->distMap[idx] = dist;
					updated = true;
				}
			}
		}

		return updated;
	}
}