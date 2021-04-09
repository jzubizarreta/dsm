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

#include "FullSystem/DSMLib.h"

#include <vector>
#include <array>
#include <deque>
#include <unordered_map>
#include <set>
#include <memory>

#include "opencv2/core.hpp"

#include <Eigen/Core>

#include "sophus/se3.hpp"

namespace dsm
{
	class Frame;

	enum KeyframeType { NONE = 0, TEMPORAL = 1, COVISIBILITY = 2, FIXED = 3 };

	class DSM_EXPORTS_DLL IVisualizer
	{
	public:

		IVisualizer();
		virtual ~IVisualizer() = 0;

		virtual void run() = 0;

		virtual void reset() = 0;

		virtual void close() = 0;

		virtual void publishCamTrackingTime(float time) = 0;
		virtual void resetCamTrackingTime() = 0;

		virtual void publishPointTrackingTime(float time) = 0;
		virtual void resetPointTrackingTime() = 0;

		virtual void publishLocalBATime(float time) = 0;
		virtual void resetLocalBATime() = 0;

		virtual void publishLiveFrame(const cv::Mat &liveFrame) = 0;
		virtual void resetLiveFrame() = 0;

		virtual void publishProcessFrame(const cv::Mat &processFrame) = 0;
		virtual void resetProcessFrame() = 0;

		virtual void publishCurrentFrame(const Eigen::Matrix4f &framePose) = 0;
		virtual void resetCurrentFrame() = 0;

		virtual void publishKeyframe(const std::shared_ptr<Frame>& keyframe, KeyframeType type) = 0;
		virtual void publishKeyframeType(const std::shared_ptr<Frame>& keyframe, KeyframeType type) = 0;
		virtual void resetKeyframes() = 0;
		virtual void resetKeyframeTypes() = 0;

		virtual void publishCovisibility(const Eigen::MatrixXi& adjacencyMatrix) = 0;
		virtual void resetCovisibility() = 0;

		virtual void publishPointDetector(const cv::Mat& img) = 0;
		virtual void publishDistanceTransformBefore(const cv::Mat& img) = 0;
		virtual void publishDistanceTransformAfter(const cv::Mat& img) = 0;
		virtual void publishTrackingResult(const cv::Mat& img) = 0;
		virtual void publishTrackingError(const cv::Mat& img) = 0;
		virtual void publishTrackingWeight(const cv::Mat& img) = 0;
		virtual void publishTrackingLight(const cv::Mat& img) = 0;
		virtual void publishTrackingDistribution(const cv::Mat& img) = 0;
		virtual void publishOptKeyframes(const cv::Mat& img) = 0;
		virtual void publishOptError(const cv::Mat& img) = 0;
		virtual void publishOptErrorDist(const cv::Mat& img) = 0;
		virtual void publishOptErrorDistLast(const cv::Mat& img) = 0;
		virtual void publishOptRawError(const cv::Mat& img) = 0;
		virtual void publishOptWeight(const cv::Mat& img) = 0;
		virtual void publishOptLight(const cv::Mat& img) = 0;
		virtual void resetDebugWindows() = 0;
	};
}

