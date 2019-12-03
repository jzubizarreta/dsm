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

#include "Utils/EigenTypes.h"

#include "sophus/se3.hpp"

#include "opencv2/core.hpp"

#include <memory>

namespace dsm
{
	class Frame;
	class ModelSelector;
	class IVisualizer;
	class WorkerThreadPool;

	// Class to initialize DSM from a sequence of images
	class MonoInitializer
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		MonoInitializer(const std::shared_ptr<WorkerThreadPool>& parallelizer = nullptr, 
						IVisualizer* outWrapper = nullptr);
		~MonoInitializer();

		// flag for reset 
		bool isResetRequired() const;
		void reset();

		// initial frame as reference
		void setReference(const std::shared_ptr<Frame>& firstFrame);

		// sequetial frames until convergence
		bool initialize(const std::shared_ptr<Frame>& frame, Sophus::SE3f& pose);

	private:

		void trackPointsInImage(const cv::Mat& refImage, 
								const cv::Mat& newImage, 
								std::vector<cv::Point2f>& refPoints, 
								std::vector<cv::Point2f>& newPoints) const;

	private:

		// flag for reset
		bool resetRequired;

		// reference data
		std::shared_ptr<Frame> reference;
		std::vector<cv::Point2f> refPoints;
		cv::Mat refImage;

		// tracked points position
		std::vector<cv::Point2f> prevPoints;

		// motion model selector
		std::unique_ptr<ModelSelector> modelSelector;

		// calibration
		cv::Mat K;

		// parameters
		int minNumTracked;
		const int maxL1Error = 7;

		// visualization
		IVisualizer* outputWrapper;
	};
}