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

#include "MonoInitializer.h"
#include "ModelSelector.h"
#include "DataStructures/Frame.h"
#include "DataStructures/CandidatePoint.h"
#include "Utils/GlobalCalibration.h"
#include "Utils/UtilFunctions.h"
#include "Visualizer/IVisualizer.h"

#include "opencv2/opencv.hpp"
#include "opencv2/video.hpp"

#include <iostream>

namespace dsm
{
	MonoInitializer::MonoInitializer(const std::shared_ptr<WorkerThreadPool>& parallelizer,
									 IVisualizer* outWrapper) :
		outputWrapper(outWrapper),
		resetRequired(false),
		minNumTracked(0)
	{
		this->modelSelector = std::make_unique<ModelSelector>(parallelizer);

		const auto& calib = GlobalCalibration::getInstance();

		this->K = cv::Mat::eye(3, 3, CV_32F);
		this->K.at<float>(0, 0) = calib.matrix3f(0)(0, 0);
		this->K.at<float>(0, 2) = calib.matrix3f(0)(0, 2);
		this->K.at<float>(1, 1) = calib.matrix3f(0)(1, 1);
		this->K.at<float>(1, 2) = calib.matrix3f(0)(1, 2);
	}

	MonoInitializer::~MonoInitializer()
	{}

	bool MonoInitializer::isResetRequired() const
	{
		return this->resetRequired;
	}

	void MonoInitializer::reset()
	{
		this->reference = nullptr;
		this->refPoints.clear();
		this->prevPoints.clear();

		this->minNumTracked = 0;

		this->resetRequired = false;
	}

	void MonoInitializer::setReference(const std::shared_ptr<Frame>& firstFrame)
	{
		this->reset();

		const auto& calib = GlobalCalibration::getInstance();

		// set first frame as reference
		this->reference = firstFrame;

		// extract reference points from candidates
		const auto& candidates = this->reference->candidates();
		this->refPoints.resize(candidates.size());

		for (int i = 0; i < candidates.size(); ++i)
		{
			const auto& cand = candidates[i];
			this->refPoints[i] = cv::Point2f(cand->u0(), cand->v0());
		}

		// at least 50% of points tracked. otherwise reset
		this->minNumTracked = static_cast<int>(0.5f*candidates.size());

		// reference image
		this->refImage = cv::Mat(calib.height(0), calib.width(0), CV_32F, (void*)this->reference->image(0));
		this->refImage.convertTo(this->refImage, CV_8U);	// convert to 8-bit
	}

	bool MonoInitializer::initialize(const std::shared_ptr<Frame>& frame, Sophus::SE3f& pose)
	{
		Utils::Time t1 = std::chrono::steady_clock::now();

		bool initOk = false;

		if (this->resetRequired)
		{
			std::cout << "MonoInitializer::initialize requires to RESET!";
			return false;
		}

		if (!this->reference)
		{
			std::cout << "MonoInitializer::initialize requires a reference keyframe!\n";
		}

		const auto& calib = GlobalCalibration::getInstance();

		// new image
		cv::Mat newImg(calib.height(0), calib.width(0), CV_32F, (void*)frame->image(0));
		newImg.convertTo(newImg, CV_8U);

		// track candidates in image
		this->trackPointsInImage(this->refImage, newImg, this->refPoints, this->prevPoints);

		if (this->refPoints.size() < this->minNumTracked)
		{
			std::cout << "MonoInitializer::initialize not enough tracked points! RESET\n";
			this->resetRequired = true;
			return false;
		}

		// select motion model
		cv::Mat motion;
		if (modelSelector->select(this->K, this->refPoints, this->prevPoints, motion))
		{
			initOk = true;

			// from cv::Mat to Sophus::SE3f
			Eigen::Matrix4f eigenPose;
			eigenPose(0, 0) = motion.at<float>(0, 0); eigenPose(0, 1) = motion.at<float>(0, 1); eigenPose(0, 2) = motion.at<float>(0, 2); eigenPose(0, 3) = motion.at<float>(0, 3);
			eigenPose(1, 0) = motion.at<float>(1, 0); eigenPose(1, 1) = motion.at<float>(1, 1); eigenPose(1, 2) = motion.at<float>(1, 2); eigenPose(1, 3) = motion.at<float>(1, 3);
			eigenPose(2, 0) = motion.at<float>(2, 0); eigenPose(2, 1) = motion.at<float>(2, 1); eigenPose(2, 2) = motion.at<float>(2, 2); eigenPose(2, 3) = motion.at<float>(2, 3);
			eigenPose(3, 0) = motion.at<float>(3, 0); eigenPose(3, 1) = motion.at<float>(3, 1); eigenPose(3, 2) = motion.at<float>(3, 2); eigenPose(3, 3) = motion.at<float>(3, 3);

			pose = Sophus::SE3f(eigenPose);
		}

		Utils::Time t2 = std::chrono::steady_clock::now();
		//std::cout << "MonoInitializer in " << Utils::elapsedTime(t1, t2) << "ms" << std::endl;

		// draw
		if (this->outputWrapper)
		{
			cv::cvtColor(newImg, newImg, cv::COLOR_GRAY2BGR);
			for (int i = 0; i < this->refPoints.size(); ++i)
			{
				cv::Point p1((int)this->refPoints[i].x, (int)this->refPoints[i].y);
				cv::Point p2((int)this->prevPoints[i].x, (int)this->prevPoints[i].y);
				cv::line(newImg, p1, p2, cv::Scalar(0, 255, 0), 1);
			}

			//if (initOk)
			//{
			//	cv::imwrite("initRefImg.png", this->refImage);
			//	cv::imwrite("initOpticalFlow.png", newImg);
			//}

			this->outputWrapper->publishProcessFrame(newImg);
		}

		return initOk;
	}

	void MonoInitializer::trackPointsInImage(const cv::Mat& refImg,
											 const cv::Mat& newImg,
											 std::vector<cv::Point2f>& refPt,
											 std::vector<cv::Point2f>& newPt) const
	{
		std::vector<unsigned char> status;
		std::vector<float> err;
		int flowFlag = (newPt.empty() ? 0 : cv::OPTFLOW_USE_INITIAL_FLOW);

		cv::calcOpticalFlowPyrLK(refImg, newImg, refPt, newPt, status, err, cv::Size(21, 21), 2,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), flowFlag, 0.001);

		// remove non matched features and outliers
		std::vector<cv::Point2f> refPointsClean, newPointsClean;
		refPointsClean.reserve(this->refPoints.size());
		newPointsClean.reserve(this->refPoints.size());
		for (int i = 0; i < this->refPoints.size(); ++i)
		{
			if (status[i] && err[i] < this->maxL1Error)
			{
				refPointsClean.push_back(refPt[i]);
				newPointsClean.push_back(newPt[i]);
			}
		}
		std::swap(refPt, refPointsClean);
		std::swap(newPt, newPointsClean);
	}
}