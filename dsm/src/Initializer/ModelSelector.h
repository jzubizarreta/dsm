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

#include <vector>
#include <memory>

#include "opencv2/core.hpp"

namespace dsm
{
	class WorkerThreadPool;

	// Selector of geometry model from correspondences
	class ModelSelector
	{
	public:

		ModelSelector(const std::shared_ptr<WorkerThreadPool>& parallelizer = nullptr);
		~ModelSelector();

		bool select(const cv::Mat& K, 
					const std::vector<cv::Point2f> &corr1, 
					const std::vector<cv::Point2f> &corr2,
					cv::Mat& pose);

	private:

		// pose from homography
		bool motionFromHomography(const cv::Mat& H, const cv::Mat& K,
								  const std::vector<cv::Point2f> &corr1, 
								  const std::vector<cv::Point2f> &corr2,
								  const std::vector<unsigned char>& inliers,
								  cv::Mat& motion) const;

		// pose from essential matrix
		bool motionFromEssential(const cv::Mat& E, const cv::Mat& K,
								 const std::vector<cv::Point2f> &corr1,
								 const std::vector<cv::Point2f> &corr2,
								 const std::vector<unsigned char>& inliers,
								 cv::Mat& motion) const;

		// selects best rotation and translation combination
		bool selectBestCombination(const cv::Mat& K, 
								   const std::vector<cv::Mat>& R_cmbs,
								   const std::vector<cv::Mat>& t_cmbs,
								   const std::vector<cv::Point2f> &corr1,
								   const std::vector<cv::Point2f> &corr2,
								   const std::vector<unsigned char>& inliers,
								   cv::Mat& motion) const;

	private:

		// workers for parallel processing
		const std::shared_ptr<WorkerThreadPool> worker;

		// thresholds
		const float heRatio = 0.8f;
		const float parallaxTh = 0.99984f;		// 1 degrees
		int minPoints;

		// Homography model estimation 
		struct HomographyModel
		{
			cv::Mat& H;								// homography
			std::vector<unsigned char>& inliers;	// inlier mask
			const std::vector<cv::Point2f> &c1;		// points in image 1
			const std::vector<cv::Point2f> &c2;		// points in image 2

			HomographyModel(const std::vector<cv::Point2f> &corr1,
							const std::vector<cv::Point2f> &corr2,
							std::vector<unsigned char>& mask,
							cv::Mat& homography);

			void operator()();
		};

		// Essential model estimation 
		struct EssentialModel
		{
			cv::Mat& E;								// homography
			std::vector<unsigned char>& inliers;	// inlier mask
			const std::vector<cv::Point2f> &c1;		// points in image 1
			const std::vector<cv::Point2f> &c2;		// points in image 2
			const cv::Mat& K;						// camera calibration

			EssentialModel(const std::vector<cv::Point2f> &corr1,
						   const std::vector<cv::Point2f> &corr2,
						   const cv::Mat& calib,
						   std::vector<unsigned char>& mask,
						   cv::Mat& essential);

			void operator()();
		};

		// R and t combination check
		struct CombinationCheck
		{
			const cv::Mat& K;
			const cv::Mat& R;
			const cv::Mat& t;
			const std::vector<cv::Point2f>& c1;
			const std::vector<cv::Point2f>& c2;
			const std::vector<unsigned char>& inliers;
			int& numGood;
			float& parallax;

			CombinationCheck(const cv::Mat& calib,
							 const cv::Mat& rot,
							 const cv::Mat& trans,
							 const std::vector<cv::Point2f>& corr1,
							 const std::vector<cv::Point2f>& corr2,
							 const std::vector<unsigned char>& mask,
							 int& numberGood, float& paral);

			void operator()();

		private:

			// check cheirality and parallax
			bool checkCheiralityAndParallax(const cv::Point2f& pt2d1, const cv::Point2f& pt2d2,
											const cv::Mat &P1, const cv::Mat &O1,
											const cv::Mat &P2, const cv::Mat &O2,
											float maxRepoError, float &parallax) const;

			// triangulate
			void triangulate(const cv::Point2f& u1, const cv::Point2f& u2,
							 const cv::Mat &P1, const cv::Mat &P2,
							 cv::Mat &pt3d) const;

		private:

			const float repoTh = 4.f;
			const float parlPerc = 0.25f;
		};
	};
}