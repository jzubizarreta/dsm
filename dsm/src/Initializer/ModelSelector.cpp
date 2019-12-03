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

#include "ModelSelector.h"
#include "Thread/WorkerThreadPool.h"

#include <iostream>

#include "opencv2/calib3d.hpp"

namespace dsm
{
	ModelSelector::HomographyModel::HomographyModel(const std::vector<cv::Point2f> &corr1,
													const std::vector<cv::Point2f> &corr2, 
													std::vector<unsigned char>& mask,
													cv::Mat& homography) :
		c1(corr1), c2(corr2), inliers(mask), H(homography)
	{}

	void ModelSelector::HomographyModel::operator()()
	{
		// find homography using RANSAC
		this->H = cv::findHomography(this->c1, this->c2, cv::RANSAC, 2.0, this->inliers);
	}

	ModelSelector::EssentialModel::EssentialModel(const std::vector<cv::Point2f> &corr1,
												  const std::vector<cv::Point2f> &corr2,
												  const cv::Mat& calib,
												  std::vector<unsigned char>& mask,
												  cv::Mat& essential) :
		c1(corr1), c2(corr2), K(calib), inliers(mask), E(essential)
	{}

	void ModelSelector::EssentialModel::operator()()
	{
		// find essential matrix using RANSAC
		this->E = cv::findEssentialMat(this->c1, this->c2, this->K, cv::RANSAC, 0.999, 1.0, this->inliers);
	}

	ModelSelector::ModelSelector(const std::shared_ptr<WorkerThreadPool>& parallelizer) :
		worker(parallelizer),
		minPoints(std::numeric_limits<int>::max())
	{}

	ModelSelector::~ModelSelector()
	{}

	bool ModelSelector::select(const cv::Mat& K,
							   const std::vector<cv::Point2f> &corr1,
							   const std::vector<cv::Point2f> &corr2,
							   cv::Mat& motion)
	{
		// check the number of correspondences
		// at least 8 to be able to estimate something
		if (corr1.size() < 8 ||corr1.size() != corr2.size())
		{
			std::cout << "ModelSelector requires valid input correspondences\n";
			return false;
		}

		// estimate both models
		std::vector<unsigned char> homoInliers;
		std::vector<unsigned char> esseInliers;
		cv::Mat H, E;

		{
			HomographyModel homoModel(corr1, corr2, homoInliers, H);
			EssentialModel essenModel(corr1, corr2, K, esseInliers, E);

			if (this->worker)
			{
				// add jobs to be processed in parallel
				this->worker->addJob(homoModel);
				this->worker->addJob(essenModel);

				// wait until finish
				this->worker->wait();
			}
			else
			{
				// do not parallelize
				homoModel();
				essenModel();
			}
		}
		
		// count number of inliers
		const int numHomoInliers = cv::countNonZero(homoInliers);
		const int numEsseInliers = cv::countNonZero(esseInliers);

		// ratio between models to select
		const float ratio = static_cast<float>(numHomoInliers) / static_cast<float>(numEsseInliers);

		// obtain the geometry and check based on the ratio
		if (ratio > this->heRatio)
		{
			this->minPoints = static_cast<int>(0.9*numHomoInliers);

			// planar
			bool found = this->motionFromHomography(H, K, corr1, corr2, homoInliers, motion);

			if (found) std::cout << "Planar Initialization!" << std::endl;

			return found;
		}
		else
		{
			this->minPoints = static_cast<int>(0.9*numEsseInliers);

			// non planar
			bool found = this->motionFromEssential(E, K, corr1, corr2, esseInliers, motion);

			if (found) std::cout << "Non-Planar Initialization!" << std::endl;

			return found;
		}

		return false;
	}

	bool ModelSelector::motionFromHomography(const cv::Mat& H, const cv::Mat& K,
											 const std::vector<cv::Point2f> &corr1,
											 const std::vector<cv::Point2f> &corr2,
											 const std::vector<unsigned char>& inliers,
											 cv::Mat& motion) const
	{
		// This function extracts relative camera motion between two views 
		// observing a planar object from the homography H induced by the plane.
		// The function may return up to four mathematical solution sets.
		std::vector<cv::Mat> R_cmbs;
		std::vector<cv::Mat> t_cmbs;
		std::vector<cv::Mat> n_cmbs;
		cv::decomposeHomographyMat(H, K, R_cmbs, t_cmbs, n_cmbs);

		// convert to float
		for (int i = 0; i < R_cmbs.size(); ++i)
		{
			R_cmbs[i].convertTo(R_cmbs[i], CV_32F);
			t_cmbs[i].convertTo(t_cmbs[i], CV_32F);
		}

		// check all possible solutions and select the best one
		return this->selectBestCombination(K, R_cmbs, t_cmbs, corr1, corr2, inliers, motion);
	}

	bool ModelSelector::motionFromEssential(const cv::Mat& E, const cv::Mat& K,
											const std::vector<cv::Point2f> &corr1,
											const std::vector<cv::Point2f> &corr2,
											const std::vector<unsigned char>& inliers,
											cv::Mat& motion) const
	{
		// This function decompose an essential matrix E using svd decomposition [HartleyZ00].
		// Generally 4 possible poses exists for a given E.They are[R_1, t], [R_1, -t], [R_2, t], [R_2, -t].
		// By decomposing E, you can only get the direction of the translation, so the function returns unit t.
		cv::Mat Ri, Rj, ti;
		cv::decomposeEssentialMat(E, Ri, Rj, ti);

		// convert to float
		Ri.convertTo(Ri, CV_32F);
		Rj.convertTo(Rj, CV_32F);
		ti.convertTo(ti, CV_32F);

		// Generate all possible projection matrix combinations.
		std::vector<cv::Mat> R_cmbs{ Ri, Rj, Ri, Rj };
		std::vector<cv::Mat> t_cmbs{ ti, ti, -ti, -ti };

		// check all possible solutions and select the best one
		return this->selectBestCombination(K, R_cmbs, t_cmbs, corr1, corr2, inliers, motion);
	}

	bool ModelSelector::selectBestCombination(const cv::Mat& K,
											  const std::vector<cv::Mat>& R_cmbs,
											  const std::vector<cv::Mat>& t_cmbs,
											  const std::vector<cv::Point2f> &corr1,
											  const std::vector<cv::Point2f> &corr2,
											  const std::vector<unsigned char>& inliers,
											  cv::Mat& motion) const
	{
		// select the best combination of rotation and translation
		std::vector<int> numGood(R_cmbs.size(), 0);
		std::vector<float> parallax(R_cmbs.size(), 1.f);

		for (int i = 0; i < R_cmbs.size(); ++i)
		{
			CombinationCheck combCheck(K, R_cmbs[i], t_cmbs[i], corr1, corr2, inliers, numGood[i], parallax[i]);

			if (this->worker)
			{
				this->worker->addJob(combCheck);
			}
			else
			{
				combCheck();
			}
		}

		// wait if parallelized
		if (this->worker)
		{
			this->worker->wait();
		}

		// select winner
		int bestGood = 0;
		int secondBestGood = 0;
		int bestGoodIdx = -1;		

		for (int i = 0; i < numGood.size(); ++i)
		{
			if (numGood[i] > bestGood)
			{
				secondBestGood = bestGood;
				bestGood = numGood[i];
				bestGoodIdx = i;
			}
			else if (numGood[i] > secondBestGood)
			{
				secondBestGood = numGood[i];
			}
		}

		// check that we have enough good points and a clear winner
		if (bestGoodIdx < 0 || 
			bestGood < this->minPoints || 
			bestGood*0.7f < secondBestGood || 
			parallax[bestGoodIdx] > this->parallaxTh)
		{
			return false;
		}

		// save best motion
		motion = cv::Mat::zeros(4, 4, CV_32F);
		R_cmbs[bestGoodIdx].copyTo(motion.rowRange(0, 3).colRange(0, 3));
		t_cmbs[bestGoodIdx].copyTo(motion.rowRange(0, 3).col(3));

		return true;
	}

	ModelSelector::CombinationCheck::CombinationCheck(const cv::Mat& calib,
													  const cv::Mat& rot,
													  const cv::Mat& trans,
													  const std::vector<cv::Point2f>& corr1,
													  const std::vector<cv::Point2f>& corr2,
													  const std::vector<unsigned char>& mask,
													  int& numberGood, float& paral) :
		K(calib), R(rot), t(trans), c1(corr1), c2(corr2), inliers(mask), numGood(numberGood), parallax(paral)
	{}

	void ModelSelector::CombinationCheck::operator()()
	{
		// Camera 1 Projection Matrix K[I|0]
		cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
		this->K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

		// Camera 2 Projection Matrix K[R|t]
		cv::Mat P2(3, 4, CV_32F);
		this->R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
		this->t.copyTo(P2.rowRange(0, 3).col(3));
		P2 = this->K*P2;

		// camera centers in camera1
		cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);
		cv::Mat O2 = -this->R.t()*this->t;

		// check all points
		this->numGood = 0;
		this->parallax = 1.f;

		std::vector<float> parallaxVector;
		parallaxVector.reserve(this->inliers.size());

		for (int i = 0; i < this->inliers.size(); ++i)
		{
			if (this->inliers[i] == 0) continue;

			// check cheirality, reprojection error and parallax
			float par = 1.f;
			if (this->checkCheiralityAndParallax(this->c1[i], this->c2[i], P1, O1, P2, O2, this->repoTh, par))
			{
				++this->numGood;
				parallaxVector.push_back(par);
			}			
		}

		// obtain parallax percentile
		if (!parallaxVector.empty())
		{
			int idx = static_cast<int>(this->parlPerc*parallaxVector.size());
			std::nth_element(parallaxVector.begin(), parallaxVector.begin() + idx, parallaxVector.end());
			this->parallax = parallaxVector[idx];
		}
	}

	bool ModelSelector::CombinationCheck::checkCheiralityAndParallax(const cv::Point2f& pt2d1, const cv::Point2f& pt2d2,
																	 const cv::Mat &P1, const cv::Mat &O1,
																	 const cv::Mat &P2, const cv::Mat &O2,
																	 float maxRepoError, float &parallax) const
	{
		// triangulate
		cv::Mat pt3d1;
		this->triangulate(pt2d1, pt2d2, P1, P2, pt3d1);

        if (!std::isfinite(pt3d1.at<float>(0)) || !std::isfinite(pt3d1.at<float>(1)) || !std::isfinite(pt3d1.at<float>(2)))
		{
			return false;
		}

		// check depth in camera1 and camera2
		const cv::Mat pt3d2 = P2.rowRange(0, 3).colRange(0, 3)*pt3d1 + P2.rowRange(0, 3).col(3);

		if (pt3d1.at<float>(2) <= 0 || pt3d2.at<float>(2) <= 0) return false;

		// parallax
		const cv::Mat normal1 = pt3d1 - O1;
		const float dist1 = static_cast<float>(cv::norm(normal1, cv::NormTypes::NORM_L2));

		const cv::Mat normal2 = pt3d1 - O2;
		const float dist2 = static_cast<float>(cv::norm(normal2, cv::NormTypes::NORM_L2));

		parallax = static_cast<float>(normal1.dot(normal2)) / (dist1*dist2 + std::numeric_limits<float>::epsilon());

		// reprojection error in camera1
		// use P1 as P1 = K[I|0]
		pt3d1 = P1.rowRange(0, 3).colRange(0, 3)*pt3d1;
		float u = pt3d1.at<float>(0) / pt3d1.at<float>(2);
		float v = pt3d1.at<float>(1) / pt3d1.at<float>(2);

		float e2 = (u - pt2d1.x)*(u - pt2d1.x) + (v - pt2d1.y)*(v - pt2d1.y);

		if (e2 > maxRepoError) return false;

		// reprojection error in camera2
		u = pt3d2.at<float>(0) / pt3d2.at<float>(2);
		v = pt3d2.at<float>(1) / pt3d2.at<float>(2);

		e2 = (u - pt2d2.x)*(u - pt2d2.x) + (v - pt2d2.y)*(v - pt2d2.y);

		if (e2 > maxRepoError) return false;

		// good point!
		return true;
	}

	void ModelSelector::CombinationCheck::triangulate(const cv::Point2f& u1, const cv::Point2f& u2,
													  const cv::Mat &P1, const cv::Mat &P2,
													  cv::Mat &pt3d) const
	{
		cv::Mat A(4, 4, CV_32F);

		A.row(0) = u1.x*P1.row(2) - P1.row(0);
		A.row(1) = u1.y*P1.row(2) - P1.row(1);
		A.row(2) = u2.x*P2.row(2) - P2.row(0);
		A.row(3) = u2.y*P2.row(2) - P2.row(1);

		cv::Mat u, w, vt;
		cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

		pt3d = vt.row(3).t();
		pt3d = pt3d.rowRange(0, 3) / pt3d.at<float>(3);
	}
}
