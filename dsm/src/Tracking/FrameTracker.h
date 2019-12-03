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
#include <array>

#include "DataStructures/AffineLight.h"
#include "Utils/Settings.h"
#include "Utils/EigenTypes.h"

#include "sophus/se3.hpp"

namespace dsm
{
	class Frame;
	class FrameTrackerReference;
	class FrameTrackerProblem;
	class IDistribution;
	class IVisualizer;

	// Tracker Settings
	// Contains:
	// - Levenberg-Marquardt parameters
	// - Convergence threshold
	// - Number of iterations per pyramid level
	//
	struct FrameTrackerSettings
	{
		FrameTrackerSettings();	// for default settings
		~FrameTrackerSettings();

		int levels;

		float lambdaSuccessFac;
		float lambdaFailFac;
		float lambdaInitial;
		float lambdaLimit;
		float convergenceEps;

		std::array<int, 6> maxItsPerLvl;

		float minimumPointUsage;
	};

	// SE3 Tracking of monocular images using Direct Image Alignment
	// It uses an inverse compositional strategy
	// For more details read: 
	//		Lucas-Kanade 20 Years On: nA unifying Framework 
	//		Simon Baker & Iain Matthews
	//
	class FrameTracker
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;		

		FrameTracker(int width, int height, const FrameTrackerSettings &settings = FrameTrackerSettings());
		~FrameTracker();

		// resets the tracker invalidating all internal data
		void reset();

		// Tracking of the new frame againts the reference
		// output: relative pose to reference keyframe
		// output: relative affine light to reference keyframe
		// output: photometric error distribution
		bool trackFrame(const std::shared_ptr<FrameTrackerReference>& reference, const std::shared_ptr<Frame>& newFrame,
						Sophus::SE3f& inOutFrameToRef, AffineLight& inOutLight, std::shared_ptr<IDistribution>& errorDistribution,
						IVisualizer* outputWrapper = nullptr);

		// Accessors
		float pointUsage() const;
		float totalResidual() const;

	private:

		// Error computations of transformed points
		float computeResiduals(const std::shared_ptr<FrameTrackerReference>& reference,
							   const std::shared_ptr<Frame>& newFrame, const int lvl, 
							   const Sophus::SE3f &refToFrame, const AffineLight& light, 
							   float* res, unsigned int* validMask) const;

		// Error distribution
		std::shared_ptr<IDistribution> computeErrorDistribution(const std::shared_ptr<FrameTrackerReference>& reference, const int lvl,
																const float* const res, const unsigned int* const validMask) const;

		// Weights
		float computeWeightAndOutliers(const std::shared_ptr<FrameTrackerReference>& reference, const int lvl,
									   std::shared_ptr<IDistribution>& errorDistribution,
									   const float* const res, unsigned int* const validMask,
									   float* const w) const;

	private:

		// statistics
		float pointUsage_;
		float finalResidual_;

		// buffers
		float* residuals;
		float* tempResiduals;
		float* weights;
		float* tempWeights;

		// mask to contral which points are valid for each iteration
		// this enables working with sse instructions
		unsigned int* validityMask;		
		unsigned int* tempValidityMask;

		// optimization problem
		std::unique_ptr<FrameTrackerProblem> problem;

		// settings
		std::unique_ptr<FrameTrackerSettings> config;
	};
}