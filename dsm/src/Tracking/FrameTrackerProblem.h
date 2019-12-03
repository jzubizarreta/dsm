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

#include "OptimizationStructures/MatrixBlock.h"
#include "OptimizationStructures/VectorBlock.h"
#include "Utils/Settings.h"
#include "Utils/EigenTypes.h"

namespace dsm
{
	class FrameTrackerReference;

	// Frame tracker optimization problem
	// This class is prepared to work with inverse compositional approach
	// First jacobians are calculated at identity in each lvl
	// Second the problem is solved at each iteration
	// 8 parameters: 6 for SE3 pose and 2 for affine light model
	// Optimized with SSE instructions for jacobian calculation

	class FrameTrackerProblem
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		FrameTrackerProblem(const int levels);
		~FrameTrackerProblem();

		void computeJacobAtIdentity(const std::shared_ptr<FrameTrackerReference>& trackingReference, int lvl);

		bool solve(float* residuals, float* weights, unsigned int* valid, int lvl,
				   const float lambda, const float convEps, Eigen::Vec8f& delta);

	private:

		// least squares problem H*x=g
		MatrixBlock<8, 8> H_;
		VectorBlock<8> g_;

		// jacobian of all the points computed at identity at each lvl
		// it is only computed when the reference changes
		std::vector<std::vector<
			Eigen::Matrix<float, 1, 8>, 
				Eigen::aligned_allocator<Eigen::Matrix<float, 1, 8>>>> jacobians;
	};
}
