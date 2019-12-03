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

#include "LeastSquares/IParameterBlock.h"
#include "DataStructures/AffineLight.h"

#include "sophus/se3.hpp"

#include "ceres/local_parameterization.h"

#include <memory>

namespace dsm
{
	// Local parameterization of frame parameters
	// In this case pose SE3 and affine light
	// Use only this parameterization if you are 
	// estimating jacobians respect to the tangent space se3
	class FrameParameterization : public ceres::LocalParameterization
	{
	public:
		FrameParameterization();
		virtual ~FrameParameterization();

		virtual bool Plus(const double* x, const double* delta,
						  double* x_plus_delta) const;

		virtual bool ComputeJacobian(const double* x,
									 double* jacobian) const;

		virtual int GlobalSize() const;

		virtual int LocalSize() const;
	};

	// Frame parameter block
	// First 7 components are the pose ( 4 quaternion + 3 translation)
	// Last 2 components are the affine light (alpha, beta)
	class FrameParameterBlock : public IParameterBlock<9>
	{
	public:

		// variable ordering
		static const int Group = 1;

		// default constructor
		FrameParameterBlock();

		// constructor with a pose
		FrameParameterBlock(const Sophus::SE3d& camToWorld, const AffineLight& affineLight);

		// destructor
		~FrameParameterBlock();

		void setPose(const Sophus::SE3d& camToWorld);
		void setAffineLight(const AffineLight& affineLight);

		Sophus::SE3d getPose() const;
		AffineLight getAffineLight() const;

		Sophus::SE3d getPoseBackup() const;
		AffineLight getAffineLightBackup() const;

		void step(Eigen::Matrix<double, 8, 1>& delta) const;
		void scaledStep(Eigen::Matrix<double, 8, 1>& delta) const;

	private:

		static const std::unique_ptr<FrameParameterization> frameParameterization;
	};
}