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

#include "ceres/local_parameterization.h"

#include <memory>

namespace dsm
{
	// Local parameterization of point parameters
	// In this case, only the inverse depth
	class PointParameterization : public ceres::LocalParameterization
	{
	public:
		PointParameterization();
		virtual ~PointParameterization();

		virtual bool Plus(const double* x, const double* delta,
						  double* x_plus_delta) const;

		virtual bool ComputeJacobian(const double* x,
									 double* jacobian) const;

		virtual int GlobalSize() const;

		virtual int LocalSize() const;
	};

	// Parameter block of points
	// Only onde dimensional parameter
	// This class is not needed but makes all the
	// optimization easier to understand
	class PointParameterBlock : public IParameterBlock<1>
	{
	public:

		// variable ordering
		static const int Group = 0;

		// default constructor
		PointParameterBlock();

		// constructor with an inverse depth
		PointParameterBlock(double iDepth);

		// destructor
		~PointParameterBlock();

		void setIDepth(double iDepth);

		double getIDepth() const;

		double getIDepthBackup() const;

		void step(double& delta) const;
		void scaledStep(double& delta) const;

	private:

		static const std::unique_ptr<PointParameterization> pointParameterization;
	};
}