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

#include "ceres/cost_function.h"

#include "LeastSquares/IResidualBlock.h"
#include "DataStructures/Visibility.h"
#include "Utils/EigenTypes.h"

namespace dsm
{
	class CeresPhotometricBA;
	class PhotometricResidual;

	class Frame;
	class ActivePoint;

	// Cost function
	class PhotometricCostFunction : public ceres::CostFunction
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		PhotometricCostFunction(PhotometricResidual* const residual);

		virtual ~PhotometricCostFunction();

		bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

	private:
	
		void discardOutlier(double** jacobians) const;
		void discardOutlier(double** jacobians, int idx) const;
		void discardOOB(double* residuals, double** jacobians) const;

	private:

		// reference to residual
		PhotometricResidual* const residual_;
	};

	// Residual block
	class PhotometricResidual : public IResidualBlock
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		friend class PhotometricCostFunction;

		PhotometricResidual(const std::unique_ptr<ActivePoint>& point, 
							const std::shared_ptr<Frame>& targetFrame,
							const std::unique_ptr<CeresPhotometricBA>& photometricBA);
		virtual ~PhotometricResidual();

		// avoid copying
		PhotometricResidual(const PhotometricResidual &) = delete;
		PhotometricResidual& operator=(const PhotometricResidual&) = delete;

		// get current state residuals
		bool evaluate(int lvl, Eigen::VecXf& residuals) const;

		// number of residual dimensions
		int dimension() const;

		// number of parameter blocks for this residual
		int numParameterBlocks() const;

		// returns the cost function
		ceres::CostFunction* getCostFunction() const;

		// point
		ActivePoint* point() const;

		// frames
		Frame* ownerFrame() const;
		Frame* targetFrame() const;

		// optimization state
		Visibility state() const;

		// point inverse depth hessian
		double iDepthHessian() const;

		// final energy
		double energy() const;
		const Eigen::VecXd& pixelEnergy() const;

		// huber loss per pixel
		double lossWeight() const;

	private:

		// reference to parameters
		Frame* const ownerFrame_;
		Frame* const targetFrame_;

		ActivePoint* const point_;

		// reference to the bundle adjustment
		CeresPhotometricBA* const photometricBA_;

		// status
		Visibility state_;

		// inverse depth hessian
		double iDepthHessian_;

		// residual
		double energy_;
		Eigen::VecXd pixelEnergy_;

		// weight
		double lossWeight_;

		// cost function
		std::unique_ptr<ceres::CostFunction> costFunction_;
	};
}