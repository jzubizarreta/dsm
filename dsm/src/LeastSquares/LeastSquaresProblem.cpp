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

#include "LeastSquaresProblem.h"

namespace dsm
{
	LeastSquaresProblem::LeastSquaresProblem(const ceres::Problem::Options& problemOptions)
	{
		// ceres problem creation
        this->problem_ = std::make_unique<ceres::Problem>(problemOptions);
	}

	LeastSquaresProblem::~LeastSquaresProblem()
	{}

    void LeastSquaresProblem::solve(const ceres::Solver::Options &solverOptions)
	{
        ceres::Solve(solverOptions, this->problem_.get(), &this->summary_);
	}

	const ceres::Solver::Summary& LeastSquaresProblem::summary() const
	{
		return this->summary_;
	}

	void LeastSquaresProblem::removeResidualBlock(ceres::ResidualBlockId residualBlockId)
	{
		this->problem_->RemoveResidualBlock(residualBlockId);
	}

	int LeastSquaresProblem::numParameterBlocks() const
	{
		return this->problem_->NumParameterBlocks();
	}

	int LeastSquaresProblem::numParameters() const
	{
		return this->problem_->NumParameters();
	}

	int LeastSquaresProblem::numResidualBlocks() const
	{
		return this->problem_->NumResidualBlocks();
	}

	int LeastSquaresProblem::numResiduals() const
	{
		return this->problem_->NumResiduals();
	}
}
