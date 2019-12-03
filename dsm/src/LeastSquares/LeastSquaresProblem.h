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

#include "ceres/ceres.h"

#include "IParameterBlock.h"
#include "IResidualBlock.h"

namespace dsm
{
	// Wrapper of ceres problem
	class LeastSquaresProblem
	{
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		LeastSquaresProblem(const ceres::Problem::Options& problemOptions);
		~LeastSquaresProblem();

		// main function to solve the least squares problem
		void solve(const ceres::Solver::Options &solverOptions);

		// optimization summary
		// solve() must be called before
		const ceres::Solver::Summary& summary() const;

		// ParameterBlock management
		template<int Dim>
		void addParameterBlock(IParameterBlock<Dim>* parameterBlock);

		template<int Dim>
		void removeParameterBlock(IParameterBlock<Dim>* parameterBlock);

		template<int Dim>
		void setParameterBlockConstant(IParameterBlock<Dim>* parameterBlock);

		template<int Dim>
		void setParameterBlockVariable(IParameterBlock<Dim>* parameterBlock);

		template<int Dim>
		void setParameterLowerBound(IParameterBlock<Dim>* parameterBlock, int index, double lowerBound);

		template<int Dim>
		void setParameterUpperBound(IParameterBlock<Dim>* parameterBlock, int index, double upperBound);

		// ResidualBlock management
		template<int Dim0>
		void addResidualBlock(IResidualBlock* residualBlock, 
							  IParameterBlock<Dim0>* x0);

		template<int Dim0, int Dim1>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1);

		template<int Dim0, int Dim1, int Dim2>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2);

		template<int Dim0, int Dim1, int Dim2, int Dim3>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2,
							  IParameterBlock<Dim3>* x3);

		template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2,
							  IParameterBlock<Dim3>* x3,
							  IParameterBlock<Dim4>* x4);

		template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
				 int Dim5>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2,
							  IParameterBlock<Dim3>* x3,
							  IParameterBlock<Dim4>* x4,
							  IParameterBlock<Dim5>* x5);

		template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
				 int Dim5, int Dim6>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2,
							  IParameterBlock<Dim3>* x3,
							  IParameterBlock<Dim4>* x4,
							  IParameterBlock<Dim5>* x5,
							  IParameterBlock<Dim6>* x6);

		template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4, 
				 int Dim5, int Dim6, int Dim7>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2,
							  IParameterBlock<Dim3>* x3,
							  IParameterBlock<Dim4>* x4,
							  IParameterBlock<Dim5>* x5,
							  IParameterBlock<Dim6>* x6,
							  IParameterBlock<Dim7>* x7);

		template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
				 int Dim5, int Dim6, int Dim7, int Dim8>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2,
							  IParameterBlock<Dim3>* x3,
							  IParameterBlock<Dim4>* x4,
							  IParameterBlock<Dim5>* x5,
							  IParameterBlock<Dim6>* x6,
							  IParameterBlock<Dim7>* x7,
							  IParameterBlock<Dim8>* x8);

		template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
				 int Dim5, int Dim6, int Dim7, int Dim8, int Dim9>
		void addResidualBlock(IResidualBlock* residualBlock,
							  IParameterBlock<Dim0>* x0,
							  IParameterBlock<Dim1>* x1,
							  IParameterBlock<Dim2>* x2,
							  IParameterBlock<Dim3>* x3,
							  IParameterBlock<Dim4>* x4,
							  IParameterBlock<Dim5>* x5,
							  IParameterBlock<Dim6>* x6,
							  IParameterBlock<Dim7>* x7,
							  IParameterBlock<Dim8>* x8,
							  IParameterBlock<Dim9>* x9);

		void removeResidualBlock(ceres::ResidualBlockId residualBlockId);

		// Statistics
		int numParameterBlocks() const;

		int numParameters() const;

		int numResidualBlocks() const;

		int numResiduals() const;

	private:

		// Ceres problem
        std::unique_ptr<ceres::Problem> problem_;
		ceres::Solver::Summary summary_;
	};

	// Implementation

	template<int Dim>
	inline void LeastSquaresProblem::addParameterBlock(IParameterBlock<Dim>* parameterBlock)
	{
		parameterBlock->setFixed(false);
		this->problem_->AddParameterBlock(parameterBlock->getParameters(),
										  parameterBlock->dimension(),
										  parameterBlock->getLocalParameterization());
	}

	template<int Dim>
	inline void LeastSquaresProblem::removeParameterBlock(IParameterBlock<Dim>* parameterBlock)
	{
		this->problem_->RemoveParameterBlock(parameterBlock->getParameters());
	}

	template<int Dim>
	inline void LeastSquaresProblem::setParameterBlockConstant(IParameterBlock<Dim>* parameterBlock)
	{
		parameterBlock->setFixed(true);
		this->problem_->SetParameterBlockConstant(parameterBlock->getParameters());
	}

	template<int Dim>
	inline void LeastSquaresProblem::setParameterBlockVariable(IParameterBlock<Dim>* parameterBlock)
	{
		parameterBlock->setFixed(false);
		this->problem_->SetParameterBlockVariable(parameterBlock->getParameters());
	}

	template<int Dim>
	inline void LeastSquaresProblem::setParameterLowerBound(IParameterBlock<Dim>* parameterBlock,
															int index, double lowerBound)
	{
		this->problem_->SetParameterLowerBound(parameterBlock->getParameters(), index, lowerBound);
	}

	template<int Dim>
	inline void LeastSquaresProblem::setParameterUpperBound(IParameterBlock<Dim>* parameterBlock,
															int index, double upperBound)
	{
		this->problem_->SetParameterUpperBound(parameterBlock->getParameters(), index, upperBound);
	}

	template<int Dim0>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2, int Dim3>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2,
													  IParameterBlock<Dim3>* x3)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters(),
																			 x3->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2,
													  IParameterBlock<Dim3>* x3,
													  IParameterBlock<Dim4>* x4)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters(),
																			 x3->getParameters(),
																			 x4->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
			 int Dim5>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2,
													  IParameterBlock<Dim3>* x3,
													  IParameterBlock<Dim4>* x4,
													  IParameterBlock<Dim5>* x5)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters(),
																			 x3->getParameters(),
																			 x4->getParameters(),
																			 x5->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
			 int Dim5, int Dim6>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2,
													  IParameterBlock<Dim3>* x3,
													  IParameterBlock<Dim4>* x4,
													  IParameterBlock<Dim5>* x5,
													  IParameterBlock<Dim6>* x6)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters(),
																			 x3->getParameters(),
																			 x4->getParameters(),
																			 x5->getParameters(),
																			 x6->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
			 int Dim5, int Dim6, int Dim7>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2,
													  IParameterBlock<Dim3>* x3,
													  IParameterBlock<Dim4>* x4,
													  IParameterBlock<Dim5>* x5,
													  IParameterBlock<Dim6>* x6,
													  IParameterBlock<Dim7>* x7)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters(),
																			 x3->getParameters(),
																			 x4->getParameters(),
																			 x5->getParameters(),
																			 x6->getParameters(),
																			 x7->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
			 int Dim5, int Dim6, int Dim7, int Dim8>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2,
													  IParameterBlock<Dim3>* x3,
													  IParameterBlock<Dim4>* x4,
													  IParameterBlock<Dim5>* x5,
													  IParameterBlock<Dim6>* x6,
													  IParameterBlock<Dim7>* x7,
													  IParameterBlock<Dim8>* x8)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters(),
																			 x3->getParameters(),
																			 x4->getParameters(),
																			 x5->getParameters(),
																			 x6->getParameters(),
																			 x7->getParameters(),
																			 x8->getParameters());
				
		// set identifier
		residualBlock->setID(residualId);
	}

	template<int Dim0, int Dim1, int Dim2, int Dim3, int Dim4,
			 int Dim5, int Dim6, int Dim7, int Dim8, int Dim9>
	inline void LeastSquaresProblem::addResidualBlock(IResidualBlock* residualBlock,
													  IParameterBlock<Dim0>* x0,
													  IParameterBlock<Dim1>* x1,
													  IParameterBlock<Dim2>* x2,
													  IParameterBlock<Dim3>* x3,
													  IParameterBlock<Dim4>* x4,
													  IParameterBlock<Dim5>* x5,
													  IParameterBlock<Dim6>* x6,
													  IParameterBlock<Dim7>* x7,
													  IParameterBlock<Dim8>* x8,
													  IParameterBlock<Dim9>* x9)
	{
		// insert into problem
		ceres::ResidualBlockId residualId = this->problem_->AddResidualBlock(residualBlock->getCostFunction(),
																			 residualBlock->getLossFunction(),
																			 x0->getParameters(),
																			 x1->getParameters(),
																			 x2->getParameters(),
																			 x3->getParameters(),
																			 x4->getParameters(),
																			 x5->getParameters(),
																			 x6->getParameters(),
																			 x7->getParameters(),
																			 x8->getParameters(),
																			 x9->getParameters());

		// set identifier
		residualBlock->setID(residualId);
	}
}
