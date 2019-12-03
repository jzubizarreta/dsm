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

#include "ceres/ceres.h"

namespace dsm
{
	// Residual block interface to Ceres
	class IResidualBlock
	{
	public:

		IResidualBlock();

		virtual ~IResidualBlock();

		// unique parameter block identifier
		void setID(ceres::ResidualBlockId parameterBlockID);

		ceres::ResidualBlockId getID() const;

		// loss function
		void setLossFunction(ceres::LossFunction* lossFunction);

		ceres::LossFunction* getLossFunction() const;

		// virtual functions to implement in each derived class

		virtual int dimension() const = 0;

		virtual int numParameterBlocks() const = 0;

		virtual ceres::CostFunction* getCostFunction() const = 0;

	protected:

		// unique identifier
		// it is a pointer to ceres residual block
		ceres::ResidualBlockId id_;

		// loss function
		// memory is not managed by this class
		ceres::LossFunction* lossFunction_;
	};
}