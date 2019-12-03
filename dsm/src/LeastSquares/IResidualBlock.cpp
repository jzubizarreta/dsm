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

#include "IResidualBlock.h"

namespace dsm
{
	IResidualBlock::IResidualBlock() : id_(nullptr), lossFunction_(nullptr)
	{}

	IResidualBlock::~IResidualBlock()
	{}

	void IResidualBlock::setID(ceres::ResidualBlockId parameterBlockID)
	{
		this->id_ = parameterBlockID;
	}

	ceres::ResidualBlockId IResidualBlock::getID() const
	{
		return this->id_;
	}

	void IResidualBlock::setLossFunction(ceres::LossFunction* lossFunction)
	{
		this->lossFunction_ = lossFunction;
	}

	ceres::LossFunction* IResidualBlock::getLossFunction() const
	{
		return this->lossFunction_;
	}
}