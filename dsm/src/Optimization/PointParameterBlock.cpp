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

#include "PointParameterBlock.h"
#include "Utils/Settings.h"

namespace dsm
{
	// Parameterization
	PointParameterization::PointParameterization()
	{
	}

	PointParameterization::~PointParameterization()
	{}

	bool PointParameterization::Plus(const double* x, const double* delta,
									 double* x_plus_delta) const
	{
		const auto& settings = Settings::getInstance();

		// variable scaling
		// "Numerical Optimization" Nocedal et al. 2006, page 95
		x_plus_delta[0] = x[0] + (delta[0] * settings.varScaleIDepth);

		return true;
	}

	bool PointParameterization::ComputeJacobian(const double* x,
												double* jacobian) const
	{
		jacobian[0] = 1.0;
		return true;
	}

	int PointParameterization::GlobalSize() const
	{
		return 1;
	}

	int PointParameterization::LocalSize() const
	{
		return 1;
	}

	// Parameter Block
	const std::unique_ptr<PointParameterization> PointParameterBlock::pointParameterization =
		std::make_unique<PointParameterization>();

	PointParameterBlock::PointParameterBlock() : IParameterBlock<1>()
	{
	}

	PointParameterBlock::PointParameterBlock(double iDepth) : IParameterBlock<1>()
	{
		this->setIDepth(iDepth);
		this->backup();
		this->setLocalParameterization(PointParameterBlock::pointParameterization.get());
	}

	PointParameterBlock::~PointParameterBlock()
	{}

	void PointParameterBlock::setIDepth(double iDepth)
	{
		this->parameters_[0] = iDepth;
	}

	double PointParameterBlock::getIDepth() const
	{
		return this->parameters_[0];
	}

	double PointParameterBlock::getIDepthBackup() const
	{
		return this->parameters_backup_[0];
	}

	void PointParameterBlock::step(double& delta) const
	{
		// linear in the euclidean space
		// currentState = backupState + delta
		delta = this->parameters_[0] - this->parameters_backup_[0];
	}

	void PointParameterBlock::scaledStep(double& delta) const
	{
		const auto& settings = Settings::getInstance();

		// linear in the euclidean space
		// currentState = backupState + delta
		delta = this->parameters_[0] - this->parameters_backup_[0];

		// scale
		delta /= settings.varScaleIDepth;
	}
}