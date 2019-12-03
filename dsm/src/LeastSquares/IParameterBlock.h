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

#include <array>

namespace ceres
{
	class LocalParameterization;
}

namespace dsm
{
	// Interface for ParameterBlock
	// Dim: dimension of parameter block
	// T: type of the parameter block
	template<int Dim>
	class IParameterBlock
	{
	public:

		inline IParameterBlock() : fixed_(false), localParameterization_(nullptr)
		{
			this->parameters_.fill(0);
			this->parameters_backup_.fill(0);
		}

		inline virtual ~IParameterBlock() {;}

		// dimensions of the parameter block
		inline int dimension() const
		{
			return Dim;
		}

		// manage if this block should be optimized
		inline void setFixed(bool fixed)
		{
			this->fixed_ = fixed;
		}

		inline bool getFixed() const
		{
			return this->fixed_;
		}

		// parameterization
		inline void setLocalParameterization(ceres::LocalParameterization* localParameterization)
		{
			this->localParameterization_ = localParameterization;
		}

		inline ceres::LocalParameterization* getLocalParameterization() const
		{
			return this->localParameterization_;
		}

		// parameters
		inline void setParameters(double* params)
		{
			std::copy(params, params + Dim, this->parameters_.data());
		}

		inline double* getParameters()
		{
			return this->parameters_.data();
		}

		inline void backup()
		{
			this->parameters_backup_ = this->parameters_;
		}

		inline double* getBackupParameters()
		{
			return this->parameters_backup_.data();
		}

	protected:

		// flag to control if optimized or not
		bool fixed_;

		// parameters
		std::array<double, Dim> parameters_;

		// back up of the parameters
		std::array<double, Dim> parameters_backup_;

		// local parameterization to use
		ceres::LocalParameterization* localParameterization_;
	};
}
