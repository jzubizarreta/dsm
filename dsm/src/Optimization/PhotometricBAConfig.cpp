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

#include "PhotometricBAConfig.h"
#include "Utils/Settings.h"

namespace dsm
{
	PhotometricBAConfig::PhotometricBAConfig()
	{
		const auto& settings = Settings::getInstance();

		// problem options

		// Do not enable ceres to take ownership of any object
		this->problemOptions.cost_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
		this->problemOptions.local_parameterization_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
		this->problemOptions.loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

		// If true, trades memory for faster RemoveResidualBlock() and
		// RemoveParameterBlock() operations
		this->problemOptions.enable_fast_removal = false;

		// By default, Ceres performs a variety of safety checks when constructing
		// the problem. There is a small but measurable performance penalty to
		// these checks, typically around 5% of construction time. If you are sure
		// your problem construction is correct, and 5% of the problem construction
		// time is truly an overhead you want to avoid, then you can set
		// disable_all_safety_checks to true.
		//
		// WARNING: Do not set this to true, unless you are absolutely sure of what
		// you are doing.
		this->problemOptions.disable_all_safety_checks = true;

		// solver options
		if (settings.showFullReport)
		{
			this->solverOptions.logging_type = ceres::PER_MINIMIZER_ITERATION;
			this->solverOptions.minimizer_progress_to_stdout = true;
		}
		else
		{
			this->solverOptions.logging_type = ceres::SILENT;
			this->solverOptions.minimizer_progress_to_stdout = false;
		}

		this->solverOptions.linear_solver_type = ceres::DENSE_SCHUR;
		this->solverOptions.minimizer_type = ceres::TRUST_REGION;
		this->solverOptions.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

		this->solverOptions.use_nonmonotonic_steps = true;
		this->solverOptions.max_consecutive_nonmonotonic_steps = 2;

		this->solverOptions.jacobi_scaling = true;
		
		// required for ceres::IterationCallback
		this->solverOptions.update_state_every_iteration = true;

		this->solverOptions.max_num_iterations = settings.maxBAIterations;

		// ignore ceres termination criteria
		this->solverOptions.function_tolerance = 0.0;
		this->solverOptions.gradient_tolerance = 0.0;
		this->solverOptions.parameter_tolerance = 0.0;
		
		// number of parallel threads
		this->solverOptions.num_threads = settings.mappingThreads;
	}
}