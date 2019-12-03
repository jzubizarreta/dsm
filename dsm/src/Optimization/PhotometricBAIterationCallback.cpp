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

#include "PhotometricBAIterationCallback.h"
#include "PhotometricResidual.h"
#include "PointParameterBlock.h"
#include "FrameParameterBlock.h"
#include "DataStructures/Frame.h"
#include "DataStructures/ActivePoint.h"
#include "DataStructures/Pattern.h"
#include "Utils/Settings.h"
#include "Utils/UtilFunctions.h"

namespace dsm
{
	PhotometricBAIterationCallback::PhotometricBAIterationCallback(const CeresPhotometricBA& bundleAdjustment) :
		bundle(bundleAdjustment)
	{}

	PhotometricBAIterationCallback::~PhotometricBAIterationCallback()
	{}

	ceres::CallbackReturnType PhotometricBAIterationCallback::operator()(const ceres::IterationSummary& summary)
	{
		const auto& settings = Settings::getInstance();

		bool canBreak = false;

		// termination criteria
		if (summary.iteration > 0 && summary.step_is_successful)
		{
			canBreak = this->checkTerminationCriteria();
		}

		if (summary.step_is_successful)
		{
			// state backup
			this->backup();
		}

		// stats
		const int lvl = this->bundle.stats->currentLvl;

		if (summary.step_is_successful)
		{
			this->bundle.stats->goodIterations[lvl]++;
		}
		else
		{
			this->bundle.stats->badIterations[lvl]++;
		}

		this->bundle.stats->iterations[lvl]++;

		// converged?
		if (canBreak && 
			summary.iteration >= settings.minBAIterations)
		{
			return ceres::CallbackReturnType::SOLVER_TERMINATE_SUCCESSFULLY;
		}
		
		return ceres::CallbackReturnType::SOLVER_CONTINUE;
	}

	void PhotometricBAIterationCallback::backup() const
	{		
		for (const Frame* const frame : this->bundle.activeKeyframes)
		{
			frame->frameBlock()->backup();

			for (const auto& point : frame->activePoints())
			{
				point->pointBlock()->backup();
			}
		}
	}

	bool PhotometricBAIterationCallback::checkTerminationCriteria() const
	{
		const auto& settings = Settings::getInstance();

		double deltaAlpha = 0;
		double deltaBeta = 0;
		double deltaRot = 0;
		double deltaTrans = 0;
		double meanIDepth = 0;
		double numPoints = 0;

		Eigen::Matrix<double, 8, 1> frameStep;

		for (const Frame* const frame : this->bundle.activeKeyframes)
		{
			// obtain last iteration steps
			frame->frameBlock()->scaledStep(frameStep);

			// sum
			deltaTrans += frameStep.segment<3>(0).squaredNorm();
			deltaRot += frameStep.segment<3>(3).squaredNorm();
			deltaAlpha += frameStep[6] * frameStep[6];
			deltaBeta += frameStep[7] * frameStep[7];

			for (const auto& point : frame->activePoints())
			{
				meanIDepth += fabs(point->pointBlock()->getIDepthBackup());
				numPoints++;
			}
		}

		int numKeyframes = (int)this->bundle.activeKeyframes.size();

		deltaAlpha /= numKeyframes;
		deltaBeta /= numKeyframes;
		deltaTrans /= numKeyframes;
		deltaRot /= numKeyframes;
		meanIDepth /= numPoints;

		bool converged = sqrt(deltaAlpha) < 0.0006 &&				// affine light a change
						 sqrt(deltaBeta) < 0.00006 &&				// affine light b change
						 sqrt(deltaRot) < 0.00006 &&				// transformation R change
						 sqrt(deltaTrans)*meanIDepth < 0.00006;		// transformation T change respet to inverse depths
		
		return converged;
	}
}
