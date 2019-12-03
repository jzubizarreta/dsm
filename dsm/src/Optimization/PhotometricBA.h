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

#include <vector>
#include <unordered_set>

#include "Utils/EigenTypes.h"

#include "ceres/ceres.h"

#include "PhotometricBAConfig.h"
#include "LeastSquares/LeastSquaresProblem.h"

namespace dsm
{
	class Frame;
	class ActivePoint;
	class IDistribution;
	class IVisualizer;
	class WorkerThreadPool;

	class PhotometricResidual;
	class PhotometricBAEvaluationCallback;
	class PhotometricBAIterationCallback;	

	struct PhotometricBAStats
	{
		PhotometricBAStats(int size);

		void reset();

		// parameters and residuals
		Eigen::VecXi numResiduals;
		Eigen::VecXi numParameters;

		// iterations management
		Eigen::VecXi iterations;
		Eigen::VecXi goodIterations;
		Eigen::VecXi badIterations;

		// current lvl
		int currentLvl;

		// error in each lvl
		Eigen::VecXf initError;
		Eigen::VecXf endError;

		// execution time
		Eigen::VecXf execTime;

		// termination
		std::vector<ceres::TerminationType> termination;
	};

	class CeresPhotometricBA
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		friend class PhotometricResidual;
		friend class PhotometricCostFunction;
		friend class PhotometricBAIterationCallback;

		CeresPhotometricBA(const PhotometricBAConfig &config = PhotometricBAConfig(),
						   IVisualizer *outputWrapper = nullptr);
		~CeresPhotometricBA();

		// main function to solve and merge optimization result
		void solve(const std::vector<std::shared_ptr<Frame>>& active);

	private:

		void reset();

		void prepareOptimization(const std::vector<std::shared_ptr<Frame>>& active, LeastSquaresProblem& problem);

		void mergeOptimization(const std::vector<std::shared_ptr<Frame>>& active, std::vector<PhotometricResidual*>& obsToRemove) const;

		void removeBadObservations(const std::vector<PhotometricResidual*>& obsToRemove) const;

		void freeFixedKeyframesMemory();

		// error distribution
		void calcNewKeyframeErrorDist(int lvl, bool lastFit) const;
		void saveErrorDistribution(const std::shared_ptr<IDistribution>& dist,
								   const std::vector<float>& allObservations,
								   int keyframeID) const;

		// prints optimization report
		void printStats();

	private:

		// optimization options
		PhotometricBAConfig options;

		// optimization stats
		std::unique_ptr<PhotometricBAStats> stats;

		// iteration callback to control thresholds and termination criteria
		std::unique_ptr<PhotometricBAIterationCallback> iterCallback;

		// visualizer
		IVisualizer* outputWrapper;

		// statistics
		int numFrames;
		int numPoints;
		int numResiduals;

		// actual optimization data
		std::vector<Frame*> activeKeyframes;
		std::vector<PhotometricResidual*> activeObservations;

		std::unordered_set<Frame*> fixedKeyframes;
	};
}