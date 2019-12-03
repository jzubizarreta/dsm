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

#include "PhotometricBA.h"
#include "PhotometricResidual.h"
#include "PhotometricBAIterationCallback.h"
#include "PointParameterBlock.h"
#include "FrameParameterBlock.h"
#include "DataStructures/Frame.h"
#include "DataStructures/ActivePoint.h"
#include "DataStructures/Visibility.h"
#include "DataStructures/Pattern.h"
#include "Statistics/Statistics.h"
#include "Statistics/IDistribution.h"
#include "Statistics/RobustNormalDistribution.h"
#include "Statistics/TDistribution.h"
#include "Utils/GlobalCalibration.h"
#include "Utils/Settings.h"
#include "Utils/Projection.h"
#include "Utils/UtilFunctions.h"
#include "Visualizer/IVisualizer.h"

#include <fstream>
#include <iomanip>

namespace dsm
{
	PhotometricBAStats::PhotometricBAStats(int size)
	{
		this->numResiduals = Eigen::VecXi::Zero(size);
		this->numParameters = Eigen::VecXi::Zero(size);
		this->iterations = Eigen::VecXi::Zero(size);
		this->goodIterations = Eigen::VecXi::Zero(size);
		this->badIterations = Eigen::VecXi::Zero(size);
		this->initError = Eigen::VecXf::Zero(size);
		this->endError = Eigen::VecXf::Zero(size);
		this->execTime = Eigen::VecXf::Zero(size);
		this->termination.resize(size);
		this->currentLvl = 0;
	}

	void PhotometricBAStats::reset()
	{
		this->numResiduals.setZero();
		this->numParameters.setZero();
		this->iterations.setZero();
		this->goodIterations.setZero();
		this->badIterations.setZero();
		this->initError.setZero();
		this->endError.setZero();
		this->execTime.setZero();
		this->currentLvl = 0;
	}

	CeresPhotometricBA::CeresPhotometricBA(const PhotometricBAConfig &config,
										   IVisualizer *outWrapper) :
		options(config), 
		numFrames(0), 
		numPoints(0), 
		numResiduals(0),
		outputWrapper(outWrapper)
	{
		const auto& settings = Settings::getInstance();

		// iteration callback
		this->iterCallback = std::make_unique<PhotometricBAIterationCallback>(*this);
		this->options.solverOptions.callbacks.push_back(this->iterCallback.get());

		this->stats = std::make_unique<PhotometricBAStats>(settings.optMaxLevel + 1);
	}

	CeresPhotometricBA::~CeresPhotometricBA()
	{
	}

	void CeresPhotometricBA::reset()
	{
		this->activeKeyframes.clear();
		this->activeObservations.clear();
	}

	void CeresPhotometricBA::solve(const std::vector<std::shared_ptr<Frame>>& active)
	{
		const auto& settings = Settings::getInstance();

		// reset internal data
		this->reset();

		// create least squares problem
		LeastSquaresProblem problem(this->options.problemOptions);

		// prepare data and observations -> very slow!!
		Utils::Time t3 = std::chrono::steady_clock::now();
		this->prepareOptimization(active, problem);
		Utils::Time t4 = std::chrono::steady_clock::now();
		//std::cout << "Prepare opt: " << Utils::elapsedTime(t3, t4) << std::endl;

		// pyramidal solve
		this->stats->reset();

		Utils::Time t5 = std::chrono::steady_clock::now();
		for (int lvl = settings.optMaxLevel; lvl >= 0; lvl--)
		{
			// set level
			this->stats->currentLvl = lvl;

			// obtain error distribution for this level
			// keep it fixed during all optimization, it changes the objective function
			Utils::Time tInitDist = std::chrono::steady_clock::now();
			this->calcNewKeyframeErrorDist(lvl, false);
			Utils::Time tEndDist = std::chrono::steady_clock::now();
			//std::cout << "Distribution " << lvl << ": " << Utils::elapsedTime(tInitDist, tEndDist) << std::endl;

			// solve
			Utils::Time tInitLvl = std::chrono::steady_clock::now();
			problem.solve(this->options.solverOptions);
			Utils::Time tEndLvl = std::chrono::steady_clock::now();

			// stats
			const ceres::Solver::Summary& summary = problem.summary();
			this->stats->numResiduals[lvl] = summary.num_residuals_reduced;
			this->stats->numParameters[lvl] = summary.num_effective_parameters_reduced;
			this->stats->initError[lvl] = (float)std::sqrt(summary.initial_cost / summary.num_residuals_reduced);
			this->stats->endError[lvl] = (float)std::sqrt(summary.final_cost / summary.num_residuals_reduced);
			this->stats->termination[lvl] = summary.termination_type;
			this->stats->execTime[lvl] = Utils::elapsedTime(tInitLvl, tEndLvl);

		}		
		Utils::Time t6 = std::chrono::steady_clock::now();
		//std::cout << "Solve opt: " << Utils::elapsedTime(t5, t6) << std::endl;

		// calculate again the error distribution with with current state
		this->calcNewKeyframeErrorDist(0, true);

		// merge solution
		// obtain a list of observations to remove
		Utils::Time t7 = std::chrono::steady_clock::now();
		std::vector<PhotometricResidual*> obsToRemove;
		this->mergeOptimization(active, obsToRemove);
		Utils::Time t8 = std::chrono::steady_clock::now();
		//std::cout << "Merge opt: " << Utils::elapsedTime(t7, t8) << std::endl;

		// remove bad observations
		Utils::Time t9 = std::chrono::steady_clock::now();
		this->removeBadObservations(obsToRemove);
		Utils::Time t10 = std::chrono::steady_clock::now();
		//std::cout << "Remove bad obs: " << Utils::elapsedTime(t9, t10) << std::endl;

		// free moemry
		this->freeFixedKeyframesMemory();

		// print result
		if (settings.printSummary)
		{
			this->printStats();

			if (settings.showFullReport)
			{
				std::string report = problem.summary().FullReport();
				std::cout << report << std::endl;
				std::cout << std::endl;
			}
		}
	}

	void CeresPhotometricBA::prepareOptimization(const std::vector<std::shared_ptr<Frame>>& active, LeastSquaresProblem& problem)
	{
		const auto& settings = Settings::getInstance();

		// ordering
		ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering();

		// add all cameras, points and observations
		for (const std::shared_ptr<Frame>& kf : active)
		{
			// add camera
			problem.addParameterBlock(kf->frameBlock().get());
			ordering->AddElementToGroup(kf->frameBlock()->getParameters(), FrameParameterBlock::Group);
			this->activeKeyframes.push_back(kf.get());

			// fix first camera, this fixes unobservable gauge freedoms
			if (kf->keyframeID() == 0)
			{
				problem.setParameterBlockConstant(kf->frameBlock().get());
			}			

			for (const auto& point : kf->activePoints())
			{
				// add point
				problem.addParameterBlock(point->pointBlock().get());
				ordering->AddElementToGroup(point->pointBlock()->getParameters(), PointParameterBlock::Group);

				// add residuals
				for (const auto& obs : point->observations())
				{
					problem.addParameterBlock(obs.first->frameBlock().get());
					ordering->AddElementToGroup(obs.first->frameBlock()->getParameters(), FrameParameterBlock::Group);

					// fix if not active
					if (!obs.first->isActive())
					{
						problem.setParameterBlockConstant(obs.first->frameBlock().get());
						
						if (settings.minimizeMemory)
						{
							this->fixedKeyframes.insert(obs.first);
						}
					}

					problem.addResidualBlock(obs.second.get(), kf->frameBlock().get(), 
						obs.first->frameBlock().get(), point->pointBlock().get());

					this->activeObservations.push_back(obs.second.get());
				}
			}
		}

		this->options.solverOptions.linear_solver_ordering.reset(ordering);
	}

	void CeresPhotometricBA::mergeOptimization(const std::vector<std::shared_ptr<Frame>>& active, std::vector<PhotometricResidual*>& obsToRemove) const
	{
		// merge all keyframes first
		for (const std::shared_ptr<Frame>& kf : active)
		{
			// pose and affine light
			kf->mergeOptimizationResult();
		}

		const std::shared_ptr<Frame>& lastKeyfame = active.back();
		const int lastIdx = lastKeyfame->keyframeID();

		// merge all points - THIS CAN BE PARALLELIZED - TODO
		for (const std::shared_ptr<Frame>& kf : active)
		{
			for (const auto& point : kf->activePoints())
			{
				const Sophus::SE3f& refToWorld = point->reference()->camToWorld();

				// inverse depth
				point->mergeOptimizationResult();

				// obtain the optimizatio hessian and 
				// maximum parallax between observations
				float iDepthHessian = 0.01f;
				float maxParallax = 1.f;

				const Eigen::Vector3f pt3d = point->pt3d();

				for (const auto& observation : point->observations())
				{
					const int obsIdx = observation.first->keyframeID();
					const Visibility state = observation.second->state();

					// visibility
					point->setVisibility(obsIdx, state);

					// remove or compute useful information based on good observations
					if (state == Visibility::OOB || state == Visibility::OUTLIER)
					{
						obsToRemove.push_back(observation.second.get());
					}
					else
					{
						const Sophus::SE3f refToTarget = observation.first->camToWorld().inverse() * refToWorld;

						// hessian
						iDepthHessian += (float)observation.second->iDepthHessian();

						// relative parallax
						const Eigen::Vector3f pt3d_cam = pt3d + refToTarget.rotationMatrix().transpose()*refToTarget.translation();
						const float dist = pt3d_cam.norm();

						const float parallax = pt3d_cam.dot(pt3d) / (dist*pt3d.norm());		// cosine

						if (parallax < maxParallax)		// if the cosine is lower, the parallax is bigger
						{
							maxParallax = parallax;
						}
					}
				}

				point->setIDepthHessian(iDepthHessian);
				point->setParallax(maxParallax);
			}
		}
	}

	void CeresPhotometricBA::removeBadObservations(const std::vector<PhotometricResidual*>& obsToRemove) const
	{
		for (int i = 0; i < obsToRemove.size(); ++i)
		{
			ActivePoint* const point = obsToRemove[i]->point();
			point->eraseObservation(obsToRemove[i]->targetFrame());
		}
	}

	void CeresPhotometricBA::freeFixedKeyframesMemory()
	{
		if (!Settings::getInstance().minimizeMemory) return;

		for (auto kf : this->fixedKeyframes)
		{
			kf->minimizeMemory();
		}
		this->fixedKeyframes.clear();
	}

	void CeresPhotometricBA::calcNewKeyframeErrorDist(int lvl, bool lastFit) const
	{
		const auto& settings = Settings::getInstance();

		// new keyframe
		Frame* const newAddedKeyframe = this->activeKeyframes.back();

		// compute all photometric observations
		std::vector<float> allObservations;
		for (const auto& obs : this->activeObservations)
		{
			// pick only observations in the new keyframe
			if (obs->targetFrame() != newAddedKeyframe)
			{
				continue;
			}

			// photometric errors for this observation
			Eigen::VecXf residuals;
			if (!obs->evaluate(lvl, residuals))
			{
				continue;
			}

			// insert into vector
			allObservations.insert(allObservations.end(), residuals.data(), residuals.data() + Pattern::size());
		}

		// this should not happen
		assert(!allObservations.empty() && "CeresPhotometricBA::calcNewKeyframeErrorDist has zero observations!!");

		// fit distribution parameters
		std::shared_ptr<IDistribution> distribution;
		if (settings.useTDistribution)
		{
			// discard outliers
			const float normalStdDev = 1.4826f*mad(allObservations);
			float maxEnergy = 3.f*normalStdDev;

			if (maxEnergy > settings.maxEnergyFit)
			{
				maxEnergy = settings.maxEnergyFit;
			}

			std::vector<float> allObservationsInlier;
			allObservationsInlier.reserve(allObservations.size());

			for (int i = 0; i < allObservations.size(); ++i)
			{
				if (fabs(allObservations[i]) > maxEnergy) continue;

				allObservationsInlier.push_back(allObservations[i]);
			}

			distribution = IDistribution::fitdist(allObservationsInlier, IDistribution::Type::TSTUDENT,
				Eigen::Vec3f(settings.nuFixed, settings.muFixed, settings.sigmaFixed));
		}
		else
		{
			distribution = IDistribution::fitdist(allObservations, IDistribution::Type::NORMAL,
				Eigen::Vec2f(settings.muFixed, settings.sigmaFixed));
		}

		newAddedKeyframe->setErrorDistribution(distribution);

		// save error distribution
		if (!lastFit && settings.saveOptErrorDist)
		{
			this->saveErrorDistribution(distribution, allObservations, newAddedKeyframe->keyframeID());
		}

		// draw error distribution
		if (this->outputWrapper && settings.debugShowOptErrorDistLast && lastFit)
		{
			// draw histogram
			cv::Mat hist = Utils::drawDistribution(allObservations, distribution,
												   Eigen::Vector2f(-255.f, 255.f), 
												   Eigen::Vector2f(0.f, 0.1f), 
												   Eigen::Vector2i(510, 510),
												   510);

			// visualize
			this->outputWrapper->publishOptErrorDistLast(hist);
		}		
	}

	void CeresPhotometricBA::saveErrorDistribution(const std::shared_ptr<IDistribution>& dist, 
												   const std::vector<float>& allObservations, 
												   int keyframeID) const
	{
		const auto& settings = Settings::getInstance();

		// check directory
		Utils::makeDir(settings.optErrorDistDir);
		
		std::string file = settings.optErrorDistDir + "/errorDist" +
						   std::to_string(keyframeID) + "_" +
						   std::to_string(this->stats->currentLvl) + ".txt";

		std::ofstream myFile;
		myFile.open(file);

		// fitted distribution
		auto tDist = std::dynamic_pointer_cast<TDistribution>(dist);
		if (tDist)
		{
			myFile << std::setprecision(7) << tDist->nu() << ", ";
			myFile << std::setprecision(7) << tDist->mu() << ", ";
			myFile << std::setprecision(7) << tDist->sigma() << "\n";

			// fit normal
			auto nDist = RobustNormalDistribution::fitdist(allObservations, Eigen::Vec2f(settings.muFixed, settings.sigmaFixed));
			myFile << std::setprecision(7) << nDist->mu() << ", ";
			myFile << std::setprecision(7) << nDist->sigma() << "\n";
		}

		auto nDist = std::dynamic_pointer_cast<RobustNormalDistribution>(dist);
		if (nDist)
		{
			// fit t-distribution
			auto tDist = TDistribution::fitdist(allObservations, Eigen::Vec3f(settings.nuFixed, settings.muFixed, settings.sigmaFixed));
			myFile << std::setprecision(7) << tDist->nu() << ", ";
			myFile << std::setprecision(7) << tDist->mu() << ", ";
			myFile << std::setprecision(7) << tDist->sigma() << "\n";

			myFile << std::setprecision(7) << nDist->mu() << ", ";
			myFile << std::setprecision(7) << nDist->sigma() << "\n";
		}

		// all the residuals
		for (int i = 0; i < allObservations.size(); ++i)
		{
			myFile << std::setprecision(9) << allObservations[i] << "\n";
		}

		myFile.close();
	}

	void CeresPhotometricBA::printStats()
	{
		const auto& settings = Settings::getInstance();

		const int width = 20;

		const std::string heading = "Least Squares Problem report";
		std::cout << std::endl << heading << std::endl;
		std::cout << std::string(std::min<int>((int)heading.size(), 78), '-') << std::endl;

		std::cout << std::right << std::setw(width) << "Residuals : ";
		for (int lvl = settings.optMaxLevel; lvl >= 0; --lvl)
		{
			std::cout << std::left << this->stats->numResiduals[lvl];

			if (lvl > 0)
			{
				std::cout << std::left << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << std::right << std::setw(width) << "Parameters : ";
		for (int lvl = settings.optMaxLevel; lvl >= 0; --lvl)
		{
			std::cout << std::left << this->stats->numParameters[lvl];

			if (lvl > 0)
			{
				std::cout << std::left << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << std::right << std::setw(width) << "Iterations : ";
		for (int lvl = settings.optMaxLevel; lvl >= 0; --lvl)
		{
			std::cout << std::left
				<< this->stats->iterations[lvl]
				<< " (" << this->stats->goodIterations[lvl] << ")";

			if (lvl > 0)
			{
				std::cout << std::left << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << std::right << std::setw(width) << "Initial RMSE : ";
		for (int lvl = settings.optMaxLevel; lvl >= 0; --lvl)
		{
			std::cout << std::right << std::setprecision(6) << this->stats->initError[lvl];
				
			if (lvl > 0)
			{
				std::cout << std::left << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << std::right << std::setw(width) << "Final RMSE : ";
		for (int lvl = settings.optMaxLevel; lvl >= 0; --lvl)
		{
			std::cout << std::right << std::setprecision(6) << this->stats->endError[lvl];

			if (lvl > 0)
			{
				std::cout << std::left << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << std::right << std::setw(width) << "Time : ";
		for (int lvl = settings.optMaxLevel; lvl >= 0; --lvl)
		{
			std::cout << std::right << std::setprecision(6) << this->stats->execTime[lvl];

			if (lvl > 0)
			{
				std::cout << std::left << ", ";
			}
		}
		std::cout << " [ms]" << std::endl;

		std::cout << std::right << std::setw(width) << "Termination : ";
		for (int lvl = settings.optMaxLevel; lvl >= 0; --lvl)
		{
			std::string termination = "";

			switch (this->stats->termination[lvl])
			{
			case ceres::CONVERGENCE:
				termination = "Convergence";
				break;
			case ceres::NO_CONVERGENCE:
				termination = "No convergence";
				break;
			case ceres::FAILURE:
				termination = "Failure";
				break;
			case ceres::USER_SUCCESS:
				termination = "User success";
				break;
			case ceres::USER_FAILURE:
				termination = "User failure";
				break;
			default:
				termination = "Unknown";
				break;
			}

			std::cout << std::right << termination;

			if (lvl > 0)
			{
				std::cout << std::left << ", ";
			}
		}
		std::cout << std::endl << std::endl;
	}
}