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

#include "LMCW.h"
#include "Log.h"
#include "DataStructures/Frame.h"
#include "DataStructures/CandidatePoint.h"
#include "DataStructures/ActivePoint.h"
#include "DataStructures/Pattern.h"
#include "DataStructures/CovisibilityGraph.h"
#include "Optimization/PhotometricResidual.h"
#include "FullSystem/DistanceTransform.h"
#include "Visualizer/IVisualizer.h"
#include "Utils/Projection.h"
#include "Utils/Settings.h"
#include "Utils/UtilFunctions.h"

namespace dsm
{
	LMCW::LMCW(int width, int height, IVisualizer *visualizer) :
		temporalWindowIndex(0), numActivePoints(0), outputWrapper_(visualizer)
	{
		const auto& settings = Settings::getInstance();
		const int levels = settings.pyramidLevels;

		// distance transformation
		this->distanceMap_ = std::make_unique<DistanceTransform>(width, height);
		this->minDistToActivate = settings.minDistToActivate;

		// covisibility graph
		this->covisibilityGraph_ = std::make_unique<CovisibilityGraph>();
	}

	LMCW::~LMCW()
	{}

	void LMCW::clear()
	{
		const auto& settings = Settings::getInstance();

		this->activeKeyframes_.clear();
		this->allKeyframes_.clear();

		this->temporalWindowIndex = 0;
		this->numActivePoints = 0;
		this->minDistToActivate = settings.minDistToActivate;

		this->covisibilityGraph_->clear();
	}

	void LMCW::insertNewKeyframe(const std::shared_ptr<Frame>& newKeyframe)
	{
		// convert to keyframe and activate
		newKeyframe->evolveToKeyframe();
		newKeyframe->activate();

		// insert into the LMCW
		newKeyframe->setKeyframeID((int)this->allKeyframes_.size());
		this->allKeyframes_.push_back(newKeyframe);
		newKeyframe->setActiveID((int)this->activeKeyframes_.size());
		this->activeKeyframes_.push_back(newKeyframe);

		// insert into covisibility graph
		this->covisibilityGraph_->addNode(newKeyframe);
	}

	void LMCW::selectWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA)
	{
		const auto& settings = Settings::getInstance();

		// temporal window
		this->selectTemporalWindow(photometricBA);

		// covisibility window
		if (!settings.doOnlyTemporalOpt)
		{
			this->selectCovisibleWindow(photometricBA);
		}
	}

	void LMCW::dropKeyframes()
	{
		const auto& settings = Settings::getInstance();

		// remove covisible keyframes
		if (!settings.doOnlyTemporalOpt)
		{
			for (int i = 0; i < this->temporalWindowIndex; ++i)
			{
				this->activeKeyframes_.front()->deactivate();
				this->activeKeyframes_.erase(this->activeKeyframes_.begin());
			}
			this->temporalWindowIndex = 0;
		}

		// drop from temporal keyframes
		for (auto it = this->activeKeyframes_.begin(); it != this->activeKeyframes_.end(); )
		{
			if ((*it)->flaggedToDrop())
			{
				(*it)->deactivate();
				(*it)->setFlaggedToDrop(false);

				// remove from keframe list
				(*it)->setActiveID(-1);
				it = this->activeKeyframes_.erase(it);
			}
			else
			{
				++it;
			}
		}

		// set active id again
		for (int i = 0; i < this->activeKeyframes_.size(); ++i)
		{
			this->activeKeyframes_[i]->setActiveID(i);
		}
	}

	void LMCW::selectTemporalWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA)
	{
		const auto& settings = Settings::getInstance();

		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
		const int w = calib.width(0);
		const int h = calib.height(0);

		const std::shared_ptr<Frame>& lastKeyframe = this->activeKeyframes_.back();
		const int lastKeyframeID = lastKeyframe->keyframeID();
		const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();

		this->numActivePoints = 0;

		int numFlaggedToDrop = 0;

		// prepare points for new active keyframes
		for (const std::shared_ptr<Frame>& kf : this->activeKeyframes_)
		{
			if (kf == lastKeyframe) continue;

			// relative pose
			const Sophus::SE3f relPose = worldToLast * kf->camToWorld();
			const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
			const Eigen::Vector3f Kt = K * relPose.translation();

			const AffineLight light = AffineLight::calcRelative(kf->affineLight(), lastKeyframe->affineLight());

			int numVisible = 0;

			for (const auto& point : kf->activePoints())
			{
				// project point to new image
				Eigen::Vector2f pt2d;
				if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
									w, h, KRKinv, Kt, pt2d))
				{
					point->setVisibility(lastKeyframeID, Visibility::OOB);
					continue;
				}

				// it is visible!
				point->setCenterProjection(pt2d);
				point->setVisibility(lastKeyframeID, Visibility::VISIBLE);

				// create new observation to the newest keyframe
				std::unique_ptr<PhotometricResidual> obs =
					std::make_unique<PhotometricResidual>(point, lastKeyframe, photometricBA);

				point->addObservation(lastKeyframe.get(), obs);

				// increase counters
				this->numActivePoints++;
				numVisible++;
			}

			// keep the last numAlwaysKeepKeyframes keyframes + the new one
			if ((kf->keyframeID() <= lastKeyframeID - settings.numAlwaysKeepKeyframes) &&
				(this->activeKeyframes_.size() - numFlaggedToDrop) >= settings.maxTemporalKeyframes)
			{
				const float numVisibleFloat = (float)numVisible;
				const float ratio = numVisibleFloat / kf->activePoints().size();

				// drop keyframes with low covisible points
				if (ratio < settings.minPointCovisible || fabs(light.alpha()) > settings.maxLightCovisible)
				{
					kf->setFlaggedToDrop(true);
					numFlaggedToDrop++;
				}
			}
		}

		// if still a lot of keyframes, drop one based on distance
		if ((this->activeKeyframes_.size() - numFlaggedToDrop) >= settings.maxTemporalKeyframes)
		{
			float maxScore = -std::numeric_limits<float>::max();
			int idx = -1;

			// keep the last N keyframes
			const int maxKeyframeID = (int)this->activeKeyframes_.size() - settings.numAlwaysKeepKeyframes;
			for (int i = 0; i < maxKeyframeID; ++i)
			{
				const Sophus::SE3f pose_i = this->activeKeyframes_[i]->camToWorld().inverse();

				// distance to other keyframes
				float score = 0.f;
				for (int j = 0; j < this->activeKeyframes_.size() - 1; ++j)
				{
					if (i == j) continue;

					const Sophus::SE3f relPose = pose_i * this->activeKeyframes_[j]->camToWorld();
					const float dist = relPose.translation().norm() + std::numeric_limits<float>::epsilon();
					score += 1.f / dist;
				}

				// distance to the latest keyframe
				const Sophus::SE3f relPose = pose_i * lastKeyframe->camToWorld();
				score *= sqrtf(relPose.translation().norm());

				if (score > maxScore)
				{
					maxScore = score;
					idx = i;
				}
			}

			if (idx >= 0)
			{
				this->activeKeyframes_[idx]->setFlaggedToDrop(true);
				numFlaggedToDrop++;
			}
		}

		if (settings.debugPrintLog && settings.debugLogActivePoints)
		{
			const std::string msg = "Act. Temporal: " + std::to_string(this->numActivePoints) + "\t";

			auto& log = Log::getInstance();
			log.addCurrentLog(lastKeyframe->frameID(), msg);
		}

		// distance map
		this->distanceMap_->compute(this->activeKeyframes_, lastKeyframe);

		if (settings.debugPrintLog && settings.debugLogDistanceMap)
		{
			const std::string msg = "DT: " + std::to_string(this->distanceMap_->getNumObstacles()) + "\t";

			auto& log = Log::getInstance();
			log.addCurrentLog(lastKeyframe->frameID(), msg);
		}

		if (settings.debugShowDistanceTransformBefore &&
			settings.doOnlyTemporalOpt &&
			this->outputWrapper_)
		{
			cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
			this->outputWrapper_->publishDistanceTransformBefore(distTransform);
		}
	}

	void LMCW::selectCovisibleWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA)
	{
		assert(this->temporalWindowIndex == 0);

		const auto& settings = Settings::getInstance();
		const auto& calib = GlobalCalibration::getInstance();

		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const int w = calib.width(0);
		const int h = calib.height(0);

		const std::shared_ptr<Frame> lastActKeyframe = this->activeKeyframes_.back();		// make a copy, required!
		const Sophus::SE3f worldToLast = lastActKeyframe->camToWorld().inverse();
		const int lastActID = lastActKeyframe->keyframeID();
		const int firstActID = this->activeKeyframes_.front()->keyframeID();

		// minimum distance based on pattern padding
		const float padding = (float)Pattern::padding();
		const float minDist = padding * padding; // squared dist

		std::vector<std::shared_ptr<Frame>> allCovisibleKeyframes;
		std::vector<int> numVisiblePoints;
		std::vector<int> numTotalVisiblePoints;

		// select some covisible keyframes
		for (const auto& kf : this->allKeyframes_)
		{
			// break if we have already reached the temporal part
			if (kf->keyframeID() >= firstActID) break;

			// relative pose
			const Sophus::SE3f kfToLast = worldToLast * kf->camToWorld();
			const Eigen::Matrix3f R = kfToLast.rotationMatrix();
			const Eigen::Vector3f t = kfToLast.translation();

			int numVisible = 0;
			int numTotalVisible = 0;

			for (const auto& point : kf->activePoints())
			{
				// project point to new image
				Eigen::Vector2f pt2d;
				if (!Utils::projectAndCheck(point->u(0), point->v(0), point->iDepth(),
											K, w, h, R, t, pt2d))
				{
					point->setVisibility(lastActID, Visibility::OOB);
					continue;
				}

				// activate point
				point->setCenterProjection(pt2d);
				point->setVisibility(lastActID, Visibility::VISIBLE);

				numTotalVisible++;

				// check distance
				int x = static_cast<int>(pt2d[0] + 0.5f);
				int y = static_cast<int>(pt2d[1] + 0.5f);

				if (this->distanceMap_->dist(x, y) < minDist) continue;

				// increase counter
				numVisible++;
			}

			float numVisibleFloat = (float)numVisible;
			float ratio = numVisibleFloat / kf->activePoints().size();

			if (ratio > settings.minPointCovisible)
			{
				allCovisibleKeyframes.push_back(kf);
				numVisiblePoints.push_back(numVisible);
				numTotalVisiblePoints.push_back(numTotalVisible);
			}
		}

		int numCovisibleKeyframes = (int)allCovisibleKeyframes.size();

		while (this->temporalWindowIndex < settings.maxCovisibleKeyframes)
		{
			if (numCovisibleKeyframes == 0) break;

			// select the keyframes with most visible points
			const auto maxElement = std::max_element(numVisiblePoints.begin(), numVisiblePoints.end());
			const int position = (int)std::distance(numVisiblePoints.begin(), maxElement);
			
			// pick the one with highest number of visible points
			const std::shared_ptr<Frame>& selectedKeyframe = allCovisibleKeyframes[position];
			selectedKeyframe->activate();
			this->activeKeyframes_.insert(this->activeKeyframes_.begin(), selectedKeyframe);
			this->numActivePoints += numTotalVisiblePoints[position];
			this->temporalWindowIndex++;
			
			// update map
			this->distanceMap_->add(selectedKeyframe, lastActKeyframe);
			
			// remove from vectors
			allCovisibleKeyframes[position] = allCovisibleKeyframes.back();
			allCovisibleKeyframes.pop_back();
			numVisiblePoints[position] = numVisiblePoints.back();
			numVisiblePoints.pop_back();
			numTotalVisiblePoints[position] = numTotalVisiblePoints.back();
			numTotalVisiblePoints.pop_back();
			numCovisibleKeyframes--;

			// early exit
			if (this->temporalWindowIndex == settings.maxCovisibleKeyframes) break;

			// update num visible
			for (int i = 0; i < numCovisibleKeyframes; ++i)
			{
				numVisiblePoints[i] = 0;

				const auto& kf = allCovisibleKeyframes[i];
				for (auto& pt : kf->activePoints())
				{
					if (pt->visibility(lastActID) == Visibility::VISIBLE)
					{
						// check distance
						const Eigen::Vec2f& pt2d = pt->centerProjection();
						int x = static_cast<int>(pt2d[0] + 0.5f);
						int y = static_cast<int>(pt2d[1] + 0.5f);

						if (this->distanceMap_->dist(x, y) < minDist) continue;

						numVisiblePoints[i]++;
					}
				}
			}
		}

		if (this->temporalWindowIndex > 0)
		{
			// create new observations to latest keyframe
			for (int i = 0; i < this->temporalWindowIndex; ++i)
			{
				for (const auto& point : this->activeKeyframes_[i]->activePoints())
				{
					if (point->visibility(lastActID) == Visibility::VISIBLE)
					{
						std::unique_ptr<PhotometricResidual> obs =
							std::make_unique<PhotometricResidual>(point, lastActKeyframe, photometricBA);

						// add residual
						point->addObservation(lastActKeyframe.get(), obs);
					}
				}
			}

			// set active id again
			for (int i = 0; i < this->activeKeyframes_.size(); ++i)
			{
				this->activeKeyframes_[i]->setActiveID(i);
			}
		}

		if (settings.debugShowDistanceTransformBefore &&
			!settings.doOnlyTemporalOpt &&
			this->outputWrapper_)
		{
			cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
			this->outputWrapper_->publishDistanceTransformBefore(distTransform);
		}
	}

	void LMCW::activatePoints(const std::unique_ptr<CeresPhotometricBA>& photometricBA)
	{
		const auto& settings = Settings::getInstance();
		const auto& calib = GlobalCalibration::getInstance();

		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
		const int width = (int)calib.width(0);
		const int height = (int)calib.height(0);

		// change heuristically the minimum distance
		const float ratio = (float)this->numActivePoints / settings.numActivePoints;

		if (ratio > 1.7f) this->minDistToActivate = this->minDistToActivate + 3;
		else if (ratio > 1.4f) this->minDistToActivate = this->minDistToActivate + 2;
		else if (ratio > 1.1f) this->minDistToActivate = this->minDistToActivate + 1;
		
		if (ratio < 0.3f) this->minDistToActivate = this->minDistToActivate - 3;
		else if (ratio < 0.6f) this->minDistToActivate = this->minDistToActivate - 2;
		else if (ratio < 0.9f) this->minDistToActivate = this->minDistToActivate - 1;

		this->minDistToActivate = std::max(this->minDistToActivate, 1);
		this->minDistToActivate = std::min(this->minDistToActivate, 2*Pattern::width());

		const float minDist = (float)this->minDistToActivate*this->minDistToActivate; // squared dist

		const std::shared_ptr<Frame>& lastKeyframe = this->activeKeyframes_.back();
		const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();

		// select active points from candidates
		int numPointsCreated = 0;

		Utils::Time t1 = std::chrono::steady_clock::now();
		int numActiveKeyframes = (int)this->activeKeyframes_.size();

		for (int i = this->temporalWindowIndex; i < numActiveKeyframes - 1; ++i)
		{
			const std::shared_ptr<Frame>& owner = this->activeKeyframes_[i];

			// relative pose
			const Sophus::SE3f ownerToLast = worldToLast * owner->camToWorld();	
			const Eigen::Matrix3f KRKinv = K * ownerToLast.rotationMatrix() * Kinv;
			const Eigen::Vector3f Kt = K * ownerToLast.translation();

			auto& candidates = owner->candidates();
			auto& activePoints = owner->activePoints();

			for (auto& cand : candidates)
			{
				CandidatePoint::PointStatus status = cand->status();

				// check status
				if (status == CandidatePoint::OPTIMIZED)
				{
					// project into new keyframe
					Eigen::Vector2f pointInFrame;
					if (!Utils::project(cand->u0(), cand->v0(), cand->iDepth(),
						width, height, KRKinv, Kt, pointInFrame))
					{
						cand = nullptr;
						continue;
					}

					int x = static_cast<int>(pointInFrame[0] + 0.5f);
					int y = static_cast<int>(pointInFrame[1] + 0.5f);

					// check squared distance
					float level = ((float)cand->detectedLevel() + 1.f);
					if (this->distanceMap_->dist(x, y) < (minDist * level * level)) continue;

					// activePoint from candidate
					std::unique_ptr<ActivePoint> point = std::make_unique<ActivePoint>(lastKeyframe->keyframeID(), cand);

					// observations & visibility
					for (const std::shared_ptr<Frame>& frame : this->activeKeyframes_)
					{
						if (frame == owner) continue;

						Visibility vis = cand->visibility(frame->activeID());

						// set visibility
						point->setVisibility(frame->keyframeID(), vis);

						if (vis == Visibility::VISIBLE)
						{
							// create new observation
							std::unique_ptr<PhotometricResidual> obs =
								std::make_unique<PhotometricResidual>(point, frame, photometricBA);

							point->addObservation(frame.get(), obs);
						}
					}

					cand = nullptr;						// delete candidate after activation

					point->setCenterProjection(pointInFrame);

					numPointsCreated++;
					this->numActivePoints++;

					// update distance map
					this->distanceMap_->add(point, lastKeyframe);

					// insert into list
					activePoints.push_back(std::move(point));
				}
				else if (status == CandidatePoint::OUTLIER)
				{
					cand = nullptr;
					continue;
				}
			}

			// reorder candidates filling the gaps
			for (int j = 0; j < candidates.size(); ++j)
			{
				if (candidates[j] == nullptr)
				{
					candidates[j] = std::move(candidates.back());
					candidates.pop_back();
					j--;						// go back again to check if last one was nullptr too
				}
			}
		}
		Utils::Time t2 = std::chrono::steady_clock::now();

		if (settings.debugPrintLog && settings.debugLogActivePoints)
		{
			const std::string msg = "Act. New: " + std::to_string(numPointsCreated) + "\t";

			auto& log = Log::getInstance();
			log.addCurrentLog(lastKeyframe->frameID(), msg);
		}

		if (settings.debugShowDistanceTransformAfter && this->outputWrapper_)
		{
			cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
			this->outputWrapper_->publishDistanceTransformAfter(distTransform);
		}
	}

	void LMCW::removeOutliers() const
	{
		const auto& settings = Settings::getInstance();

		const int lastKeyframeID = this->activeKeyframes_.back()->keyframeID();

		// remove outlier points
		for (const std::shared_ptr<Frame>& keyframe : this->activeKeyframes_)
		{
			auto& activePoints = keyframe->activePoints();

			for (int i = 0; i < activePoints.size(); ++i)
			{
				auto& point = activePoints[i];

				bool remove = false;

				// check if the point is new
				const int32_t age = point->age(lastKeyframeID);
				if (age < settings.minNumKFToConsiderNew && !keyframe->flaggedToDrop())
				{
					// for new points check that have been observed in all new keyframes
					if (point->numObservations() < (age + 1))
					{
						remove = true;
					}
				}
				else
				{
					// for old points check that have been observed at least in "minNumGoodObservations" keyframes
					if (point->numObservations() < settings.minNumGoodObservations)
					{
						remove = true;
					}
				}

				if (remove)
				{
					activePoints[i] = std::move(activePoints.back());
					activePoints.pop_back();
					i--;
				}
			}
		}
	}

	void LMCW::updateConnectivity() const
	{
		const int numActiveKeyframes = (int)this->activeKeyframes_.size();

		Eigen::MatrixXi adj;
		adj.resize(numActiveKeyframes, numActiveKeyframes);
		adj.setZero();

		// update connectivity of active keyframes
		for (int i = 0; i < this->activeKeyframes_.size(); ++i)
		{
			const std::shared_ptr<Frame>& keyframe = this->activeKeyframes_[i];

			for (int j = 0; j < this->activeKeyframes_.size(); ++j)
			{
				if (i == j) continue;

				int idx = this->activeKeyframes_[j]->keyframeID();

				int numVisible = 0;

				// look for all active point residuals
				const auto& activePoints = keyframe->activePoints();
				for (const auto& point : activePoints)
				{
					if (point->visibility(idx) == Visibility::VISIBLE)
					{
						numVisible++;
					}
				}

				adj(i, j) = numVisible;
			}
		}

		for (int i = 0; i < numActiveKeyframes; ++i)
		{
			for (int j = i + 1; j < numActiveKeyframes; ++j)
			{
				int totalVisible = adj(i, j) + adj(j, i);
				this->covisibilityGraph_->connect(this->activeKeyframes_[i], this->activeKeyframes_[j], totalVisible);
			}
		}

		if (this->outputWrapper_)
		{
			this->outputWrapper_->publishCovisibility(this->covisibilityGraph_->adjacencyMatrix());
		}
	}
}