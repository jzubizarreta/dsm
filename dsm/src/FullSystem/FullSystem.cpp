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

#include "FullSystem.h"
#include "PointDetector.h"
#include "DistanceTransform.h"
#include "Log.h"
#include "LMCW.h"
#include "Initializer/MonoInitializer.h"
#include "Utils/Settings.h"
#include "Utils/UtilFunctions.h"
#include "Utils/Projection.h"
#include "Utils/Interpolation.h"
#include "Utils/GlobalCalibration.h"
#include "DataStructures/Frame.h"
#include "DataStructures/CandidatePoint.h"
#include "DataStructures/ActivePoint.h"
#include "Memory/Buffer.h"
#include "Memory/BufferPool.h"
#include "Thread/WorkerThreadPool.h"
#include "Tracking/FrameTracker.h"
#include "Tracking/FrameTrackerReference.h"
#include "Optimization/PhotometricBA.h"
#include "Optimization/PhotometricResidual.h"
#include "Visualizer/IVisualizer.h"

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <fstream>
#include <time.h>

namespace dsm
{
	FullSystem::FullSystem(int w, int h, const Eigen::Matrix3f &calib,
		const std::string &settingsFile,
		IVisualizer *outputWrapper) :
		initialized(false),
		trackingIsGood(true),
		createNewKeyframe(false),
		createNewKeyframeID(-1),
		numMappedFramesFromLastKF(0),
		shouldStop(false),
		newFrameMappedDone(true),
		outputWrapper(outputWrapper)
	{
		// First: initialize pattern types
		Pattern::initialize();

		// Second: read settings from file
		auto& settings = Settings::getInstance();
		if (!settingsFile.empty())
		{
			std::cout << "Reading settings...";
			if (settings.fromFile(settingsFile))
			{
				std::cout << "succesfully!" << std::endl;
			}
			else
			{
				std::cout << "failed!" << std::endl;
			}
		}

		// Third: set global calibration
		auto& globalCalib = GlobalCalibration::getInstance();
		globalCalib.setCalibration(w, h, calib, settings.pyramidLevels);

		const int width = (int)globalCalib.width(0);
		const int height = (int)globalCalib.height(0);

		// thread pool
		this->threadPool = std::make_shared<WorkerThreadPool>(settings.mappingThreads);

		// initializer
		this->initializer = std::make_unique<MonoInitializer>(this->threadPool, this->outputWrapper);

		// pixel detector
		this->pointDetector = std::make_unique<PointDetector>(width, height, (int)settings.pointDetectionLevels,
															  (int)settings.numBlocksPerDimension, settings.minGradAdd,
															  this->threadPool);

		this->pixelMask = (int32_t*)Eigen::internal::aligned_malloc(width * height * sizeof(int32_t));

		// Fourth: initialize member variables
		this->tracker = std::make_unique<FrameTracker>(width, height);
		this->trackingReference = std::make_shared<FrameTrackerReference>(width, height);
		this->newTrackingReference = std::make_shared<FrameTrackerReference>(width, height);
		this->trackingReferenceUpdated = false;

		// optimization window
		this->lmcw = std::make_unique<LMCW>(width, height, this->outputWrapper);

		// optimizer
		PhotometricBAConfig config;
		this->ceresOptimizer = std::make_unique<CeresPhotometricBA>(config, this->outputWrapper);

		// initial tracking priors
		this->lastTrackedMotion = Sophus::SE3f();
		this->lastTrackedFrame = nullptr;

		this->lastTrackedResidual = std::numeric_limits<float>::max();

		this->lastWasKF = false;

		// depth map save counter
		this->saveID = 0;

		// depth map visualization image
		this->depthMapImage = cv::Mat(h >> settings.showDepthMapLvl, w >> settings.showDepthMapLvl, CV_8UC3);
		this->minIDepthTracker = -1.f;
		this->maxIDepthTracker = -1.f;
		this->minIDepthOpt = -1.f;
		this->maxIDepthOpt = -1.f;

		// start mapping thread
		if (!settings.singleThreaded)
		{
			this->mappingThread = std::make_unique<std::thread>(&FullSystem::mappingThreadLoop, this);
		}
	}

	FullSystem::~FullSystem()
	{
		const auto& settings = Settings::getInstance();

		// make sure none is waiting for something
		std::cout << "Waiting for DSM to finish.." << std::endl;

		// wait all frames to be processed
		if (!settings.singleThreaded)
		{
			this->waitUntilMappingFinished();

			// send a signal to finish!
			{
				std::lock_guard<std::mutex> lock(this->unmappedTrackedFramesMutex);
				this->shouldStop = true;
			}
			this->unmappedTrackedFramesSignal.notify_all();

			// wait the thread to exit
			if (this->mappingThread->joinable())
			{
				this->mappingThread->join();
			}
		}

		std::cout << " .. DSM has finished." << std::endl;

		// delete main classes in order
		if (this->initializer) this->initializer = nullptr;
		if (this->pointDetector) this->pointDetector = nullptr;
		if (this->tracker) this->tracker = nullptr;
		if (this->trackingReference) this->trackingReference = nullptr;
		if (this->newTrackingReference) this->newTrackingReference = nullptr;
		if (this->ceresOptimizer) this->ceresOptimizer = nullptr;
		if (this->lastTrackedFrame) lastTrackedFrame = nullptr;
		if (this->threadPool) this->threadPool = nullptr;
		if (this->lmcw) this->lmcw = nullptr;

		// clear keyframes and return all buffers to their pools
		// make sure std::shared_ptr<Frame> live only in this class
		this->unmappedTrackedFrames.clear();

		// delete mask for pixel detector
		Eigen::internal::aligned_free(this->pixelMask);

		// delete buffers
		// guarantee that all buffers have been returned to the pool
		BufferPool<bool>::getInstance().clearPool();				// bool pool
		BufferPool<char>::getInstance().clearPool();				// char pool
		BufferPool<unsigned char>::getInstance().clearPool();		// unsigned char pool
		BufferPool<float>::getInstance().clearPool();				// float pool
	}

	bool FullSystem::isInitialized() const
	{
		return this->initialized;
	}

	bool FullSystem::isLost() const
	{
		return !this->trackingIsGood;
	}

	void FullSystem::getTrajectory(std::vector<Eigen::Matrix4f> &poses, std::vector<double> &timestamps) const
	{
		const auto& allKeyframes = this->lmcw->allKeyframes();

		poses.clear();
		timestamps.clear();
		poses.reserve(allKeyframes.size());
		timestamps.reserve(allKeyframes.size());

		for (const auto& kf : allKeyframes)
		{
			poses.push_back(kf->camToWorld().matrix());
			timestamps.push_back(kf->timestamp());
		}
	}

	void FullSystem::getStructure(std::vector<Eigen::Vector3f>& structure) const
	{
		structure.clear();

		const auto& calib = GlobalCalibration::getInstance();
		const auto& K = calib.matrix3f(0);

		const auto& allKeyframes = this->lmcw->allKeyframes();

		for (const auto& kf : allKeyframes)
		{
			const auto& pose = kf->camToWorld();

			for (const auto& pt : kf->activePoints())
			{
				const float u = pt->u(0);
				const float v = pt->v(0);
				const float depth = 1.f / pt->iDepth();

				if (depth <= 0.f) continue;

				for (int idx = 0; idx < Pattern::size(); ++idx)
				{
					float uj = u + (float)Pattern::at(idx, 0);
					float vj = v + (float)Pattern::at(idx, 1);

					// point in camera
					Eigen::Vector3f pt = depth*Eigen::Vector3f((uj - K(0, 2)) / K(0, 0),
															   (vj - K(1, 2)) / K(1, 1),
															   1.f);

					// point in world
					pt = pose * pt;

					// save
					structure.push_back(pt);
				}
			}
		}
	}

	float FullSystem::getCamTrackingMeanTime() const
	{
		float time = 0.f;
		if (this->camTrackingTime.size() > 0)
		{
			for (int i = 0; i < this->camTrackingTime.size(); ++i)
			{
				time += this->camTrackingTime[i];
			}
			time /= this->camTrackingTime.size();
		}
		return time;
	}

	float FullSystem::getPointTrackingMeanTime() const
	{
		float time = 0.f;
		if (this->pointTrackingTime.size() > 0)
		{
			for (int i = 0; i < this->pointTrackingTime.size(); ++i)
			{
				time += this->pointTrackingTime[i];
			}
			time /= this->pointTrackingTime.size();
		}
		return time;
	}

	float FullSystem::getLocalBAMeanTime() const
	{
		float time = 0.f;
		if (this->localBATime.size() > 0)
		{
			for (int i = 0; i < this->localBATime.size(); ++i)
			{
				time += this->localBATime[i];
			}
			time /= this->localBATime.size();
		}
		return time;
	}

	int FullSystem::getNumPoints() const
	{
		const auto& allKeyframes = this->lmcw->allKeyframes();

		int num = 0;
		for (const auto& kf : allKeyframes)
		{
			num += (int)kf->activePoints().size();
		}
		return num;
	}

	int FullSystem::getNumKeyframes() const
	{
		return (int)this->lmcw->allKeyframes().size();
	}

	bool FullSystem::getLastWasKF() const
	{
		return this->lastWasKF;
	}

	bool FullSystem::initialize(const std::shared_ptr<Frame>& frame)
	{
		if (this->initialized) return true;

		Utils::Time t1 = std::chrono::steady_clock::now();

		const auto& allKeyframes = this->lmcw->allKeyframes();

		if (allKeyframes.empty())
		{
			// insert frame as first keyframe
			frame->setTrackingResult(nullptr, Sophus::SE3f(), AffineLight());
			this->lmcw->insertNewKeyframe(frame);

			// create candidates
			this->createCandidates(frame);

			// set as initializer reference
			this->initializer->setReference(frame);
		}
		else
		{
			// try to initialize
			Sophus::SE3f firstToSecond;
			if (this->initializer->initialize(frame, firstToSecond))
			{
				// rescale to norm(t) = 0.1m
				firstToSecond.translation() /= firstToSecond.translation().norm();
				firstToSecond.translation() *= 0.1f;

				// set initialization pose as tracking result
				const auto& firstKF = allKeyframes[0];
				frame->setTrackingResult(firstKF.get(), firstToSecond.inverse(), AffineLight());

				// initialize some values: motion, flags
				this->lastTrackedFrame = frame;
				this->lastTrackedMotion = Sophus::SE3f();

				Utils::Time t2 = std::chrono::steady_clock::now();
				std::cout << "Done initialization in " << Utils::elapsedTime(t1, t2) << "ms" << std::endl;

				// insert frame as keyframe and optimize
				this->createKeyframeAndOptimize(frame);

				return true;
			}
		}

		// reset if it cannot initialize and start again
		if (this->initializer->isResetRequired())
		{
			this->initialized = false;

			this->lastTrackedFrame = nullptr;

			this->lmcw->clear();

			this->initializer->reset();
		}

		Utils::Time t2 = std::chrono::steady_clock::now();
		std::cout << "Trying to initialize... " << Utils::elapsedTime(t1, t2) << "ms" << std::endl;

		return false;
	}

	void FullSystem::trackFrame(int id, double timestamp, unsigned char* image)
	{
		this->lastWasKF = false;

		auto& settings = Settings::getInstance();

		// track frame
		Utils::Time t1 = std::chrono::steady_clock::now();

		// Create new frame
		std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, image);

		if (settings.debugPrintLog)
		{
			auto& log = Log::getInstance();
			log.addNewLog(trackingNewFrame->frameID());
		}

		if (!this->trackingIsGood)
		{
			std::cout << "LOST..." << std::endl;
			return;
		}

		// initialization
		if (!this->initialized)
		{
			this->initialized = this->initialize(trackingNewFrame);
			return;
		}

		// track
		this->trackNewFrame(trackingNewFrame);

		if (!this->trackingIsGood)
		{
			std::cout << "Tracking LOST!!" << std::endl;
			return;
		}

		if (settings.debugPrintLog && settings.debugLogTracking)
		{
			const float* pose = trackingNewFrame->thisToParentPose().data();
			const AffineLight& light = trackingNewFrame->thisToParentLight();
			const float energy = this->tracker->totalResidual();

			std::string msg = "trackingPose: " + std::to_string(pose[0]) + "\t" + std::to_string(pose[1]) + "\t" + std::to_string(pose[2]) + "\t" + std::to_string(pose[3]) + "\t"
											   + std::to_string(pose[4]) + "\t" + std::to_string(pose[5]) + "\t" + std::to_string(pose[6]) + "\t";
			msg += "affineLight: " + std::to_string(light.alpha()) + "\t" + std::to_string(light.beta()) + "\t";
			msg += "cost: " + std::to_string(energy) + "\t";

			auto& log = Log::getInstance();
			log.addCurrentLog(trackingNewFrame->frameID(), msg);
		}

		// Keyframe selection
		// If we have already asked to create one, dont do it again
		bool localCreateNewKeyframe = false;
		if (!this->createNewKeyframe)
		{
			if(this->numMappedFramesFromLastKF >= settings.minNumMappedFramesToCreateKF)
			{
				localCreateNewKeyframe = this->isNewKeyframeRequired(trackingNewFrame);
			}
						
			this->createNewKeyframe = localCreateNewKeyframe;
		}

		// insert current frame to unmapped queue
		Utils::Time t2;
		if (!settings.singleThreaded)
		{
			{
				std::lock_guard<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
				this->unmappedTrackedFrames.push_back(trackingNewFrame);

				if (localCreateNewKeyframe)
				{
					this->createNewKeyframeID = trackingNewFrame->frameID();
				}

				// control flag for blocking
				{
					std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
					this->newFrameMappedDone = false;
				}
			}
			this->unmappedTrackedFramesSignal.notify_one();

			t2 = std::chrono::steady_clock::now();
		}
		else
		{
			if (localCreateNewKeyframe)
			{
				this->createNewKeyframeID = trackingNewFrame->frameID();
			}

			t2 = std::chrono::steady_clock::now();

			this->doMapping(trackingNewFrame);
		}

		const float time = Utils::elapsedTime(t1, t2);
		this->camTrackingTime.push_back(time);

		if (this->outputWrapper)
		{
			// current camera pose
			const Eigen::Matrix4f camPose = (this->lastTrackedFrame->parent()->camToWorld() * 
											 this->lastTrackedFrame->thisToParentPose()).matrix();
			this->outputWrapper->publishCurrentFrame(camPose);

			//timings			
			this->outputWrapper->publishCamTrackingTime(time);
		}

		// implement blocking
		// required for debugging
		if (!settings.singleThreaded && settings.blockUntilMapped && this->trackingIsGood)
		{
			this->waitUntilMappingFinished();
		}
	}

	void FullSystem::trackNewFrame(const std::shared_ptr<Frame>& frame)
	{
		Utils::Time t1 = std::chrono::steady_clock::now();

		// select tracking reference keyframe
		Sophus::SE3f lastToWorldPose = this->lastTrackedFrame->parent()->camToWorld() *
									   this->lastTrackedFrame->thisToParentPose();

		AffineLight lastToWorldLight = AffineLight::calcGlobal(this->lastTrackedFrame->parent()->affineLight(),
															   this->lastTrackedFrame->thisToParentLight());

		if (this->trackingReferenceUpdated)
		{
			std::lock_guard<std::mutex> lock(this->trackingReferenceMutex);
			std::swap(this->trackingReference, this->newTrackingReference);
			this->createNewKeyframe = false;
			this->trackingReferenceUpdated = false;

			Frame* const newReference = this->trackingReference->reference();
			Frame* const oldReference = this->newTrackingReference->reference();

			if (newReference && oldReference)
			{
				// correct last frame pose with PBA result
				// oldReference = parent of newReference

				// pose
				const Sophus::SE3f poseCorrection = newReference->camToWorld().inverse() * 
													oldReference->camToWorld() * 
													newReference->thisToParentPose();

				lastToWorldPose *= poseCorrection.inverse();

				// light
				const AffineLight newRefOldLight = AffineLight::calcGlobal(newReference->parent()->affineLight(),
																		   newReference->thisToParentLight());
				const AffineLight lightCorrection = AffineLight::calcRelative(newRefOldLight,
																		      newReference->affineLight());
				lastToWorldLight = AffineLight::calcGlobal(lastToWorldLight,
														   lightCorrection);
			}
		}

		Frame* const reference = this->trackingReference->reference();

		// pose prior -> constant velocity model
		const Sophus::SE3f lastToRef = reference->camToWorld().inverse() * lastToWorldPose;
		Sophus::SE3f frameToRefPose = lastToRef * this->lastTrackedMotion;

		// affine light prior
		AffineLight frameToRefLight = AffineLight::calcRelative(reference->affineLight(),
																lastToWorldLight);

		// Error distribution
		std::shared_ptr<IDistribution> errorDistribution;

		// Track
		// relative pose to reference keyframe
		// relative affine light to reference keyframe
		bool goodTracked = this->tracker->trackFrame(this->trackingReference, frame, frameToRefPose, frameToRefLight,
													 errorDistribution, this->outputWrapper);

		//tracking lost? - reset tracking internal data
		if (!goodTracked)
		{
			this->trackingIsGood = false;

			this->lastTrackedFrame = nullptr;
			this->lastTrackedMotion = Sophus::SE3f();

			this->trackingReference->reset();
			this->tracker->reset();

			this->unmappedTrackedFramesSignal.notify_one();

			return;
		}

		// save result
		frame->setTrackingResult(reference, frameToRefPose, frameToRefLight);

		// error distribution
		frame->setErrorDistribution(errorDistribution);

		// residuals per level
		this->lastTrackedResidual = this->tracker->totalResidual();

		if (this->trackingReference->firstFrameResidual() < 0.f)
		{
			this->trackingReference->setFirstFrameResidual(this->lastTrackedResidual);
		}

		// save info for tracking priors
		this->lastTrackedMotion = lastToRef.inverse() * frameToRefPose;
		this->lastTrackedFrame = frame;

		Utils::Time t2 = std::chrono::steady_clock::now();
		//std::cout << "Tracking: " << Utils::elapsedTime(t1, t2) << "\n";
	}

	bool FullSystem::isNewKeyframeRequired(const std::shared_ptr<Frame>& frame) const
	{
		const auto& settings = Settings::getInstance();

		Frame* const reference = this->trackingReference->reference();

		// check camera translation relative to mean inverse depth
		const Eigen::Vector3f dist = frame->thisToParentPose().translation() * 
									 this->trackingReference->meanIDepth();

		// check point usage by tracker 
		const float pointUsage = this->tracker->pointUsage();

		// check illumination change
		const AffineLight& relativeLight = frame->thisToParentLight();

		// new keyframe required?
		if ((dist.norm()*settings.newKFDistWeight +
			(1.f - pointUsage) * settings.newKFUsageWeight +
			fabs(relativeLight.alpha()) * settings.newKFAffineWeight) > 1.f ||
			settings.newKFResidualWeight*this->trackingReference->firstFrameResidual() < this->lastTrackedResidual)
		{
			return true;
		}

		return false;
	}

	void FullSystem::mappingThreadLoop()
	{
		std::cout << "Started mapping thread!" << std::endl;

		const auto& settings = Settings::getInstance();

		Utils::Time t1, t2;

		while (true)
		{
			std::shared_ptr<Frame> frame;

			// waiting condition. It will wake up when a new frame is set in the queue 
			// or when shutting down
			{
				std::unique_lock<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
				if (this->unmappedTrackedFrames.empty())
				{
					// signal to stop blocking
					{
						std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
						this->newFrameMappedDone = true;
						//std::cout << "wake up!" << std::endl;
					}
					this->newFrameMappedSignal.notify_all();

					this->unmappedTrackedFramesSignal.wait(
						unMappedLock, [&]() {return this->shouldStop || !this->unmappedTrackedFrames.empty(); }
					);

					if (this->shouldStop)
					{
						return;
					}
				}	

				t1 = std::chrono::steady_clock::now();

				// take a new frame from the front
				frame = std::move(this->unmappedTrackedFrames.front());
				this->unmappedTrackedFrames.pop_front();

				// some checkings
				if (!this->unmappedTrackedFrames.empty())
				{
					const auto& activeKeyframes = this->lmcw->activeWindow();

					// if new keyframe required, discard all the frames until the need was requested
					if (this->createNewKeyframeID > activeKeyframes.back()->frameID())
					{
						while (this->unmappedTrackedFrames.size() > 0 &&
							   this->unmappedTrackedFrames.front()->frameID() <= this->createNewKeyframeID)
						{
							frame = std::move(this->unmappedTrackedFrames.front());
							this->unmappedTrackedFrames.pop_front();
						}
					}
				
					// check if mapping thread is running too slow
					// and the number of unmapped frames is growing
					if (this->unmappedTrackedFrames.size() > settings.maxUnmappedFrames)
					{
						std::cout << "Unmapped frames is growing.." << std::endl;
				
						// then, skip the following frame
						this->unmappedTrackedFrames.pop_front();
					}
				}				
			}

			// do mapping stuff
			this->doMapping(frame);
		}
		std::cout << "Finished mapping thread!" << std::endl;
	}

	void FullSystem::doMapping(const std::shared_ptr<Frame>& frame)
	{
		assert(frame != nullptr);

		Utils::Time t1 = std::chrono::steady_clock::now();

		const auto& activeKeyframes = this->lmcw->activeWindow();

		// do mapping stuff
		if (this->createNewKeyframeID > activeKeyframes.back()->frameID())
		{
			// create new keyframe and optimize
			this->createKeyframeAndOptimize(frame);
			this->numMappedFramesFromLastKF = 0;

			Utils::Time t2 = std::chrono::steady_clock::now();
			const float time = Utils::elapsedTime(t1, t2);
			this->localBATime.push_back(time);

			if (this->outputWrapper)
			{
				this->outputWrapper->publishLocalBATime(time);
			}
		}
		else
		{
			// track candidates with the tracked frame
			this->trackCandidates(frame);
			this->numMappedFramesFromLastKF++;

			Utils::Time t2 = std::chrono::steady_clock::now();
			const float time = Utils::elapsedTime(t1, t2);
			this->pointTrackingTime.push_back(time);

			if (this->outputWrapper)
			{
				this->outputWrapper->publishPointTrackingTime(time);
			}
		}
	}

	void FullSystem::waitUntilMappingFinished()
	{
		std::unique_lock<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
		//std::cout << "waiting..." << std::endl;
		this->newFrameMappedSignal.wait(newFrameMappedLock, [&]() {return this->newFrameMappedDone; });
	}

	void FullSystem::createCandidates(const std::shared_ptr<Frame>& frame)
	{
		const auto& calib = GlobalCalibration::getInstance();
		const auto& settings = Settings::getInstance();

		const int32_t width = calib.width(0);
		const int32_t height = calib.height(0);
		const int32_t distToBorder = Pattern::padding() + 1;

		// Create new candidates in the frame
		// They have to be homogeneously distributed in the image	
		int num = this->pointDetector->detect(frame, (int)settings.numCandidates, this->pixelMask,
											  this->outputWrapper);

		// create candidates
		auto& candidates = frame->candidates();
		candidates.reserve(num);

		Utils::Time t1 = std::chrono::steady_clock::now();

		for (int32_t row = distToBorder; row < height - distToBorder; ++row)
		{
			for (int32_t col = distToBorder; col < width - distToBorder; ++col)
			{
				int32_t idx = col + row * width;

				if (this->pixelMask[idx] < 0) continue;

				// create a point
				candidates.emplace_back(std::make_unique<CandidatePoint>((float)col, (float)row, this->pixelMask[idx], frame));
			}
		}

		candidates.shrink_to_fit();

		Utils::Time t2 = std::chrono::steady_clock::now();
		
		if (settings.debugPrintLog && settings.debugLogPixelDetection)
		{
			const std::string msg = "PointDetect: " + std::to_string(candidates.size()) + "\t";

			auto& log = Log::getInstance();
			log.addCurrentLog(frame->frameID(), msg);
		}

		// reserve memory
		auto& activePoints = frame->activePoints();
		activePoints.reserve(candidates.size());
		
		//std::cout << "Select pixels: " << Utils::elapsedTime(t1, t2) << std::endl;
	}

	void FullSystem::trackCandidates(const std::shared_ptr<Frame>& frame)
	{
		const auto& settings = Settings::getInstance();

		const auto& activeKeyframes = this->lmcw->activeWindow();

		if (settings.debugCandidates || (settings.debugPrintLog && settings.debugLogCandidatesTracking))
		{
			int numGood = 0;
			int numBadError = 0;
			int numBadEpiLine = 0;
			int numOOB = 0;
			int numSkipped = 0;
			int numBadConditioned = 0;
			int numNegativeIDepth = 0;

			for (const auto& kf : activeKeyframes)
			{
				// clean candidate vector
				// remove outliers
				const auto& candidates = kf->candidates();

				for (const auto& cand : candidates)
				{
					// skip if the point is an outlier or is not visible
					if (cand->status() == CandidatePoint::OUTLIER ||
						cand->lastObservation() == CandidatePoint::OOB)
					{
						continue;
					}
					else
					{
						CandidatePoint::ObserveStatus status =  cand->observe(frame);

						if (status == CandidatePoint::GOOD) numGood++;
						else if (status == CandidatePoint::BAD_ERROR) numBadError++;
						else if (status == CandidatePoint::BAD_EPILINE) numBadEpiLine++;
						else if (status == CandidatePoint::OOB) numOOB++;
						else if (status == CandidatePoint::SKIPPED) numSkipped++;
						else if (status == CandidatePoint::BAD_CONDITIONED) numBadConditioned++;
						else if (cand->iDepth() < 0.f) numNegativeIDepth++;
					}
				}
			}

			if (settings.debugCandidates)
			{
				std::cout << "Candidates good: " << numGood << std::endl;
				std::cout << "Candidates bad error: " << numBadError << std::endl;
				std::cout << "Candidates bad epiline: " << numBadEpiLine << std::endl;
				std::cout << "Candidates oob: " << numOOB << std::endl;
				std::cout << "Candidates skipped: " << numSkipped << std::endl;
				std::cout << "Candidates bad_cond: " << numBadConditioned << std::endl;
				std::cout << "Candidates neg_iDepth: " << numNegativeIDepth << std::endl;
				std::cout << std::endl;
			}

			if (settings.debugPrintLog && settings.debugLogCandidatesTracking)
			{
				const std::string msg = "Candidates: " + std::to_string(numGood) + "\t" + std::to_string(numBadError) + "\t" + std::to_string(numBadEpiLine) + "\t"
										+ std::to_string(numOOB) + "\t" + std::to_string(numSkipped) + "\t" + std::to_string(numBadConditioned) + "\t";

				auto& log = Log::getInstance();
				log.addCurrentLog(frame->frameID(), msg);
			}
		}
		else
		{
			// vector of candidates to proccess
			std::vector<CandidatePoint*> toObserve;

			for (const auto& kf : activeKeyframes)
			{
				const auto& candidates = kf->candidates();
				for (const auto& cand : candidates)
				{
					// skip if the point is an outlier or is not visible
					if (cand->status() == CandidatePoint::OUTLIER ||
						cand->lastObservation() == CandidatePoint::OOB)
					{
						continue;
					}
					else
					{
						// if still visible
						toObserve.push_back(cand.get());
					}
				}
			}

			// try to initialize candidates
			// do it in parallel
			int itemsCount = (int)toObserve.size();

			int start = 0;
			int end = 0;
			int step = (itemsCount + settings.mappingThreads - 1) / settings.mappingThreads;

			for (int i = 0; i < settings.mappingThreads; ++i)
			{
				start = end;
				end = std::min(end + step, itemsCount);

				this->threadPool->addJob(PointObserver(toObserve, frame, start, end));
			}

			// wait until finish
			this->threadPool->wait();
		}
	}

	void FullSystem::refineCandidates()
	{
		const auto& settings = Settings::getInstance();

		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
		const int width = (int)calib.width(0);
		const int height = (int)calib.height(0);

		const auto& activeKeyframes = this->lmcw->activeWindow();
		const auto temporalKeyframes = this->lmcw->temporalWindow();
		const std::shared_ptr<Frame>& lastKeyframe = temporalKeyframes.back();
		const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();

		// vector of candidates to proccess
		std::vector<CandidatePoint*> toOptimize;

		for (int j = 0; j < temporalKeyframes.size() - 1; ++j)
		{
			// relative pose
			const Sophus::SE3f relPose = worldToLast * temporalKeyframes[j]->camToWorld();
			const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
			const Eigen::Vector3f Kt = K * relPose.translation();

			auto& candidates = temporalKeyframes[j]->candidates();
			for (auto& cand : candidates)
			{
				CandidatePoint::PointStatus status = cand->status();
				CandidatePoint::ObserveStatus lastObs = cand->lastObservation();

				// remove if the point is an outlier
				// or if the point has never been observed				
				if (status == CandidatePoint::PointStatus::UNINITIALIZED ||
					lastObs == CandidatePoint::ObserveStatus::BAD_ERROR || 
					lastObs == CandidatePoint::ObserveStatus::BAD_EPILINE)
				{
					cand = nullptr;
					continue;
				}

				bool isGoodCandidate = cand->matchUncertainty() < settings.maxCandidateUncertainty &&
									   cand->matchQuality() > settings.minCandidateQuality;

				if (!isGoodCandidate)
				{
					// erase if it is not visible anymore
					if (lastObs == CandidatePoint::OOB ||
						temporalKeyframes[j]->flaggedToDrop())
					{
						cand = nullptr;
					}

					continue;
				}

				// project to last keyframe
				Eigen::Vector2f pointInFrame;
				if (!Utils::project(cand->u0(), cand->v0(), cand->iDepth(),
									width, height, KRKinv, Kt, pointInFrame))
				{
					cand = nullptr;
					continue;
				}

				// we have a good candidate!
				// insert to list
				toOptimize.push_back(cand.get());
			}	
		}

		if (settings.debugCandidates || (settings.debugPrintLog && settings.debugLogCandidatesOpt))
		{
			int numGood = 0;
			int numBad = 0;
			int numSkipped = 0;

			int itemsCount = (int)toOptimize.size();

			for (int i = 0; i < itemsCount; ++i)
			{
				toOptimize[i]->optimize(activeKeyframes);

				if (toOptimize[i]->status() == CandidatePoint::PointStatus::OPTIMIZED) numGood++;
				else if (toOptimize[i]->status() == CandidatePoint::PointStatus::INITIALIZED) numSkipped++;
				else if (toOptimize[i]->status() == CandidatePoint::PointStatus::OUTLIER) numBad++;
			}

			if (settings.debugCandidates)
			{
				std::cout << "Candidates opt good: " << numGood << std::endl;
				std::cout << "Candidates opt skipped: " << numSkipped << std::endl;
				std::cout << "Candidates opt bad: " << numBad << std::endl;
				std::cout << std::endl;
			}

			if (settings.debugPrintLog && settings.debugLogCandidatesOpt)
			{
				const std::string msg = "Candidates Opt: " + std::to_string(numGood) + "\t" + std::to_string(numSkipped) + "\t" + std::to_string(numBad) + "\t";

				auto& log = Log::getInstance();
				log.addCurrentLog(lastKeyframe->frameID(), msg);
			}
		}
		else
		{
			// try to refine candidates
			// do it in parallel
			int itemsCount = (int)toOptimize.size();

			int start = 0;
			int end = 0;
			int step = (itemsCount + settings.mappingThreads - 1) / settings.mappingThreads;

			for (int i = 0; i < settings.mappingThreads; ++i)
			{
				start = end;
				end = std::min(end + step, itemsCount);

				this->threadPool->addJob(PointOptimizer(toOptimize, activeKeyframes, start, end));
			}
		}

		// meantime reorder candidates
		for (int j = 0; j < temporalKeyframes.size() - 1; ++j)
		{
			auto& candidates = temporalKeyframes[j]->candidates();		
			for (int i = 0; i < candidates.size(); ++i)
			{
				if (candidates[i] == nullptr)
				{
					candidates[i] = std::move(candidates.back());
					candidates.pop_back();
					i--;
				}
			}
		}

		// wait until finish
		this->threadPool->wait();
	}

	void FullSystem::createKeyframeAndOptimize(const std::shared_ptr<Frame>& frame)
	{
		this->lastWasKF = true;

		const auto& settings = Settings::getInstance();

		Utils::Time t1 = std::chrono::steady_clock::now();

		// initialize candidates
		this->trackCandidates(frame);

		// insert new keyframe
		this->lmcw->insertNewKeyframe(frame);

		if (settings.debugPrintLog && settings.debugLogKeyframes)
		{
			const std::string msg = "Keyframe: " + std::to_string(frame->keyframeID()) + "\t";

			auto& log = Log::getInstance();
			log.addCurrentLog(frame->frameID(), msg);
		}

		// select window
		this->lmcw->selectWindow(this->ceresOptimizer);

		// refine candidates from the temporal window
		Utils::Time t_refine_init = std::chrono::steady_clock::now();
		this->refineCandidates();
		Utils::Time t_refine_end = std::chrono::steady_clock::now();
		//std::cout << "Refine Candidates: " << Utils::elapsedTime(t_refine_init, t_refine_end) << std::endl;

		// select new active points from the temporal window
		this->lmcw->activatePoints(this->ceresOptimizer);

		// optimize
		const auto& activeKeyframes = this->lmcw->activeWindow();
		this->ceresOptimizer->solve(activeKeyframes);

		// remove outliers
		this->lmcw->removeOutliers();

		// set new tracking reference
		// use covisibility keyframes here too
		Utils::Time t13 = std::chrono::steady_clock::now();
		{
			std::lock_guard<std::mutex> lock(this->trackingReferenceMutex);
			this->newTrackingReference->setNewReference(activeKeyframes);
			this->trackingReferenceUpdated = true;

			if ((this->outputWrapper && settings.showDepthMap) ||
				settings.saveDepthMaps)
			{
				// depth map
				this->drawTrackDepthMap(this->newTrackingReference, this->depthMapImage, settings.showDepthMapLvl);
			}

			if (settings.saveDepthMaps)
			{
				// check directory
				dsm::Utils::makeDir(settings.depthMapsDir);

				std::string file = settings.depthMapsDir + "/depth" + std::to_string(this->saveID) + ".png";
				cv::imwrite(file, this->depthMapImage);
				this->saveID++;
			}

			if (this->outputWrapper && settings.showDepthMap)
			{
				this->outputWrapper->publishProcessFrame(this->depthMapImage);
			}
		}
		Utils::Time t14 = std::chrono::steady_clock::now();
		//std::cout << "Tracking ref time: " << Utils::elapsedTime(t13, t14) << std::endl;

		// Debug windows
		if (settings.debugShowOptKeyframes && this->outputWrapper)
			this->drawActiveKeyframes();

		if (settings.debugShowOptError && this->outputWrapper)
			this->drawOptErrorMap();

		if (settings.debugShowOptWeight && this->outputWrapper)
			this->drawOptWeight();

		if (settings.debugShowOptLight && this->outputWrapper)
			this->drawOptLight();

		if (settings.debugShowOptErrorDist && this->outputWrapper)
			this->drawOptErrorDist();

		// Publish to output wrapper
		Utils::Time t15 = std::chrono::steady_clock::now();
		if (this->outputWrapper)
		{
			// reset types
			this->outputWrapper->resetKeyframeTypes();

			// 3d points & cameras
			const auto temporalKeyframes = this->lmcw->temporalWindow();
			const auto covisibleKeyfames = this->lmcw->covisibleWindow();

			for (const auto& kf : temporalKeyframes)
			{
				this->outputWrapper->publishKeyframe(kf, KeyframeType::TEMPORAL);
			}
			for (const auto& kf : covisibleKeyfames)
			{
				this->outputWrapper->publishKeyframe(kf, KeyframeType::COVISIBILITY);
			}
		}
		Utils::Time t16 = std::chrono::steady_clock::now();
		//std::cout << "Visualization time: " << Utils::elapsedTime(t15, t16) << std::endl;

		// update covisibility graph
		Utils::Time t17 = std::chrono::steady_clock::now();
		this->lmcw->updateConnectivity();
		Utils::Time t18 = std::chrono::steady_clock::now();
		//std::cout << "Update connectivity time: " << Utils::elapsedTime(t9, t10) << std::endl;

		// drop keyframes and remove covisible keyframes
		// we will only estimate new candidates from the temporal window
		this->lmcw->dropKeyframes();

		// create new candidates for last keyframe
		Utils::Time t19 = std::chrono::steady_clock::now();
		this->createCandidates(frame);
		Utils::Time t20 = std::chrono::steady_clock::now();
		//std::cout << "Create candidates time: " << Utils::elapsedTime(t19, t20) << std::endl;

		Utils::Time t2 = std::chrono::steady_clock::now();
		std::cout << "createKeyframeAndOptimize in " << Utils::elapsedTime(t1, t2) << std::endl;
	}

	void FullSystem::drawTrackDepthMap(const std::shared_ptr<FrameTrackerReference>& trackRef, cv::Mat& depthMapBGR, int lvl)
	{
		if (trackRef == nullptr) return;

		// constant values
		const float* pyrImg = trackRef->reference()->image(lvl);
		const float* iDepthMap = trackRef->iDepthMap(lvl);

		const int w = GlobalCalibration::getInstance().width(lvl);
		const int h = GlobalCalibration::getInstance().height(lvl);

		if (pyrImg == nullptr || iDepthMap == nullptr)
		{
			return;
		}

		// obtain inverse depth range
		std::vector<float> allIDepths;
		for (int i = 0; i < w*h; i++)
		{
			if (!(iDepthMap[i] <= 0.f))
			{
				allIDepths.push_back(iDepthMap[i]);
			}		
		}
		std::sort(allIDepths.begin(), allIDepths.end());

		if (allIDepths.empty()) return;

		this->minIDepthTracker = std::max(allIDepths[(int)(allIDepths.size()*0.1f)], 0.f);
		this->maxIDepthTracker = std::max(allIDepths[(int)(allIDepths.size()*0.9f)], 0.f);

		// copy image
		for (int row = 0; row < h; ++row)
		{
			for (int col = 0; col < w; ++col)
			{
				int index = col + row * w;
				unsigned char c = (unsigned char)pyrImg[index];
				depthMapBGR.at<cv::Vec3b>(row, col) = cv::Vec3b(c, c, c);
			}
		}

		// draw points
		for (int row = 0; row < h; ++row)
		{
			for (int col = 0; col < w; ++col)
			{
				int index = col + row * w;
				if (!(iDepthMap[index] <= 0.f))
				{
					Eigen::Vector3f rgb = Utils::colorMap(this->minIDepthTracker, this->maxIDepthTracker, iDepthMap[index]) * 255;
					cv::Vec3b bgr = cv::Vec3b((unsigned char)rgb[2], (unsigned char)rgb[1], (unsigned char)rgb[0]);

					Utils::draw3x3Square(depthMapBGR, row, col, bgr);
				}
			}
		}
	}

	void FullSystem::drawActiveKeyframes()
	{
		const auto& activeKeyframes = this->lmcw->activeWindow();
		int numActiveKeyframes = (int)activeKeyframes.size();

		if (!(numActiveKeyframes > 1)) return;

		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
		const int w = (int)calib.width(0);
		const int h = (int)calib.height(0);

		const std::shared_ptr<Frame>& lastKeyframe = activeKeyframes.back();
		const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
		int lastIdx = lastKeyframe->keyframeID();

		// obtain inverse depth range
		std::vector<float> allIDepths;
		for (int i = 0; i < numActiveKeyframes - 1; ++i)
		{
			const auto& activePoints = activeKeyframes[i]->activePoints();
			for (const auto& point : activePoints)
			{
				allIDepths.push_back(point->iDepth());
			}
		}
		std::sort(allIDepths.begin(), allIDepths.end());

		if (allIDepths.empty()) return;

		this->minIDepthOpt = std::max(allIDepths[(int)(allIDepths.size()*0.1f)], 0.f);
		this->maxIDepthOpt = std::max(allIDepths[(int)(allIDepths.size()*0.9f)], 0.f);

		// create images
		std::vector<cv::Mat> images(numActiveKeyframes);
		for (int i = 0; i < numActiveKeyframes; ++i)
		{
			// create image
			images[i] = cv::Mat(h, w, CV_8UC1);

			// copy values
			const float* imagePtr = activeKeyframes[i]->image(0);
			for (int idx = 0; idx < w*h; idx++)
			{
				images[i].data[idx] = (unsigned char)imagePtr[idx];
			}

			// change from gray to bgr
			cv::cvtColor(images[i], images[i], cv::COLOR_GRAY2BGR);
		}

		const int rectSize = 6;
		const int rectPad = rectSize / 2;

		// draw for each active keyframe their active points
		// draw also the active points in the last keyframe
		for (int i = 0; i < numActiveKeyframes - 1; ++i)
		{
			// relative pose data
			const Sophus::SE3f relPose = worldToLast * activeKeyframes[i]->camToWorld();
			const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
			const Eigen::Vector3f Kt = K * relPose.translation();

			// active points
			// use all active points
			const auto& activePoints = activeKeyframes[i]->activePoints();
			for (const auto& point : activePoints)
			{
				const float u = point->u(0);
				const float v = point->v(0);

				if (point->visibility(lastIdx) == Visibility::VISIBLE)
				{
					Eigen::Vector2f pt2d;
					if (!Utils::project(u, v, point->iDepth(),
										w, h, KRKinv, Kt, pt2d))
					{
						continue;
					}
					
					Eigen::Vector3f rgb = Utils::colorMap(this->minIDepthOpt, this->maxIDepthOpt, point->iDepth()) * 255;
					
					// draw in the reference image
					cv::rectangle(images[i], cv::Rect((int)u - rectPad, (int)v - rectPad, rectSize, rectSize), cv::Scalar(rgb[2], rgb[1], rgb[0]), 2);
					
					// draw in last keyframe
					cv::rectangle(images[numActiveKeyframes - 1], cv::Rect((int)pt2d[0] - rectPad, (int)pt2d[1] - rectPad, rectSize, rectSize), cv::Scalar(rgb[2], rgb[1], rgb[0]), 2);
				}
				else
				{
					cv::rectangle(images[i], cv::Rect((int)u - rectPad, (int)v - rectPad, rectSize, rectSize), cv::Scalar(0, 0, 0), 2);
				}		
			}
		}

		// show images
		cv::Mat output;
		Utils::concatImages(images, output);

		if (this->outputWrapper)
		{
			this->outputWrapper->publishOptKeyframes(output);
		}
	}

	void FullSystem::drawOptErrorMap()
	{
		const auto& settings = Settings::getInstance();

		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
		const int w = calib.width(0);
		const int h = calib.height(0);

		const auto& activeKeyframes = this->lmcw->activeWindow();
		const auto& lastKeyframe = activeKeyframes.back();
		const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
		int idx = lastKeyframe->keyframeID();

		// initialize an opencv image
		cv::Mat errorMap(h, w, CV_8UC1);

		// copy values
		const float* imagePtr = lastKeyframe->image(0);
		for (int idx = 0; idx < w*h; idx++)
		{
			errorMap.data[idx] = (unsigned char)imagePtr[idx];
		}

		// change from gray to bgr
		cv::cvtColor(errorMap, errorMap, cv::COLOR_GRAY2BGR);

		for (auto& kf : activeKeyframes)
		{
			// relative pose data
			const Sophus::SE3f relPose = worldToLast * kf->camToWorld();
			const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
			const Eigen::Vector3f Kt = K * relPose.translation();

			for (auto& point : kf->activePoints())
			{
				if (point->visibility(idx) == Visibility::VISIBLE)
				{
					auto it = point->observations().find(lastKeyframe.get());

					if (it == point->observations().end()) continue;

					Eigen::Vector2f pt2d;
					if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
										w, h, KRKinv, Kt, pt2d))
					{
						continue;
					}

					double error = it->second->energy();

					Eigen::Vector3f rgb = Utils::colorMap(0.f, 15.f*15.f*Pattern::size(), (float)error) * 255;

					// draw in last keyframe
					cv::circle(errorMap, cv::Point((int)(pt2d[0] + 0.5f), (int)(pt2d[1] + 0.5f)), 2, cv::Scalar(rgb[2], rgb[1], rgb[0]), -1);
				}
			}
		}

		if (this->outputWrapper)
		{
			this->outputWrapper->publishOptError(errorMap);
		}
	}

	void FullSystem::drawOptWeight()
	{
		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
		const int w = calib.width(0);
		const int h = calib.height(0);

		const auto& activeKeyframes = this->lmcw->activeWindow();
		const auto& lastKeyframe = activeKeyframes.back();
		const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
		int idx = lastKeyframe->keyframeID();

		// initialize an opencv image
		cv::Mat weightMap(h, w, CV_8UC1);

		// copy values
		const float* imagePtr = lastKeyframe->image(0);
		for (int idx = 0; idx < w*h; idx++)
		{
			weightMap.data[idx] = (unsigned char)imagePtr[idx];
		}

		// change from gray to bgr
		cv::cvtColor(weightMap, weightMap, cv::COLOR_GRAY2BGR);

		for (auto& kf : activeKeyframes)
		{
			// relative pose data
			const Sophus::SE3f relPose = worldToLast * kf->camToWorld();
			const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
			const Eigen::Vector3f Kt = K * relPose.translation();

			for (auto& point : kf->activePoints())
			{
				if (point->visibility(idx) == Visibility::VISIBLE)
				{
					auto it = point->observations().find(lastKeyframe.get());

					if (it == point->observations().end()) continue;

					Eigen::Vector2f pt2d;
					if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
										w, h, KRKinv, Kt, pt2d))
					{
						continue;
					}

					double weight = it->second->lossWeight();

					Eigen::Vector3f rgb = Utils::colorMap(0.f, 1.f, (float)weight) * 255;

					// draw in last keyframe
					cv::circle(weightMap, cv::Point((int)(pt2d[0] + 0.5f), (int)(pt2d[1] + 0.5f)), 2, cv::Scalar(rgb[2], rgb[1], rgb[0]), -1);
				}
			}
		}

		if (this->outputWrapper)
		{
			this->outputWrapper->publishOptWeight(weightMap);
		}
	}

	void FullSystem::drawOptLight()
	{
		const auto& activeKeyframes = this->lmcw->activeWindow();
		int numActiveKeyframes = (int)activeKeyframes.size();

		if (!(numActiveKeyframes > 1)) return;

		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& K = calib.matrix3f(0);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
		const int w = (int)calib.width(0);
		const int h = (int)calib.height(0);

		// create images
		std::vector<cv::Mat> images(numActiveKeyframes);
		for (int i = 0; i < numActiveKeyframes; ++i)
		{
			// create image
			images[i] = cv::Mat(255, 255, CV_8UC3);
			images[i].setTo(cv::Scalar(255, 255, 255));
		}

		for (int i = 0; i < numActiveKeyframes; ++i)
		{
			const auto& kf1 = activeKeyframes[i];
			const Sophus::SE3f worldToKf1 = kf1->camToWorld().inverse();

			const AffineLight& light = kf1->affineLight();
			const float* image = kf1->image(0);

			// draw points
			for (const auto& kf2 : activeKeyframes)
			{
				if(kf1 == kf2) continue;

				// relative data
				const Sophus::SE3f relPose = worldToKf1 * kf2->camToWorld();
				const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
				const Eigen::Vector3f Kt = K * relPose.translation();

				const AffineLight& affLight = kf2->affineLight();
				const float affLight_a = affLight.a();
				const float affLight_b = affLight.b();

				for (const auto& point : kf2->activePoints())
				{
					if (point->visibility(kf1->keyframeID()) != Visibility::VISIBLE)
					{
						continue;
					}

					const Eigen::VecXf& color = point->colors(0);

					for (int idx = 0; idx < Pattern::size(); ++idx)
					{
						float uj = point->u(0) + (float)Pattern::at(idx, 0);
						float vj = point->v(0) + (float)Pattern::at(idx, 1);

						// project point to new image
						Eigen::Vector2f pt2d;
						if (!Utils::project(uj, vj, point->iDepth(), w, h, KRKinv, Kt, pt2d))
						{
							continue;
						}
						
						float newImgColor = bilinearInterpolation(image, pt2d[0], pt2d[1], w);
						
						// image coordiantes
						int row = static_cast<int>(affLight_a*color[idx] + affLight_b + 0.5f);
						int col = static_cast<int>(newImgColor + 0.5f);
						
						col = std::min(254, col);
						col = std::max(0, col);
						
						row = std::min(254, row);
						row = std::max(0, row);
						
						// draw
						images[i].at<cv::Vec3b>(254 - row, col) = cv::Vec3b(0, 0, 0);
					}
				}
			}

			// draw affine light line
			const float light_a = light.a();
			const float light_b = light.b();

			cv::Point p1, p2;

			int pt1x = (int)(-light_b / light_a + 0.5f);
			if (pt1x >= 0)
			{
				// cross with y = 0
				pt1x = std::min(254, pt1x);
				p1 = cv::Point(pt1x, 0);
			}
			else
			{
				// cross with x = 0
				int pt1y = (int)(light_b + 0.5f);
				pt1y = std::min(254, pt1y);
				p1 = cv::Point(0, pt1y);
			}

			int pt2x = (int)((254.f - light_b) / light_a + 0.5f);
			if (pt2x <= 254)
			{
				// cross with y = 254
				pt2x = std::max(0, pt2x);
				p2 = cv::Point(pt2x, 254);
			}
			else
			{
				// cross with x = 254
				int pt2y = (int)(light_a*254.f + light_b + 0.5f);
				pt2y = std::max(0, pt2y);
				p2 = cv::Point(254, pt2y);
			}

			cv::line(images[i], cv::Point(p1.x, 254 - p1.y), cv::Point(p2.x, 254 - p2.y), cv::Scalar(0, 0, 255), 1);
		}

		// show images
		cv::Mat output;
		Utils::concatImages(images, output);

		if (this->outputWrapper)
		{
			this->outputWrapper->publishOptLight(output);
		}
	}

	void FullSystem::drawOptErrorDist()
	{
		const auto& activeKeyframes = this->lmcw->activeWindow();
		int numActiveKeyframes = (int)activeKeyframes.size();

		const auto& calib = GlobalCalibration::getInstance();
		const int w = (int)calib.width(0);
		const int h = (int)calib.height(0);

		// initialize vector
		std::vector<cv::Mat> images(numActiveKeyframes);

		for (int i = 0; i < numActiveKeyframes; ++i)
		{
			const auto& kf1 = activeKeyframes[i];

			std::vector<float> allObservations;

			// take all observations
			for (const auto& kf2 : activeKeyframes)
			{
				if (kf1 == kf2) continue;

				for (const auto& point : kf2->activePoints())
				{
					if (point->visibility(kf1->keyframeID()) == Visibility::UNINITIALIZED ||
						point->visibility(kf1->keyframeID()) == Visibility::OOB)
					{
						continue;
					}

					const auto& obs = point->observations();

					// find kf1
					auto it = obs.find(kf1.get());
					if (it == obs.end()) continue;

					// take residual vector
					const auto& pixelEnergy = it->second->pixelEnergy();
					for (int idx = 0; idx < Pattern::size(); ++idx)
					{
						allObservations.push_back((float)pixelEnergy[idx]);
					}
				}
			}

			// draw distribution
			const auto& distribution = kf1->errorDistribution();
			images[i] = Utils::drawDistribution(allObservations, distribution, 
												Eigen::Vector2f(-255.f, 255.f),
												Eigen::Vector2f(0, 0.1f),
												Eigen::Vector2i(510, 510),
												510);
		}

		// show images
		cv::Mat output;
		Utils::concatImages(images, output);

		if (this->outputWrapper)
		{
			this->outputWrapper->publishOptErrorDist(output);
		}
	}

	void FullSystem::printLog() const
	{
		const auto& settings = Settings::getInstance();
		if (settings.debugPrintLog)
		{
			const auto& log = Log::getInstance();
			log.printLog();
		}
	}

	// classes for parallelization
	FullSystem::PointObserver::PointObserver(std::vector<CandidatePoint*>& theCandidates, const std::shared_ptr<Frame> &targetFrame,
											 int aBegin, int anEnd) :
		candidates(theCandidates), target(targetFrame), begin(aBegin), end(anEnd)
	{
	}

	void FullSystem::PointObserver::operator()()
	{
		// compute candidate observation from begin to end only
		for (int i = this->begin; i < this->end; ++i)
		{
			CandidatePoint* candPoint = this->candidates[i];

			// observe in the new tracked frame
			candPoint->observe(this->target);
		}
	}

	FullSystem::PointOptimizer::PointOptimizer(std::vector<CandidatePoint*>& theCandidates,
											   const std::vector<std::shared_ptr<Frame>> &activeFrames,
											   int aBegin, int anEnd) :
		candidates(theCandidates), activeFrames(activeFrames), begin(aBegin), end(anEnd)
	{
	}

	void FullSystem::PointOptimizer::operator()()
	{
		// compute candidate observation from begin to end only
		for (int i = this->begin; i < this->end; ++i)
		{
			CandidatePoint* candPoint = this->candidates[i];

			// optimize inverse depth in all active keyframes
			candPoint->optimize(this->activeFrames);
		}
	}
}
