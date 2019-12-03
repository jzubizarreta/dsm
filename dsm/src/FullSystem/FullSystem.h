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

#include "DSMLib.h"

#include <Eigen/Core>
#include <Eigen/StdVector>
#include "sophus/se3.hpp"

#include "opencv2/core.hpp"

#include <deque>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <map>
#include <thread>

namespace dsm
{
	class Frame;
	class CandidatePoint;
	class ActivePoint;
	class PointDetector;
	class CeresPhotometricBA;
	class MonoInitializer;
	class FrameTracker;
	class FrameTrackerReference;
	class DistanceTransform;
	class CovisibilityGraph;
	class IVisualizer;
	class WorkerThreadPool;
	class LMCW;

	class DSM_EXPORTS_DLL FullSystem
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		FullSystem(int w, int h, const Eigen::Matrix3f &calib,
				   const std::string &settingsFile = "",
				   IVisualizer *outputWrapper = nullptr);

		FullSystem(const FullSystem&) = delete;
		FullSystem& operator=(const FullSystem&) = delete;
		~FullSystem();

		////////////////////////////////////////
		//		  MAIN TRACKING FUNC		  //
		////////////////////////////////////////


		// Tracks the stereo frame. It calculates the system pose and the 3D scene*/
		void trackFrame(int id, double timestamp, unsigned char* image);

		////////////////////////////////////////
		//		       ACCESORS	 	          //
		////////////////////////////////////////

		void getTrajectory(std::vector<Eigen::Matrix4f> &poses, std::vector<double> &timestamps) const;

		void getStructure(std::vector<Eigen::Vector3f>& structure) const;

		float getCamTrackingMeanTime() const;
		float getPointTrackingMeanTime() const;
		float getLocalBAMeanTime() const;

		int getNumPoints() const;
		int getNumKeyframes() const;

		bool getLastWasKF() const;

		bool isInitialized() const;

		bool isLost() const;

		void printLog() const;

	private:

		// initialize from video sequence
		bool initialize(const std::shared_ptr<Frame>& frame);

		// track new frame using image alignment
		void trackNewFrame(const std::shared_ptr<Frame>& frame);

		// keyframe management
		bool isNewKeyframeRequired(const std::shared_ptr<Frame>& frame) const;

		// mapping thread infinite loop
		void mappingThreadLoop();
		void doMapping(const std::shared_ptr<Frame>& frame);
		void waitUntilMappingFinished();

		// candidates management
		void createCandidates(const std::shared_ptr<Frame>& frame);
		void trackCandidates(const std::shared_ptr<Frame>& frame);
		void refineCandidates();

		// Optimization
		void createKeyframeAndOptimize(const std::shared_ptr<Frame>& frame);

		// visualization
		void drawTrackDepthMap(const std::shared_ptr<FrameTrackerReference>& trackRef, cv::Mat& depthMapBGR, int lvl);

		// optimization visualization
		void drawActiveKeyframes();	// draws active keyframes with their points

		void drawOptErrorMap();	
		void drawOptWeight();
		void drawOptLight();
		void drawOptErrorDist();

	private:

		// control variables
		std::atomic_bool initialized;
		std::atomic_bool trackingIsGood;

		// threads
		std::unique_ptr<std::thread> mappingThread;
		std::atomic_bool shouldStop;

		// for blocking operation. Set in Mapping, read in Tracking.
		std::condition_variable  newFrameMappedSignal;
		std::mutex newFrameMappedMutex;
		bool newFrameMappedDone;

		// initializer
		std::unique_ptr<MonoInitializer> initializer;

		// detector
		std::unique_ptr<PointDetector> pointDetector;
		int32_t* pixelMask;

		// tracker, only in tracking thread
		std::unique_ptr<FrameTracker> tracker;

		// parallelization
		std::shared_ptr<WorkerThreadPool> threadPool;

		// only in tracking thread
		// last tracked frame information for priors
		std::shared_ptr<Frame> lastTrackedFrame;		// last tracked frame
		Sophus::SE3f lastTrackedMotion;					// last tracked frame motion based on constant velocity model
		float lastTrackedResidual;

		// tracking reference
		std::mutex trackingReferenceMutex;
		std::shared_ptr<FrameTrackerReference> trackingReference;			// tracking reference
		std::shared_ptr<FrameTrackerReference> newTrackingReference;		// updated tracking reference
		bool trackingReferenceUpdated;

		// write in tracking, read in mapping
		std::mutex unmappedTrackedFramesMutex;
		std::condition_variable  unmappedTrackedFramesSignal;
		std::deque<std::shared_ptr<Frame>> unmappedTrackedFrames;	

		// flag to set if new keyframe is required
		bool createNewKeyframe;							// only in tracking thread
		int createNewKeyframeID;						// write in tracking thread, read in mapping
		std::atomic_int numMappedFramesFromLastKF;		// write in mapping thread, read in tracking

		// Optimization window
		std::unique_ptr<LMCW> lmcw;

		// ceres optimizer
		std::unique_ptr<CeresPhotometricBA> ceresOptimizer;				// photometric bundle adjustment

		// statistics
		std::vector<float> camTrackingTime;
		std::vector<float> pointTrackingTime;
		std::vector<float> localBATime;
		bool lastWasKF;

		// visualization
		IVisualizer* outputWrapper;

		cv::Mat depthMapImage;
		float minIDepthTracker;
		float maxIDepthTracker;
		float minIDepthOpt;
		float maxIDepthOpt;

		// image writing
		int saveID;
		
		// structures for parallelization
		struct PointObserver
		{
			PointObserver(std::vector<CandidatePoint*>& theCandidates, 
						  const std::shared_ptr<Frame> &targetFrame,
						  int aBegin, int anEnd);

			void operator()();

		private:
			std::vector<CandidatePoint*>& candidates;
			const std::shared_ptr<Frame>& target;
			int begin, end;
		};

		struct PointOptimizer
		{
			PointOptimizer(std::vector<CandidatePoint*>& theCandidates,
						   const std::vector<std::shared_ptr<Frame>> &activeFrames,
						   int aBegin, int anEnd);

			void operator()();

		private:
			std::vector<CandidatePoint*>& candidates;
			const std::vector<std::shared_ptr<Frame>>& activeFrames;
			int begin, end;
		};
	};
}
