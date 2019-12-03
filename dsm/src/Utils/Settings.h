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

#include <string>
#include <Eigen/Core>

namespace dsm
{
	class Settings
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		static Settings& getInstance();

		bool fromFile(const std::string &file);
		
		void reset();

		// Dont forget to declare these two. You want to make sure they
		// are unacceptable otherwise you may accidentally get copies of
		// your singleton appearing.
		Settings(Settings const&) = delete;
		void operator=(Settings const&) = delete;

	public:

		//////////////////////////////////
		//	   Changeable Parameters	//
		//////////////////////////////////

		// Debug

		// writes a log file
		bool debugPrintLog;							
		bool debugLogTracking;
		bool debugLogKeyframes;
		bool debugLogPixelDetection;
		bool debugLogCandidatesTracking;
		bool debugLogCandidatesOpt;
		bool debugLogDistanceMap;
		bool debugLogActivePoints;

		bool debugCandidates;

		bool debugShowTrackResult;					// shows tracking result
		bool debugShowTrackError;					// tracking error map
		bool debugShowTrackWeights;
		bool debugShowTrackLight;					// tracking light result
		bool debugShowTrackDistribution;			// tracking error distribution

		bool debugShowPointDetection;				// shows point detection	

		bool debugShowDistanceTransformBefore;
		bool debugShowDistanceTransformAfter;

		bool debugShowOptKeyframes;

		bool debugShowOptError;						// error of active keyframes
		bool debugShowOptWeight;					// weights of active keyframes
		bool debugShowOptLight;						// affine light of active keyframes
		bool debugShowOptErrorDist;					// error distribution of active keyframes
		bool debugShowOptErrorDistLast;				// error distribution of last keyframe

		std::string depthMapsDir;
		bool saveDepthMaps;

		std::string optErrorDistDir;
		bool saveOptErrorDist;	

		// Parallelization
		bool blockUntilMapped;						// blocks tracking until mapping has finished
		bool singleThreaded;						// sequentialize tracking and mapping threads
													// required for visualization
		int mappingThreads;							// numer of parallel threads in mapping thread
													// at leas 1

		// Memory
		bool minimizeMemory;						// uses memory pool to reduce memory requeriments

		// Huber
		bool useFixedHuberScale;
		float huberScale;

		// Error Distribution
		bool useTDistribution;
		float nuFixed;								// fixed if > 0
		float muFixed;								// fixed if >= 0
		float sigmaFixed;							// fixed if > 0

		// Default Distribution
		float defaultNu;
		float defaultMu;		
		float defaultSigma;

		// outliers
		float inlierPercentile;						// inlier percentile
		float maxPixelOutlier;						// maximum number of pixels to consider the point outlier
		float maxPixelDiscard;						// maximum number of pixels to discard from optimization

		float maxEnergyFit;							// maximum energy to fit the error distribution

		// gradient weight constant w = c^2 / ( c^2 + grad^2 )
		float weightConstant;

		// Tracking
		int trackingMaxLevel;									// Coarsest tracking level

		// candidate selection
		int pointDetectionLevels;			// maximum number of pyramid levels to detect candidate points
		int numCandidates;					// maximum number of candidates per frame
		int numBlocksPerDimension;			// number of block in "x" and "y" image dimensions
		float minGradAdd;					// threshold constant addition to histogram for candidate pixel detection

		// Maxium number of unmapped frames in the queue
		int maxUnmappedFrames;

		// candidate tracking
		float maxEplLengthFactor;									// maximum length factor of epl to search based on width and height
		float minEplLengthSkip;										// minimum length of epl to search	
		float stereoMaxEnergy;										// maximum point tracking energy per pixel
		int secondBestRadius;										// minimum radius to the second-best match
		float epiLineSigma;											// epipolar line uncertainty
		int subpixelIterations;										// number of gauss newton iterations for subpixel refinement
		float subpixelStepThreshold;								// if step < thr the gauss newton optimization stops

		// perspective change: cos(viewChange)
		float maxViewChange;

		// candidate optimization
		int candidateOptIterations;				// number of iterations for candidate refinement
		int minDistToActivate;					// minimum neighbour distance to activate
		float maxCandidateUncertainty;			// maximum match uncertainty
		float minCandidateQuality;				// minimum distance to the second best match to activate

		// optimization
		bool doOnlyTemporalOpt;
		bool printSummary;
		bool showFullReport;
		float minOptimizationGrad;				// minimum gradient to consider a observation as outlier
		int minBAIterations;					// minimum number of bundle adjustment iterations
		int maxBAIterations;					// maximum number of bundle adjustment iterations
		int optMaxLevel;

		// Optimization scale factors
		float varScaleRot;
		float varScaleTrans;
		float varScaleAlpha;
		float varScaleBeta;
		float varScaleIDepth;

		// Active window
		int numActivePoints;					// number of observations in the last keyframe
	
		int maxTemporalKeyframes;				// maximum number of temporal active keyframes in the opt window
		int maxCovisibleKeyframes;				// maximum number of covisible active keyframes in the opt window
		int numAlwaysKeepKeyframes;				// number of temporal keyframes to always keep in the optimization window + the new one

		float minPointCovisible;				// minimum ratio of visible points to consider a keyframe as covisible
		float maxLightCovisible;				// maximum light change to consider a keyframe covisible

		// Outlier removal
		int minNumKFToConsiderNew;				// minimum number of keyframes to consider an active point as a new (temporal)
		int minNumGoodObservations;				// minimum number of observation to consider an active point as inlier

		// Keyframe Selection criteria
		float newKFDistWeight;						// weight of the camera translation relative to scene depth (parallax)
		float newKFUsageWeight;						// weight of point usage by the frame tracker
		float newKFAffineWeight;					// weight of light change in the scene
		float newKFResidualWeight;					// weight of the frame tracker residual
		int minNumMappedFramesToCreateKF;			// minimum number of tracked frames to create a new one		

		// Number of pyramids
		int pyramidLevels;

		// Depth map
		bool showDepthMap;							// shows depth map in the gui
		int showDepthMapLvl;

		// Secure Division
		static const float DIVISION_EPS;

		// Pi number
		static const float PI;

	private:

		Settings();
		~Settings();

		// updates argument
		void parseArgument(const char* arg);

		// updates some parameters based on the values of others
		void updateNonIndependentParameters();

		// bool compare
		bool compare(const std::string &value);

		// check parameters for each allowed type
		bool checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, std::string &var);
		bool checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, bool &var);
		bool checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, int &var);
		bool checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, float &var);
	};
}