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

#include "Settings.h"
#include "DataStructures/Pattern.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cassert>
#include <cmath>

namespace dsm
{
	// float variables must be initialized in the cpp
	const float Settings::DIVISION_EPS = 1e-10f;
	const float Settings::PI = 3.14159265358979323846f;

	Settings::Settings() 
	{
		this->reset();
	}

	Settings::~Settings()
	{
	}

	Settings& Settings::getInstance()
	{
		static Settings theOneAndOnly;
		return theOneAndOnly;
	}

	void Settings::reset()
	{
		// log
		this->debugPrintLog = false;
		this->debugLogTracking = false;
		this->debugLogKeyframes = false;
		this->debugLogPixelDetection = false;
		this->debugLogCandidatesTracking = false;
		this->debugLogCandidatesOpt = false;
		this->debugLogDistanceMap = false;
		this->debugLogActivePoints = false;

		// candidate tracking
		this->debugCandidates = false;

		// coarse tracking
		this->debugShowTrackResult = false;
		this->debugShowTrackError = false;
		this->debugShowTrackWeights = false;
		this->debugShowTrackLight = false;
		this->debugShowTrackDistribution = false;

		// pixel detector
		this->debugShowPointDetection = false;

		// distance transform
		this->debugShowDistanceTransformBefore = false;
		this->debugShowDistanceTransformAfter = false;

		// debug optimization
		this->debugShowOptKeyframes = false;

		this->debugShowOptError = false;
		this->debugShowOptWeight = false;
		this->debugShowOptLight = false;
		this->debugShowOptErrorDist = false;
		this->debugShowOptErrorDistLast = false;

		this->depthMapsDir = "./DepthMaps";
		this->saveDepthMaps = false;

		this->optErrorDistDir = "./ErrorDistribution";
		this->saveOptErrorDist = false;

		// Parallelization
		this->blockUntilMapped = false;
		this->singleThreaded = true;									
		this->mappingThreads = 6;										// 6

		this->minimizeMemory = false;

		this->useFixedHuberScale = false;								// false
		this->huberScale = 9.f;											// 9

		this->useTDistribution = true;									// true
		this->nuFixed = -1.f;											// -1
		this->muFixed = 0.f;											// 0
		this->sigmaFixed = -1.f;										// -1

		this->defaultNu = 5.f;											// 5
		this->defaultMu = 0.f;											// 0
		this->defaultSigma = 7.5f;										// 7.5

		this->inlierPercentile = 0.95f;									// 0.95
		this->maxPixelOutlier = 0.3f;									// 0.3
		this->maxPixelDiscard = 0.6f;									// 0.6

		this->maxEnergyFit = 60.f;										// 60

		this->weightConstant = 50.f*50.f;								// 50*50

		this->trackingMaxLevel = 3;										// 3
			
		this->pointDetectionLevels = 2;									// 2
		this->numCandidates = 1500;										// 1500
		this->numBlocksPerDimension = 20;								// 20
		this->minGradAdd = 7.f;											// 7

		this->maxUnmappedFrames = 3;									// 3

		this->maxEplLengthFactor = 0.0375f;								// 0.0375‬
		this->minEplLengthSkip = 1.5f;									// 1.5
		this->stereoMaxEnergy = 20.f;									// 20
		this->secondBestRadius = 2;										// 2
		this->epiLineSigma = 0.5f*0.5f;									// 0.25
		this->subpixelIterations = 3;									// 3
		this->subpixelStepThreshold = 0.1f;								// 0.1

		this->maxViewChange = cos(60.f*Settings::PI / 180.f);			// cos(60)

		this->candidateOptIterations = 3;								// 3
		this->minDistToActivate = Pattern::width();						// Pattern::width()
		this->maxCandidateUncertainty = 2.f;							// 2
		this->minCandidateQuality = 2.f;								// 2

		this->doOnlyTemporalOpt = false;								// false
		this->printSummary = true;										// true
		this->showFullReport = false;									// false
		this->minOptimizationGrad = 2.f;								// 2
		this->minBAIterations = 1;										// 1
		this->maxBAIterations = 100;									// 100
		this->optMaxLevel = 2;											// 2

		this->numActivePoints = 1500;									// 1500

		this->maxTemporalKeyframes = 5;									// 5
		this->maxCovisibleKeyframes = 3;								// 3
		this->numAlwaysKeepKeyframes = 3;								// 3: 2 + latest one

		this->minPointCovisible = 0.05f;								// 0.05
		this->maxLightCovisible = 0.7f;									// 0.7

		this->minNumKFToConsiderNew = 2;								// 2
		this->minNumGoodObservations = 3;								// 3

		this->newKFDistWeight = 4.f;									// 4		
		this->newKFUsageWeight = 3.f;									// 3 
		this->newKFAffineWeight = 4.f;									// 4
		this->newKFResidualWeight = 2.f;								// 2
		this->minNumMappedFramesToCreateKF = 1;							// 1

		this->showDepthMap = true;										// true
		this->showDepthMapLvl = 0;										// 0

		// update the rest
		this->updateNonIndependentParameters();
	}

	void Settings::updateNonIndependentParameters()
	{
		// number of pyramids
		this->pyramidLevels = std::max(std::max(this->trackingMaxLevel + 1,
												this->pointDetectionLevels + 1),
									   this->optMaxLevel + 1);

		// num always keep frames
		this->numAlwaysKeepKeyframes = std::min(this->numAlwaysKeepKeyframes,
												this->maxTemporalKeyframes - 1);

		// variable scaling
		// values obtained from the diag(Hessian)
		this->varScaleTrans = 0.5f;
		this->varScaleRot = 1.f;
		this->varScaleAlpha = 10.f;
		this->varScaleBeta = 1000.f;
		this->varScaleIDepth = 1.f;
	}

	bool Settings::fromFile(const std::string &file)
	{
		// open file
		std::ifstream infile(file);
		if (!infile.good())
		{
			return false;
		}

		// update parameters
		std::string argument;
		while (std::getline(infile, argument))
		{
			if(!argument.empty())
				this->parseArgument(argument.c_str());
		}

		// update the rest
		this->updateNonIndependentParameters();

		return true;
	}

	void Settings::parseArgument(const char* arg)
	{
		std::string argument(arg);

		// transform to lower
		std::transform(argument.begin(), argument.end(), argument.begin(), ::tolower);

		// remove spaces
		argument.erase(std::remove_if(argument.begin(), argument.end(), ::isspace), argument.end());

		// split by "="
		const size_t pos = argument.find("=");

		const std::string name = argument.substr(0, pos);
		const std::string value = argument.substr(pos + 1);

		// check
		if (this->checkParameter("debugprintlog", name, value, this->debugPrintLog)) return;
		if (this->checkParameter("debuglogtracking", name, value, this->debugLogTracking)) return;
		if (this->checkParameter("debuglogkeyframes", name, value, this->debugLogKeyframes)) return;
		if (this->checkParameter("debuglogpixeldetection", name, value, this->debugLogPixelDetection)) return;
		if (this->checkParameter("debuglogcandidatestracking", name, value, this->debugLogCandidatesTracking)) return;
		if (this->checkParameter("debuglogcandidatesopt", name, value, this->debugLogCandidatesOpt)) return;
		if (this->checkParameter("debuglogdistancemap", name, value, this->debugLogDistanceMap)) return;
		if (this->checkParameter("debuglogactivepoints", name, value, this->debugLogActivePoints)) return;
		if (this->checkParameter("debugcandidates", name, value, this->debugCandidates)) return;
		if (this->checkParameter("debugshowtrackresult", name, value, this->debugShowTrackResult)) return;
		if (this->checkParameter("debugshowtrackerror", name, value, this->debugShowTrackError)) return;
		if (this->checkParameter("debugshowtrackweights", name, value, this->debugShowTrackWeights)) return;
		if (this->checkParameter("debugshowtracklight", name, value, this->debugShowTrackLight)) return;
		if (this->checkParameter("debugshowtrackdistribution", name, value, this->debugShowTrackDistribution)) return;
		if (this->checkParameter("debugshowpointdetection", name, value, this->debugShowPointDetection)) return;
		if (this->checkParameter("debugshowdistancetransformbefore", name, value, this->debugShowDistanceTransformBefore)) return;
		if (this->checkParameter("debugshowdistancetransformafter", name, value, this->debugShowDistanceTransformAfter)) return;
		if (this->checkParameter("debugshowoptkeyframes", name, value, this->debugShowOptKeyframes)) return;
		if (this->checkParameter("debugshowopterror", name, value, this->debugShowOptError)) return;
		if (this->checkParameter("debugshowoptweight", name, value, this->debugShowOptWeight)) return;
		if (this->checkParameter("debugshowoptlight", name, value, this->debugShowOptLight)) return;
		if (this->checkParameter("debugshowopterrordist", name, value, this->debugShowOptErrorDist)) return;
		if (this->checkParameter("debugshowopterrordistlast", name, value, this->debugShowOptErrorDistLast)) return;
		if (this->checkParameter("depthmapsdir", name, value, this->depthMapsDir)) return;
		if (this->checkParameter("savedepthmaps", name, value, this->saveDepthMaps)) return;
		if (this->checkParameter("opterrordistdir", name, value, this->optErrorDistDir)) return;
		if (this->checkParameter("saveopterrordist", name, value, this->saveOptErrorDist)) return;
		if (this->checkParameter("blockuntilmapped", name, value, this->blockUntilMapped)) return;
		if (this->checkParameter("singlethreaded", name, value, this->singleThreaded)) return;
		if (this->checkParameter("mappingthreads", name, value, this->mappingThreads)) return;
		if (this->checkParameter("minimizememory", name, value, this->minimizeMemory)) return;
		if (this->checkParameter("usefixedhuberscale", name, value, this->useFixedHuberScale)) return;
		if (this->checkParameter("huberscale", name, value, this->huberScale)) return;
		if (this->checkParameter("usetdistribution", name, value, this->useTDistribution)) return;
		if (this->checkParameter("nufixed", name, value, this->nuFixed)) return;
		if (this->checkParameter("mufixed", name, value, this->muFixed)) return;
		if (this->checkParameter("sigmafixed", name, value, this->sigmaFixed)) return;
		if (this->checkParameter("defaultnu", name, value, this->defaultNu)) return;
		if (this->checkParameter("defaultmu", name, value, this->defaultMu)) return;
		if (this->checkParameter("defaultsigma", name, value, this->defaultSigma)) return;
		if (this->checkParameter("inlierpercentile", name, value, this->inlierPercentile)) return;
		if (this->checkParameter("maxpixeloutlier", name, value, this->maxPixelOutlier)) return;
		if (this->checkParameter("maxpixeldiscard", name, value, this->maxPixelDiscard)) return;
		if (this->checkParameter("maxenergyfit", name, value, this->maxEnergyFit)) return;
		if (this->checkParameter("weightconstant", name, value, this->weightConstant)) return;
		if (this->checkParameter("trackingmaxlevel", name, value, this->trackingMaxLevel)) return;
		if (this->checkParameter("pointdetectionlevels", name, value, this->pointDetectionLevels)) return;
		if (this->checkParameter("numcandidates", name, value, this->numCandidates)) return;
		if (this->checkParameter("numblocksperdimension", name, value, this->numBlocksPerDimension)) return;
		if (this->checkParameter("mingradadd", name, value, this->minGradAdd)) return;
		if (this->checkParameter("maxunmappedframes", name, value, this->maxUnmappedFrames)) return;
		if (this->checkParameter("maxepllengthfactor", name, value, this->maxEplLengthFactor)) return;
		if (this->checkParameter("minepllengthskip", name, value, this->minEplLengthSkip)) return;
		if (this->checkParameter("stereomaxenergy", name, value, this->stereoMaxEnergy)) return;
		if (this->checkParameter("secondbestradius", name, value, this->secondBestRadius)) return;
		if (this->checkParameter("epilinesigma", name, value, this->epiLineSigma)) return;
		if (this->checkParameter("subpixeliterations", name, value, this->subpixelIterations)) return;
		if (this->checkParameter("subpixelstepthreshold", name, value, this->subpixelStepThreshold)) return;
		if (this->checkParameter("maxviewchange", name, value, this->maxViewChange)) return;
		if (this->checkParameter("candidateoptiterations", name, value, this->candidateOptIterations)) return;
		if (this->checkParameter("mindisttoactivate", name, value, this->minDistToActivate)) return;
		if (this->checkParameter("maxcandidateuncertainty", name, value, this->maxCandidateUncertainty)) return;
		if (this->checkParameter("mincandidatequality", name, value, this->minCandidateQuality)) return;
		if (this->checkParameter("doonlytemporalopt", name, value, this->doOnlyTemporalOpt)) return;
		if (this->checkParameter("printsummary", name, value, this->printSummary)) return;
		if (this->checkParameter("showfullreport", name, value, this->showFullReport)) return;
		if (this->checkParameter("minoptimizationgrad", name, value, this->minOptimizationGrad)) return;
		if (this->checkParameter("minbaiterations", name, value, this->minBAIterations)) return;
		if (this->checkParameter("maxbaiterations", name, value, this->maxBAIterations)) return;
		if (this->checkParameter("optmaxlevel", name, value, this->optMaxLevel)) return;
		if (this->checkParameter("numactivepoints", name, value, this->numActivePoints)) return;
		if (this->checkParameter("maxtemporalkeyframes", name, value, this->maxTemporalKeyframes)) return;
		if (this->checkParameter("maxcovisiblekeyframes", name, value, this->maxCovisibleKeyframes)) return;
		if (this->checkParameter("numalwayskeepkeyframes", name, value, this->numAlwaysKeepKeyframes)) return;
		if (this->checkParameter("minpointcovisible", name, value, this->minPointCovisible)) return;
		if (this->checkParameter("maxlightcovisible", name, value, this->maxLightCovisible)) return;
		if (this->checkParameter("minnumkftoconsidernew", name, value, this->minNumKFToConsiderNew)) return;
		if (this->checkParameter("minnumgoodobservations", name, value, this->minNumGoodObservations)) return;
		if (this->checkParameter("newkfdistweight", name, value, this->newKFDistWeight)) return;
		if (this->checkParameter("newkfusageweight", name, value, this->newKFUsageWeight)) return;
		if (this->checkParameter("newkfaffineweight", name, value, this->newKFAffineWeight)) return;
		if (this->checkParameter("newkfresidualweight", name, value, this->newKFResidualWeight)) return;
		if (this->checkParameter("minnummappedframestocreatekf", name, value, this->minNumMappedFramesToCreateKF)) return;
		if (this->checkParameter("showdepthmap", name, value, this->showDepthMap)) return;
		if (this->checkParameter("showdepthmaplvl", name, value, this->showDepthMapLvl)) return;

		// not found
		std::cout << "Could not parse argument: " << argument << std::endl;
	}

	bool Settings::compare(const std::string &value)
	{
		if (value.compare("true") == 0)
		{
			return true;
		}
		else if (value.compare("false") == 0)
		{
			return false;
		}
		else
		{
			std::cout << "Settings::compare() could not compare: " << value << " (false as default)" << std::endl;
			return false;
		}
	}

	bool Settings::checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, std::string &var)
	{
		if (nameValue.compare(name) == 0)
		{
			var = value;
			return true;
		}
		return false;
	}

	bool Settings::checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, bool &var)
	{
		if (nameValue.compare(name) == 0)
		{
			var = this->compare(value);
			return true;
		}
		return false;
	}

	bool Settings::checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, int &var)
	{
		if (nameValue.compare(name) == 0)
		{
			var = std::atoi(value.c_str());
			return true;
		}
		return false;
	}

	bool Settings::checkParameter(const std::string& name, const std::string& nameValue, const std::string& value, float &var)
	{
		if (nameValue.compare(name) == 0)
		{
			var = static_cast<float>(std::atof(value.c_str()));
			return true;
		}
		return false;
	}
}
