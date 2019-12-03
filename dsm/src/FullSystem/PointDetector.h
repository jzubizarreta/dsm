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
#include <array>
#include <memory>
#include <random>
#include <atomic>

#include <Eigen/Core>

namespace dsm
{
	class Frame;
	class IVisualizer;
	class WorkerThreadPool;

	// Selected pixels will be homogeneously distributed in the image
	// and will have enough intensity contrast with its neighbours
	class PointDetector
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		PointDetector(int w, int h, int numLevels = 2, int numBlocks = 20, float threshold = 7.f,
					  const std::shared_ptr<WorkerThreadPool>& threadPool = nullptr);
		~PointDetector();

		// detects pixels for the input frame
		// it will generate a mask with selected pixels
		// frame: input frame for pixel selection
		// mask: out selected pixels. if > 0 tells the level in which the pixel was selected
		// return: number of selected pixels
		int detect(const std::shared_ptr<Frame>& frame, int numPoints, int32_t* const mask,
				   IVisualizer* outputWrapper = nullptr);

	private:

		struct CandidatePixel
		{
			int idx;				// idx in zero lvl image
			int lvl;				// detected lvl
			float mag;				// pixel gradient magnitude

			CandidatePixel() : idx(-1), lvl(-1), mag(-1.f)
			{}
		};

		void calcThresholds(const float* gradMag2, int yMin, int yMax);
		void smoothThresholdMap();

		// function for all image
		int selectPixels(const int winSize, int32_t* const mask, int yMin, int yMax) const;

		// function for each coarse window
		int selectMaxPixel(const int row, const int col, const int winSize, int32_t* const mask) const;

		// recursive function for each fine window
		void selectMaxPixelRecursive(const int oy, const int ox, const int rowIdx, const int colIdx,
									 const int numFineWindows, const int fineWindowSize, const int lvl, 
									 int& maxLvl, std::vector<CandidatePixel>& salientPixels) const;

		// recursive function for all cancidates
		int selectBestPixel(const std::vector<CandidatePixel>& salientPixels, const int row, const int col,
							const int numBlocks, const int lvl, int32_t* const mask) const;

		void selectMaxPixelLvl(const int row, const int col, const int winSize, int& maxLvl, CandidatePixel& candidate) const;

	private:

		// image size
		std::vector<int> width;
		std::vector<int> height;

		std::vector<int> maxWidth;
		std::vector<int> maxHeight;

		// image gradients
		std::vector<const float*> imgGradients;

		// number of pyramid levels
		const int numPyramids;

		// additive threshold
		const float additiveThreshold;

		// window size for maximal suppresion
		// the size is adaptative
		int window;

		// block size for threshold calculation
		int blockSizeWidth;		// size in pixels
		int blockSizeHeight;
		int numBlockWidth;
		int numBlockHeight;

		// image of thresholds: numBlockWidth*numBlockHeight
		float* thresholdMapDuplication;
		float* thresholdMap;

		// worker thread
		const std::shared_ptr<WorkerThreadPool> worker;
		std::vector<int> numSelectedVector;

		// structures for parallelization
		struct CalcThresholdParallel
		{
			CalcThresholdParallel(PointDetector& thisDetector, int aBegin, int anEnd,
								  const float* gradientImg);

			void operator()();

		private:
			PointDetector& detector;
			const int begin, end;

			const float* const gradient;
		};

		struct Selector
		{
			Selector(PointDetector& thisDetector, int aBegin, int anEnd, 
					 int threadNumber, int aWinSize, int32_t* const aMask);

			void operator()();

		private:
			PointDetector& detector;
			const int begin, end;
			const int threadNum;
			const int winSize;

			int32_t* const mask;
		};
	};
}
