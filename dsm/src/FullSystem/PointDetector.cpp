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

#include "PointDetector.h"
#include "DataStructures/Frame.h"
#include "Utils/UtilFunctions.h"
#include "Utils/Interpolation.h"
#include "Utils/Kernel.h"
#include "Statistics/Statistics.h"
#include "Thread/WorkerThreadPool.h"

#include "Visualizer/IVisualizer.h"

#include "dsm/BuildFlags.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

namespace dsm
{
	// Detector
	PointDetector::PointDetector(int w, int h, int numLevels, int numBlocks, float threshold,
								 const std::shared_ptr<WorkerThreadPool>& threadPool) :
		numPyramids(numLevels),
		additiveThreshold(threshold),
		window(5),
		worker(threadPool)
	{
		// image pyramid sizes
		this->width.resize(this->numPyramids + 1);
		this->height.resize(this->numPyramids + 1);
		this->maxWidth.resize(this->numPyramids + 1);
		this->maxHeight.resize(this->numPyramids + 1);
		for (int i = 0; i <= this->numPyramids; ++i)
		{
			this->width[i] = w >> i;
			this->height[i] = h >> i;
			this->maxWidth[i] = this->width[i] - 2;
			this->maxHeight[i] = this->height[i] - 2;
		}

		this->imgGradients.resize(this->numPyramids + 1);

		// get the best block size
		this->blockSizeWidth = (int)std::round((float)w / (float)numBlocks);
		this->blockSizeHeight = (int)std::round((float)h / (float)numBlocks);

		this->numBlockWidth = (int)ceil((float)w / this->blockSizeWidth);
		this->numBlockHeight = (int)ceil((float)h / this->blockSizeHeight);

		this->thresholdMap = (float*)Eigen::internal::aligned_malloc(this->numBlockWidth * this->numBlockHeight * sizeof(float));
		this->thresholdMapDuplication = (float*)Eigen::internal::aligned_malloc(this->numBlockWidth * this->numBlockHeight * sizeof(float));
	}

	PointDetector::~PointDetector()
	{
		Eigen::internal::aligned_free(this->thresholdMap);
		Eigen::internal::aligned_free(this->thresholdMapDuplication);
	}

	int PointDetector::detect(const std::shared_ptr<Frame>& frame, int numPoints, int32_t* const mask,
							  IVisualizer* outputWrapper)
	{
		assert(mask != nullptr);

		// set gradient pyramids
		// do this before to avoid deadlocks during parallel processing
		for (int lvl = 0; lvl <= this->numPyramids; ++lvl)
		{
			imgGradients[lvl] = frame->gradient(lvl);
		}

		// compute gradient histograms and thresholds
		if (this->worker)
		{
			const int numThreads = (int)this->worker->getNumThreads();
			const int step = (this->numBlockHeight + numThreads - 1) / numThreads;
		
			int start = 0;
			int end = 0;
		
			for (int t = 0; t < numThreads; ++t)
			{
				start = end;
				end = std::min(end + step, this->numBlockHeight);
		
				this->worker->addJob(CalcThresholdParallel(*this, start, end, imgGradients[0]));
			}
			this->worker->wait();
		}
		else
		{
			this->calcThresholds(imgGradients[0], 0, this->numBlockHeight);
		}	

		// smooth threshold map to obtain more point on
		// edges than in white walls
		this->smoothThresholdMap();

		Utils::Time t1 = std::chrono::steady_clock::now();
		
		// iterate until the desired number of points is selected
		// we will set a maximum number of iterations to avoid infinite loop
		// in each iteration the window size is adapted heuristically
		int numSelected = 0;

		for (int it = 0; it <= 1; ++it)
		{
			// reset mask
			std::fill(mask, mask + this->width[0] * this->height[0], -1);
			numSelected = 0;

			// start from coarse level and select pixels recursively
			int coarseWindow = this->window << this->numPyramids;

			if (this->worker)
			{
				const int numThreads = (int)this->worker->getNumThreads();
				numSelectedVector.resize(numThreads);

				int numItems = (int)ceil((float)this->height[0] / (float)coarseWindow);

				int step = (numItems + numThreads - 1) / numThreads;
				step *= coarseWindow;	// in pixels

				int start = 0;
				int end = 0;

				for (int t = 0; t < numThreads; ++t)
				{
					start = end;
					end = std::min(end + step, this->height[0]);

					numSelectedVector[t] = 0;

					this->worker->addJob(Selector(*this, start, end, t, coarseWindow, mask));
				}
				this->worker->wait();

				for (int t = 0; t < numThreads; ++t)
				{
					numSelected += numSelectedVector[t];
				}
			}
			else
			{
				numSelected = this->selectPixels(coarseWindow, mask, 0, this->height[0]);
			}

			// change heuristically the window size based on the density
			const float ratioOfSucces = static_cast<float>(numSelected) / numPoints;
			this->window = static_cast<int>(sqrtf(ratioOfSucces)*this->window + 0.5f);

			// check dimensions
			// at least 3x3
			if (this->window < 3) this->window = 3;

			if (ratioOfSucces > 0.9f && ratioOfSucces < 1.1f)
			{
				break;
			}
		}

		Utils::Time t2 = std::chrono::steady_clock::now();
		//std::cout << "Detect points: " << Utils::elapsedTime(t1, t2) << "\n";

		// visualize
		if (outputWrapper && Settings::getInstance().debugShowPointDetection)
		{
			cv::Mat img(this->height[0], this->width[0], CV_8UC3);
			const float* image = frame->image(0);

			// copy image
			for (int i = 0; i < this->height[0]; ++i)
			{
				for (int j = 0; j < this->width[0]; ++j)
				{
					int id = i * this->width[0] + j;
					img.at<cv::Vec3b>(i, j)[0] = (unsigned char)image[id];
					img.at<cv::Vec3b>(i, j)[1] = (unsigned char)image[id];
					img.at<cv::Vec3b>(i, j)[2] = (unsigned char)image[id];
				}
			}

			// draw pixels
			for (int i = 0; i < this->height[0]; ++i)
			{
				for (int j = 0; j < this->width[0]; ++j)
				{
					int id = i * this->width[0] + j;
					if (mask[id] >= 0)
					{
						if (mask[id] == 0)
						{
							cv::circle(img, cv::Point(j, i), 2, cv::Scalar(0, 255, 0), -1);
						}
						else if (mask[id] == 1)
						{
							cv::circle(img, cv::Point(j, i), 2, cv::Scalar(255, 0, 0), -1);
						}
						else if (mask[id] == 2)
						{
							cv::circle(img, cv::Point(j, i), 2, cv::Scalar(0, 0, 255), -1);
						}
						else
						{
							cv::circle(img, cv::Point(j, i), 2, cv::Scalar(255, 102, 255), -1);
						}
					}
				}
			}

			outputWrapper->publishPointDetector(img);
		}

		return numSelected;
	}

	int PointDetector::selectPixels(const int winSize, int32_t* const mask, int yMin, int yMax) const
	{
		int numSelectedPoints = 0;

		// maximal pixel in the neighbourhood
		for (int y = yMin; y < yMax; y += winSize)
		{
			for (int x = 0; x < this->width[0]; x += winSize)
			{
				numSelectedPoints += this->selectMaxPixel(y, x, winSize, mask);
			}
		}

		return numSelectedPoints;
	}

	int PointDetector::selectMaxPixel(const int row, const int col, const int winSize, int32_t* const mask) const
	{
		// num zero lvl blocks in each dimension
		const int numFineWindows = (int)1 << this->numPyramids;	
		const int fineWindowSize = winSize / numFineWindows;

		// candidates for each fine block
		std::vector<CandidatePixel> salientPixels(numFineWindows*numFineWindows);

		// compute all fine windows
		int maxLvl = this->numPyramids;
		this->selectMaxPixelRecursive(row, col, 0, 0, numFineWindows, fineWindowSize, 
									  this->numPyramids, maxLvl, salientPixels);

		// select best candidates
		return this->selectBestPixel(salientPixels, 0, 0, numFineWindows, this->numPyramids, mask);
	}

	void PointDetector::selectMaxPixelRecursive(const int oy, const int ox, const int rowIdx, const int colIdx,
												const int numFineWindows, const int fineWindowSize, const int lvl, 
												int& maxLvl, std::vector<CandidatePixel>& salientPixels) const
	{
		// if lvl > 0 go to (lvl-1)
		if (lvl > 0)
		{
			const int newLvl = (lvl - 1);

			const int numBlockInLvlm1 = (int)1 << newLvl;
			int maxLvlChild = maxLvl;

			// try to select pixels in finer resolution
			for (int i = 0; i < 2; ++i)
			{
				const int yIdx = rowIdx + i*numBlockInLvlm1;
				for (int j = 0; j < 2; ++j)
				{
					const int xIdx = colIdx + j*numBlockInLvlm1;
					this->selectMaxPixelRecursive(oy, ox, yIdx, xIdx, 
												  numFineWindows, fineWindowSize, newLvl, 
												  maxLvlChild, salientPixels);
				}
			}

			// change parent maxLvl
			if (maxLvlChild < maxLvl)
			{
				maxLvl = std::max(lvl, maxLvlChild);
			}

			return;
		}

		// if lvl == 0, select the max pixel
		const int x = ox + colIdx*fineWindowSize;
		const int y = oy + rowIdx*fineWindowSize;
		const int index = colIdx + rowIdx*numFineWindows;

		this->selectMaxPixelLvl(y, x, fineWindowSize, maxLvl, salientPixels[index]);
	}

	int PointDetector::selectBestPixel(const std::vector<CandidatePixel>& salientPixels, const int row, const int col, 
									   const int numBlocks, const int lvl, int32_t* const mask) const
	{
		if (lvl > 0)
		{
			// try to select pixels in finer resolution
			int numSelected = 0;
			const int finerNumBlocks = numBlocks >> 1;
			for (int i = 0; i < 2; ++i)
			{
				const int y = row + i*finerNumBlocks;
				for (int j = 0; j < 2; ++j)
				{					
					int x = col + j*finerNumBlocks;

					numSelected += this->selectBestPixel(salientPixels, y, x, finerNumBlocks, lvl - 1, mask);
				}
			}

			if (numSelected > 0) return numSelected;
		}

		// best values
		int bestIdx = -1;
		float bestMag = -1.f;

		const int step = (int)1 << this->numPyramids;

		for (int r = row; r < (row + numBlocks); ++r)
		{
			const int index_y = r*step;
			for (int c = col; c < (col + numBlocks); ++c)
			{
				const int index = index_y + c;
				const CandidatePixel& candidate = salientPixels[index];

				if (candidate.lvl == lvl && candidate.mag > bestMag)
				{
					bestMag = candidate.mag;
					bestIdx = candidate.idx;
				}
			}
		}

		if (bestIdx > 0)
		{
			mask[bestIdx] = lvl;
			return 1;
		}

		return 0;
	}

	void PointDetector::selectMaxPixelLvl(const int row, const int col, const int winSize, int& maxLvl, CandidatePixel& candidate) const
	{
		// window boundary
		const int min_x = std::min(std::max(col, 2), this->maxWidth[0] - 1);
		const int min_y = std::min(std::max(row, 2), this->maxHeight[0] - 1);
		const int max_x = std::min(std::max(col + winSize, 2), this->maxWidth[0] - 1);
		const int max_y = std::min(std::max(row + winSize, 2), this->maxHeight[0] - 1);

		// output candidates
		float higherMag = -1.f;

		for (int y = row; y < max_y; ++y)
		{
			const int idxThr_y = (y / this->blockSizeHeight)*this->numBlockWidth;
			const int idxZero_y = y * this->width[0];

			for (int x = col; x < max_x; ++x)
			{
				// idx for threshold selection
				const int idxThr = (x / this->blockSizeWidth) + idxThr_y;

				// idx in zero lvl
				const int idxZero = x + idxZero_y;

				// gradient threshold
				float threshold = this->thresholdMap[idxThr];

				// iterate in different lvls until we found a salient pixel
				for (int lvl = 0; lvl <= maxLvl; ++lvl)
				{
					// gradient value
					const float* const gradMag = this->imgGradients[lvl];

					float mag;
					if (lvl == 0)
					{
						// do not interpolate
						mag = gradMag[idxZero];
					}
					else
					{
						// pixel location in lvl
						const float u = ((x + 0.5f) / ((int)1 << lvl)) - 0.5f;
						const float v = ((y + 0.5f) / ((int)1 << lvl)) - 0.5f;

						// skip pixels without gradient information
						if (!(v > 1 && v < this->maxHeight[lvl] && u > 1 && u < this->maxWidth[lvl])) break;

						// interpolate
						mag = bilinearInterpolation(gradMag, u, v, this->width[lvl]);
					}

					// check if salient
					if (mag > threshold && 
						((lvl < maxLvl) || (maxLvl == lvl && mag > higherMag)))
					{
						maxLvl = lvl;
						higherMag = mag;

						candidate.lvl = lvl;
						candidate.idx = idxZero;
						candidate.mag = mag;
						
						break;
					}

					// downweight threshold in each lvl
					threshold *= 0.75f;
				}
			}
		}
	}

	void PointDetector::calcThresholds(const float* gradMag2, int yMin, int yMax)
	{
		for (int row = yMin; row < yMax; ++row)
		{
			// skip pixels without gradient information
			const int vInit = std::max(1, row * this->blockSizeHeight);
			const int vEnd = std::min(vInit + this->blockSizeHeight, this->maxHeight[0]);

			for (int col = 0; col < this->numBlockWidth; ++col)
			{
				// reset histogram
				const int uInit = std::max(1, col * this->blockSizeWidth);
				const int uEnd = std::min(uInit + this->blockSizeWidth, this->maxWidth[0]);

				// obtain all gradient magnitudes in the block
				const int blockSize = (uEnd - uInit)*(vEnd - vInit);
				std::vector<float> allGradientMag(blockSize);
				int idx = 0;

#if defined(ENABLE_SSE)

				int gap = (uEnd - uInit) % 4;
				int maxU = uEnd - gap;
				
				for (int y = vInit; y < vEnd; ++y)
				{
					const int y_idx = y * this->width[0];
					for (int x = uInit; x < maxU; x += 4)
					{
						_mm_storeu_ps(allGradientMag.data() + idx, _mm_loadu_ps(gradMag2 + y_idx + x));
						idx += 4;
					}

					// compute the rest by hand
					for (int x = maxU; x < uEnd; x++)
					{
						allGradientMag[idx] = gradMag2[y_idx + x];
						++idx;
					}
				}
#else
				// compute histogran for each block
				for (int y = vInit; y < vEnd; ++y)
				{
					const int y_idx = y * this->width[0];
					for (int x = uInit; x < uEnd; ++x)
					{
						allGradientMag[idx] = gradMag2[y_idx + x];
						++idx;
					}
				}
#endif
				// compute threshold using median value plus an additive constant
				this->thresholdMapDuplication[row*this->numBlockWidth + col] = sqrt(dsm::median(allGradientMag)) + this->additiveThreshold;
			}
		}
	}

	void PointDetector::smoothThresholdMap()
	{
		// we will use a gaussian kernel 3x3
		for (int row = 0; row < this->numBlockHeight; ++row)
		{
			const int row_idx = row * this->numBlockWidth;
			for (int col = 0; col < this->numBlockWidth; ++col)
			{
				const int idx = row_idx + col;
				Kernel::convolveSafe<Kernel::Gaussian3x3>(this->thresholdMapDuplication, col, row, this->numBlockWidth,
														  this->numBlockHeight, this->thresholdMap[idx]);

				// use squared values to avoid sqrt operation
				this->thresholdMap[idx] *= this->thresholdMap[idx];
			}
		}
	}

	// classes for parallelization

	PointDetector::CalcThresholdParallel::CalcThresholdParallel(PointDetector& thisDetector, int aBegin, int anEnd,
																const float* gradientImg) :
		detector(thisDetector), begin(aBegin), end(anEnd), gradient(gradientImg)
	{
	}

	void PointDetector::CalcThresholdParallel::operator()()
	{
		this->detector.calcThresholds(this->gradient, this->begin, this->end);
	}

	PointDetector::Selector::Selector(PointDetector& thisDetector, int aBegin, int anEnd, 
									  int threadNumber, int aWinSize, int32_t* const aMask) :
		detector(thisDetector), begin(aBegin), end(anEnd), 
		threadNum(threadNumber), winSize(aWinSize), mask(aMask)
	{
	}

	void PointDetector::Selector::operator()()
	{
		this->detector.numSelectedVector[this->threadNum] += 
			this->detector.selectPixels(this->winSize, this->mask, this->begin, this->end);
	}
}
