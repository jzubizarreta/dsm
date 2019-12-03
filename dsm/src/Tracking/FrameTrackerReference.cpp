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

#include "FrameTrackerReference.h"
#include "DataStructures/Frame.h"
#include "DataStructures/ActivePoint.h"
#include "DataStructures/Pattern.h"
#include "Utils/GlobalCalibration.h"
#include "Utils/Settings.h"
#include "Utils/Projection.h"
#include "Utils/EigenTypes.h"
#include "Utils/Kernel.h"
#include "Utils/UtilFunctions.h"

#include <iostream>

namespace dsm
{
	FrameTrackerReference::FrameTrackerReference(const int32_t width, const int32_t height) :
		reference_(nullptr)
	{
		const auto& calib = GlobalCalibration::getInstance();
		const int levels = Settings::getInstance().trackingMaxLevel + 1;

		// initialize all vectors
		this->u_.resize(levels);
		this->v_.resize(levels);
		this->x_.resize(levels);
		this->y_.resize(levels);
		this->iDepth_.resize(levels);
		this->gx_.resize(levels);
		this->gy_.resize(levels);
		this->color_.resize(levels);
		this->weight_.resize(levels);
		this->numPoints_.resize(levels);
		this->iDepthMap_.resize(levels);
		this->validMap_.resize(levels);

		for (int lvl = 0; lvl < levels; ++lvl)
		{
			const int levelW = calib.width(lvl);
			const int levelH = calib.height(lvl);
			const int levelSize = levelW*levelH;

			this->u_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->v_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->x_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->y_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->iDepth_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->gx_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->gy_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->color_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->weight_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->iDepthMap_[lvl] = (float*)Eigen::internal::aligned_malloc(levelSize * sizeof(float));
			this->validMap_[lvl] = (bool*)Eigen::internal::aligned_malloc(levelSize * sizeof(bool));
		}

		const int size0 = calib.width(0)*calib.height(0);
		this->iDepthMapNonSmooth_ = (float*)Eigen::internal::aligned_malloc(size0 * sizeof(float));

		this->reset();
	}

	FrameTrackerReference::~FrameTrackerReference()
	{
		std::unique_lock<std::mutex> lock(this->accessMutex_);

		// delete all buffers
		for (int lvl = 0; lvl < this->u_.size(); ++lvl)
		{
			Eigen::internal::aligned_free((void*)this->u_[lvl]);
			Eigen::internal::aligned_free((void*)this->v_[lvl]);
			Eigen::internal::aligned_free((void*)this->x_[lvl]);
			Eigen::internal::aligned_free((void*)this->y_[lvl]);
			Eigen::internal::aligned_free((void*)this->iDepth_[lvl]);
			Eigen::internal::aligned_free((void*)this->gx_[lvl]);
			Eigen::internal::aligned_free((void*)this->gy_[lvl]);
			Eigen::internal::aligned_free((void*)this->color_[lvl]);
			Eigen::internal::aligned_free((void*)this->weight_[lvl]);
			Eigen::internal::aligned_free((void*)this->iDepthMap_[lvl]);
			Eigen::internal::aligned_free((void*)this->validMap_[lvl]);
		}

		Eigen::internal::aligned_free((void*)this->iDepthMapNonSmooth_);
	}

	Frame* FrameTrackerReference::reference() const
	{
		return this->reference_;
	}

	const float* FrameTrackerReference::iDepthMap(const int32_t level) const
	{
		return this->iDepthMap_[level];
	}

	float FrameTrackerReference::meanIDepth() const
	{
		return this->meanIDepth_;
	}

	const float* FrameTrackerReference::u(const int32_t level) const
	{
		return this->u_[level];
	}

	const float* FrameTrackerReference::v(const int32_t level) const
	{
		return this->v_[level];
	}

	const float* FrameTrackerReference::x(const int32_t level) const
	{
		return this->x_[level];
	}

	const float* FrameTrackerReference::y(const int32_t level) const
	{
		return this->y_[level];
	}

	const float* FrameTrackerReference::iDepth(const int32_t level) const
	{
		return this->iDepth_[level];
	}

	const float* FrameTrackerReference::gx(const int32_t level) const
	{
		return this->gx_[level];
	}

	const float* FrameTrackerReference::gy(const int32_t level) const
	{
		return this->gy_[level];
	}

	const float* FrameTrackerReference::color(const int32_t level) const
	{
		return this->color_[level];
	}

	const float* FrameTrackerReference::weight(const int32_t level) const
	{
		return this->weight_[level];
	}

	int FrameTrackerReference::numPoints(const int32_t level) const
	{
		return this->numPoints_[level];
	}

	void FrameTrackerReference::setFirstFrameResidual(float val)
	{
		this->firstFrameResidual_ = val;
	}

	float FrameTrackerReference::firstFrameResidual() const
	{
		return this->firstFrameResidual_;
	}

	void FrameTrackerReference::reset()
	{
		std::unique_lock<std::mutex> lock(this->accessMutex_);

		// invalidate all the data, do not release memory
		for (int lvl = 0; lvl < this->numPoints_.size(); ++lvl)
		{
			this->numPoints_[lvl] = 0;
		}

		this->reference_ = nullptr;

		this->meanIDepth_ = -1.f;
		this->firstFrameResidual_ = -1.f;
	}

	void FrameTrackerReference::setNewReference(const std::vector<std::shared_ptr<Frame>>& activeKeyframes)
	{
		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Mat33f& K = calib.matrix3f(0);
		const Eigen::Mat33f& Kinv = calib.invMatrix3f(0);
		const int width0 = calib.width(0);
		const int height0 = calib.height(0);

		this->reset();		// has its own lock

		std::unique_lock<std::mutex> lock(this->accessMutex_);

		// the last active keyframe will be the reference
		this->reference_ = activeKeyframes.back().get();
		const Sophus::SE3f worldToReference = this->reference_->camToWorld().inverse();

		// reset zero level inverse deth map
		// use mask to allow zero/negative values: points at infinity
		const int size0 = width0*height0;
		std::fill(this->iDepthMap_[0], this->iDepthMap_[0] + size0, 0.f);
		std::fill(this->validMap_[0], this->validMap_[0] + size0, false);
		std::fill(this->iDepthMapNonSmooth_, this->iDepthMapNonSmooth_ + size0, 0.f);

		float sumIDepth = 0.f;
		float number = 0.f;

		const int border = Pattern::padding();

		// generate inverse depth maps for zero level
		// use all active keyframes except the last one that is empty of points
		for (int i = 0; i < activeKeyframes.size() - 1; ++i)
		{
			// relative pose data
			const Sophus::SE3f relPose = worldToReference * activeKeyframes[i]->camToWorld();
			const Eigen::Mat33f KRKinv = K * relPose.rotationMatrix() * Kinv;
			const Eigen::Vec3f Kt = K * relPose.translation();

			// use all active points				
			Eigen::Vec2f pt2d;
			float newIDepth;

			const auto& activePoints = activeKeyframes[i]->activePoints();
			for (const auto& ap : activePoints)
			{
				// check if the point was visible in the last keyframe
				if (activeKeyframes[i].get() == this->reference_ ||
					ap->visibility(this->reference_->keyframeID()) == Visibility::VISIBLE)
				{
					// TODO: avoid projecting active points to the reference so many times
					if (!Utils::project(ap->u(0), ap->v(0), ap->iDepth(),
										width0, height0, KRKinv, Kt, 
										pt2d, newIDepth))
					{
						continue;
					}

					sumIDepth += newIDepth;
					number++;

					const int u = (int)(pt2d[0] + 0.5f);
					const int v = (int)(pt2d[1] + 0.5f);

					// check image boundaries
					if (!(u >= border && u < (width0 - border) && v >= border && v < (height0 - border)))
					{
						continue;
					}

					// for each point, set the pattern in zero lvl
					for (int j = 0; j < Pattern::size(); ++j)
					{
						const int uj = u + Pattern::at(j, 0);
						const int vj = v + Pattern::at(j, 1);

						const int index = uj + vj * width0;
						this->iDepthMapNonSmooth_[index] = newIDepth;
						this->validMap_[0][index] = true;
					}					
				}
			}
		}

		if (!(number > 0.1f)) return;

		this->meanIDepth_ = sumIDepth / number;

		Utils::Time t4 = std::chrono::steady_clock::now();

		// smooth inverse depth map with gaussian kernel 5x5
		for (int32_t y = 2; y < height0 - 2; y++)
		{
			const int rowIdx = y*width0;
			for (int32_t x = 2; x < width0 - 2; x++)
			{
				const int idx = rowIdx + x;
				if (this->validMap_[0][idx])
				{
					Kernel::convolve<Kernel::Gaussian5x5>(this->iDepthMapNonSmooth_, this->validMap_[0],
														  idx, width0, this->iDepthMap_[0][idx]);
				}
			}
		}

		Utils::Time t1 = std::chrono::steady_clock::now();

		// generate inverse depth maps for the rest of levels
		// assign a depth value if at least one of the 
		// source pixels has a depth value
		for (int lvl = 1; lvl < this->iDepthMap_.size(); ++lvl)
		{
			const int32_t width = calib.width(lvl - 1);
			const int32_t height = calib.height(lvl - 1);

			const float* const iDepthSource = this->iDepthMap_[lvl - 1];
			const bool* const validSource = this->validMap_[lvl - 1];
			float* iDepthDest = this->iDepthMap_[lvl];
			bool* validDest = this->validMap_[lvl];

			for (int32_t y = 0; y < height; y += 2)
			{
				const int rowIdx = y * width;
				for (int32_t x = 0; x < width; x += 2)
				{
					*validDest = Kernel::convolve<Kernel::Box2x2>(iDepthSource, validSource, 
																  rowIdx + x, width,
																  *iDepthDest);
					validDest++;
					iDepthDest++;
				}
			}
		}

		Utils::Time t2 = std::chrono::steady_clock::now();

		//std::cout << "Ref pyramids: " << Utils::elapsedTime(t1, t2) << std::endl;
		//std::cout << "Ref smooth: " << Utils::elapsedTime(t4, t1) << std::endl;
	}

	bool FrameTrackerReference::generateReferencePointCloud(const int32_t level)
	{
		const auto& calib = GlobalCalibration::getInstance();
		const auto& settings = Settings::getInstance();

		std::unique_lock<std::mutex> lock(this->accessMutex_);

		if (this->numPoints_[level] > 0)
		{
			// already computed
			return false;
		}

		assert(this->reference_ != nullptr);

		// frame data
		const float* imagePtr = this->reference_->image(level);
		const float* gxPtr = this->reference_->gx(level);
		const float* gyPtr = this->reference_->gy(level);
		const float* gradPtr = this->reference_->gradient(level);
		const float* iDepthPtr = this->iDepthMap_[level];	// inverse depth map referenced to left frame
		const bool* validPtr = this->validMap_[level];

		const float weightConstant = settings.weightConstant;

		// camera data
		const Eigen::Mat33f& invK = calib.invMatrix3f(level);
		const int width = calib.width(level);
		const int height = calib.height(level);

		int number = 0;

		for (int row = 1; row < height - 1; ++row)
		{
			const int rowIdx = row*width;
			for (int col = 1; col < width - 1; ++col)
			{
				const int idx = rowIdx + col;

				if (!validPtr[idx]) continue;

				this->u_[level][number] = (float)col;
				this->v_[level][number] = (float)row;
				this->x_[level][number] = col*invK(0, 0) + invK(0, 2);
				this->y_[level][number] = row*invK(1, 1) + invK(1, 2);
				this->iDepth_[level][number] = iDepthPtr[idx];
				this->gx_[level][number] = gxPtr[idx];
				this->gy_[level][number] = gyPtr[idx];
				this->color_[level][number] = imagePtr[idx];
				this->weight_[level][number] = sqrtf(weightConstant / (weightConstant + gradPtr[idx]));	// weight = sqrt(c^2 / (c^2 + grad^2))
				++number;
			}
		}

		this->numPoints_[level] = number;

		return true;
	}
}