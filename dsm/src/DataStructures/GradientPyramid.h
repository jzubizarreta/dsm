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
#include <mutex>

#include "ImagePyramid.h"
#include "Memory/BufferPool.h"
#include "Utils/GlobalCalibration.h"
#include "dsm/BuildFlags.h"

namespace dsm
{
	template<typename T, typename S = T>
	class GradientPyramid
	{
	public:

		GradientPyramid(int numLevels, const ImagePyramid<S>& imgPyramid);
		~GradientPyramid();

		// gradients for each lvl
		const T* const gx(int lvl) const;
		const T* const gy(int lvl) const;
		const T* const gradient(int lvl) const;

		// returns buffers to the pool
		void freeMemory();

	protected:

		// initialize gradient
		void initialize(int lvl);

		// build with template specialization
		void buildGradients(int lvl);

	protected:

		std::mutex mut;

		// reference to image pyramids
		const ImagePyramid<S>& imagePyramid;

		// image gradients
		std::vector<Buffer<T>*> gxPyramid;
		std::vector<Buffer<T>*> gyPyramid;
		std::vector<Buffer<T>*> gradientPyramid;
		std::vector<bool> valid;

		const int levels;
	};

	// Implementation

	template<typename T, typename S>
	inline GradientPyramid<T, S>::GradientPyramid(int numLevels, const ImagePyramid<S>& imgPyramid) :
		levels(numLevels), 
		gxPyramid(numLevels, nullptr),
		gyPyramid(numLevels, nullptr),
		gradientPyramid(numLevels, nullptr),
		valid(numLevels, false),
		imagePyramid(imgPyramid)
	{
	}

	template<typename T, typename S>
	inline GradientPyramid<T, S>::~GradientPyramid()
	{
		this->freeMemory();
	}

	template<typename T, typename S>
	inline const T* const GradientPyramid<T, S>::gx(int lvl) const
	{
		assert(lvl < this->levels);

		if (!this->valid[lvl])
		{
			const_cast<GradientPyramid<T, S>*>(this)->initialize(lvl);
		}

		return gxPyramid[lvl]->data;
	}

	template<typename T, typename S>
	inline const T* const GradientPyramid<T, S>::gy(int lvl) const
	{
		assert(lvl < this->levels);

		if (!this->valid[lvl])
		{
			const_cast<GradientPyramid<T, S>*>(this)->initialize(lvl);
		}

		return gyPyramid[lvl]->data;
	}

	template<typename T, typename S>
	inline const T* const GradientPyramid<T, S>::gradient(int lvl) const
	{
		assert(lvl < this->levels);

		if (!this->valid[lvl])
		{
			const_cast<GradientPyramid<T, S>*>(this)->initialize(lvl);
		}

		return gradientPyramid[lvl]->data;
	}

	template<typename T, typename S>
	inline void GradientPyramid<T, S>::freeMemory()
	{
		// return buffers to memory pool
		for (int lvl = 0; lvl < this->levels; ++lvl)
		{
			auto& bufferPool = BufferPool<T>::getInstance();

			if (this->valid[lvl])
			{
				bufferPool.pushBuffer(this->gxPyramid[lvl]);
				bufferPool.pushBuffer(this->gyPyramid[lvl]);
				bufferPool.pushBuffer(this->gradientPyramid[lvl]);

				this->gxPyramid[lvl] = nullptr;
				this->gyPyramid[lvl] = nullptr;
				this->gradientPyramid[lvl] = nullptr;

				this->valid[lvl] = false;
			}
		}
	}

	template<typename T, typename S>
	inline void GradientPyramid<T, S>::initialize(int lvl)
	{
		std::lock_guard<std::mutex> lock(this->mut);

		// double check to allow multiple threads call this function but compute only once
		if (this->valid[lvl]) return;

		const auto& calib = GlobalCalibration::getInstance();
		const int width = calib.width(lvl);
		const int height = calib.height(lvl);

		// allocate memory
		const int size = width*height;
		auto& bufferPool = BufferPool<T>::getInstance();
		this->gxPyramid[lvl] = bufferPool.popBuffer(size);
		this->gyPyramid[lvl] = bufferPool.popBuffer(size);
		this->gradientPyramid[lvl] = bufferPool.popBuffer(size);

		// build the gradients
		this->buildGradients(lvl);
		this->valid[lvl] = true;
	}

	template<typename T, typename S>
	inline void GradientPyramid<T, S>::buildGradients(int lvl)
	{
		const auto& calib = GlobalCalibration::getInstance();
		const int width = calib.width(lvl);
		const int height = calib.height(lvl);

		// we will skip a border of 1 pixel
		const S* const imgPtr = this->imagePyramid.image(lvl);
		T* const gxPtr = this->gxPyramid[lvl]->data;
		T* const gyPtr = this->gyPyramid[lvl]->data;
		T* const magPtr = this->gradientPyramid[lvl]->data;

		for (int y = 1; y < height - 1; ++y)
		{
			const int rowIdx = y*width;
			for (int x = 1; x < width - 1; ++x)
			{
				const int idx = rowIdx + x;
				gxPtr[idx] = (T)0.5*(imgPtr[idx + 1] - imgPtr[idx - 1]);
				gyPtr[idx] = (T)0.5*(imgPtr[idx + width] - imgPtr[idx - width]);
				magPtr[idx] = gxPtr[idx] * gxPtr[idx] + gyPtr[idx] * gyPtr[idx];
			}
		}
	}
}