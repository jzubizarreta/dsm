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

#include "Memory/BufferPool.h"
#include "Utils/GlobalCalibration.h"
#include "Utils/Kernel.h"
#include "dsm/BuildFlags.h"

namespace dsm
{
	template<typename T>
	class ImagePyramid
	{
	public:

		ImagePyramid(int numLevels, const unsigned char* const img);
		~ImagePyramid();

		// images for each lvl
		const T* const image(int lvl) const;

		// returns buffers to the pool
		void freeMemory(bool onlyPyramids = false);

	protected:

		// initialize image
		void initialize(int lvl);	

		// build with template specialization
		void buildImage(int lvl);

	protected:

		std::mutex mut;

		std::vector<Buffer<T>*> imagePyramid;
		std::vector<bool> valid;

		const int levels;
	};

	// Implementation

	template<typename T>
	inline ImagePyramid<T>::ImagePyramid(int numLevels, const unsigned char* const img) : 
		levels(numLevels), 
		imagePyramid(numLevels, nullptr),
		valid(numLevels, false)
	{
		const auto& calib = GlobalCalibration::getInstance();
		const int32_t width = calib.width(0);
		const int32_t height = calib.height(0);

		// copy zero lvl image
		this->imagePyramid[0] = BufferPool<T>::getInstance().popBuffer(width*height);

		for (int i = 0; i < width*height; ++i)
		{
			this->imagePyramid[0]->data[i] = static_cast<T>(img[i]);
		}

		this->valid[0] = true;
	}

	template<typename T>
	inline ImagePyramid<T>::~ImagePyramid()
	{
		this->freeMemory(false);
	}

	template<typename T>
	inline const T* const ImagePyramid<T>::image(int lvl) const
	{
		assert(lvl < this->levels);

		if (!this->valid[lvl])
		{
			const_cast<ImagePyramid<T>*>(this)->initialize(lvl);
		}

		return imagePyramid[lvl]->data;
	}

	template<typename T>
	inline void ImagePyramid<T>::freeMemory(bool onlyPyramids)
	{
		int startLvl = 0;

		if (onlyPyramids) startLvl = 1;

		// return buffers to memory pool
		for (int lvl = startLvl; lvl < this->levels; ++lvl)
		{
			if (this->valid[lvl])
			{
				BufferPool<T>::getInstance().pushBuffer(this->imagePyramid[lvl]);
				this->imagePyramid[lvl] = nullptr;
				this->valid[lvl] = false;
			}
		}
	}

	template<typename T>
	inline void ImagePyramid<T>::initialize(int lvl)
	{
		// check recursively if we have the previous
		// lvl available
		if (!this->valid[lvl - 1])
		{
			this->initialize(lvl - 1);
		}

		std::lock_guard<std::mutex> lock(this->mut);

		// double check to allow multiple threads call this function but compute only once
		if (this->valid[lvl]) return;

		// source and destiny sizes
		const auto& calib = GlobalCalibration::getInstance();
		const int destWidth = calib.width(lvl);
		const int destHeight = calib.height(lvl);
		
		// allocate memory
		this->imagePyramid[lvl] = BufferPool<T>::getInstance().popBuffer(destWidth*destHeight);
		
		// build the image using optimizations when possible
		this->buildImage(lvl);
		this->valid[lvl] = true;
	}

	template<typename T>
	inline void ImagePyramid<T>::buildImage(int lvl)
	{
		// general implementation for all T

		const auto& calib = GlobalCalibration::getInstance();
		const int width = calib.width(lvl - 1);
		const int height = calib.height(lvl - 1);

		//gaps if the image is not multiple of 2
		const int wgap = width % 2;
		const int hgap = height % 2;
		const int wminusgap = width - wgap;
		const int hminusgap = height - hgap;

		// build image
		const T* const source = this->imagePyramid[lvl - 1]->data;
		T* dest = this->imagePyramid[lvl]->data;

		for (int y = 0; y < hminusgap; y += 2)
		{
			const int rowIdx = y*width;
			for (int x = 0; x < wminusgap; x += 2)
			{
				Kernel::convolve<Kernel::Box2x2>(source, rowIdx + x, width, *dest);
				dest++;
			}
		}
	}

	template<>
	inline void ImagePyramid<float>::buildImage(int lvl)
	{
		// special implementation for float images

		const auto& calib = GlobalCalibration::getInstance();
		const int width = calib.width(lvl - 1);
		const int height = calib.height(lvl - 1);

		// gaps if the image is not multiple of 2
		const int wgap = width % 2;
		const int hgap = height % 2;
		const int wminusgap = width - wgap;
		const int hminusgap = height - hgap;

		// build image
		const float* const source = this->imagePyramid[lvl - 1]->data;
		float* dest = this->imagePyramid[lvl]->data;

#if defined(ENABLE_SSE)

		// Do only if it does not break the 16-byte alignment
		// i.e. the image has a width divisible by 8
		if (wminusgap % 8 == 0)
		{
			const __m128 fac_025 = _mm_set_ps1(0.25f);

			for (int y = 0; y < hminusgap; y += 2)
			{
				const float* rowSource = source + y*width;
				for (int x = 0; x < wminusgap; x += 8)
				{
					// summation of 4 pixels at a time
					const __m128 sum = _mm_hadd_ps(_mm_add_ps(_mm_load_ps(rowSource), _mm_load_ps(rowSource + width)), 
												   _mm_add_ps(_mm_load_ps(rowSource + 4), _mm_load_ps(rowSource + 4 + width)));

					// store average
					_mm_store_ps(dest, _mm_mul_ps(sum, fac_025));

					dest += 4;
					rowSource += 8;
				}
			}

			return;
		}

#endif

		for (int y = 0; y < hminusgap; y += 2)
		{
			const int rowIdx = y*width;
			for (int x = 0; x < wminusgap; x += 2)
			{
				Kernel::convolve<Kernel::Box2x2>(source, rowIdx + x, width, *dest);
				dest++;
			}
		}
	}
}