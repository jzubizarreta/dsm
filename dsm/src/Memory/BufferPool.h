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

#include <mutex>
#include <unordered_map>
#include <deque>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include "Buffer.h"

namespace dsm
{
	template<typename Type>
	class BufferPool
	{

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Returns the global instance. Creates it when the method is first called.
		static BufferPool<Type>& getInstance();

		// get a buffer from the pool
		Buffer<Type>* popBuffer(const size_t numElements);

		// return a buffer to the pool
		void pushBuffer(Buffer<Type>* buffer);

		// clear all memory of the pool
		void clearPool();

		/** Dont forget to declare these two. You want to make sure they
		* are unacceptable otherwise you may accidentally get copies of
		* your singleton appearing. */
		BufferPool(BufferPool const&) = delete;
		void operator=(BufferPool const&) = delete;

	private:

		BufferPool();
		~BufferPool();

	private:

		// mutex
		std::mutex accessMutex;

		// all buffers created in this pool
		// this manages memory
		std::vector<std::unique_ptr<Buffer<Type>>> allBuffers;

		// available buffer than are not beeing used
		// ordered by buffer size
		// map <numElements, buffer>		
		std::unordered_map<size_t, std::deque<Buffer<Type>*>> availableBuffers;
	};

	// implementation
	template<typename Type>
	inline BufferPool<Type>::BufferPool()
	{}

	template<typename Type>
	inline BufferPool<Type>::~BufferPool()
	{
		this->clearPool();
	}

	template<typename Type>
	inline BufferPool<Type>& BufferPool<Type>::getInstance()
	{
		static BufferPool<Type> theBufferPool;
		return theBufferPool;
	}

	template<typename Type>
	inline Buffer<Type>* BufferPool<Type>::popBuffer(const size_t numElements)
	{
		Buffer<Type>* buffer = nullptr;

		std::lock_guard<std::mutex> lock(this->accessMutex);

		if (this->availableBuffers.find(numElements) != this->availableBuffers.end())
		{
			std::deque<Buffer<Type>*>& bufferVector = this->availableBuffers[numElements];
			if (!bufferVector.empty())
			{
				buffer = bufferVector.front();
				bufferVector.pop_front();
				
				return buffer;
			}
		}

		int id = (int)this->allBuffers.size();
		std::unique_ptr<Buffer<Type>> newBuffer = std::make_unique<Buffer<Type>>(numElements, id);
		buffer = newBuffer.get();
		this->allBuffers.push_back(std::move(newBuffer));
		
		return buffer;
	}

	template<typename Type>
	inline void BufferPool<Type>::pushBuffer(Buffer<Type>* buffer)
	{
		if (buffer == nullptr) return;

		std::lock_guard<std::mutex> lock(this->accessMutex);

		// crash if the std::unique_ptr has been destroyed before and 
		// buffer is pointer to rubish
		assert(buffer->idx < (int)this->allBuffers.size());

		if (this->availableBuffers.find(buffer->numElements) == this->availableBuffers.end())
		{
			this->availableBuffers.emplace(buffer->numElements, std::deque<Buffer<Type>*>());
		}

		this->availableBuffers[buffer->numElements].push_back(buffer);
	}

	template<typename Type>
	inline void BufferPool<Type>::clearPool()
	{
		std::lock_guard<std::mutex> lock(this->accessMutex);
		this->availableBuffers.clear();
		this->allBuffers.clear();
	}
}