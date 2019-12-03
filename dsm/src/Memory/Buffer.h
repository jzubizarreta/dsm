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

#include <Eigen/Core>

namespace dsm
{
	template<typename Type>
	class Buffer
	{

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Type* const data;				// buffer
		const size_t numElements;		// number of elements of Type
		const size_t idx;				// id in the pool vector

		// constructor: create the buffer and saves the size internally
		inline Buffer(size_t size, size_t id) :
			numElements(size), 
			idx(id),
			data((Type*)Eigen::internal::aligned_malloc(size * sizeof(Type)))
		{
		}

		// destructor: deletes buffer memory
		inline ~Buffer()
		{
			Eigen::internal::aligned_free(this->data);
		}

		// avoid copying
		Buffer(Buffer const&) = delete;
		void operator=(Buffer const&) = delete;
	};
}