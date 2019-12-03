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
	namespace Kernel
	{
		enum Type { UserDefined = 0, Box2x2 = 1, Box3x3 = 12, Gaussian3x3 = 3, Gaussian5x5 = 5 };

		template<int T>
		struct WrongType : public std::false_type {};

		// wrong call! implement the required Type
		template<int T>
		inline bool convolve(const float* const image, const bool* const mask,
							 const int idx, const int width, float& value,
							 const Eigen::Matrix<float, -1, -1>* const kernel = nullptr)
		{
			// should never enter here
			static_assert(WrongType<T>::value, "Kernel::convolve() not suitable template parameters");
			return false;
		}

		template<int T>
		inline bool convolve(const float* const image, const int idx, const int width, float& value,
							 const Eigen::Matrix<float, -1, -1>* const kernel = nullptr)
		{
			// should never enter here
			static_assert(WrongType<T>::value, "Kernel::convolve() not suitable template parameters");
			return false;
		}

		// convolve with image boundary check
		template<int T>
		inline bool convolveSafe(const float* const image, const int x, const int y,
								 const int width, const int height, float& value,
								 const Eigen::Matrix<float, -1, -1>* const kernel = nullptr)
		{
			// should never enter here
			static_assert(WrongType<T>::value, "Kernel::convolve() not suitable template parameters");
			return false;
		}

		// general implementation with a user defined kernel
		// the kernel has to be centerd in the idx pixel
		template<>
		inline bool convolve<UserDefined>(const float* const image, const bool* const mask,
										  const int idx, const int width, float& value,
										  const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			if (kernel == nullptr) { value = 0.f; return false; }

			const int rows = (int)(*kernel).rows() / 2;
			const int cols = (int)(*kernel).cols() / 2;

			float sum = 0.f;
			float num = 0.f;

			for (int r = -rows, i = 0; r <= rows; ++r, ++i)
			{
				const int rowIdx = r*width + idx;
				for (int c = -cols, j = 0; c <= cols; ++c, ++j)
				{
					const int idx = rowIdx + c;		
					if (mask[idx])
					{
						sum += image[idx] * (*kernel)(i, j);
						num += (*kernel)(i, j);
					}
				}
			}

			if (num > 0.1f) { value = sum / num; return true; }
			else { value = 0.f; return false; }
		}

		template<>
		inline bool convolve<UserDefined>(const float* const image, const int idx, const int width, float& value,
										  const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			if (kernel == nullptr) { value = 0.f; return false; }

			const int rows = (int)(*kernel).rows() / 2;
			const int cols = (int)(*kernel).cols() / 2;

			float sum = 0.f;
			float num = 0.f;

			for (int r = -rows, i = 0; r <= rows; ++r, ++i)
			{
				const int rowIdx = r*width + idx;
				for (int c = -cols, j = 0; c <= cols; ++c, ++j)
				{
					const int idx = rowIdx + c;
					sum += image[idx] * (*kernel)(i, j);
					num += (*kernel)(i, j);
				}
			}

			if (num > 0.1f) { value = sum / num; return true; }
			else { value = 0.f; return false; }
		}

		// 2x2 Box implementation
		// input kernel is ignored
		// 
		//	x	-	width	1	1
		//	-	-			1	1
		//

		template<>
		inline bool convolve<Box2x2>(const float* const image, const bool* const mask,
									 const int idx, const int width, float& value,
									 const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			float sum = 0.f;
			float num = 0.f;

			if (mask[idx]) { sum += image[idx]; num++; }
			if (mask[idx + 1]) { sum += image[idx + 1]; num++; }
			if (mask[idx + width]) { sum += image[idx + width]; num++; }
			if (mask[idx + width + 1]) { sum += image[idx + width + 1]; num++; }

			if (num > 0.1f) { value = sum / num; return true; }
			else { value = 0.f; return false; }
		}

		template<>
		inline bool convolve<Box2x2>(const float* const image, const int idx, const int width, float& value,
									 const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			float sum = image[idx];
			sum += image[idx + 1];
			sum += image[idx + width];
			sum += image[idx + width + 1];

			value = sum * 0.25f;

			return true;
		}

		// 3x3 Box implementation
		// input kernel is ignored
		//
		//	-	-	-			1	1	1
		//	-	x	-	with	1	1	1
		//	-	-	-			1	1	1
		//

		template<>
		inline bool convolve<Box3x3>(const float* const image, const bool* const mask,
									 const int idx, const int width, float& value,
									 const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			float sum = 0.f;
			float num = 0.f;

			if (mask[idx]) { sum += image[idx]; num++; }
			if (mask[idx + 1]) { sum += image[idx + 1]; num++; }
			if (mask[idx - 1]) { sum += image[idx - 1]; num++; }
			if (mask[idx + width]) { sum += image[idx + width]; num++; }
			if (mask[idx - width]) { sum += image[idx - width]; num++; }
			if (mask[idx + 1 + width]) { sum += image[idx + 1 + width]; num++; }
			if (mask[idx + 1 - width]) { sum += image[idx + 1 - width]; num++; }
			if (mask[idx - 1 + width]) { sum += image[idx - 1 + width]; num++; }
			if (mask[idx - 1 - width]) { sum += image[idx - 1 - width]; num++; }

			if (num > 0.1f) { value = sum / num; return true; }
			else { value = 0.f; return false; }
		}

		template<>
		inline bool convolve<Box3x3>(const float* const image, const int idx, const int width, float& value,
									 const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			float sum = image[idx];
			sum += image[idx + 1];
			sum += image[idx - 1];
			sum += image[idx + width];
			sum += image[idx - width];
			sum += image[idx + 1 + width];
			sum += image[idx + 1 - width];
			sum += image[idx - 1 + width];
			sum += image[idx - 1 - width];

			value = sum / 9.f;

			return true;
		}

		// 3x3 Gaussian implementation
		// input kernel is ignored
		//
		//	-	-	-			1	2	1
		//	-	x	-	with	2	4	2
		//	-	-	-			1	2	1
		//

		template<>
		inline bool convolve<Gaussian3x3>(const float* const image, const bool* const mask,
										  const int idx, const int width, float& value,
										  const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			float sum = 0.f;
			float num = 0.f;

			if (mask[idx]) { sum += 4.f*image[idx]; num += 4.f; }
			if (mask[idx + 1]) { sum += 2.f*image[idx + 1]; num += 2.f; }
			if (mask[idx - 1]) { sum += 2.f*image[idx - 1]; num += 2.f; }
			if (mask[idx + width]) { sum += 2.f*image[idx + width]; num += 2.f; }
			if (mask[idx - width]) { sum += 2.f*image[idx - width]; num += 2.f; }
			if (mask[idx + 1 + width]) { sum += image[idx + 1 + width]; num++; }
			if (mask[idx + 1 - width]) { sum += image[idx + 1 - width]; num++; }
			if (mask[idx - 1 + width]) { sum += image[idx - 1 + width]; num++; }
			if (mask[idx - 1 - width]) { sum += image[idx - 1 - width]; num++; }

			if (num > 0.1f) { value = sum / num; return true; }
			else { value = 0.f; return false; }
		}

		template<>
		inline bool convolve<Gaussian3x3>(const float* const image, const int idx, const int width, float& value,
										  const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			float sum = 4.f*image[idx];
			sum += 2.f*image[idx + 1];
			sum += 2.f*image[idx - 1];
			sum += 2.f*image[idx + width];
			sum += 2.f*image[idx - width];
			sum += image[idx + 1 + width];
			sum += image[idx + 1 - width];
			sum += image[idx - 1 + width];
			sum += image[idx - 1 - width];

			sum = value / 16.f;

			return true;
		}

		template<>
		inline bool convolveSafe<Gaussian3x3>(const float* const image, const int x, const int y, 
											  const int width, const int height, float& value,
											  const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			float sum = 0.f;
			float num = 0.f;

			const bool convolveUp = (y - 1) >= 0;
			const bool convolveDown = (y + 1) < height;
			const bool convolveLeft = (x - 1) >= 0;
			const bool convolveRight = (x + 1) < width;

			// convolve with image boundary check
			if (convolveUp)
			{
				const int rowIdx = (y - 1)*width + x;
				if (convolveLeft) { sum += image[rowIdx - 1]; num++; }
				if (convolveRight) { sum += image[rowIdx + 1]; num++; }
				sum += 2.f*image[rowIdx]; num += 2.f;
			}

			if (convolveDown)
			{
				const int rowIdx = (y + 1)*width + x;
				if (convolveLeft) { sum += image[rowIdx - 1]; num++; }
				if (convolveRight) { sum += image[rowIdx + 1]; num++; }
				sum += 2.f*image[rowIdx]; num += 2.f;
			}

			const int rowIdx = y*width + x;
			if (convolveLeft) { sum += 2.f*image[rowIdx - 1]; num += 2.f; }
			if (convolveRight) { sum += 2.f*image[rowIdx + 1]; num += 2.f; }
			sum += 4.f*image[rowIdx]; num += 4.f;

			value = sum / num; 
			return true;
		}

		// 5x5 Gaussian implementation
		// input kernel is ignored
		//
		//	-	-	-	-	-			1	4	6	4	1
		//	-	-	-	-	-			4	16	24	16	4
		//	-	-	x	-	-	with	6	24	36	24	6
		//	-	-	-	-	-			4	16	24	16	4
		//	-	-	-	-	-			1	4	6	4	1
		//

		template<>
		inline bool convolve<Gaussian5x5>(const float* const image, const bool* const mask,
										  const int idx, const int width, float& value,
										  const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			const int w2 = 2 * width;

			float sum = 0.f;
			float num = 0.f;

			if (mask[idx]) { sum += 36.f*image[idx]; num += 36.f; };
			if (mask[idx + 1]) { sum += 24.f*image[idx + 1]; num += 24.f; }
			if (mask[idx - 1]) { sum += 24.f*image[idx - 1]; num += 24.f; }
			if (mask[idx + width]) { sum += 24.f*image[idx + width]; num += 24.f; }
			if (mask[idx - width]) { sum += 24.f*image[idx - width]; num += 24.f; }
			if (mask[idx + 1 + width]) { sum += 16.f*image[idx + 1 + width]; num += 16.f; }
			if (mask[idx + 1 - width]) { sum += 16.f*image[idx + 1 - width]; num += 16.f; }
			if (mask[idx - 1 + width]) { sum += 16.f*image[idx - 1 + width]; num += 16.f; }
			if (mask[idx - 1 - width]) { sum += 16.f*image[idx - 1 - width]; num += 16.f; }
			if (mask[idx + 2]) { sum += 6.f*image[idx + 2]; num += 6.f; }
			if (mask[idx - 2]) { sum += 6.f*image[idx - 2]; num += 6.f; }
			if (mask[idx + w2]) { sum += 6.f*image[idx + w2]; num += 6.f; }
			if (mask[idx - w2]) { sum += 6.f*image[idx - w2]; num += 6.f; }
			if (mask[idx + 2 + width]) { sum += 4.f*image[idx + 2 + width]; num += 4.f; }
			if (mask[idx + 2 - width]) { sum += 4.f*image[idx + 2 - width]; num += 4.f; }
			if (mask[idx - 2 + width]) { sum += 4.f*image[idx - 2 + width]; num += 4.f; }
			if (mask[idx - 2 - width]) { sum += 4.f*image[idx - 2 - width]; num += 4.f; }
			if (mask[idx + 1 + w2]) { sum += 4.f*image[idx + 1 + w2]; num += 4.f; }
			if (mask[idx + 1 - w2]) { sum += 4.f*image[idx + 1 - w2]; num += 4.f; }
			if (mask[idx - 1 + w2]) { sum += 4.f*image[idx - 1 + w2]; num += 4.f; }
			if (mask[idx - 1 - w2]) { sum += 4.f*image[idx - 1 - w2]; num += 4.f; }
			if (mask[idx + 2 + w2]) { sum += image[idx + 2 + w2]; num++; }
			if (mask[idx + 2 - w2]) { sum += image[idx + 2 - w2]; num++; }
			if (mask[idx - 2 + w2]) { sum += image[idx - 2 + w2]; num++; }
			if (mask[idx - 2 - w2]) { sum += image[idx - 2 - w2]; num++; }

			if (num > 0.1f) { value = sum / num; return true; }
			else { value = 0.f; return false; }
		}

		template<>
		inline bool convolve<Gaussian5x5>(const float* const image, const int idx, const int width, float& value,
										  const Eigen::Matrix<float, -1, -1>* const kernel)
		{
			const int w2 = 2 * width;

			float sum = 36.f*image[idx];
			sum += 24.f*image[idx + 1];
			sum += 24.f*image[idx - 1];
			sum += 24.f*image[idx + width];
			sum += 24.f*image[idx - width];
			sum += 16.f*image[idx + 1 + width];
			sum += 16.f*image[idx + 1 - width];
			sum += 16.f*image[idx - 1 + width];
			sum += 16.f*image[idx - 1 - width];
			sum += 6.f*image[idx + 2];
			sum += 6.f*image[idx - 2];
			sum += 6.f*image[idx + w2];
			sum += 6.f*image[idx - w2];
			sum += 4.f*image[idx + 2 + width];
			sum += 4.f*image[idx + 2 - width];
			sum += 4.f*image[idx - 2 + width];
			sum += 4.f*image[idx - 2 - width];
			sum += 4.f*image[idx + 1 + w2];
			sum += 4.f*image[idx + 1 - w2];
			sum += 4.f*image[idx - 1 + w2];
			sum += 4.f*image[idx - 1 - w2];
			sum += image[idx + 2 + w2];
			sum += image[idx + 2 - w2];
			sum += image[idx - 2 + w2];
			sum += image[idx - 2 - w2];

			sum = value / 256.f;

			return true;
		}
	}
}