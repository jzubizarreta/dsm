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

#include <vector>
#include <shared_mutex>

#include "FullSystem/DSMLib.h"

namespace dsm
{
	class DSM_EXPORTS_DLL GlobalCalibration
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		static GlobalCalibration& getInstance();

		// sets the calibration and computes calibration pyramids
		void setCalibration(const int32_t width, const int32_t height,
							const Eigen::Matrix3f &calib,
							const int32_t pyramidLevels);

		// constant accessors
		int32_t levels() const;

		int32_t width(int32_t level) const;
		int32_t height(int32_t level) const;		

		const Eigen::Matrix3f& matrix3f(int32_t level) const;
		const Eigen::Matrix3f& invMatrix3f(int32_t level) const;

		const Eigen::Matrix3d& matrix3d(int32_t level) const;		
		const Eigen::Matrix3d& invMatrix3d(int32_t level) const;

		float pixelAngleX() const;
		float pixelAngleY() const;
		float pixelAngle() const;

		GlobalCalibration(GlobalCalibration const&) = delete;
		void operator=(GlobalCalibration const&) = delete;

	private:

		GlobalCalibration();
		~GlobalCalibration();

		void resize(const int32_t size);

	private:

		// image pyramids
		std::vector<int32_t> width_;
		std::vector<int32_t> height_;

		// camera intrinsic parameters
		std::vector<Eigen::Matrix3f> K_, KInv_;
		std::vector<Eigen::Matrix3d> Kd_, KdInv_;

		// angle per pixel
		float pixelAngle_x, pixelAngle_y;
		float pixelAngle_;

		// numer of pyramid levels
		int32_t levels_;
	};
}