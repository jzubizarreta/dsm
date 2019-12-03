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

#include "GlobalCalibration.h"

#include <Eigen/LU>

namespace dsm
{
	GlobalCalibration::GlobalCalibration() : 
		levels_(0),
		pixelAngle_x(0.f),
		pixelAngle_y(0.f),
		pixelAngle_(0.f)
	{
	}

	GlobalCalibration::~GlobalCalibration()
	{
	}

	GlobalCalibration& GlobalCalibration::getInstance()
	{
		static GlobalCalibration theCalibration;
		return theCalibration;
	}

	void GlobalCalibration::setCalibration(const int32_t width, const int32_t height,
										   const Eigen::Matrix3f &calib, const int32_t pyramidLevels)
	{
		this->levels_ = pyramidLevels;

		// resize all vectors
		this->resize(this->levels_);

		// set zero level
		this->width_[0] = width;
		this->height_[0] = height;

		// pinhole
		this->K_[0] = calib;
		this->KInv_[0] = this->K_[0].inverse();

		this->Kd_[0] = this->K_[0].cast<double>();
		this->KdInv_[0] = this->KInv_[0].cast<double>();

		// angle per pixel
		// take the central pixel
		this->pixelAngle_x = std::atan(1.f / this->K_[0](0, 0));
		this->pixelAngle_y = std::atan(1.f / this->K_[0](1, 1));
		this->pixelAngle_ = 0.5f*(this->pixelAngle_x + this->pixelAngle_y);

		// pyramid levels
		for (int32_t level = 1; level < this->levels_; ++level)
		{
			this->width_[level] = this->width_[0] >> level;
			this->height_[level] = this->height_[0] >> level;

			// left
			const float fx = this->K_[level - 1](0, 0) * 0.5f;
			const float fy = this->K_[level - 1](1, 1) * 0.5f;
			const float cx = (this->K_[0](0, 2) + 0.5f) / ((int)1 << level) - 0.5f;
			const float cy = (this->K_[0](1, 2) + 0.5f) / ((int)1 << level) - 0.5f;

			this->K_[level] <<	fx, 0.f, cx,
								0.f, fy, cy,
								0.f, 0.f, 1.f;

			this->KInv_[level] = this->K_[level].inverse();

			this->Kd_[level] = this->K_[level].cast<double>();
			this->KdInv_[level] = this->KInv_[level].cast<double>();
		}
	}

	void GlobalCalibration::resize(const int32_t size)
	{
		this->width_.resize(size);
		this->height_.resize(size);

		this->K_.resize(size);
		this->KInv_.resize(size);

		this->Kd_.resize(size);
		this->KdInv_.resize(size);
	}

	int32_t GlobalCalibration::levels() const
	{
		return this->levels_;
	}

	int32_t GlobalCalibration::width(int32_t level) const
	{
		assert(level < this->levels_);
		return this->width_[level];
	}

	int32_t GlobalCalibration::height(int32_t level) const
	{
		assert(level < this->levels_);
		return this->height_[level];
	}

	const Eigen::Matrix3f& GlobalCalibration::matrix3f(int32_t level) const
	{
		assert(level < this->levels_);
		return this->K_[level];
	}

	const Eigen::Matrix3f& GlobalCalibration::invMatrix3f(int32_t level) const
	{
		assert(level < this->levels_);
		return this->KInv_[level];
	}

	const Eigen::Matrix3d& GlobalCalibration::matrix3d(int32_t level) const
	{
		assert(level < this->levels_);
		return this->Kd_[level];
	}

	const Eigen::Matrix3d& GlobalCalibration::invMatrix3d(int32_t level) const
	{
		assert(level < this->levels_);
		return this->KdInv_[level];
	}

	float GlobalCalibration::pixelAngleX() const
	{
		return this->pixelAngle_x;
	}

	float GlobalCalibration::pixelAngleY() const
	{
		return this->pixelAngle_y;
	}

	float GlobalCalibration::pixelAngle() const
	{
		return this->pixelAngle_;
	}
}