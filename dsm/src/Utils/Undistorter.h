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

#include <opencv2/core.hpp>

#include "FullSystem/DSMLib.h"

namespace dsm
{
	// OpenCV based undistorter

	class DSM_EXPORTS_DLL Undistorter
	{
	public:

		Undistorter(const std::string& calibFilename);

		~Undistorter();

		// main undistortion function
		void undistort(const cv::Mat& image, cv::Mat& result) const;

		// camera intrinsic paramaters of original image
		const cv::Mat& getOriginalK() const;

		// distortion parameters of original image
		const cv::Mat& getDist() const;

		// camera intrinsic parameters of undistorted image
		const cv::Mat& getK() const;

		// original image size
		int getInputWidth() const;
		int getInputHeight() const;
		
		// undistorted image size
		int getOutputWidth() const;
		int getOutputHeight() const;

		// flag to check if initialized successfully
		bool isValid() const;

	private:

		cv::Mat K_;
		cv::Mat originalK_;
		cv::Mat distCoeffs_;

		int out_width, out_height;
		int in_width, in_height;
		cv::Mat mapx, mapy;

		bool valid;
	};
}