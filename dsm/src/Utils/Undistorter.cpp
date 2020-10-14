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

#include "Undistorter.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <array>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace dsm
{
	Undistorter::Undistorter(const std::string& calibFilename) :
		valid(false)
	{
		std::cout << "Reading Calibration from file: " << calibFilename << "\n";

		std::ifstream infile(calibFilename);
		if (!infile.good())
		{
			return;
		}

		// read lines
		std::string l1, l2, l3, l4;

		std::getline(infile, l1);
		std::getline(infile, l2);
		std::getline(infile, l3);
        std::getline(infile, l4);

		std::istringstream iss1(l1);
		std::istringstream iss2(l2);
		std::istringstream iss3(l3);
		std::istringstream iss4(l4);

		// first line: camera type
		std::string camera_type;
		iss1 >> camera_type;

		if(camera_type != "radtan" && camera_type != "equidistant") {
			std::cout << "Camera type should be either 'radtan' or 'equidistant'. Setting to 'radtan'." << std::endl;
			camera_type = "radtan";
		}


		// second line: camera parameters
		size_t count = 1;
		std::array<double, 12> inputCalibration;		// max 12 params
		iss2 >> inputCalibration[count - 1];
		while (std::istream::goodbit == iss2.rdstate())
		{
			count++;
			iss2 >> inputCalibration[count - 1];
		}

		// set inputK and distortion params
		this->originalK_ = cv::Mat::zeros(3, 3, CV_64FC1);
		this->originalK_.at<double>(0, 0) = inputCalibration[0];
		this->originalK_.at<double>(1, 1) = inputCalibration[1];
		this->originalK_.at<double>(2, 2) = 1.0;
		this->originalK_.at<double>(0, 2) = inputCalibration[2];
		this->originalK_.at<double>(1, 2) = inputCalibration[3];

		size_t numDistortionParams = count - 4;
		this->distCoeffs_ = cv::Mat::zeros((int)numDistortionParams, 1, CV_64FC1);
		for (int i = 0; i < (int)numDistortionParams; ++i)
		{
			this->distCoeffs_.at<double>(i, 0) = inputCalibration[4 + i];
		}

		// third line input image size
		if (iss3 >> this->in_width >> this->in_height)
		{
			std::cout << "Input resolution: " << this->in_width << ", " << this->in_height << std::endl;
			std::cout << "Input calibration: " << this->originalK_.at<double>(0, 0) << " " << this->originalK_.at<double>(1, 1) << " " <<
												  this->originalK_.at<double>(0, 2) << " " << this->originalK_.at<double>(1, 2);
			for (int i = 0; i < numDistortionParams; ++i)
			{
				std::cout << " " << this->distCoeffs_.at<double>(i, 0);
			}
			std::cout << std::endl;

		}
		else
		{
			std::cout << "Failed to read input image size" << std::endl;
			valid = false;
			return;
		}

		// fourth line output image size
		if (iss4 >> this->out_width >> this->out_height)
		{
			std::cout << "Output resolution: " << this->out_width << ", " << this->out_height << std::endl;
		}
		else
		{
			std::cout << "Failed to read output image size" << std::endl;
			valid = false;
			return;
		}

		// undistortion remap
		this->K_ = cv::getOptimalNewCameraMatrix(this->originalK_, this->distCoeffs_, cv::Size(this->in_width, this->in_height),
												 0, cv::Size(this->out_width, this->out_height), nullptr, false);

		if(camera_type == "radtan")
		{
			cv::initUndistortRectifyMap(this->originalK_, this->distCoeffs_, cv::Mat(), this->K_,
				cv::Size(this->out_width, this->out_height), CV_32F, this->mapx, this->mapy);
		}
		else if(camera_type == "equidistant")
		{
			cv::fisheye::initUndistortRectifyMap(this->originalK_, this->distCoeffs_, cv::Mat(), this->K_,
				cv::Size(this->out_width, this->out_height), CV_32F, this->mapx, this->mapy);
		}

		std::cout << "Output calibration: " << this->K_.at<double>(0, 0) << " " << this->K_.at<double>(1, 1) << " " <<
											   this->K_.at<double>(0, 2) << " " << this->K_.at<double>(1, 2) << std::endl;

		valid = true;
	}

	Undistorter::~Undistorter()
	{}

	void Undistorter::undistort(const cv::Mat& image, cv::Mat& result) const
	{
		cv::remap(image, result, this->mapx, this->mapy, cv::INTER_LINEAR);
	}

	const cv::Mat& Undistorter::getOriginalK() const
	{
		return this->originalK_;
	}

	const cv::Mat& Undistorter::getDist() const
	{
		return this->distCoeffs_;
	}

	const cv::Mat& Undistorter::getK() const
	{
		return this->K_;
	}

	int Undistorter::getInputWidth() const
	{
		return this->in_width;
	}

	int Undistorter::getInputHeight() const
	{
		return this->in_height;
	}

	int Undistorter::getOutputWidth() const
	{
		return this->out_width;
	}

	int Undistorter::getOutputHeight() const
	{
		return this->out_height;
	}

	bool Undistorter::isValid() const
	{
		return this->valid;
	}
}
