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

#include <string>
#include <vector>

#include "opencv2/core.hpp"

#include "FullSystem/DSMLib.h"

namespace dsm
{
	class DSM_EXPORTS_DLL EurocReader
	{
	public:

		EurocReader(const std::string &imageFolder, const std::string &timestampFile, bool reverse);
		~EurocReader();

		// grab image files
		bool open();

		// resets sequence
		void reset();

		// check if successfully opened
		bool isOpened() const;

		// reads next image
		bool read(cv::Mat &img, double &timestamp);

		// sequence frames per second
		double fps() const;

	private:

		bool readImageNames();

	private:

		// input data
		const std::string imagePath;
		const std::string timestampPath;
		const int inc;

		// dataset data
		std::vector<std::string> files;
		std::vector<double> timestamps;
		double fps_;

		// image counter
		int id;	
	};
}