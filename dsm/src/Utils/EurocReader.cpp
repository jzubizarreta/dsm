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

#include "EurocReader.h"

#include <iostream>
#include <fstream>

#include "opencv2/highgui.hpp"

namespace dsm
{
	EurocReader::EurocReader(const std::string &imageFolder, const std::string &timestampFile, bool reverse) :
		imagePath(imageFolder), timestampPath(timestampFile), id(0), inc(reverse ? -1 : 1)
	{}

	EurocReader::~EurocReader()
	{}

	bool EurocReader::open()
	{
		bool readOK = this->readImageNames();

		if (!readOK) {
			std::cout << "Could not read timestamps file at " << this->timestampPath << std::endl;
			return false;
		}

		//sequence length in seconds
		double diff = this->timestamps.back() - this->timestamps.front();

		//fps
		this->fps_ = this->timestamps.size() / diff;

		// reset
		this->reset();

		std::cout << "EurocMav sequence found!" << std::endl;

		return true;
	}

	void EurocReader::reset()
	{
		if (this->inc > 0) this->id = 0;
		else this->id = (int)this->files.size() - 1;
	}

	bool EurocReader::isOpened() const
	{
		return (this->files.size() > 0);
	}

	bool EurocReader::read(cv::Mat &img, double &timestamp)
	{
		if (this->id < this->files.size() && this->id >= 0)
		{
			img = cv::imread(this->files[this->id], cv::IMREAD_UNCHANGED);
			timestamp = this->timestamps[this->id];

			this->id += this->inc;

			return true;
		}

		return false;
	}

	double EurocReader::fps() const
	{
		return this->fps_;
	}

	bool EurocReader::readImageNames()
	{
		//clear all data
		this->timestamps.clear();
		this->files.clear();

		// read timestamps and images with names equal to timestamps
		std::ifstream infile;
		infile.open(this->timestampPath);
		while (!infile.eof() && infile.good())
		{
			std::string line;
			std::getline(infile, line);

			if (!line.empty())
			{
				this->files.push_back(imagePath + "/" + line + ".png");
				this->timestamps.push_back(std::atof(line.c_str()) / 1e9);		// transform to seconds
			}
		}

		if (this->timestamps.size() > 0 && this->timestamps.size() == this->files.size())
		{
			return true;
		}
		else
		{
			this->timestamps.clear();
			this->files.clear();
		}

		return false;
	}
}