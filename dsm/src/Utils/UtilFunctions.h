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

#include <iostream>
#include <chrono>
#include <fstream>

#if defined(_WIN32)
#include <direct.h>
#endif

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include "Statistics/TDistribution.h"
#include "Settings.h"
#include "FullSystem/DSMLib.h"

#include <sys/stat.h>

namespace dsm
{
	namespace Utils
	{
		typedef std::chrono::steady_clock::time_point Time;

		inline float elapsedTime(const Time &t1, const Time &t2)
		{
			return (float)std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.f;
		}

		inline Eigen::Vector3f colorMap(const float min, const float max, float value)
		{
			// https://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV

			// blend over HSV-values
			Eigen::Vector3f rgb;

			// convert to the range [0, 1]*0.8
			// 0.8 is taken to limit H range to [0, 280]: from red to magenta
			value = std::min(std::max((value - min) / (max - min), 0.f), 1.f)*0.8f;		// H

			const float s = 1.f;		// S
			const float v = 1.f;		// V

			value -= floor(value);
			value *= 6.f;					// H'
			
			const int h = (int)floor(value);
			const float c = s * v;
			
			float f = value - h;
			if (!(h & 1))
			{
				f = 1.f - f; // if i is even
			}

			const float x = v * (1.f - s * f);

			switch (h) 
			{
				case 0:
					rgb[0] = c; rgb[1] = x; rgb[2] = 0.f;
					break;
				case 1:
					rgb[0] = x; rgb[1] = c; rgb[2] = 0.f;
					break;
				case 2:
					rgb[0] = 0.f; rgb[1] = c; rgb[2] = x;
					break;
				case 3:
					rgb[0] = 0.f; rgb[1] = x; rgb[2] = c;
					break;
				case 4:
					rgb[0] = x; rgb[1] = 0.f; rgb[2] = c;
					break;
				case 5:
					rgb[0] = c; rgb[1] = 0.f; rgb[2] = x;
					break;
				default:
					rgb[0] = 0.f; rgb[1] = 0.f; rgb[2] = 0.f;		// black
					break;
			}

			return rgb;
		}

		inline void concatImages(const std::vector<cv::Mat>& images, cv::Mat &output)
		{
			const int num = (int)images.size();

			if (num == 0) return;

			const int width = images[0].cols;
			const int height = images[0].rows;

			// dimensions for 16:9 ratio
			const float desiredRatio = 16.f / 9.f;
			const int maxColumns = 5;
			const int maxRows = (int)std::ceil((float)num / maxColumns);

			if (maxColumns*maxRows < num) return;

			int numColumns = 0;
			float bestError = std::numeric_limits<float>::max();
			for (int c = 1; c <= maxColumns; c++)
			{
				const int r = (int)std::ceil((float)num / c);

				const int totalWidth = c * width;
				const int totalHeight = r * height;

				const float currentRatio = (float)totalWidth / totalHeight;
				const float error = std::abs(currentRatio - desiredRatio);

				if (error < bestError)
				{
					bestError = error;
					numColumns = c;
				}
			}

			const int numRows = (int)std::ceil((float)num / numColumns);

			// create output image
			output = cv::Mat(numRows*height, numColumns*width, images[0].type());
			output.setTo(255);

			// insert all images into the new one
			for (int r = 0; r < numRows; ++r)
			{
				for (int c = 0; c < numColumns; ++c)
				{
					const int index = r*numColumns + c;

					if (index >= num) continue;

					images[index].copyTo(output(cv::Rect(c*width, r*height, width, height)));
				}
			}
		}

		inline void draw3x3Square(cv::Mat& output, int row, int col, const cv::Vec3b& bgr)
		{
			// boundary
			const int w = output.cols;
			const int h = output.rows;

			for (int i = -1; i < 1; ++i)
			{
				for (int j = -1; j < 1; ++j)
				{
					const int x = col + j;
					const int y = row + i;

					if (!(x >= 0 && y >= 0 && x < w && y < h)) continue;

					output.at<cv::Vec3b>(y, x) = bgr;
				}
			}
		}

		inline cv::Mat drawDistribution(const std::vector<float>& values, const std::shared_ptr<IDistribution>& dist,
										const Eigen::Vector2f& xRange, const Eigen::Vector2f& yRange,
										const Eigen::Vector2i& winSize, int numBins,
										const Eigen::Vector3i& lineColor = Eigen::Vector3i(255, 0, 0), int lineWidth = 2,
										const Eigen::Vector3i& histColor = Eigen::Vector3i(128, 128, 128))
		{
			// image with white background
			cv::Mat hist(winSize[1], winSize[0], CV_8UC3);
			hist.setTo(cv::Scalar(255, 255, 255));

			// check range values
			if (xRange[0] > xRange[1]) return hist;
			if (yRange[0] > yRange[1]) return hist;

			// the maximum bin resolution is the image width
			if (numBins > winSize[0]) numBins = winSize[0];

			// horizontal bin size in value/bin
			const float xBin = (xRange[1] - xRange[0]) / numBins;
			
			// total number of values
			const int num = (int)values.size();

			// counter in each bin
			std::vector<int> count(numBins, 0);
			for (int i = 0; i < num; i++)
			{
				if (values[i] < xRange[0] || values[i] > xRange[1]) continue;

				int index = static_cast<int>((values[i] - xRange[0]) / xBin);
				index = std::min(std::max(0, index), numBins - 1); 

				count[index]++;
			}

			// pdf in each bin
			std::vector<float> pdf(numBins, 0);
			for (int i = 0; i < count.size(); i++)
			{
				pdf[i] = ((float)count[i]) / (num*xBin);
			}

			// bin size in px/value
			const float xBinPx = std::max((float)winSize[0] / numBins, 1.f);
			const float yBinPx = winSize[1] / ((yRange[1] - yRange[0])*1.1f);

			// draw histogram
			for (int bin_i = 0; bin_i < pdf.size(); ++bin_i)
			{
				int minRow = static_cast<int>(yBinPx * (pdf[bin_i] - yRange[0]));
				minRow = std::max(std::min(minRow, winSize[1] - 1), 0);

				int col_init = static_cast<int>(bin_i*xBinPx);
				col_init = std::max(std::min(col_init, winSize[0] - 1), 0);

				int col_end = static_cast<int>((bin_i + 1)*xBinPx);
				col_end = std::max(std::min(col_end, winSize[0] - 1), 0);

				// bin line
				for (int col = col_init; col < col_end; ++col)
				{
					cv::Point p1(col, winSize[1] - 1);
					cv::Point p2(col, winSize[1] - 1 - minRow);
					cv::line(hist, p1, p2, cv::Scalar(histColor[2], histColor[1], histColor[0]), 1);
				}
			}

			// compute tdist pdf for each bin
			std::vector<float> tpdf(numBins, 0.f);
			for (int i = 0; i < numBins; ++i)
			{
				float res = (i * xBin) + xRange[0];
				tpdf[i] = dist->pdf(res);
			}

			// draw tdist curve
			for (int bin_i = 1; bin_i < tpdf.size(); ++bin_i)
			{
				int row1 = static_cast<int>(yBinPx * (tpdf[bin_i - 1] - yRange[0]));
				row1 = std::max(std::min(winSize[1] - 1, row1), 0);

				int col1 = static_cast<int>((bin_i - 1)*xBinPx);
				col1 = std::max(std::min(col1, winSize[0] - 1), 0);

				int row2 = static_cast<int>(yBinPx * (tpdf[bin_i] - yRange[0]));
				row2 = std::max(std::min(winSize[1] - 1, row2), 0);

				int col2 = static_cast<int>(bin_i*xBinPx);
				col2 = std::max(std::min(col2, winSize[0] - 1), 0);
				
				cv::Point p1(col1, winSize[1] - 1 - row1);
				cv::Point p2(col2, winSize[1] - 1 - row2);

				// tdist line
				cv::line(hist, p1, p2, cv::Scalar(lineColor[2], lineColor[1], lineColor[0]), lineWidth);
			}

			return hist;
		}

		DSM_EXPORTS_DLL inline bool savePLY(const std::string &file, const std::vector<Eigen::Vector3f>& vertices,
											const std::vector<Eigen::Vector3f>& normals = std::vector<Eigen::Vector3f>(),
											const std::vector<Eigen::Matrix<unsigned char, 3, 1>>& colors = std::vector<Eigen::Matrix<unsigned char, 3, 1>>())
		{
			std::cout << "Saving PLY...";

			std::ofstream plyFile;
			plyFile.open(file);

			if (!plyFile.is_open())
			{
				std::cout << "savePLY could not open " << file << std::endl;
				return false;
			}

			const size_t numVertex = vertices.size();

			if (numVertex <= 0) return false;

			const uint32_t numFaces = 0;
			const bool hasVertex = (vertices.size() > 0);
			const bool hasNormal = (normals.size() > 0);
			const bool hasFaces = false;
			const bool hasColor = (colors.size() > 0);

			// write header
			plyFile << "ply\n";
			plyFile << "format ascii 1.0\n";
			plyFile << "comment author: ply from DSM\n";

			if (hasVertex)
			{
				plyFile << "element vertex " << numVertex << "\n";
				plyFile << "property float x\n";
				plyFile << "property float y\n";
				plyFile << "property float z\n";
			}

			if (hasNormal)
			{
				plyFile << "property float nx\n";
				plyFile << "property float ny\n";
				plyFile << "property float nz\n";
			}

			if (hasColor)
			{
				plyFile << "property uchar red\n";
				plyFile << "property uchar green\n";
				plyFile << "property uchar blue\n";
			}

			if (hasFaces)
			{
				// TODO
			}

			plyFile << "end_header\n";

			// write vertices
			for (size_t i = 0; i < numVertex; ++i)
			{
				// vertex
				plyFile << vertices[i].x() << " " << vertices[i].y() << " " << vertices[i].z();

				if (hasNormal)
				{
					// normal
					plyFile << " " << normals[i].x() << " " << normals[i].y() << " " << normals[i].z();
				}

				if (hasColor)
				{
					// color
					unsigned int r = static_cast<unsigned int>(colors[i][0]);
					unsigned int g = static_cast<unsigned int>(colors[i][1]);
					unsigned int b = static_cast<unsigned int>(colors[i][2]);
					plyFile << " " << r << " " << g << " " << b;
				}

				plyFile << "\n";
			}

			plyFile.close();

			std::cout << "Finished!" << std::endl;

			return true;
		}

		inline bool dirExists(const std::string& dir)
		{
			struct stat info;

			if (stat(dir.c_str(), &info) != 0)
			{
				return false;
			}	
			else if (info.st_mode & S_IFDIR)
			{
				return true;
			}				
			else
			{
				return false;
			}
		}

		inline void makeDir(const std::string& dir)
		{
			if (!dirExists(dir))
			{
				// create folder
#if defined(_WIN32)
				_mkdir(dir.c_str());
#else 
				mkdir(dir.c_str(), 0777); // notice that 777 is different than 0777
#endif
			}
		}
	}
}
