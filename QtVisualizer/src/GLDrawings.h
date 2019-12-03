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

#include <array>

#include <Eigen/Core>

#include "opencv2/core.hpp"

namespace dsm
{
	// draw coordinate system
	void drawAxis(float scale, float lineWidth, float alpha = 1.f);

	// draw wired cube
	void drawWireCube(const std::array<float, 6> &aabb, float lineWidth, const Eigen::Vector3f &color, float alpha = 1.f);

	// draw solid cube with wire frame (optional)
	void drawSolidCube(const std::array<float, 6> &aabb, const Eigen::Vector3f &color, float alpha = 1.f, bool drawWireframe = false);

	// draw wired camera frame
	void drawCamera(float scale, float lineWidth, const Eigen::Vector3f &color, float alpha = 1.f);

	// draw opencv image as 2D texture
	void drawTextureImg(const cv::Mat &image, unsigned int &textureId);
}