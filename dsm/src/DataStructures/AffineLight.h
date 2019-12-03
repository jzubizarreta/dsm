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
	// Affine light model from exposure time correction
	//
	// Model:	I_cam = exp(alpha)*I_real + beta
	//			I_real = exp(-alpha)*(I_cam - beta)
	//
	// Mode of usage:	I_real = a*I_cam + b
	//
	class AffineLight
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		AffineLight();
		AffineLight(float alpha, float beta);
		AffineLight(const Eigen::Vector2f& affLight);
		AffineLight(const AffineLight& affLight);

		// I_real = exp(-alpha)*(I_cam - beta)
		float alpha() const;
		float beta() const;

		// I_real = exp(-v[0])*(I_cam - v[1])
		Eigen::Vector2f toEigenVec() const;

		// I_real = a*I_cam + b
		float a() const;
		float b() const;

		// copy operator
		AffineLight& operator=(const AffineLight& affLight);

		// Calculates the affine light relative to two frames
		// Input: exp(-alpha_from)*(I_from - beta_from) = exp(-alpha_to)*(I_to - beta_to)
		// Output: I_from = exp(-alpha)*(I_to - beta)
		static AffineLight calcRelative(const AffineLight& fromGlobal, const AffineLight& toGlobal);
		
		// Calculates the global affine light of a camera
		// Input: global of "from" and relative to "from" of "to"
		// Output: global of "to"
		static AffineLight calcGlobal(const AffineLight& fromGlobal, const AffineLight& toRelative);

		// degree of freedom of group
		static const int DoF = 2;

		// number of internal parameters used
		static const int num_parameters = 2;

	private:

		float alpha_;
		float beta_;
	};
}