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

#include "Utils/EigenTypes.h"

#include <vector>

namespace dsm
{
	// Derivative free minimization using Nelder - Mead method
	// The algorithm is based on :
	// 'Convergence properties of the Nelder-Mead simplex method in
	// low dimensions' Lagarias et al. 1998
	//
	// Additional info :
	// http://www.scholarpedia.org/article/Nelder-Mead_algorithm
	class NelderMead
	{
		typedef std::function<float(const Eigen::VecXf&, const std::vector<float>&)> Func;

	public:

		// Minimization options
		struct Options
		{
			Options() {}
			float tolVol = 1e-02f;		// relative volume tolerance(params)
			float tolFunc = 1e-02f;		// relative function tolerance
			int maxFunc = 400;			// maximum function evaluations
			int maxIter = 200;			// maximum iterations

			float h = 0.05f;			// initialization delta percentaje
			float h0 = 0.00025f;		// initialization delta for zero elements
		};
		
		// Constructor & Destructor
		NelderMead();
		~NelderMead();

		// Main function to minimize a function using only its values
		Eigen::VecXf minimize(const Func &func,
							  const Eigen::VecXf &x0,
							  const std::vector<float> &data,
							  const Options &options = Options()) const;

	private:

		// function to sort the simplex/list of vertices
		void sortSimplex(Eigen::MatXXf &v, Eigen::VecXf &fv) const;

		// function to obtain the centroid of the best n points
		Eigen::VecXf meanSimplex(const Eigen::MatXXf &v) const;

		// index comparison
		struct IndexComp
		{
			IndexComp(const Eigen::VecXf& fv);
			bool operator()(const int a, const int b) const;

		private:

			const Eigen::VecXf& values;
		};
	};
}