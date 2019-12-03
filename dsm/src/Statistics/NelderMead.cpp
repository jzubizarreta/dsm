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

#include "NelderMead.h"

#include <numeric>

namespace dsm
{
	NelderMead::NelderMead()
	{}

	NelderMead::~NelderMead()
	{}

	Eigen::VecXf NelderMead::minimize(const Func &func, 
									  const Eigen::VecXf& x0, 
									  const std::vector<float> &data, 
									  const Options& options) const
	{
		// number of dimensions
		const int n = (int)x0.size();

		// Initialize adaptative parameters from :
		// 'Implementing the Nelder-Mead simplex algortihm
		// with adaptative parameters' Gao et al. 2010, Eq. 4.1
		const float	rho = 1.f;
		const float chi = 1.f + 2.f / n;
		const float psi = 0.75f - 1.f / (2.f * n);
		const float sigma = 1.f - 1.f / n;

		// Convergence parameters
		const float tolVol = options.tolVol;
		const float tolFunc = options.tolFunc;
		const int maxFunc = options.maxFunc;
		const int maxIter = options.maxIter;

		// Set up a simplex near the initial guess
		Eigen::MatXXf v(n, n + 1);				// simplex: list of verices
		Eigen::VecXf fv(n + 1);					// vertex function values

		// Set the initial guess as a vertex
		v.col(0) = x0;
		fv[0] = func(x0, data);

		// Set the rest of vertices
		const float h = options.h;
		const float h0 = options.h0;
		for (int i = 0; i < n; ++i)
		{
			Eigen::VecXf y = x0;
			if (y[i] != 0)
			{
				y[i] = (1 + h)*y[i];
			}
			else
			{
				y[i] = h0;
			}
			v.col(i + 1) = y;
			fv[i + 1] = func(y, data);
		}

		// Additional parameters
		int funcEvals = n + 1;
		int iterCount = 1;
		float vol = 1.f;            // simplex relative volume

		// main algorithm 
		while (funcEvals < maxFunc && iterCount < maxIter)
		{
			// 1. Sort so v.col(0) has the lowest function value Eq. 2.3
			this->sortSimplex(v, fv);

			// Convergence criteria base on :
			// 'Efficient Implementation of the Nelder-Mead
			// Search Algorithm' Singer et al. 2004
			const float termFunc = 2 * fabs(fv[n] - fv[0]) / (fabs(fv[n]) + fabs(fv[0]));     // Eq. 12
			const float termVol = pow(vol, 1.f / n);		// Eq. 18

			if (termFunc <= tolFunc && termVol <= tolVol)
			{
				break;
			}

			// 2. Compute the reflection point Eq. 2.4
			// xbar = average of the n (NOT n + 1) best points
			const Eigen::VecXf xbar = this->meanSimplex(v);
			const Eigen::VecXf xr = (1 + rho)*xbar - rho * v.col(n);
			const float fr = func(xr, data);
			funcEvals = funcEvals + 1;

			if (fr < fv[0])
			{
				// 3. Calculate the expansion point Eq. 2.5
				const Eigen::VecXf xe = (1 + rho * chi)*xbar - rho * chi*v.col(n);
				const float fe = func(xe, data);
				funcEvals = funcEvals + 1;

				if (fe < fr)
				{
					v.col(n) = xe;
					fv[n] = fe;
					vol = vol * rho*chi;
				}
				else
				{
					v.col(n) = xr;
					fv[n] = fr;
					vol = vol * rho;
				}
			}
			else if (fr >= fv[n - 1])
			{
				// 4. Perform a contraction between xbar and the better
				//    of xn + 1 and xr
				bool doShrink = false;

				if (fr < fv[n])
				{
					// 4.a Perform an outside contraction Eq. 2.6
					const Eigen::VecXf xc = (1 + psi * rho)*xbar - psi * rho*v.col(n);
					const float fc = func(xc, data);
					funcEvals = funcEvals + 1;

					if (fc <= fr)
					{
						v.col(n) = xc;
						fv[n] = fc;
						vol = vol * rho*psi;
					}
					else
					{
						// shrink
						doShrink = true;
					}
				}
				else
				{
					// 4.b Perform an inside contraction Eq. 2.7
					const Eigen::VecXf xcc = (1 - psi)*xbar + psi * v.col(n);
					const float fcc = func(xcc, data);
					funcEvals = funcEvals + 1;

					if (fcc < fv[n])
					{
						v.col(n) = xcc;
						fv[n] = fcc;
						vol = vol * psi;
					}
					else
					{
						// shrink
						doShrink = true;
					}
				}	// end contraction

				// 5. Perform a shrink step if required
				if (doShrink)
				{
					for (int i = 1; i < (n + 1); ++i)
					{
						v.col(i) = v.col(0) + sigma * (v.col(i) - v.col(0));
						fv[i] = func(v.col(i), data);
					}
					vol = vol * pow(sigma, n);
					funcEvals = funcEvals + n;
				}
			}
			else
			{
				// Accept the reflection point
				v.col(n) = xr;
				fv[n] = fr;
				vol = vol * rho;
			}

			// Increase iteration counter
			iterCount = iterCount + 1;
		}

		// take the best vertex
		return v.col(0);
	}

	void NelderMead::sortSimplex(Eigen::MatXXf &v, Eigen::VecXf &fv) const
	{
		const int s = (int)fv.size();

		// original unsorted index map
		Eigen::VecXi index(s);
		std::iota(index.data(), index.data() + s, 0);
		
		// sort indices by function values
		std::sort(index.data(), index.data() + s, IndexComp(fv));
		
		// reorder vectors by ordered indices
		Eigen::MatXXf vu = v;
		Eigen::VecXf fvu = fv;
		for (int i = 0; i < s; ++i)
		{
			v.col(i) = vu.col(index[i]);
			fv[i] = fvu[index[i]];
		}
	}

	Eigen::VecXf NelderMead::meanSimplex(const Eigen::MatXXf &v) const
	{
		// xbar = average of the n(NOT n + 1) best points
		Eigen::VecXf xbar(v.rows());
		xbar.setZero();

		for (int i = 0; i < v.rows(); ++i)
		{
			for (int j = 0; j < v.cols() - 1; ++j)
			{
				xbar[i] += v(i, j);
			}

			xbar[i] /= (v.cols() - 1);
		}

		return xbar;
	}

	// sort using index
	NelderMead::IndexComp::IndexComp(const Eigen::VecXf& fv) :
		values(fv)
	{}

	bool NelderMead::IndexComp::operator()(const int a, const int b) const
	{
		return values[a] < values[b];
	}

}