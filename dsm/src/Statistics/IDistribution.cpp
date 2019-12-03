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

#include "IDistribution.h"
#include "RobustNormalDistribution.h"
#include "TDistribution.h"

namespace dsm
{
	IDistribution::IDistribution()
	{}

	IDistribution::~IDistribution()
	{}

	std::shared_ptr<IDistribution> IDistribution::fitdist(const std::vector<float> &dist, IDistribution::Type type,
														  const Eigen::VecXf& fixed)
	{
		switch (type)
		{
		case IDistribution::NORMAL:
			// robust normal distribution
			return RobustNormalDistribution::fitdist(dist, fixed);
			break;
		case IDistribution::TSTUDENT:
			// student's t-distribution
			return TDistribution::fitdist(dist, fixed);
			break;
		default:
			// default robust normal distribution
			return RobustNormalDistribution::fitdist(dist, fixed);
			break;
		}
	}
}