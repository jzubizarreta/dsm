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

#include "Settings.h"

namespace dsm
{
	namespace Utils
	{
		template<typename T>
		inline bool checkImageBoundaries(const Eigen::Matrix<T, 2, 1>& pixel, int width, int height)
		{
			return (pixel[0] > 1.1 && pixel[0] < width - 2.1 && pixel[1] > 1.1 && pixel[1] < height - 2.1);
		}

		// Projection functions
		// We will use inverse depth parameterization to achieve a continuous parameterization.
		// In addition, we scale the transformation function to get a continuous function:
		//
		//		pt2d = K*R*inv(K)*u + K*t*iDepth
		//
		// This scaled function is continuous and avoids computing 3D points, which will lead to
		// a discontinuity in iDepth = 0.
		// We will also use this function during jacobians estimation to make sure they are
		// continuous in all the values of the inverse depth.

		// This function transforms a pixel from the reference image to a new one.
		// It also checks image boundaries and inverse depth consistency even if
		// its values is < 0. Its output contains useful values for jacobians computation.
		//
		// in - uj: pixel x coordinate
		// in - vj: pixel y coordinate
		// in - iDepth: pixel inverse depth
		// in - normal: point normal, or camera ray
		// in - K: camera intrinsic matrix
		// in - width, height: image dimensions
		// in - R: rotation from reference to new image
		// in - t: translation from reference to new image
		// out - Xpn: Xp normalized at z=1
		// out - pt2d: projected point in new image
		// out - newIDepth: inverse depth in new image
		// out - rescale: scale between both inverse depths (newIDepth / iDepth)
		// return: if successfully projected or not due to OOB
		template<typename T>
		bool project(T uj, T vj, T iDepth, 
					 const Eigen::Matrix<T, 3, 3>& K, int width, int height,
					 const Eigen::Matrix<T, 3, 3>& R, const Eigen::Matrix<T, 3, 1>& t,
					 Eigen::Matrix<T, 2, 1>& Xpn, Eigen::Matrix<T, 2, 1>& pt2d, 
					 T& newIDepth, T& rescale)
		{
			static const Settings& settings = Settings::getInstance();

			// unproject
			const Eigen::Matrix<T, 3, 1> Xinv((uj - K(0, 2)) / K(0, 0),
											  (vj - K(1, 2)) / K(1, 1),
											  1);

			// Xp: R*Xinv + t*iDepth
			const Eigen::Matrix<T, 3, 1> Xp = R * Xinv + t * iDepth;

			// new iDepth and rescale factor
			rescale = 1 / Xp[2];

			// if the point was in the range [0, Inf] in camera1
			// it has to be also in the same range in camera2
			// This allows using negative inverse depth values
			// i.e. same iDepth sign in both cameras
			if (!(rescale > 0)) return false;

			// inverse depth in new image
			newIDepth = iDepth*rescale;

			// normalize
			Xpn[0] = Xp[0] * rescale;
			Xpn[1] = Xp[1] * rescale;

			// project: K * Xp
			pt2d[0] = Xpn[0] * K(0, 0) + K(0, 2);
			pt2d[1] = Xpn[1] * K(1, 1) + K(1, 2);

			// check image boundaries
			return checkImageBoundaries(pt2d, width, height);
		}

		// This function transforms a pixel from the reference image to a new one.
		// It also checks image boundaries and inverse depth consistency even if
		// its values is < 0.
		//
		// in - uj: pixel x coordinate
		// in - vj: pixel y coordinate
		// in - iDepth: pixel inverse depth
		// in - width, height: image dimensions
		// in - KRKinv: K*rotation*inv(K) from reference to new image
		// in - Kt: K*translation from reference to new image
		// out - pt2d: projected point in new image
		// out - newIDepth: inverse depth in new image
		// return: if successfully projected or not due to OOB
		template<typename T>
		bool project(T uj, T vj, T iDepth, int width, int height,
					 const Eigen::Matrix<T, 3, 3>& KRKinv, const Eigen::Matrix<T, 3, 1>& Kt,
					 Eigen::Matrix<T, 2, 1>& pt2d, T& newIDepth)
		{
			// transform and project
			const Eigen::Matrix<T, 3, 1> pt = KRKinv * Eigen::Matrix<T, 3, 1>(uj, vj, 1) + Kt*iDepth;

			// rescale factor
			const T rescale = 1 / pt[2];

			// if the point was in the range [0, Inf] in camera1
			// it has to be also in the same range in camera2
			// This allows using negative inverse depth values
			// i.e. same iDepth sign in both cameras
			if (!(rescale > 0)) return false;

			// inverse depth in new image
			newIDepth = iDepth*rescale;

			// normalize
			pt2d[0] = pt[0] * rescale;
			pt2d[1] = pt[1] * rescale;

			// check image boundaries
			return checkImageBoundaries(pt2d, width, height);
		}

		// This function transforms a pixel from the reference image to a new one.
		// It also checks image boundaries and inverse depth consistency even if
		// its values is < 0.
		//
		// in - uj: pixel x coordinate
		// in - vj: pixel y coordinate
		// in - iDepth: pixel inverse depth
		// in - width, height: image dimensions
		// in - KRKinv: K*rotation*inv(K) from reference to new image
		// in - Kt: K*translation from reference to new image
		// out - pt2d: projected point in new image
		// return: if successfully projected or not due to OOB
		template<typename T>
		bool project(T uj, T vj, T iDepth, int width, int height,
					 const Eigen::Matrix<T, 3, 3>& KRKinv, const Eigen::Matrix<T, 3, 1>& Kt,
					 Eigen::Matrix<T, 2, 1>& pt2d)
		{
			// transform and project
			const Eigen::Matrix<T, 3, 1> pt = KRKinv * Eigen::Matrix<T, 3, 1>(uj, vj, 1) + Kt*iDepth;

			// rescale factor
			const T rescale = 1 / pt[2];

			// if the point was in the range [0, Inf] in camera1
			// it has to be also in the same range in camera2
			// This allows using negative inverse depth values
			// i.e. same iDepth sign in both cameras
			if (!(rescale > 0)) return false;

			// normalize
			pt2d[0] = pt[0] * rescale;
			pt2d[1] = pt[1] * rescale;

			// check image boundaries
			return checkImageBoundaries(pt2d, width, height);
		}

		template<typename T, bool check = true>
		bool project(const Eigen::Matrix<T, 3, 1>& KRray, const Eigen::Matrix<T, 3, 1>& Kt, 
					 T iDepth, int width, int height, Eigen::Matrix<T, 2, 1>& pt2d)
		{
			// transform and project
			const Eigen::Matrix<T, 3, 1> pt = KRray + Kt*iDepth;

			// rescale factor
			const T rescale = 1 / pt[2];

			// if the point was in the range [0, Inf] in camera1
			// it has to be also in the same range in camera2
			// This allows using negative inverse depth values
			// i.e. same iDepth sign in both cameras
			if (!(rescale > 0)) return false;

			// normalize
			pt2d[0] = pt[0] * rescale;
			pt2d[1] = pt[1] * rescale;

			// check image boundaries
			return !check || checkImageBoundaries(pt2d, width, height);
		}

		// This function transforms a pixel from the reference image to a new one.
		// It also checks image boundaries and inverse depth consistency even if
		// its values is < 0 and the viewpoint change
		//
		// in - uj: pixel x coordinate
		// in - vj: pixel y coordinate
		// in - iDepth: pixel inverse depth
		// in - K: camera intrinsic matrix
		// in - width, height: image dimensions
		// in - R: rotation from reference to new image
		// in - t: translation from reference to new image
		// out - pt2d: projected point in new image
		// return: if successfully projected or not due to OOB
		template<typename T>
		bool projectAndCheck(T uj, T vj, T iDepth,
							 const Eigen::Matrix<T, 3, 3>& K, int width, int height,
							 const Eigen::Matrix<T, 3, 3>& R, const Eigen::Matrix<T, 3, 1>& t,
							 Eigen::Matrix<T, 2, 1>& pt2d)
		{
			static const Settings& settings = Settings::getInstance();

			// unproject
			const Eigen::Matrix<T, 3, 1> Xinv((uj - K(0, 2)) / K(0, 0),
											  (vj - K(1, 2)) / K(1, 1),
											  1);

			// Xp: R*Xinv + t*iDepth
			const Eigen::Matrix<T, 3, 1> Xp = R * Xinv + t * iDepth;

			// new iDepth and rescale factor
			const T rescale = 1 / Xp[2];

			// if the point was in the range [0, Inf] in camera1
			// it has to be also in the same range in camera2
			// This allows using negative inverse depth values
			// i.e. same iDepth sign in both cameras
			if (!(rescale > 0)) return false;

			// view change
			const Eigen::Matrix<T, 3, 1> Xinv2 = Xinv + R.transpose() * t * iDepth;
			const T viewChange = Xinv2.dot(Xinv) / (Xinv.norm()*Xinv2.norm());
			if (viewChange < settings.maxViewChange)
			{
				return false;
			}

			// normalize
			const Eigen::Matrix<T, 2, 1> Xpn = Xp.template head<2>() * rescale;

			// project: K * Xp
			pt2d[0] = Xpn[0] * K(0, 0) + K(0, 2);
			pt2d[1] = Xpn[1] * K(1, 1) + K(1, 2);

			// check image boundaries
			return checkImageBoundaries(pt2d, width, height);
		}
	}
}
