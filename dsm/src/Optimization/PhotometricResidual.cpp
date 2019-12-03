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

#include "PhotometricResidual.h"
#include "PointParameterBlock.h"
#include "FrameParameterBlock.h"
#include "PhotometricBA.h"
#include "DataStructures/ActivePoint.h"
#include "DataStructures/Pattern.h"
#include "DataStructures/Frame.h"
#include "Utils/Interpolation.h"
#include "Utils/Settings.h"
#include "Utils/GlobalCalibration.h"
#include "Utils/Projection.h"
#include "Utils/LossFunction.h"

namespace dsm
{
	// Cost Function

	PhotometricCostFunction::PhotometricCostFunction(PhotometricResidual* const residual) :
		residual_(residual)
	{
		// number of residual
		this->set_num_residuals(Pattern::size());

		// parameter block sizes
		this->mutable_parameter_block_sizes()->push_back(9);		// owner frame: pose(7) + light(2)
		this->mutable_parameter_block_sizes()->push_back(9);		// target frame: pose(7) + light(2)
		this->mutable_parameter_block_sizes()->push_back(1);		// point inverse depth(1)
	}

	PhotometricCostFunction::~PhotometricCostFunction()
	{}

	bool PhotometricCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
	{
		// current level
		const int lvl = this->residual_->photometricBA_->stats->currentLvl;

		ActivePoint* point = this->residual_->point();
		Frame* ownerFrame = this->residual_->ownerFrame();
		Frame* targetFrame = this->residual_->targetFrame();

		// discard the point if is OOB in the reference frame
		if (!point->valid(lvl))
		{
			this->residual_->state_ = Visibility::OOB;
			this->discardOOB(residuals, jacobians);
			return true;
		}

		const auto& settings = Settings::getInstance();
		const auto& calib = GlobalCalibration::getInstance();

		const Eigen::Matrix3d& K = calib.matrix3d(lvl);
		const Eigen::Matrix3d& Kinv = calib.invMatrix3d(lvl);
		const int32_t width = calib.width(lvl);
		const int32_t height = calib.height(lvl);

		const std::shared_ptr<IDistribution>& dist = targetFrame->errorDistribution();
		const float energyThreshold = targetFrame->energyThreshold();

		const Eigen::VecXf& color = point->colors(lvl);
		const Eigen::VecXf& weight = point->weights(lvl);

		const float* image = targetFrame->image(lvl);
		const float* gx = targetFrame->gx(lvl);
		const float* gy = targetFrame->gy(lvl);

		// residual computation

		// map pointer for efficiency. this avoids copying data
		Eigen::Map<Sophus::SE3d const> const ownerToWorld(parameters[0]);
		Eigen::Map<Sophus::SE3d const> const targetToWorld(parameters[1]);

		const Sophus::SE3d worldToTarget = targetToWorld.inverse();
		const Sophus::SE3d ownerToTarget = worldToTarget * ownerToWorld;
		const Eigen::Matrix3d R = ownerToTarget.rotationMatrix();
		const Eigen::Vector3d& t = ownerToTarget.translation();

		// affine light
		const AffineLight ownerLight((float)parameters[0][7], (float)parameters[0][8]);
		const AffineLight targetLight((float)parameters[1][7], (float)parameters[1][8]);

		const AffineLight relLight = AffineLight::calcRelative(ownerLight, targetLight);

		const double light_a = relLight.a();
		const double light_b = relLight.b();
		const double targetLight_beta = (double)targetLight.beta();

		// inverse depth
		const double iDepth = parameters[2][0];

		// obtain geometric jacobians for central pixel
		Eigen::Matrix<double, 2, 6> JupJT;
		Eigen::Matrix<double, 2, 1> JupJid;
		if (jacobians != NULL)
		{
			Eigen::Vector2d Xpn, pt2d;
			double newIDepth, rescale;

			// project using central pixel
			if (!Utils::project((double)point->u(lvl), (double)point->v(lvl), iDepth,
								K, width, height, R, t,
								Xpn, pt2d, newIDepth, rescale))
			{
				// discard this residual
				this->residual_->state_ = Visibility::OOB;
				this->discardOOB(residuals, jacobians);

				return true;
			}

			// inverse depth jacobian
			if (jacobians[2] != NULL)
			{
				JupJid(0, 0) = K(0, 0)*rescale*(t[0] - t[2] * Xpn[0]);
				JupJid(1, 0) = K(1, 1)*rescale*(t[1] - t[2] * Xpn[1]);
			}

			// pose jacobian
			if (jacobians[0] != NULL || jacobians[1] != NULL)
			{
				// relative pose jacobian
				JupJT(0, 0) = K(0, 0) * newIDepth;
				JupJT(0, 1) = 0.0;
				JupJT(0, 2) = -K(0, 0) * newIDepth * Xpn[0];
				JupJT(0, 3) = -K(0, 0) * Xpn[0] * Xpn[1];
				JupJT(0, 4) = K(0, 0) * (1.0 + Xpn[0] * Xpn[0]);
				JupJT(0, 5) = -K(0, 0) * Xpn[1];

				JupJT(1, 0) = 0.0;
				JupJT(1, 1) = K(1, 1) * newIDepth;
				JupJT(1, 2) = -K(1, 1) * newIDepth * Xpn[1];
				JupJT(1, 3) = -K(1, 1) * (1.0 + Xpn[1] * Xpn[1]);
				JupJT(1, 4) = K(1, 1) * Xpn[0] * Xpn[1];
				JupJT(1, 5) = K(1, 1) * Xpn[0];

				// adjoint
				JupJT *= worldToTarget.Adj();
			}
		}

		// compute residual for all pattern points
		double energy = 0.0;
		double hessian = 0.0;
		double gradient = 0.0;
		double avgWeight = 0.0;
		double numBad = 0.0;

		const Eigen::Matrix3d KRKinv = K * R * Kinv;
		const Eigen::Vector3d Kt = K * t;

		for (int32_t idx = 0; idx < Pattern::size(); ++idx)
		{
			double uj = (double)point->u(lvl) + (double)Pattern::at(idx, 0);
			double vj = (double)point->v(lvl) + (double)Pattern::at(idx, 1);

			Eigen::Vector2d pt2d;

			if (!Utils::project(uj, vj, iDepth, width, height,
								KRKinv, Kt, pt2d))
			{
				// discard this residual
				this->residual_->state_ = Visibility::OOB;
				this->discardOOB(residuals, jacobians);
				
				return true;
			}

			// obtain bilinear interpolated intensity values
			double newColor = bilinearInterpolation(image, pt2d[0], pt2d[1], width);

			// residual with light compensation and weight
			double res = (double)color[idx] - light_a*newColor - light_b;

			// image jacobian: dIj/du'
			Eigen::Matrix<double, 1, 2> JIJup;
			JIJup(0, 0) = light_a*bilinearInterpolation(gx, pt2d[0], pt2d[1], width);
			JIJup(0, 1) = light_a*bilinearInterpolation(gy, pt2d[0], pt2d[1], width);

			double squaredGrad = JIJup.squaredNorm();

			// observation weight based on gradient magnitude
			// set as the mean value between owner and target frames
			double gradWeight = sqrt(settings.weightConstant / (settings.weightConstant + squaredGrad));
			gradWeight = 0.5*((double)weight[idx] + gradWeight);

			// apply gradient weight first
			const double resgw = res * gradWeight;

			// weight function
			const double distWeight = dist->weight((float)resgw);
			const double totalWeight = gradWeight*sqrt(distWeight);

			// energy
			residuals[idx] = totalWeight*res;
			this->residual_->pixelEnergy_[idx] = resgw;
			energy += distWeight * resgw*resgw;			

			// weight average
			avgWeight += distWeight;

			// sum of gradients to avoid residuals in white walls
			gradient += distWeight * gradWeight * gradWeight * squaredGrad;

			// discard bad pixels
			if (fabs(resgw) > energyThreshold)
			{
				numBad++;
			}

			// compute jacobian
			if (jacobians != NULL)
			{
				// E = (1+alpha_i)*I_i(u) + beta_i - (1+alpha_j)*I(u') - beta_j
				// u' = project(X') = project(Tji * X) = project(Tji * unproject(u, iDepth) )
				// Tji = Tj^-1 * Ti

				// jacobians[i][r * parameter_block_sizes_[i] + c]
				// i: parameter block 
				// r: residual
				// c: parameter from block

				const Eigen::Matrix<double, 1, 6> JIJT = totalWeight * JIJup * JupJT;
				const double JIJalpha = totalWeight * light_a*(newColor - targetLight_beta);

				// owner frame: J1x9
				if (jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 9>> Jowner(jacobians[0] + (idx * 9));
					Jowner.segment<6>(0) = -JIJT;														// owner pose
					Jowner[6] = -JIJalpha;																// owner alpha
					Jowner[7] = -totalWeight;															// owner beta
					Jowner[8] = 0.0;

					// variable scaling
					// "Numerical Optimization" Nocedal et al. 2006, page 95
					Jowner.segment<3>(0) *= settings.varScaleTrans;
					Jowner.segment<3>(3) *= settings.varScaleRot;
					Jowner[6] *= settings.varScaleAlpha;
					Jowner[7] *= settings.varScaleBeta;
				}

				// target frame: J1x9
				if (jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 9>> Jtarget(jacobians[1] + (idx * 9));
					Jtarget.segment<6>(0) = JIJT;														// target pose
					Jtarget[6] = JIJalpha;																// target alpha
					Jtarget[7] = totalWeight * light_a;													// target beta
					Jtarget[8] = 0.0;

					// variable scaling
					// "Numerical Optimization" Nocedal et al. 2006, page 95
					Jtarget.segment<3>(0) *= settings.varScaleTrans;
					Jtarget.segment<3>(3) *= settings.varScaleRot;
					Jtarget[6] *= settings.varScaleAlpha;
					Jtarget[7] *= settings.varScaleBeta;
				}

				// point inverse depth: J1x1
				if (jacobians[2] != NULL)
				{
					double iDepthJacob = -totalWeight * JIJup * JupJid;

					jacobians[2][idx] = iDepthJacob * settings.varScaleIDepth;

					hessian += iDepthJacob * iDepthJacob;
				}
			}
		}

		// check outlier
		if (numBad / Pattern::size() > settings.maxPixelOutlier ||
			gradient < (double)settings.minOptimizationGrad)
		{
			this->residual_->state_ = Visibility::OUTLIER;

			if (numBad / Pattern::size() > settings.maxPixelDiscard)
			{
				this->discardOutlier(jacobians);
			}
		}
		else
		{
			this->residual_->state_ = Visibility::VISIBLE;
		}

		// store useful values
		this->residual_->energy_ = energy;
		this->residual_->lossWeight_ = avgWeight / Pattern::size();

		if (jacobians != NULL && jacobians[2] != NULL)
		{
			this->residual_->iDepthHessian_ = hessian;
		}

		return true;
	}

	void PhotometricCostFunction::discardOOB(double* residuals, double** jacobians) const
	{
		const int numPoints = Pattern::size();

		std::fill(residuals, residuals + numPoints, 0.0);

		if (jacobians != NULL)
		{
			// all jacobians to zero
			if (jacobians[0] != NULL)
			{
				std::fill(jacobians[0], jacobians[0] + (numPoints * 9), 0.0);
			}
			if (jacobians[1] != NULL)
			{
				std::fill(jacobians[1], jacobians[1] + (numPoints * 9), 0.0);
			}
			if (jacobians[2] != NULL)
			{
				std::fill(jacobians[2], jacobians[2] + numPoints, 0.0);
			}
		}
	}

	void PhotometricCostFunction::discardOutlier(double** jacobians) const
	{
		if (jacobians != NULL)
		{
			const int numPoints = Pattern::size();

			// all jacobians to zero
			if (jacobians[0] != NULL)
			{
				std::fill(jacobians[0], jacobians[0] + (numPoints * 9), 0.0);
			}
			if (jacobians[1] != NULL)
			{
				std::fill(jacobians[1], jacobians[1] + (numPoints * 9), 0.0);
			}
			if (jacobians[2] != NULL)
			{
				std::fill(jacobians[2], jacobians[2] + numPoints, 0.0);
			}
		}
	}

	void PhotometricCostFunction::discardOutlier(double** jacobians, int idx) const
	{
		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Map<Eigen::Matrix<double, 1, 9>> Jowner(jacobians[0] + (idx * 9));
				Jowner.setZero();
			}
			if (jacobians[1] != NULL)
			{
				Eigen::Map<Eigen::Matrix<double, 1, 9>> Jtarget(jacobians[1] + (idx * 9));
				Jtarget.setZero();
			}
			if (jacobians[2] != NULL)
			{
				jacobians[2][idx] = 0.0;
			}
		}
	}

	// Residual

	PhotometricResidual::PhotometricResidual(const std::unique_ptr<ActivePoint>& point,
											 const std::shared_ptr<Frame>& targetFrame,
											 const std::unique_ptr<CeresPhotometricBA>& photometricBA) :
		point_(point.get()), 
		ownerFrame_(point->reference()), 
		targetFrame_(targetFrame.get()), 
		photometricBA_(photometricBA.get())
	{
		this->state_ = Visibility::VISIBLE;

		this->iDepthHessian_ = 0.0;
		this->energy_ = -1.0;
		this->lossWeight_ = 1.0;

		this->pixelEnergy_.resize(Pattern::size());
		this->pixelEnergy_.fill(-1);

		this->costFunction_ = std::make_unique<PhotometricCostFunction>(this);
	}

	PhotometricResidual::~PhotometricResidual()
	{
	}

	int PhotometricResidual::dimension() const
	{
		return this->costFunction_->num_residuals();
	}

	int PhotometricResidual::numParameterBlocks() const
	{
		return (int)this->costFunction_->parameter_block_sizes().size();
	}

	ceres::CostFunction* PhotometricResidual::getCostFunction() const
	{
		return this->costFunction_.get();
	}

	ActivePoint* PhotometricResidual::point() const
	{
		return this->point_;
	}

	Frame* PhotometricResidual::ownerFrame() const
	{
		return this->ownerFrame_;
	}

	Frame* PhotometricResidual::targetFrame() const
	{
		return this->targetFrame_;
	}

	Visibility PhotometricResidual::state() const
	{
		return this->state_;
	}

	double PhotometricResidual::iDepthHessian() const
	{
		return this->iDepthHessian_;
	}

	double PhotometricResidual::energy() const
	{
		return this->energy_;
	}

	const Eigen::VecXd& PhotometricResidual::pixelEnergy() const
	{
		return this->pixelEnergy_;
	}

	double PhotometricResidual::lossWeight() const
	{
		return this->lossWeight_;
	}

	bool PhotometricResidual::evaluate(int lvl, Eigen::VecXf& residuals) const
	{
		const auto& settings = Settings::getInstance();

		if (!this->point_->valid(lvl))
		{
			return false;
		}
		
		// output residuals
		residuals.resize(Pattern::size());

		// calibration
		const auto& calib = GlobalCalibration::getInstance();
		const Eigen::Matrix3f& K = calib.matrix3f(lvl);
		const Eigen::Matrix3f& Kinv = calib.invMatrix3f(lvl);
		const int32_t width = calib.width(lvl);
		const int32_t height = calib.height(lvl);

		// parameters
		const float iDepth = (float)this->point_->pointBlock()->getIDepth();
		const Sophus::SE3f ownerToWorld = this->ownerFrame_->frameBlock()->getPose().cast<float>();
		const Sophus::SE3f targetToWorld = this->targetFrame_->frameBlock()->getPose().cast<float>();
		const AffineLight ownerLight = this->ownerFrame_->frameBlock()->getAffineLight();
		const AffineLight targetLight = this->targetFrame_->frameBlock()->getAffineLight();

		const Sophus::SE3f worldToTarget = targetToWorld.inverse();
		const Sophus::SE3f ownerToTarget = worldToTarget * ownerToWorld;
		const Eigen::Matrix3f R = ownerToTarget.rotationMatrix();
		const Eigen::Vector3f& t = ownerToTarget.translation();

		const Eigen::Matrix3f KRKinv = K * R * Kinv;
		const Eigen::Vector3f Kt = K * t;

		const AffineLight relLight = AffineLight::calcRelative(ownerLight, targetLight);
		const float light_a = relLight.a();
		const float light_b = relLight.b();

		// reference
		const Eigen::VecXf& color = this->point_->colors(lvl);
		const Eigen::VecXf& weight = this->point_->weights(lvl);

		const float* image = this->targetFrame_->image(lvl);
		const float* gx = this->targetFrame_->gx(lvl);
		const float* gy = this->targetFrame_->gy(lvl);

		for (int32_t idx = 0; idx < Pattern::size(); ++idx)
		{
			const float uj = this->point_->u(lvl) + (float)Pattern::at(idx, 0);
			const float vj = this->point_->v(lvl) + (float)Pattern::at(idx, 1);

			Eigen::Vector2f pt2d;
			if (!Utils::project(uj, vj, iDepth, width, height,
								KRKinv, Kt, pt2d))
			{
				return false;
			}

			// obtain bilinear interpolated intensity values
			const float newColor = bilinearInterpolation(image, pt2d[0], pt2d[1], width);

			// residual with light compensation
			const float res = color[idx] - light_a*newColor - light_b;

			// image gradient: dIj/du'
			Eigen::Matrix<float, 1, 2> JIJup;
			JIJup(0, 0) = light_a*bilinearInterpolation(gx, pt2d[0], pt2d[1], width);
			JIJup(0, 1) = light_a*bilinearInterpolation(gy, pt2d[0], pt2d[1], width);

			// observation weight based on gradient magnitude
			// set as the mean value between owner and target frames
			float gradWeight = sqrt(settings.weightConstant / (settings.weightConstant + JIJup.squaredNorm()));
			gradWeight = 0.5f*(weight[idx] + gradWeight);

			// apply gradient weight first
			residuals[idx] = res * gradWeight;
		}

		return true;
	}
}