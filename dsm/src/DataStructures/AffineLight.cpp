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

#include "AffineLight.h"
#include <cmath>

namespace dsm
{
	AffineLight::AffineLight() :
		alpha_(0.f), beta_(0.f)
	{
	}

	AffineLight::AffineLight(float alpha, float beta) :
		alpha_(alpha), beta_(beta)
	{
	}

	AffineLight::AffineLight(const Eigen::Vector2f& affLight) :
		alpha_(affLight[0]), beta_(affLight[1])
	{
	}

	AffineLight::AffineLight(const AffineLight& affLight) :
		alpha_(affLight.alpha_), beta_(affLight.beta_)
	{
	}

	float AffineLight::alpha() const
	{
		return this->alpha_;
	}

	float AffineLight::beta() const
	{
		return this->beta_;
	}

	Eigen::Vector2f AffineLight::toEigenVec() const
	{
		return Eigen::Vector2f(this->alpha_, this->beta_);
	}

	float AffineLight::a() const
	{
		return exp(-alpha_);
	}

	float AffineLight::b() const
	{
		return -exp(-alpha_)*this->beta_;
	}

	AffineLight& AffineLight::operator=(const AffineLight& affLight)
	{
		this->alpha_ = affLight.alpha_;
		this->beta_ = affLight.beta_;
		return *this;
	}

	AffineLight AffineLight::calcRelative(const AffineLight& fromGlobal, const AffineLight& toGlobal)
	{
		float relAlpha = toGlobal.alpha() - fromGlobal.alpha();
		float relBeta = toGlobal.beta() - fromGlobal.beta()*exp(relAlpha);

		return AffineLight(relAlpha, relBeta);
	}

	AffineLight AffineLight::calcGlobal(const AffineLight& fromGlobal, const AffineLight& toRelative)
	{
		float toGlobalAlpha = toRelative.alpha() + fromGlobal.alpha();
		float toGlobalBeta = toRelative.beta() + fromGlobal.beta()*exp(toRelative.alpha());

		return AffineLight(toGlobalAlpha, toGlobalBeta);
	}
}
