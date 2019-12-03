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

#include "MouseInterface.h"
#include <algorithm>
#include <QGLWidget>	// OpenGL includes

#include <Eigen/Dense>

namespace dsm
{
	float MouseInterface::rotationSensibility = 4.5f;
	float MouseInterface::zoomSensibility = 20.0f;
	float MouseInterface::translationSensibility = 20.0f;

	MouseInterface::MouseInterface()
	{
		reset();
	}

	MouseInterface::~MouseInterface()
	{
	}

	void MouseInterface::mousePressed(int x, int y, MouseMotionType codMov)
	{
		using namespace std;

		switch (codMov)
		{
		case TRANSLATION:
		case SCALE:
			translationTmp.setZero();	// initialize translationTmp to 0
			break;
		case ROTATION:
			rotationTmp.setZero();	// initialize rotationTmp to 0
			break;
		default:
			break;
		}

		mouseStartPosition[0] = x;
		mouseStartPosition[1] = y;
	}

	void MouseInterface::mouseMotion(int x, int y, MouseMotionType codMov)
	{
		Eigen::Vector3f mouse(mouseStartPosition[0] - x, mouseStartPosition[1] - y, 0.0f);
		Eigen::Vector3f rotationAxis;

		switch (codMov)
		{
		case TRANSLATION:
		{
			translationTmp[0] = -mouse[0] / translationSensibility;
			translationTmp[1] = mouse[1] / translationSensibility;
			translationTmp[2] = 0;
		}
		break;
		case ROTATION:
		{
			Eigen::Vector3f z(0.0f, 0.0f, -1.f);
			rotationAxis = z.cross(mouse);
			rotationTmp[0] = rotationAxis.norm() / rotationSensibility;
			rotationTmp[1] = -rotationAxis[0];
			rotationTmp[2] = rotationAxis[1];
			rotationTmp[3] = rotationAxis[2];
		}
		break;
		case SCALE:
		{
			translationTmp[2] = mouse[1] / zoomSensibility;
		}
		break;
		default:
			break;
		}
	}

	void MouseInterface::mouseReleased(int x, int y, MouseMotionType codMov)
	{
		using namespace std;
		switch (codMov)
		{
		case TRANSLATION:
		case SCALE:
			translation[0] += translationTmp[0];
			translation[1] += translationTmp[1];
			translation[2] += mouseWheelZoom + translationTmp[2];
			translationTmp.setZero();
			mouseWheelZoom = 0;
			break;
		case ROTATION:
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glRotatef(rotationTmp[0], rotationTmp[1], rotationTmp[2], rotationTmp[3]);
			glPushMatrix();
			glMultMatrixf(rotation.data());
			glGetFloatv(GL_MODELVIEW_MATRIX, rotation.data());
			glPopMatrix();
			rotationTmp.setZero();
			break;
		default:
			break;
		}
	}

	void MouseInterface::mouseWheel(int value)
	{
		mouseWheelZoom += value;
	}

	void MouseInterface::reset()
	{
		translation.setZero();
		translationTmp.setZero();
		rotationTmp.setZero();
		rotation.setIdentity();

		mouseWheelZoom = 0.0f;
	}

	void MouseInterface::applyMouseTransform(float scale)
	{
		glTranslatef(scale*translationTmp[0], scale*translationTmp[1], scale*(mouseWheelZoom + translationTmp[2]));
		glTranslatef(scale*translation[0], scale*translation[1], scale*translation[2]);
		glRotatef(rotationTmp[0], rotationTmp[1], rotationTmp[2], rotationTmp[3]);
		glMultMatrixf(rotation.data());
	}
}
