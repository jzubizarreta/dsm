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

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QTimer>

#include "MouseInterface.h"

namespace dsm
{
	class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
	{
		Q_OBJECT

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	public:
		GLWidget(QWidget *parent = 0);
		~GLWidget();

		void applyMouseTransform(float scale) { mouseHandler.applyMouseTransform(scale); }

	signals:
		void initOpenGL();
		void renderOpenGL();
		void doubleClicked(int x, int y);		

	protected:
		// redefined functions
		void initializeGL();
		void resizeGL(int width, int height);
		void paintGL();

		// mouse events
		void mousePressEvent(QMouseEvent *event) override;
		void mouseMoveEvent(QMouseEvent *event) override;
		void mouseReleaseEvent(QMouseEvent *event) override;

		void mouseDoubleClickEvent(QMouseEvent *event) override;

		void wheelEvent(QWheelEvent *event) override;

	private:
		// mouse control
		MouseInterface mouseHandler;

		// printer
		static bool doPrint;
	};
}