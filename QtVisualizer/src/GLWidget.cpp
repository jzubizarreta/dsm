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

#include "GLWidget.h"
#include <QMouseEvent>
#include <iostream>

namespace dsm
{
	bool GLWidget::doPrint = true;

	GLWidget::GLWidget(QWidget *parent) : QOpenGLWidget(parent)
	{
		QSurfaceFormat format;
		format.setDepthBufferSize(24);
		format.setVersion(2, 1);
		format.setProfile(QSurfaceFormat::CompatibilityProfile);
		setFormat(format);

		// Do not call OpenGL functions here. Call OpenGL initialization code
		// in initializeGL
	}

	GLWidget::~GLWidget()
	{
	}

	//
	// GL configuration
	//
	void GLWidget::initializeGL()
	{
		this->initializeOpenGLFunctions();

		if (GLWidget::doPrint)
		{
			std::cout << "OpenGL Version= " << glGetString(GL_VERSION) << "\n";
			GLWidget::doPrint = false;
		}

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_AUTO_NORMAL);

		glEnable(GL_NORMALIZE);

		glDepthFunc(GL_LESS);
		glEnable(GL_DEPTH_TEST);

		glEnable(GL_CULL_FACE);

		emit initOpenGL();
	}

	void GLWidget::resizeGL(int width, int height)
	{
		glViewport(0, 0, width, height);
		update();
	}

	//
	// Rendering, continiously called
	//
	void GLWidget::paintGL()
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		emit renderOpenGL();
	}

	//
	// Mouse events
	//

	/**
	Performs picking. The function that draws pickable objects must be a slot connected with renderPicking() signal.
	The results are emitted using mouseClicked signal
	*/
	void GLWidget::mousePressEvent(QMouseEvent *event)
	{
		makeCurrent();

		if (event->button() == Qt::LeftButton)
			mouseHandler.mousePressed(event->x(), event->y(), ROTATION);
		if (event->button() == Qt::MidButton)
			mouseHandler.mousePressed(event->x(), event->y(), SCALE);
		if (event->button() == Qt::RightButton)
			mouseHandler.mousePressed(event->x(), event->y(), TRANSLATION);

		update();
	}

	void GLWidget::mouseMoveEvent(QMouseEvent *event)
	{
		makeCurrent();

		if (event->buttons() == Qt::LeftButton)
			mouseHandler.mouseMotion(event->x(), event->y(), ROTATION);
		if (event->buttons() == Qt::MidButton)
			mouseHandler.mouseMotion(event->x(), event->y(), SCALE);
		if (event->buttons() == Qt::RightButton)
			mouseHandler.mouseMotion(event->x(), event->y(), TRANSLATION);

		update();
	}

	void GLWidget::mouseReleaseEvent(QMouseEvent *event)
	{
		makeCurrent();

		if (event->button() == Qt::LeftButton)
			mouseHandler.mouseReleased(event->x(), event->y(), ROTATION);
		if (event->button() == Qt::MidButton)
			mouseHandler.mouseReleased(event->x(), event->y(), SCALE);
		if (event->button() == Qt::RightButton)
			mouseHandler.mouseReleased(event->x(), event->y(), TRANSLATION);

		update();
	}

	void GLWidget::mouseDoubleClickEvent(QMouseEvent *event)
	{
		emit doubleClicked(event->x(), event->y());
		update();
	}

	void GLWidget::wheelEvent(QWheelEvent * event)
	{
		makeCurrent();
		QPoint numPixels = event->pixelDelta();
		QPoint numDegrees = event->angleDelta() / 8;

		if (!numPixels.isNull())
		{
			mouseHandler.mouseWheel(numPixels.y());
		}
		else if (!numDegrees.isNull())
		{
			QPoint numSteps = numDegrees / 15;
			mouseHandler.mouseWheel(numSteps.y());
		}

		event->accept();
		update();
	}
}
