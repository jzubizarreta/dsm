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

#include "QtWindow.h"
#include "QtVisualizer.h"
#include "GLDrawings.h"

#include <iostream>
#include <cmath>

#ifdef WIN32
	#include <windows.h>
#endif

#ifdef __APPLE__
	#include <OpenGL/gl.h>
	#include <OpenGL/glu.h>
#else
	#include <GL/gl.h>
	#include <GL/glu.h>
#endif

#include <QFileDialog>

#include "opencv2/imgproc.hpp"

namespace dsm
{
	QtWindow::QtWindow(QtVisualizer& outWrapper, QWidget *parent) :
		QMainWindow(parent),
		outputWrapper(outWrapper),
		imgWidget(nullptr),
		procWidget(nullptr),
		VRWidget(nullptr),
		renderTimer(nullptr),
		doPlay(false),
		doReset(false),
		imgWidth(0),
		imgHeight(0)
	{
		std::cout << "Qt version " << QT_VERSION_STR << std::endl;

		ui.setupUi(this);

		// white central widget
		ui.centralWidget->setStyleSheet("QWidget{background-color:white;}");

		// status bar
		this->statusBar()->setStyleSheet("background-color:white;");

		QString buttonStyle = "QPushButton{border-radius:10px;border:white;background-color:white;}"
			"QPushButton:hover{border-radius:10px;background-color:rgb(220,220,220);}"
			"QPushButton:pressed{border-style:outset;border-width:1px;border-radius:10px;border-color:rgb(200,200,200);background-color:rgb(220,220,220);}"
			"QPushButton:checked{border-style:outset;border-width:1px;border-radius:10px;border-color:rgb(200,200,200);background-color:rgb(220,220,220);}";

		ui.playButton->setStyleSheet(buttonStyle);
		ui.resetButton->setStyleSheet(buttonStyle);
		ui.saveButton->setStyleSheet(buttonStyle);

		// CHECK BOXES
		// set the same spacing to align the check box
		QString spacing = "QCheckBox{spacing:%1px}";
		int width = ui.originCheckBox->fontMetrics().width(ui.originCheckBox->text());
		ui.originCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.cameraCheckBox->fontMetrics().width(ui.cameraCheckBox->text());
		ui.cameraCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.trajectoryCheckBox->fontMetrics().width(ui.trajectoryCheckBox->text());
		ui.trajectoryCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.keyframesCheckBox->fontMetrics().width(ui.keyframesCheckBox->text());
		ui.keyframesCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.activeKeyframesCheckBox->fontMetrics().width(ui.activeKeyframesCheckBox->text());
		ui.activeKeyframesCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.covisibilityCheckBox->fontMetrics().width(ui.covisibilityCheckBox->text());
		ui.covisibilityCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.activeCovisibilityCheckBox->fontMetrics().width(ui.activeCovisibilityCheckBox->text());
		ui.activeCovisibilityCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.pointCloudCheckBox->fontMetrics().width(ui.pointCloudCheckBox->text());
		ui.pointCloudCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));
		width = ui.localPointCloudCheckBox->fontMetrics().width(ui.localPointCloudCheckBox->text());
		ui.localPointCloudCheckBox->setStyleSheet(spacing.arg(ui.optionWidget->width() - width - 50));

		this->showOrigin = true;
		this->showCamera = true;
		this->showTrajectory = true;
		this->showKeyframes = true;
		this->showActiveKeyframes = true;
		this->showCovisibility = true;
		this->showActiveCovisibility = true;
		this->showPointCloud = true;
		this->showLocalPointCloud = false;

		ui.originCheckBox->setChecked(this->showOrigin);
		ui.cameraCheckBox->setChecked(this->showCamera);
		ui.trajectoryCheckBox->setChecked(this->showTrajectory);
		ui.keyframesCheckBox->setChecked(this->showKeyframes);
		ui.activeKeyframesCheckBox->setChecked(this->showActiveKeyframes);
		ui.covisibilityCheckBox->setChecked(this->showCovisibility);
		ui.activeCovisibilityCheckBox->setChecked(this->showActiveCovisibility);
		ui.pointCloudCheckBox->setChecked(this->showPointCloud);
		ui.localPointCloudCheckBox->setChecked(this->showLocalPointCloud);

		// threshold sliders
		ui.varSlider->setTracking(true);
		ui.varSlider->setRange(0, 100);		// slider works with integers
		ui.parSlider->setTracking(true);
		ui.parSlider->setRange(0, 100);

		const float varThreshold = 10.f;
		const float parThreshold = 0.f;

		const int varValue = int((varThreshold + 10.f) * (100.f / 20.f) + 0.5f);
		const int parValue = int(parThreshold * (100.f / 90.f) + 0.5f);

		ui.varSlider->setValue(varValue);
		ui.parSlider->setValue(parValue);

		this->outputWrapper.setVarThreshold(pow(10.f, varThreshold));
		this->outputWrapper.setParThreshold(cos(parThreshold*(float)M_PI / 180.f));

		// scale slider
		ui.scaleSlider->setTracking(true);
		ui.scaleSlider->setRange(1, 100);

		const float scale = 1.f;
		const int scaleValue = int(scale + 0.5f);

		ui.scaleSlider->setValue(scaleValue);
		this->outputWrapper.setCamScale(scaleValue);

		// OPENGL WIDGETS

		// VR widget (parent)
		this->VRWidget = new GLWidget(this);
		this->VRWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		
		// input image widget (child)
		this->imgWidget = new GLWidget(this->VRWidget);

		// process image widget (child)
		this->procWidget = new GLWidget(this->VRWidget);

		// add to layout
		this->VRWidget->lower();
		ui.layoutVR->addWidget(this->VRWidget);

		// CONNECTIONS

		// --------------------------------------------------------------------------------------------------
		// IMPORTANT: these connections must be QueuedConnection
		// We have to make sure the rendering is called in the window (main) thread

		//render AR -> image, objects...
		QObject::connect(&this->outputWrapper, SIGNAL(updateARWidget()), this->imgWidget, SLOT(update()), Qt::QueuedConnection);

		//display processing image
		QObject::connect(&this->outputWrapper, SIGNAL(updateProcessImage()), this->procWidget, SLOT(update()), Qt::QueuedConnection);

		//debug windows for visualization with opencv
		QObject::connect(&this->outputWrapper, SIGNAL(updateCVDebugWindow(int)), this, SLOT(renderOpencvWindows(int)), Qt::QueuedConnection);

		// use a QTimer for VR and StatusBar updates
		this->renderTimer = new QTimer(this);
		this->renderTimer->setInterval(50);		// 20 fps

		//render VR -> pointcloud, octree, keyframes, camera
		QObject::connect(this->renderTimer, SIGNAL(timeout()), this->VRWidget, SLOT(update()));		// QueuedConnection not required

		//timings & other information
		QObject::connect(this->renderTimer, SIGNAL(timeout()), this, SLOT(updateStatusBar()));		// QueuedConnection not required
		// --------------------------------------------------------------------------------------------------

		// --------------------------------------------------------------------------------------------------
		// VR OpenGL initialization
		QObject::connect(this->VRWidget, SIGNAL(initOpenGL()), this, SLOT(initVROpenGL()), Qt::DirectConnection);

		// the signal must be executed manually
		QObject::connect(this->imgWidget, SIGNAL(renderOpenGL()), this, SLOT(renderCamera()), Qt::DirectConnection);
		QObject::connect(this->procWidget, SIGNAL(renderOpenGL()), this, SLOT(renderProcess()), Qt::DirectConnection);
		QObject::connect(this->VRWidget, SIGNAL(renderOpenGL()), this, SLOT(renderOpenGL()), Qt::DirectConnection);

		// resize manually
		QObject::connect(this->VRWidget, SIGNAL(resized()), this, SLOT(changeImgSizes()), Qt::DirectConnection);
		// --------------------------------------------------------------------------------------------------

		// --------------------------------------------------------------------------------------------------
		// Buttons
		QObject::connect(ui.playButton, SIGNAL(clicked(bool)), this, SLOT(play()));
		QObject::connect(ui.resetButton, SIGNAL(clicked(bool)), this, SLOT(reset()));
		QObject::connect(ui.saveButton, SIGNAL(clicked(bool)), this, SLOT(save()));

		// checkboxes
		QObject::connect(ui.originCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.cameraCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.trajectoryCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.keyframesCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.activeKeyframesCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.covisibilityCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.activeCovisibilityCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.pointCloudCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));
		QObject::connect(ui.localPointCloudCheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeChecBoxState(int)));

		// sliders
		QObject::connect(ui.varSlider, SIGNAL(valueChanged(int)), this, SLOT(changeVarSliderValue(int)));
		QObject::connect(ui.parSlider, SIGNAL(valueChanged(int)), this, SLOT(changeParSliderValue(int)));
		QObject::connect(ui.scaleSlider, SIGNAL(valueChanged(int)), this, SLOT(changeScaleSliderValue(int)));
		// --------------------------------------------------------------------------------------------------

		// --------------------------------------------------------------------------------------------------
		// start timers
		this->renderTimer->start();
	}

	QtWindow::~QtWindow()
	{
		// all QWidgets are deleted automatically from this (parent)
	}

	void QtWindow::keyPressEvent(QKeyEvent *k)
	{
		if (k->key() == Qt::Key_Escape)
		{
			this->close();
		}
	}

	bool QtWindow::getDoPlay() const
	{
		return this->doPlay;
	}

	bool QtWindow::getDoReset() const
	{
		return this->doReset;
	}

	bool QtWindow::getShowOrigin() const
	{
		return this->showOrigin;
	}

	bool QtWindow::getShowCamera() const
	{
		return this->showCamera;
	}

	bool QtWindow::getShowTrajectory() const
	{
		return this->showTrajectory;
	}

	bool QtWindow::getShowKeyframes() const
	{
		return this->showKeyframes;
	}

	bool QtWindow::getShowActiveKeyframes() const
	{
		return this->showActiveKeyframes;
	}

	bool QtWindow::getShowCovisibility() const
	{
		return this->showCovisibility;
	}

	bool QtWindow::getShowActiveCovisibility() const
	{
		return this->showActiveCovisibility;
	}

	bool QtWindow::getShowPointCloud() const
	{
		return this->showPointCloud;
	}

	bool QtWindow::getShowLocalPointCloud() const
	{
		return this->showLocalPointCloud;
	}

	void QtWindow::setImageSize(int w, int h)
	{
		this->imgWidth = w;
		this->imgHeight = h;
		this->imgRatio = (float)this->imgHeight / this->imgWidth;
	}

	void QtWindow::save()
	{
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save Reconstruction"), QDir::homePath(), tr("PLY Model (*.ply)"));

		if (!fileName.isEmpty())
		{
			this->outputWrapper.saveReconstruction(fileName.toStdString());
		}
	}

	void QtWindow::play(bool value)
	{
		if (value)
		{
			this->doPlay = !this->doPlay;
		}
		else
		{
			this->doPlay = false;
		}
	}

	void QtWindow::reset(bool value)
	{
		this->doReset = value;
	}

	void QtWindow::changeChecBoxState(int state)
	{
		this->showOrigin = ui.originCheckBox->isChecked();
		this->showCamera = ui.cameraCheckBox->isChecked();
		this->showTrajectory = ui.trajectoryCheckBox->isChecked();
		this->showKeyframes = ui.keyframesCheckBox->isChecked();
		this->showActiveKeyframes = ui.activeKeyframesCheckBox->isChecked();
		this->showCovisibility = ui.covisibilityCheckBox->isChecked();
		this->showActiveCovisibility = ui.activeCovisibilityCheckBox->isChecked();
		this->showPointCloud = ui.pointCloudCheckBox->isChecked();
		this->showLocalPointCloud = ui.localPointCloudCheckBox->isChecked();
	}

	void QtWindow::changeVarSliderValue(int value)
	{
		// interpolate values from 0:100 to -10:10
		// threshold = 10^value
		float varValue = value * (20.f / 100.f) - 10.f;
		this->outputWrapper.setVarThreshold(pow(10.f, varValue));

		//std::cout << "Sliding variance: " << varValue << std::endl;
	}

	void QtWindow::changeParSliderValue(int value)
	{
		// interpolate values from 0:100 (int) to 0:90 (degrees float)
		// threshold = cos(degrees)
		float pValue = value * (90.f / 100.f);
		this->outputWrapper.setParThreshold(cos(pValue*(float)M_PI / 180.f));

		//std::cout << "Sliding parallax: " << pValue << " degrees" << std::endl;
	}

	void QtWindow::changeScaleSliderValue(int value)
	{
		// interpolate values from 0:100 (int) to 0:100 (float)
		float sValue = (float)value;
		this->outputWrapper.setCamScale(sValue);
	}

	void QtWindow::renderCamera()
	{
		this->imgWidget->makeCurrent();
		this->outputWrapper.augmentCamera();
		this->imgWidget->doneCurrent();
	}

	void QtWindow::renderProcess()
	{
		this->procWidget->makeCurrent();
		this->outputWrapper.augmentProcCamera();
		this->procWidget->doneCurrent();
	}

	void QtWindow::changeImgSizes()
	{
		if (this->imgWidth == 0 || this->imgHeight == 0) return;

		// parent size
		const auto& parentSize = this->VRWidget->size();

		// new size
		float w = parentSize.width() / 4;
		if (w < 160) w = 160;
		float h = this->imgRatio * w;

		// resize image widget
		this->imgWidget->setGeometry(0, 0, (int)w, (int)h);

		// resize proc widget
		this->procWidget->setGeometry(0, (int)h, (int)w, (int)h);
	}

	void QtWindow::updateStatusBar()
	{
		QString message = this->outputWrapper.getStatusBarMessage();
		this->statusBar()->showMessage(message);
	}

	void QtWindow::initVROpenGL()
	{
		// Light initialization:
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHTING);
		glEnable(GL_COLOR_MATERIAL);

		std::array<float, 4> illumination = { -1.f, 1.f, 1.f, 0.f };

		// Directional light
		glLightfv(GL_LIGHT0, GL_POSITION, illumination.data());

		// clear
		glClearColor(1.f, 1.f, 1.f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	void QtWindow::renderOpenGL()
	{
		this->VRWidget->makeCurrent();

		glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			GLint viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);
			gluPerspective(45, (float)viewport[2] / viewport[3], 0.001, 1000);
		glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();

			float scale = this->outputWrapper.getCamScale();

			// use mouse interface
			this->VRWidget->applyMouseTransform(scale);

			//outputwrapper rendering
			this->outputWrapper.renderOpenGL();

			glPopMatrix();
		glMatrixMode(GL_PROJECTION);
			glPushMatrix();
		glMatrixMode(GL_MODELVIEW);

		glFlush();

		this->VRWidget->doneCurrent();
	}

	void QtWindow::renderOpencvWindows(int id)
	{
		this->outputWrapper.renderDebugWindows(id);
	}
}




