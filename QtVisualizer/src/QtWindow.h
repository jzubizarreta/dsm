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

#include <memory>

#include <QMainWindow>
#include <QTimer>
#include <QKeyEvent>
#include <QThread>
#include "ui_qtwindow.h"

#include "GLWidget.h"

#include "opencv2/core.hpp"

namespace dsm
{
	class QtVisualizer;

	class QtWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		explicit QtWindow(QtVisualizer& outWrapper, QWidget *parent = 0);
		~QtWindow();

		// communication functions
		bool getDoPlay() const;
		bool getDoReset() const;

		bool getShowOrigin() const;
		bool getShowCamera() const;
		bool getShowTrajectory() const;
		bool getShowKeyframes() const;
		bool getShowActiveKeyframes() const;
		bool getShowCovisibility() const;
		bool getShowActiveCovisibility() const;
		bool getShowPointCloud() const;
		bool getShowLocalPointCloud() const;

		// image size
		void setImageSize(int w, int h);

	public slots:
		void initVROpenGL();		// light initialization
		void renderOpenGL();		// VR window rendering
		void renderCamera();		// live camera rendering
		void renderProcess();		// processing window rendering

		void changeImgSizes();

		void updateStatusBar();

		void changeChecBoxState(int state);

		void changeVarSliderValue(int value);
		void changeParSliderValue(int value);

		void changeScaleSliderValue(int value);

		void renderOpencvWindows(int id);

		void play(bool value = true);
		void reset(bool value = true);
		void save();

	private:
		void keyPressEvent(QKeyEvent *);	// redefined

	private:
		Ui::QtWindowClass ui;

		//output wrapper reference
		QtVisualizer& outputWrapper;

		// image size
		int imgWidth, imgHeight;
		float imgRatio;

		//rendering widgets
		GLWidget* imgWidget;
		GLWidget* procWidget;
		GLWidget* VRWidget;

		// timers
		QTimer* renderTimer;

		// communication
		bool doPlay;
		bool doReset;

		bool showOrigin;
		bool showCamera;
		bool showTrajectory;
		bool showKeyframes;
		bool showActiveKeyframes;
		bool showCovisibility;
		bool showActiveCovisibility;
		bool showPointCloud;
		bool showLocalPointCloud;
	};
}