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

#include "opencv2/core.hpp"

#include <Eigen/Core>

#include "QtVisualizerLib.h"
#include "Visualizer/IVisualizer.h"
#include "KeyframeVisualizer.h"

#include <mutex>
#include <shared_mutex>
#include <array>
#include <deque>
#include <set>
#include <atomic>
#include <unordered_map>
#include <memory>

#include <QApplication>
#include <QObject>

namespace dsm
{
	class QtWindow;
	class ObjectLoader;

	class QT_VISUALIZER_EXPORTS_DLL QtVisualizer : public QObject, public IVisualizer
	{
		Q_OBJECT		

	public:

		// make the construction require an application
		// it must be created before the wrapper
		QtVisualizer(const QApplication& anApp);
		~QtVisualizer();

		//main function that runs the application UI
		void run();

		// reset everything
		void reset();

		// close the window
		void close();

		// image size
		void setImageSize(int w, int h);

		// buttons
		bool getDoProcessing() const;
		bool getDoReset() const;

		// set visualization thresholds
		void setVarThreshold(float value);
		void setParThreshold(float value);

		// set visualization scale
		void setCamScale(float value);
		float getCamScale();
		void setPointScale(float value);

		// function that updates status bar message
		QString getStatusBarMessage();

		//main function called by the main window to render in openGL - VR
		//call only in the thread of the main window!
		void renderOpenGL();

		//main function called by the main window to render in openGL - AR
		//call only in the thread of the main window!
		void augmentCamera();

		//main function called by the main window to render in openGL - process image
		//call only in the thread of the main window!
		void augmentProcCamera();

		//functions that store the data to be displayed by renderOpenGL() and augmentCamera()
		//call this functions whenever you want
		void publishCamTrackingTime(float time);		// it has its own emit
		void resetCamTrackingTime();

		void publishPointTrackingTime(float time);		// it has its own emit
		void resetPointTrackingTime();

		void publishLocalBATime(float time);		// it has its own emit
		void resetLocalBATime();

		void publishLiveFrame(const cv::Mat &image);		// it has its own emit
		void resetLiveFrame();

		void publishProcessFrame(const cv::Mat &image);		// it has its own emit
		void resetProcessFrame();

		void publishCurrentFrame(const Eigen::Matrix4f &framePose);	// you must call updateVR() to update the widget
		void resetCurrentFrame();

		void publishKeyframe(const std::shared_ptr<dsm::Frame>& keyframe, KeyframeType type);	// you must call updateVR() to update the widget
		void publishKeyframeType(const std::shared_ptr<dsm::Frame>& keyframe, KeyframeType type);
		void resetKeyframes();
		void resetKeyframeTypes();

		void publishCovisibility(const Eigen::MatrixXi& adjacencyMatrix);
		void resetCovisibility();

		// pixel detector visualization
		void publishPointDetector(const cv::Mat& img);

		// distance transform visualization
		void publishDistanceTransformBefore(const cv::Mat& img);
		void publishDistanceTransformAfter(const cv::Mat& img);

		// tracking visualization
		void publishTrackingResult(const cv::Mat& img);
		void publishTrackingError(const cv::Mat& img);
		void publishTrackingWeight(const cv::Mat& img);
		void publishTrackingLight(const cv::Mat& img);
		void publishTrackingDistribution(const cv::Mat& img);

		// optimization visualization
		void publishOptKeyframes(const cv::Mat& img);
		void publishOptError(const cv::Mat& img);
		void publishOptErrorDist(const cv::Mat& img);
		void publishOptErrorDistLast(const cv::Mat& img);
		void publishOptRawError(const cv::Mat& img);
		void publishOptWeight(const cv::Mat& img);
		void publishOptLight(const cv::Mat& img);
		void resetDebugWindows();

		// draws all debug windows
		void renderDebugWindows(int id);

		// save reconstruction
		void saveReconstruction(const std::string& fileName);

	signals:
		void updateARWidget();				// each time a new image is published
		void updateProcessImage();			// each time a new processed image is published
		void updateCVDebugWindow(int);		// each time a new cv image is published

	private:
		
		void renderCamera();
		void renderKeyframes();
		void renderCovisibility();
		void renderTrajectory();

		void updateKeyframes();

	private:

		// QApplication
		const QApplication& app;

		// Main window
		std::unique_ptr<QtWindow> mainWindow;

		//Current camera image
		std::mutex imgMutex;
		cv::Mat cameraImage;
		unsigned int liveImgTextureId;

		//Current process image
		std::mutex processImgMutex;
		cv::Mat processImage;
		unsigned int procImgTextureId;

		//timings
		std::mutex timeMutex;
		float camTrackingTime;
		float ptTrackingTime;
		float localBATime;

		// current frame pose
		std::mutex frameMutex;
		Eigen::Matrix4f currentFrame;

		// keyframes
		std::mutex keyframeMutex;
		std::vector<std::unique_ptr<KeyframeVisualizer>> keyframes;
		std::unordered_map<int, KeyframeVisualizer*> idToKeyframes;
		int numPoints;

		std::mutex thresholdMutex;
		float varThreshold;
		float parThreshold;

		std::mutex scaleMutex;
		float camScale;
		float pointScale;

		// covisibility
		std::mutex covisibilityMutex;
		Eigen::MatrixXi covisibility;

		// optimization visualization
		std::mutex cvDebugMutex;

		cv::Mat cvPointDetector;			// 0

		cv::Mat cvDistTransformBefore;		// 1
		cv::Mat cvDistTransformAfter;		// 2

		cv::Mat cvTrackingResult;			// 3
		cv::Mat cvTrackingError;			// 4
		cv::Mat cvTrackingWeight;			// 5
		cv::Mat cvTrackingLight;			// 6
		cv::Mat cvTrackingDist;				// 7

		cv::Mat cvOptKeyframes;				// 8
		cv::Mat cvOptError;					// 9
		cv::Mat cvOptErrorDist;				// 10
		cv::Mat cvOptErrorDistLast;			// 11
		cv::Mat cvOptRawError;				// 12
		cv::Mat cvOptWeight;				// 13
		cv::Mat cvOptLight;					// 14
	};

}