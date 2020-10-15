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

#include "QtVisualizer.h"
#include "QtWindow.h"
#include "GLDrawings.h"

#include "DataStructures/Frame.h"
#include "DataStructures/ActivePoint.h"
#include "Utils/UtilFunctions.h"

#include <Eigen/Geometry>
#include "sophus/se3.hpp"

#include "opencv2/highgui.hpp"

#include <iostream>
#include <fstream>

namespace dsm
{
	QtVisualizer::QtVisualizer(const QApplication& anApp) :
		app(anApp)
	{
		this->liveImgTextureId = -1;
		this->procImgTextureId = -1;

		this->currentFrame.setZero();

		this->covisibility.setZero();

		this->camTrackingTime = 0.f;
		this->ptTrackingTime = 0.f;
		this->localBATime = 0.f;

		this->numPoints = 0;

		this->varThreshold = std::numeric_limits<float>::max();
		this->parThreshold = 1.f;

		this->camScale = 10.f;
		this->pointScale = 1.f;

		// Create the application before the window always!
		this->app.setAttribute(Qt::AA_UseDesktopOpenGL);

		this->mainWindow = std::make_unique<QtWindow>(*this);
		this->mainWindow->show();						//initialize everything! OpenGL rendering context too!
	}

	QtVisualizer::~QtVisualizer()
	{
		cv::destroyAllWindows();	
	}

	void QtVisualizer::run()
	{
		this->app.exec();
	}

	void QtVisualizer::reset()
	{
		this->resetCamTrackingTime();
		this->resetPointTrackingTime();
		this->resetLocalBATime();
		this->resetLiveFrame();
		this->resetProcessFrame();
		this->resetCurrentFrame();
		this->resetKeyframes();
		this->resetCovisibility();
		this->resetDebugWindows();

		this->mainWindow->reset(false);
		this->mainWindow->play(false);
	}

	void QtVisualizer::close()
	{
		this->mainWindow->close();
	}

	void QtVisualizer::setImageSize(int w, int h)
	{
		this->mainWindow->setImageSize(w, h);
	}

	bool QtVisualizer::getDoProcessing() const
	{
		return this->mainWindow->getDoPlay();
	}

	bool QtVisualizer::getDoReset() const
	{
		return this->mainWindow->getDoReset();
	}

	void QtVisualizer::setVarThreshold(float value)
	{
		std::lock_guard<std::mutex> lock(this->thresholdMutex);
		this->varThreshold = value;
	}

	void QtVisualizer::setParThreshold(float value)
	{
		std::lock_guard<std::mutex> lock(this->thresholdMutex);
		this->parThreshold = value;
	}

	void QtVisualizer::setCamScale(float value)
	{
		std::lock_guard<std::mutex> lock(this->scaleMutex);
		this->camScale = value;
	}

	float QtVisualizer::getCamScale()
	{
		std::lock_guard<std::mutex> lock(this->scaleMutex);
		return this->camScale;
	}

	void QtVisualizer::setPointScale(float value)
	{
		std::lock_guard<std::mutex> lock(this->scaleMutex);
		this->pointScale = value;
	}

	void QtVisualizer::renderOpenGL()
	{
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		// clear
		glClearColor(1.f, 1.f, 1.f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);

		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);

		// update data
		this->updateKeyframes();

		// render world axis
		if (this->mainWindow->getShowOrigin())
		{
			float cs;
			{
				std::lock_guard<std::mutex> lock(this->scaleMutex);
				cs = this->camScale;
			}

			dsm::drawAxis(0.2f*cs, 3.f);
		}

		//covisibility between keyframes
		if (this->mainWindow->getShowCovisibility()) this->renderCovisibility();

		// trajectory
		if (this->mainWindow->getShowTrajectory()) this->renderTrajectory();

		// render keyframes and pointcloud
		this->renderKeyframes();

		//current camera pose
		if (this->mainWindow->getShowCamera()) this->renderCamera();

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		float time = (float)std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.f;
		//std::cout << "renderOpenGL(): " << time << std::endl;
	}

	void QtVisualizer::augmentCamera()
	{
		// DO NOT MODIFY MEMBER VARIABLES OF THE CLASS HERE DUE TO MULTITHREADING!!!
		std::lock_guard<std::mutex> lock(this->imgMutex);

		if (!this->cameraImage.empty())
		{
			//render current camera image
			dsm::drawTextureImg(this->cameraImage, this->liveImgTextureId);

			glFlush();
		}
	}

	void QtVisualizer::augmentProcCamera()
	{
		// DO NOT MODIFY MEMBER VARIABLES OF THE CLASS HERE DUE TO MULTITHREADING!!!
		std::lock_guard<std::mutex> lock(this->processImgMutex);

		if (!this->processImage.empty())
		{
			//render current camera image
			dsm::drawTextureImg(this->processImage, this->procImgTextureId);

			glFlush();
		}
	}

	QString QtVisualizer::getStatusBarMessage()
	{
		int size = 20;

		std::lock_guard<std::mutex> lock(this->timeMutex);
		std::lock_guard<std::mutex> lock2(this->keyframeMutex);

		// count number of points
		this->numPoints = 0;
		for (int i = 0; i < this->keyframes.size(); ++i)
		{
			this->numPoints += this->keyframes[i]->getNumPoints();
		}

		QString space;
		space = space.leftJustified(5, ' ', true);

		const float totalTracking = this->camTrackingTime + this->ptTrackingTime;

		QString trackingMessage("Tracking: " + QString::number(totalTracking, 'f', 1) + "ms");
		trackingMessage = trackingMessage.leftJustified(20, ' ', false);

		QString mappingMessage("Local PBA: " + QString::number(this->localBATime, 'f', 1) + "ms");
		mappingMessage = mappingMessage.leftJustified(25, ' ', false);

		QString keyframeMessage("KFs: " + QString::number(this->keyframes.size(), 'f', 0));
		keyframeMessage = keyframeMessage.leftJustified(10, ' ', false);

		QString pointCloudMessage("MPs: " + QString::number(this->numPoints, 'f', 0));
		pointCloudMessage = pointCloudMessage.leftJustified(10, ' ', false);

		const QString message = space + trackingMessage + mappingMessage + keyframeMessage + pointCloudMessage;

		return message;
	}

	void QtVisualizer::publishCamTrackingTime(float time)
	{
		std::lock_guard<std::mutex> lock(this->timeMutex);
		this->camTrackingTime = time;
	}

	void QtVisualizer::resetCamTrackingTime()
	{
		std::lock_guard<std::mutex> lock(this->timeMutex);
		this->camTrackingTime = 0.f;
	}

	void QtVisualizer::publishPointTrackingTime(float time)
	{
		std::lock_guard<std::mutex> lock(this->timeMutex);
		this->ptTrackingTime = time;
	}

	void QtVisualizer::resetPointTrackingTime()
	{
		std::lock_guard<std::mutex> lock(this->timeMutex);
		this->ptTrackingTime = 0.f;
	}

	void QtVisualizer::publishLocalBATime(float time)
	{
		std::lock_guard<std::mutex> lock(this->timeMutex);
		this->localBATime = time;
	}

	void QtVisualizer::resetLocalBATime()
	{
		std::lock_guard<std::mutex> lock(this->timeMutex);
		this->localBATime = 0.f;
	}

	void QtVisualizer::publishLiveFrame(const cv::Mat &image)
	{
		std::lock_guard<std::mutex> lock(this->imgMutex);
		image.copyTo(this->cameraImage);
		emit updateARWidget();
	}

	void QtVisualizer::resetLiveFrame()
	{
		std::lock_guard<std::mutex> lock(this->imgMutex);
		this->cameraImage.release();
		emit updateARWidget();
	}

	void QtVisualizer::publishProcessFrame(const cv::Mat &image)
	{
		std::lock_guard<std::mutex> lock(this->processImgMutex);
		image.copyTo(this->processImage);
		emit updateProcessImage();
	}

	void QtVisualizer::resetProcessFrame()
	{
		std::lock_guard<std::mutex> lock(this->processImgMutex);
		this->processImage.release();
		emit updateProcessImage();
	}

	void QtVisualizer::publishCurrentFrame(const Eigen::Matrix4f &pose)
	{
		std::lock_guard<std::mutex> lock(this->frameMutex);
		this->currentFrame = pose;
	}

	void QtVisualizer::resetCurrentFrame()
	{
		std::lock_guard<std::mutex> lock(this->frameMutex);
		this->currentFrame.setZero();
	}

	void QtVisualizer::publishKeyframe(const std::shared_ptr<dsm::Frame>& keyframe, KeyframeType type)
	{
		std::lock_guard<std::mutex> lock(this->keyframeMutex);

		// create visualizer if not created yet
		if (this->idToKeyframes.find(keyframe->keyframeID()) == this->idToKeyframes.end())
		{
			std::unique_ptr<KeyframeVisualizer> keyframeVis = std::make_unique<KeyframeVisualizer>();
			this->idToKeyframes[keyframe->keyframeID()] = keyframeVis.get();
			this->keyframes.push_back(std::move(keyframeVis));
		}

		// set new data
		this->idToKeyframes[keyframe->keyframeID()]->compute(keyframe, type);
	}

	void QtVisualizer::publishKeyframeType(const std::shared_ptr<dsm::Frame>& keyframe, KeyframeType type)
	{
		std::lock_guard<std::mutex> lock(this->keyframeMutex);

		if (this->idToKeyframes.find(keyframe->keyframeID()) != this->idToKeyframes.end())
		{
			this->idToKeyframes[keyframe->keyframeID()]->computeType(type);
		}
	}

	void QtVisualizer::resetKeyframes()
	{
		std::lock_guard<std::mutex> lock(this->keyframeMutex);
		this->idToKeyframes.clear();
		this->keyframes.clear();
		this->numPoints = 0;
	}

	void QtVisualizer::resetKeyframeTypes()
	{
		std::lock_guard<std::mutex> lock(this->keyframeMutex);
		for (auto& kf : this->keyframes)
		{
			kf->computeType(KeyframeType::NONE);
		}
	}

	void QtVisualizer::publishCovisibility(const Eigen::MatrixXi& adjacencyMatrix)
	{
		std::lock_guard<std::mutex> lock(this->covisibilityMutex);
		this->covisibility = adjacencyMatrix;
	}

	void QtVisualizer::resetCovisibility()
	{
		std::lock_guard<std::mutex> lock(this->covisibilityMutex);
		this->covisibility = Eigen::MatrixXi();
	}

	void QtVisualizer::renderCamera()
	{
		// disable depth test to guarantee that the current camera overlaps everything
		glDisable(GL_DEPTH_TEST);

		Eigen::Matrix4f pose;

		if (!this->currentFrame.isZero())
		{
			{
				std::lock_guard<std::mutex> lock(this->frameMutex);
				pose = this->currentFrame;
			}

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

				// pose
				glMultMatrixf(pose.data());

				// scale
				float cs;
				{
					std::lock_guard<std::mutex> lock(this->scaleMutex);
					cs = this->camScale;
				}

				// render camera
				dsm::drawCamera(0.1f*cs, 1.5f, Eigen::Vector3f(1.f, 0.f, 0.f), 1.f);

			glPopMatrix();
		}

		glEnable(GL_DEPTH_TEST);
	}

	void QtVisualizer::updateKeyframes()
	{
		std::lock_guard<std::mutex> lock(this->keyframeMutex);

		float varianceThreshold, parallaxThreshold;
		{
			std::lock_guard<std::mutex> lock2(this->thresholdMutex);
			varianceThreshold = this->varThreshold;
			parallaxThreshold = this->parThreshold;
		}

		int numUpdated = 0;
		for (int i = 0; i < this->keyframes.size(); ++i)
		{
			// update only 10 keyframes each time
			if (!(numUpdated < 10)) break;

			// update rendering data if required
			if (this->keyframes[i]->update(varianceThreshold, parallaxThreshold))
			{
				numUpdated++;
			}
		}
	}

	void QtVisualizer::renderKeyframes()
	{
		std::lock_guard<std::mutex> lock(this->keyframeMutex);

		float cs, ps;
		{
			std::lock_guard<std::mutex> lock2(this->scaleMutex);
			cs = this->camScale;
			ps = this->pointScale;
		}
		

		for (int i = 0; i < this->keyframes.size(); ++i)
		{
			// keyframes
			if (this->mainWindow->getShowActiveKeyframes() && this->mainWindow->getShowKeyframes())
			{
				this->keyframes[i]->drawCamera(cs, true);
			}
			else if (this->mainWindow->getShowKeyframes())
			{
				this->keyframes[i]->drawCamera(cs, false);
			}
			else if (this->mainWindow->getShowActiveKeyframes() &&
					(this->keyframes[i]->getType() == dsm::KeyframeType::COVISIBILITY || 
					 this->keyframes[i]->getType() == dsm::KeyframeType::TEMPORAL))
			{
				this->keyframes[i]->drawCamera(cs, true);
			}
			
			// pointcloud
			if (this->mainWindow->getShowPointCloud() && this->mainWindow->getShowLocalPointCloud())
			{
				this->keyframes[i]->drawPointCloud(ps, true);
			}
			else if (this->mainWindow->getShowPointCloud())
			{
				this->keyframes[i]->drawPointCloud(ps, false);
			}
			else if (this->mainWindow->getShowLocalPointCloud() &&
					(this->keyframes[i]->getType() == dsm::KeyframeType::COVISIBILITY ||
					this->keyframes[i]->getType() == dsm::KeyframeType::TEMPORAL))
			{
				this->keyframes[i]->drawPointCloud(ps, false);
			}
		}
	}

	void QtVisualizer::renderCovisibility()
	{
		std::lock_guard<std::mutex> lock(this->covisibilityMutex);
		std::lock_guard<std::mutex> lock2(this->keyframeMutex);

		const int numCameras = (int)this->keyframes.size();

		if (numCameras != this->covisibility.cols()) return;

		glDisable(GL_LIGHTING);

		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		glEnable(GL_LINE_SMOOTH);

		glColor3f(0.f, 1.f, 0.f);
		glLineWidth(1.f);

		glBegin(GL_LINES);

		Eigen::Vector4f t0;
		Eigen::Vector4f t1;

		for (int i = 0; i < numCameras; ++i)
		{
			if (this->mainWindow->getShowActiveCovisibility() &&
				(this->keyframes[i]->getType() != dsm::KeyframeType::COVISIBILITY &&
				this->keyframes[i]->getType() != dsm::KeyframeType::TEMPORAL))
			{
				continue;
			}

			t0 = this->keyframes[i]->getPose().col(3);

			for (int j = i + 1; j < numCameras; ++j)
			{
				if (j == i) continue;

				if (this->mainWindow->getShowActiveCovisibility() &&
					(this->keyframes[j]->getType() != dsm::KeyframeType::COVISIBILITY &&
					this->keyframes[j]->getType() != dsm::KeyframeType::TEMPORAL))
				{
					continue;
				}

				if (this->covisibility(i, j) > 0)
				{
					t1 = this->keyframes[j]->getPose().col(3);

					glVertex3f(t0[0], t0[1], t0[2]);
					glVertex3f(t1[0], t1[1], t1[2]);
				}
			}
		}
		glEnd();

		glDisable(GL_LINE_SMOOTH);

		glEnable(GL_LIGHTING);
	}

	void QtVisualizer::renderTrajectory()
	{
		std::lock_guard<std::mutex> lock(this->keyframeMutex);

		glDisable(GL_LIGHTING);

		//draw trajectory
		glColor3f(1.f, 0.f, 0.f);
		glLineWidth(2.f);
		glBegin(GL_LINE_STRIP);

		// all keyframes
		for (int i = 0; i < this->keyframes.size(); ++i)
		{
			const Eigen::Matrix4f& pose = this->keyframes[i]->getPose();
			glVertex3f(pose(0, 3), pose(1, 3), pose(2, 3));
		}

		// current frame
		glVertex3f(currentFrame(0, 3), currentFrame(1, 3), currentFrame(2, 3));

		glEnd();

		glEnable(GL_LIGHTING);
	}

	void QtVisualizer::publishPointDetector(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvPointDetector = img.clone();
		emit updateCVDebugWindow(0);
	}

	void QtVisualizer::publishDistanceTransformBefore(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvDistTransformBefore = img.clone();
		emit updateCVDebugWindow(1);
	}

	void QtVisualizer::publishDistanceTransformAfter(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvDistTransformAfter = img.clone();
		emit updateCVDebugWindow(2);
	}

	void QtVisualizer::publishTrackingResult(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvTrackingResult = img.clone();
		emit updateCVDebugWindow(3);
	}

	void QtVisualizer::publishTrackingError(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvTrackingError = img.clone();
		emit updateCVDebugWindow(4);
	}

	void QtVisualizer::publishTrackingWeight(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvTrackingWeight = img.clone();
		emit updateCVDebugWindow(5);
	}

	void QtVisualizer::publishTrackingLight(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvTrackingLight = img.clone();
		emit updateCVDebugWindow(6);
	}

	void QtVisualizer::publishTrackingDistribution(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvTrackingDist = img.clone();
		emit updateCVDebugWindow(7);
	}

	void QtVisualizer::publishOptKeyframes(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvOptKeyframes = img.clone();
		emit updateCVDebugWindow(8);
	}

	void QtVisualizer::publishOptError(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvOptError = img.clone();
		emit updateCVDebugWindow(9);
	}

	void QtVisualizer::publishOptErrorDist(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvOptErrorDist = img.clone();
		emit updateCVDebugWindow(10);
	}

	void QtVisualizer::publishOptErrorDistLast(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvOptErrorDistLast = img.clone();
		emit updateCVDebugWindow(11);
	}

	void QtVisualizer::publishOptRawError(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvOptRawError = img.clone();
		emit updateCVDebugWindow(12);
	}

	void QtVisualizer::publishOptWeight(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvOptWeight = img.clone();
		emit updateCVDebugWindow(13);
	}

	void QtVisualizer::publishOptLight(const cv::Mat& img)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);
		this->cvOptLight = img.clone();
		emit updateCVDebugWindow(14);
	}

	void QtVisualizer::resetDebugWindows()
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);

		this->cvPointDetector.release();

		this->cvDistTransformBefore.release();
		this->cvDistTransformAfter.release();

		this->cvTrackingResult.release();
		this->cvTrackingError.release();
		this->cvTrackingWeight.release();
		this->cvTrackingLight.release();

		this->cvOptKeyframes.release();
		this->cvOptError.release();
		this->cvOptErrorDist.release();
		this->cvOptErrorDistLast.release();
		this->cvOptRawError.release();
		this->cvOptWeight.release();
		this->cvOptLight.release();

		cv::destroyAllWindows();
	}

	void QtVisualizer::renderDebugWindows(int id)
	{
		std::lock_guard<std::mutex> lock(this->cvDebugMutex);

		switch (id)
		{
		case 0:
			if (!this->cvPointDetector.empty())
			{
				cv::namedWindow("PointDetector", cv::WINDOW_NORMAL);
				cv::imshow("PointDetector", this->cvPointDetector);
			}
		case 1:
			if (!this->cvDistTransformBefore.empty())
			{
				cv::namedWindow("DistTransformBefore", cv::WINDOW_NORMAL);
				cv::imshow("DistTransformBefore", this->cvDistTransformBefore);
			}
		case 2:
			if (!this->cvDistTransformAfter.empty())
			{
				cv::namedWindow("DistTransformAfter", cv::WINDOW_NORMAL);
				cv::imshow("DistTransformAfter", this->cvDistTransformAfter);
			}
		case 3:
			if (!this->cvTrackingResult.empty())
			{
				cv::namedWindow("TrackResult", cv::WINDOW_NORMAL);
				cv::imshow("TrackResult", this->cvTrackingResult);
			}
		case 4:
			if (!this->cvTrackingError.empty())
			{
				cv::namedWindow("TrackError", cv::WINDOW_NORMAL);
				cv::imshow("TrackError", this->cvTrackingError);
			}
		case 5:
			if (!this->cvTrackingWeight.empty())
			{
				cv::namedWindow("TrackWeights", cv::WINDOW_NORMAL);
				cv::imshow("TrackWeights", this->cvTrackingWeight);
			}
		case 6:
			if (!this->cvTrackingLight.empty())
			{
				cv::namedWindow("TrackLight", cv::WINDOW_NORMAL);
				cv::imshow("TrackLight", this->cvTrackingLight);
			}
		case 7:
			if (!this->cvTrackingDist.empty())
			{
				cv::namedWindow("TrackDist", cv::WINDOW_NORMAL);
				cv::imshow("TrackDist", this->cvTrackingDist);
			}
		case 8:
			if (!this->cvOptKeyframes.empty())
			{
				cv::namedWindow("ActiveKeyframes", cv::WINDOW_NORMAL);
				cv::imshow("ActiveKeyframes", this->cvOptKeyframes);
			}
		case 9:
			if (!this->cvOptError.empty())
			{
				cv::namedWindow("OptError", cv::WINDOW_NORMAL);
				cv::imshow("OptError", this->cvOptError);
			}
		case 10:
			if (!this->cvOptErrorDist.empty())
			{
				cv::namedWindow("OptErrorDist", cv::WINDOW_NORMAL);
				cv::imshow("OptErrorDist", this->cvOptErrorDist);
			}
		case 11:
			if (!this->cvOptErrorDistLast.empty())
			{
				cv::namedWindow("OptErrorDistLast", cv::WINDOW_NORMAL);
				cv::imshow("OptErrorDistLast", this->cvOptErrorDistLast);
			}
		case 12:
			if (!this->cvOptRawError.empty())
			{
				cv::namedWindow("OptRawError", cv::WINDOW_NORMAL);
				cv::imshow("OptRawError", this->cvOptRawError);
			}
		case 13:
			if (!this->cvOptWeight.empty())
			{
				cv::namedWindow("OptWeight", cv::WINDOW_NORMAL);
				cv::imshow("OptWeight", this->cvOptWeight);
			}
		case 14:
			if (!this->cvOptLight.empty())
			{
				cv::namedWindow("Optlight", cv::WINDOW_NORMAL);
				cv::imshow("Optlight", this->cvOptLight);
			}
		default:
			break;
		}		
	}

	void QtVisualizer::saveReconstruction(const std::string& fileName)
	{
		if (fileName.empty()) return;

		std::lock_guard<std::mutex> lock(this->keyframeMutex);

		// obtain 3d points and their color
		std::vector<Eigen::Vector3f> pc;
		std::vector<Eigen::Matrix<unsigned char, 3, 1>> color;

		for (int i = 0; i < this->keyframes.size(); ++i)
		{
			// get data in camera frame
			const auto& kfPC = this->keyframes[i]->getPointCloud();
			const auto& kfColor = this->keyframes[i]->getColors();
			const Sophus::SE3f pose(this->keyframes[i]->getPose());

			// transform into world frame
			std::vector<Eigen::Vector3f> kfPCWorld(kfPC.size());
			for (int i = 0; i < kfPC.size(); ++i)
			{
				kfPCWorld[i] = pose*kfPC[i];
			}

			// insert into list
			pc.insert(pc.end(), kfPCWorld.begin(), kfPCWorld.end());
			color.insert(color.end(), kfColor.begin(), kfColor.end());
		}

		// save
		Utils::savePLY(fileName, pc, std::vector<Eigen::Vector3f>(), color);
	}
}