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

#include <iostream>
#include <thread>
#include <memory>

#include "glog/logging.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "QtVisualizer.h"
#include "FullSystem/FullSystem.h"
#include "Utils/Undistorter.h"
#include "Utils/EurocReader.h"

// Use the best GPU available for rendering (visualization)
#ifdef WIN32
extern "C"
{
	__declspec(dllexport) uint32_t NvOptimusEnablement = 0x00000001;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace dsm
{
	class Processor
	{
	public:

		inline Processor() { this->shouldStop = false; }
		inline ~Processor() { this->join(); }

		inline void run(cv::VideoCapture& reader, Undistorter& undistorter, QtVisualizer& visualizer, std::string& settingsFile)
		{
			this->processThread = std::make_unique<std::thread>(&Processor::doRun, this,
				std::ref(reader), std::ref(undistorter), std::ref(visualizer), std::ref(settingsFile));
		}

		inline void join()
		{
			this->shouldStop = true;

			// wait the thread to exit
			if (this->processThread->joinable())
			{
				std::cout << "Waiting Processor to finish.." << std::endl;

				this->processThread->join();

				std::cout << " .. Processor has finished." << std::endl;
			}
		}

	private:

		inline void doRun(cv::VideoCapture& reader, Undistorter& undistorter, QtVisualizer& visualizer, std::string& settingsFile)
		{
			int id = 0;
			cv::Mat image;
			double timestamp;

			const double fps = reader.get(cv::CAP_PROP_FPS);

			const cv::Mat& cvK = undistorter.getK();
			const Eigen::Matrix3f K((Eigen::Matrix3f() << cvK.at<double>(0, 0), cvK.at<double>(0, 1), cvK.at<double>(0, 2),
				cvK.at<double>(1, 0), cvK.at<double>(1, 1), cvK.at<double>(1, 2),
				cvK.at<double>(2, 0), cvK.at<double>(2, 1), cvK.at<double>(2, 2)).finished());

			// create DSM
			std::unique_ptr<FullSystem> DSM;

			while (!this->shouldStop)
			{
				// reset
				if (visualizer.getDoReset())
				{
					// reset slam system
					DSM.reset();

					// reset variables
					id = 0;
					timestamp = 0;
					image.release();

					// reset visualizer
					visualizer.reset();

					// reset dataset reader
					reader.set(cv::CAP_PROP_POS_FRAMES, 0.0);
				}

				// capture
				if (visualizer.getDoProcessing() && reader.read(image))
				{
					double time = (double)cv::getTickCount();

					//gray image from source
					if (image.channels() == 3)
					{
						cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
					}

					// undistort
					undistorter.undistort(image, image);

					if (DSM == nullptr)
					{
						DSM = std::make_unique<FullSystem>(undistorter.getOutputWidth(),
														   undistorter.getOutputHeight(),
														   K, settingsFile,
														   &visualizer);
					}

					// process
					DSM->trackFrame(id, timestamp, image.data);

					// visualize image
					visualizer.publishLiveFrame(image);

					// increase counter
					++id;
					++timestamp;

					// wait 
					time = 1000.0*(cv::getTickCount() - time) / cv::getTickFrequency();
					const double delay = (1000.0 / fps) - time;

					if (delay > 0.0)
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(delay)));
					}
				}
				else
				{
					// allow other threads to run
					std::this_thread::yield();
				}
			}

			// print log
			if (DSM)
			{
				DSM->printLog();
			}
		}

	private:

		bool shouldStop;

		std::unique_ptr<std::thread> processThread;
	};
}

int main(int argc, char *argv[])
{
	// input arguments
	std::string videoFile, calibFile, settingsFile;

	// Configuration
	if (argc == 4)
	{
		videoFile = argv[1];
		calibFile = argv[2];
		settingsFile = argv[3];
	}
	else if (argc == 3)
	{
		videoFile = argv[1];
		calibFile = argv[2];
	}
	else
	{
		std::cout << "The VideoExample requires at least 2 arguments: videoFile, calibFile, settingsFile (optional)\n";
		return 0;
	}

	// Initialize logging
	google::InitGoogleLogging(argv[0]);
	//FLAGS_logtostderr = 1;
	//FLAGS_v = 9;

	std::cout << "\n";

	// Create the application before the window always!
	// create visualizer in the main thread
	QApplication app(argc, argv);
	dsm::QtVisualizer visualizer(app);

	std::cout << "\n";

	// read calibration
	dsm::Undistorter undistorter(calibFile);
	if (!undistorter.isValid())
	{
		std::cout << "need camera calibration file..." << std::endl;
		return 0;
	}

	std::cout << "\n";

	// read sequence
	cv::VideoCapture reader(videoFile);
	if (!reader.isOpened())
	{
		std::cout << "no video found ..." << std::endl;
		return 0;
	}

	std::cout << "\n";

	// add image size to the visualizer
	visualizer.setImageSize(undistorter.getOutputWidth(), undistorter.getOutputHeight());

	// run processing in a second thread
	dsm::Processor processor;
	processor.run(reader, undistorter, visualizer, settingsFile);

	// run main window
	// it will block the main thread until closed
	visualizer.run();

	// join processing thread
	processor.join();

	std::cout << "Finished!" << std::endl;

	return 1;
}