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

#include <thread>
#include <functional>
#include <memory>
#include <mutex>
#include <deque> 

namespace dsm
{
	// This class allows reusing a single thread by inserting tasks to a queue 
	// If (blockUntilFinish == false) the thread finishes after the end of the job currently running. 
	// Execution of other pending jobs is not guaranteed after wantExit is set
	class WorkerThread
	{
	public:

		inline WorkerThread() : wantExit(false), isDone(true)
		{
			this->worker = std::make_unique<std::thread>(&WorkerThread::workerLoop, this);
		}

		inline ~WorkerThread()
		{
			this->join(false);
		}

		// adds a job to the queue to be processed
		inline void addJob(const std::function<void()>& job)
		{
			// add to list
			{
				std::lock_guard<std::mutex> lock(this->queueMutex);
				this->taskQueue.push_back(job);

				// set working control variable
				{
					std::lock_guard<std::mutex> waitLock(this->waitMutex);
					this->isDone = false;
				}
			}

			// start working!
			this->queueSignal.notify_all();
		}

		// blocks until the worker finishes
		inline void wait()
		{
			std::unique_lock<std::mutex> waitLock(this->waitMutex);
			this->waitSignal.wait(waitLock, [&]() {return this->isDone; });
		}

		// blocks until the worker finishes and deletes the thread
		inline void join(const bool blockUntilFinish = false)
		{
			if (blockUntilFinish)
			{
				// wait until all tasks are finished
				this->wait();
			}

			{
				std::lock_guard<std::mutex> lock(this->queueMutex);
				this->wantExit = true;
			}

			// send a signal to finish!
			this->queueSignal.notify_all();

			// wait the thread to exit
			if (this->worker->joinable())
			{
				this->worker->join();
			}
		}

		// gets the thread unique identifier
		inline std::thread::id get_id()
		{
			return this->worker->get_id();
		}

		WorkerThread(const WorkerThread&) = delete;					// no copying!
		WorkerThread& operator=(const WorkerThread&) = delete;		// no copying!

	private:
		inline void workerLoop()
		{
			std::function<void()> task;

			while (true)
			{
				// pick the first job in the queue or wait otherwise
				{
					std::unique_lock<std::mutex> lock(this->queueMutex);

					// check if the worker has finished all the jobs
					if (this->taskQueue.empty())
					{
						// signal to stop waiting
						{
							std::lock_guard<std::mutex> waitLock(this->waitMutex);
							this->isDone = true;
						}
						this->waitSignal.notify_all();

						// wait for next task
						this->queueSignal.wait(lock, [&]() { return this->wantExit || !this->taskQueue.empty(); });
					}

					// check if we want to exit
					if (this->wantExit)
					{
						return;
					}

					// take a new taks
					task = std::move(this->taskQueue.front());

					this->taskQueue.pop_front();
				}

				// work!
				task();
			}
		}

	private:

		// thread
		std::unique_ptr<std::thread>			worker;

		// tasks in queue
		std::deque<std::function<void()>>       taskQueue;

		// synchronization
		std::condition_variable					queueSignal;
		std::mutex								queueMutex;

		std::condition_variable					waitSignal;
		std::mutex								waitMutex;

		bool									wantExit;
		bool									isDone;
	};
}