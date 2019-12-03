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
#include <mutex>
#include <deque> 
#include <vector>
#include <condition_variable>

namespace dsm
{
	// This thread pool works for numberOfThreads > 1. If its value is 1 it will execute everything in the main thread.
	// If you want to use a unique parallel thread use WorkerThread instead.
	// If (blockUntilFinish == false) the thread finishes after the end of the job currently running. 
	// Execution of other pending jobs is not guaranteed after wantExit is set.
	class WorkerThreadPool
	{
	public:

		inline WorkerThreadPool(size_t numberOfThreads) :
			wantExit(false),
			allDone(true),
			numThreads(numberOfThreads > 0 ? numberOfThreads : std::thread::hardware_concurrency())
		{
			if (this->numThreads != 1)
			{
				this->isDone.assign(this->numThreads, true);

				for (size_t i = 0; i < this->numThreads; ++i)
				{
					//this->isDone.emplace_back(true);
					this->workers.emplace_back(&WorkerThreadPool::workerLoop, this, i);
				}
			}
		}

		inline ~WorkerThreadPool()
		{
			this->join(false);
		}

		inline size_t getNumThreads()
		{
			return this->numThreads;
		}

		// adds a job to the queue to be processed
		inline void addJob(const std::function<void()>& job)
		{
			if (this->numThreads != 1)
			{
				// add to list
				{
					std::lock_guard<std::mutex> lock(this->queueMutex);
					this->taskQueue.push_back(job);

					// set working control variable
					{
						std::lock_guard<std::mutex> waitLock(this->waitMutex);
						this->allDone = false;
					}
				}

				// tell to a worker that is waiting to start!
				this->queueSignal.notify_one();
			}
			else
			{
				job();
			}
		}

		// blocks until the workers finish
		inline void wait()
		{
			if (this->numThreads != 1)
			{
				std::unique_lock<std::mutex> waitLock(this->waitMutex);
				this->waitSignal.wait(waitLock, [&]() {return this->allDone; });
			}
		}

		// blocks until the worker finish and delete all the thread
		inline void join(bool blockUntilFinish = false)
		{
			if (this->numThreads != 1)
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

				// wait all threads to exit
				for (auto& worker : this->workers)
				{
					if (worker.joinable())
					{
						worker.join();
					}
				}
			}
		}

		WorkerThreadPool(const WorkerThreadPool&) = delete;					// no copying!
		WorkerThreadPool& operator=(const WorkerThreadPool&) = delete;		// no copying!

	private:
		inline void workerLoop(const size_t id)
		{
			std::function<void()> task;

			while (true)
			{
				// pick the first job in the queue or wait otherwise
				{
					std::unique_lock<std::mutex> lock(this->queueMutex);

					// check if workers have finished all the jobs
					if (this->taskQueue.empty())
					{
						this->isDone[id] = true;

						// check if actually all are finished
						bool allFinished = true;
						for (size_t i = 0; i < this->numThreads; ++i)
						{
							allFinished = allFinished && this->isDone[i];
						}

						// all have finished! signal to stop waiting
						if (allFinished)
						{
							{
								std::lock_guard<std::mutex> waitLock(this->waitMutex);
								this->allDone = true;
							}
							this->waitSignal.notify_all();
						}

						// wait for next task
						this->queueSignal.wait(lock, [&]() { return this->wantExit || !this->taskQueue.empty(); });
					}

					if (this->wantExit)
					{
						return;
					}

					this->isDone[id] = false;

					task = std::move(this->taskQueue.front());

					this->taskQueue.pop_front();
				}

				// work!
				task();
			}
		}

	private:

		// threads
		std::vector<std::thread>				workers;
		std::vector<bool>						isDone;

		// tasks queue
		std::deque<std::function<void()>>		taskQueue;

		// synchronization
		std::condition_variable					queueSignal;
		std::mutex								queueMutex;

		std::condition_variable					waitSignal;
		std::mutex								waitMutex;

		bool									allDone;
		bool									wantExit;

		// constant values
		const size_t							numThreads;
	};
}
