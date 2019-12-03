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

#include "Log.h"

#include <fstream>
#include <time.h>

namespace dsm
{
	Log::Log()
	{
	}

	Log::~Log()
	{
	}

	Log& Log::getInstance()
	{
		static Log theOneAndOnly;
		return theOneAndOnly;
	}

	void Log::clear()
	{
		std::lock_guard<std::mutex> lock(this->m);
		this->logHistory_.clear();
	}

	void Log::addNewLog(int idx)
	{
		std::lock_guard<std::mutex> lock(this->m);

		const auto it = this->logHistory_.find(idx);
		if (it == this->logHistory_.end())
		{
			std::string logFrame = "Frame " + std::to_string(idx) + ": \t";
			this->logHistory_[idx] = logFrame;
		}		
	}

	void Log::addCurrentLog(int idx, const std::string& log)
	{
		std::lock_guard<std::mutex> lock(this->m);

		const auto it = this->logHistory_.find(idx);
		if (it == this->logHistory_.end())
		{
			std::string logFrame = "Frame " + std::to_string(idx) + ": \t";
			logFrame += log;
			this->logHistory_[idx] = logFrame;
		}
		else
		{
			std::string& logFrame = this->logHistory_[idx];
			logFrame += log;
		}
	}

	void Log::printLog() const
	{
		time_t t = time(0);					// get time now
		struct tm now;

#if WIN32
		localtime_s(&now, &t);
#else
		localtime_r(&t, &now);
#endif

		std::string buffer(7, '\0');
		strftime(&buffer[0], buffer.size(), "%H%M%S", &now);
		buffer.pop_back();

		std::string fileName = "log_" + buffer + ".txt";

		std::ofstream out(fileName);
		for (auto it = this->logHistory_.begin(); it != this->logHistory_.end(); ++it)
		{
			out << (*it).second << "\n";
		}
		out.close();
	}
}