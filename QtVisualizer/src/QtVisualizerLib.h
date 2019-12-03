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

#if defined (WIN32)
#if defined (_MSC_VER)
#pragma warning(disable: 4251)
#endif
#if defined(QtVisualizer_EXPORTS)
#define  QT_VISUALIZER_EXPORTS_DLL __declspec(dllexport)
#else
#define  QT_VISUALIZER_EXPORTS_DLL __declspec(dllimport)
#endif
#else
#define QT_VISUALIZER_EXPORTS_DLL
#endif

extern "C"
{
	bool QT_VISUALIZER_EXPORTS_DLL printQtVisualizerLibVersion();
}