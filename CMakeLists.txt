# 
# This file is part of DSM.
# 
# Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza
# Developed by Jon Zubizarreta,
# for more information see <https://github.com/jzubizarreta/dsm>.
# If you use this code, please cite the respective publications as
# listed on the above website.
# 
# DSM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DSM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DSM. If not, see <http://www.gnu.org/licenses/>.
# 

cmake_minimum_required(VERSION 3.5)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(PROJECT_NAME DSM)

project(${PROJECT_NAME})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(${WIN32})
	#This is to tell the location of opengl32.lib, glu32.lib and other Windows SDK libraries
	set (CMAKE_LIBRARY_PATH "C:/Program Files (x86)/Windows Kits/8.0/Lib/win8/um/x64") 
	add_definitions(-DNOMINMAX)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -O2 -pthread -march=native")
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(dsm)
add_subdirectory(QtVisualizer)
add_subdirectory(Examples)