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

######### Find the dependencies of this project #####
find_package(OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Set the name of the Library
set(LIBRARY_NAME QtVisualizer)

#Current library
set(root ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Define the header and source files of the library. Also for Qt based projects
# define the form and resource files of the project
set(HEADER_MOC_FILES
	${root}/GLWidget.h
	${root}/QtWindow.h
    ${root}/QtVisualizer.h
	) 

set(HEADER_FILES
    ${root}/QtVisualizerLib.h
    ${root}/MouseInterface.h
    ${root}/KeyframeVisualizer.h
    ${root}/GLDrawings.h
        )
	
set(SOURCE_FILES 
	${root}/QtVisualizerLib.cpp
	${root}/QtVisualizer.cpp
	${root}/GLWidget.cpp
	${root}/MouseInterface.cpp
	${root}/QtWindow.cpp
	${root}/KeyframeVisualizer.cpp
	${root}/GLDrawings.cpp
	)
	
set(FORM_FILES
	${root}/qtwindow.ui 
	)
	
set(RESOURCE_FILES 
	${root}/qtwindow.qrc 
	)
	
qt5_wrap_ui(FORM_HEADERS ${FORM_FILES})
qt5_wrap_cpp(HEADERS_MOC ${HEADER_MOC_FILES})
qt5_add_resources(RESOURCES_RCC ${RESOURCE_FILES})
	
# Organize the source and header files into groups
source_group("Headers" FILES ${HEADER_FILES} ${HEADER_MOC_FILES})
source_group("Source" FILES ${SOURCE_FILES})
source_group("Forms" FILES ${FORM_FILES})
source_group("Resources" FILES ${RESOURCE_FILES})

if(CMAKE_VERSION VERSION_GREATER 3.8)
    source_group(TREE ${root} FILES ${HEADER_FILES} ${HEADER_MOC_FILES} ${SOURCE_FILES})
endif()

# Define the folder containing the header files for this library
set(QT_VISUALIZER_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src PARENT_SCOPE)

# Define the include directory search file for this project
include_directories(${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/src
	${CMAKE_SOURCE_DIR}/thirdparty/Sophus
	${EIGEN3_INCLUDE_DIR}
	${DSM_LIB_INCLUDE_DIR}
	)

# Add the library
add_library(${LIBRARY_NAME} SHARED  ${SOURCE_FILES} ${HEADER_FILES} ${HEADER_MOC_FILES} ${FORM_HEADERS} ${HEADERS_MOC} ${RESOURCES_RCC})

# Use the Widgets module from Qt 5.
qt5_use_modules(${LIBRARY_NAME} Core  Gui Widgets OpenGL)

# Define the link libraries used in the project
target_link_libraries(${LIBRARY_NAME} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} dsm)

# avoid compiler warnings from OpenGL on macos
if(APPLE)  
    target_compile_options(${LIBRARY_NAME} PRIVATE -Wno-deprecated-declarations -Wno-inconsistent-missing-override)
endif()

set_target_properties(${LIBRARY_NAME} PROPERTIES FOLDER "DSM")
