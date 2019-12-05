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

#include "GLDrawings.h"

#ifdef WIN32
	#include <windows.h>
#endif

#ifdef __APPLE__
	#include <OpenGL/gl.h>
	#include <OpenGL/glu.h>
#else
	#include <GL/gl.h>
	#include <GL/glu.h>
#endif

#include "opencv2/imgproc.hpp"

namespace dsm
{
	void drawAxis(float scale, float lineWidth, float alpha)
	{
		glDisable(GL_LIGHTING);

		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		glEnable(GL_LINE_SMOOTH);

		glLineWidth(lineWidth);

		glBegin(GL_LINES);

		glColor4f(1.f, 0.f, 0.f, alpha);
		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(scale, 0.f, 0.f);

		glColor4f(0.f, 1.f, 0.f, alpha);
		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(0.f, scale, 0.f);

		glColor4f(0.f, 0.f, 1.f, alpha);
		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(0.f, 0.f, scale);

		glEnd();

		glColor3f(0.f, 0.f, 0.f);

		glDisable(GL_LINE_SMOOTH);

		glEnable(GL_LIGHTING);
	}

	void drawWireCube(const std::array<float, 6> &aabb, float lineWidth, const Eigen::Vector3f &color, float alpha)
	{
		glDisable(GL_LIGHTING);

		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		glEnable(GL_LINE_SMOOTH);

		// set color
		glColor4f(color[0], color[1], color[2], alpha);

		// draw
		glLineWidth(lineWidth);
		glBegin(GL_LINES);

		// low face
		glVertex3f(aabb[0], aabb[2], aabb[4]);
		glVertex3f(aabb[1], aabb[2], aabb[4]);

		glVertex3f(aabb[1], aabb[2], aabb[4]);
		glVertex3f(aabb[1], aabb[3], aabb[4]);

		glVertex3f(aabb[1], aabb[3], aabb[4]);
		glVertex3f(aabb[0], aabb[3], aabb[4]);

		glVertex3f(aabb[0], aabb[3], aabb[4]);
		glVertex3f(aabb[0], aabb[2], aabb[4]);

		// top face
		glVertex3f(aabb[0], aabb[2], aabb[5]);
		glVertex3f(aabb[1], aabb[2], aabb[5]);

		glVertex3f(aabb[1], aabb[2], aabb[5]);
		glVertex3f(aabb[1], aabb[3], aabb[5]);

		glVertex3f(aabb[1], aabb[3], aabb[5]);
		glVertex3f(aabb[0], aabb[3], aabb[5]);

		glVertex3f(aabb[0], aabb[3], aabb[5]);
		glVertex3f(aabb[0], aabb[2], aabb[5]);

		// side faces
		glVertex3f(aabb[0], aabb[2], aabb[4]);
		glVertex3f(aabb[0], aabb[2], aabb[5]);

		glVertex3f(aabb[1], aabb[2], aabb[4]);
		glVertex3f(aabb[1], aabb[2], aabb[5]);

		glVertex3f(aabb[1], aabb[3], aabb[4]);
		glVertex3f(aabb[1], aabb[3], aabb[5]);

		glVertex3f(aabb[0], aabb[3], aabb[4]);
		glVertex3f(aabb[0], aabb[3], aabb[5]);

		glEnd();

		glDisable(GL_LINE_SMOOTH);

		glEnable(GL_LIGHTING);
	}

	void drawSolidCube(const std::array<float, 6> &aabb, const Eigen::Vector3f &color, float alpha, bool drawWireframe)
	{
		// set color
		glColor4f(color[0], color[1], color[2], alpha);

		// draw solid faces
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_QUADS);

		// top face
		glNormal3f(0.0f, 0.0f, 1.0f);
		glVertex3f(aabb[0], aabb[2], aabb[5]);
		glVertex3f(aabb[1], aabb[2], aabb[5]);
		glVertex3f(aabb[1], aabb[3], aabb[5]);
		glVertex3f(aabb[0], aabb[3], aabb[5]);

		// bottom face
		glNormal3f(0.0f, 0.0f, -1.0f);
		glVertex3f(aabb[0], aabb[2], aabb[4]);
		glVertex3f(aabb[0], aabb[3], aabb[4]);
		glVertex3f(aabb[1], aabb[3], aabb[4]);
		glVertex3f(aabb[1], aabb[2], aabb[4]);

		// right face
		glNormal3f(0.0f, 1.0f, 0.0f);
		glVertex3f(aabb[0], aabb[3], aabb[4]);
		glVertex3f(aabb[0], aabb[3], aabb[5]);
		glVertex3f(aabb[1], aabb[3], aabb[5]);
		glVertex3f(aabb[1], aabb[3], aabb[4]);

		// left face
		glNormal3f(0.0f, -1.0f, 0.0f);
		glVertex3f(aabb[0], aabb[2], aabb[4]);
		glVertex3f(aabb[1], aabb[2], aabb[4]);
		glVertex3f(aabb[1], aabb[2], aabb[5]);
		glVertex3f(aabb[0], aabb[2], aabb[5]);

		// front face
		glNormal3f(1.0f, 0.0f, 0.0f);
		glVertex3f(aabb[1], aabb[2], aabb[4]);
		glVertex3f(aabb[1], aabb[3], aabb[4]);
		glVertex3f(aabb[1], aabb[3], aabb[5]);
		glVertex3f(aabb[1], aabb[2], aabb[5]);

		// back face
		glNormal3f(-1.0f, 0.0f, 0.0f);
		glVertex3f(aabb[0], aabb[2], aabb[4]);
		glVertex3f(aabb[0], aabb[2], aabb[5]);
		glVertex3f(aabb[0], aabb[3], aabb[5]);
		glVertex3f(aabb[0], aabb[3], aabb[4]);

		glEnd();

		// draw cube wireframe
		if (drawWireframe)
		{
			dsm::drawWireCube(aabb, 2.f, Eigen::Vector3f(0.f, 0.f, 0.f), 1.f);
		}
	}

	void drawCamera(float scale, float lineWidth, const Eigen::Vector3f &color, float alpha)
	{
		// camera size
		const float w = scale;
		const float h = w*0.75f;
		const float z = w*0.6f;

		glDisable(GL_LIGHTING);

		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		glEnable(GL_LINE_SMOOTH);

		// set color
		glColor4f(color[0], color[1], color[2], alpha);

		// line width
		glLineWidth(lineWidth);

		// draw camera
		glBegin(GL_LINES);
			glVertex3f(0, 0, 0);
			glVertex3f(w, h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(w, -h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(-w, -h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(-w, h, z);
			glVertex3f(w, h, z);
			glVertex3f(w, -h, z);
			glVertex3f(-w, h, z);
			glVertex3f(-w, -h, z);
			glVertex3f(-w, h, z);
			glVertex3f(w, h, z);
			glVertex3f(-w, -h, z);
			glVertex3f(w, -h, z);
		glEnd();

		glDisable(GL_LINE_SMOOTH);

		glEnable(GL_LIGHTING);
	}

	void drawTextureImg(const cv::Mat &image, unsigned int &textureId)
	{
		glEnable(GL_TEXTURE_2D);

		if (textureId == -1)
		{
			glEnable(GL_TEXTURE_2D);
			glGenTextures(1, &textureId);
		}

		glBindTexture(GL_TEXTURE_2D, textureId);

		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // GL_LINEAR (interpolacion linear)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // fastest
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL); //Se selecciona el modo de texturas "calco"

		cv::Mat imageRGB;
		if (image.channels() == 3)
		{
			cv::cvtColor(image, imageRGB, cv::COLOR_BGR2RGB);
		}
		else if (image.channels() == 1)
		{
			cv::cvtColor(image, imageRGB, cv::COLOR_GRAY2RGB);
		}

		// check that the image dimensions are multiple of two. required for graphic cards
		if (imageRGB.cols % 2 != 0 || imageRGB.rows % 2 != 0)
		{
			int newWidth = imageRGB.cols - (imageRGB.cols % 2);
			int newHeight = imageRGB.rows - (imageRGB.rows % 2);

			cv::resize(imageRGB, imageRGB, cv::Size(newWidth, newHeight));
		}

		GLenum format = imageRGB.channels() == 3 ? GL_RGB : GL_LUMINANCE;

		// it has to be at least 64x64
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageRGB.cols, imageRGB.rows, 0, format, GL_UNSIGNED_BYTE, imageRGB.data);
		imageRGB.release();

		const float zfar = -999.99f;
		const float vertices[] = {
			0, 0, zfar,
			static_cast<float>(image.cols), 0, zfar,
			static_cast<float>(image.cols), static_cast<float>(image.rows), zfar,
			0, static_cast<float>(image.rows), zfar };

		const float coords[] = {
			0, 1,
			1, 1,
			1, 0,
			0, 0
		};

		glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0, image.cols, 0, image.rows, 1, 1000);

		glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			glTexCoordPointer(2, GL_FLOAT, 0, &coords[0]);
			glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);
			glDisableClientState(GL_VERTEX_ARRAY);
			glPopMatrix();

		glMatrixMode(GL_PROJECTION);
			glPopMatrix();

		glMatrixMode(GL_MODELVIEW);

		glDisable(GL_TEXTURE_2D);
	}
}
