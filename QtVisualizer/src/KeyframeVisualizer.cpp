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

#include "KeyframeVisualizer.h"
#include "GLDrawings.h"
#include "DataStructures/Frame.h"
#include "DataStructures/ActivePoint.h"
#include "DataStructures/Pattern.h"
#include "Utils/GlobalCalibration.h"

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

namespace dsm
{
	KeyframeVisualizer::KeyframeVisualizer()
	{
		this->type = KeyframeType::NONE;
		this->camToWorld.setIdentity();

		this->varThr = std::numeric_limits<float>::max();
		this->numBufferPoints = 0;

		this->needUpdate = false;
	}

	KeyframeVisualizer::~KeyframeVisualizer()
	{}

	const Eigen::Matrix4f& KeyframeVisualizer::getPose() const
	{
		return this->camToWorld;
	}

	const std::vector<Eigen::Vector3f>& KeyframeVisualizer::getPointCloud() const
	{
		return this->vertexBuffer;
	}

	const std::vector<Eigen::Matrix<unsigned char, 3, 1>>& KeyframeVisualizer::getColors() const
	{
		return this->colorBuffer;
	}

	KeyframeType KeyframeVisualizer::getType() const
	{
		return this->type;
	}

	int KeyframeVisualizer::getNumPoints() const
	{
		return (int)this->pointCloud.size();
	}

	void KeyframeVisualizer::compute(const std::shared_ptr<dsm::Frame>& keyframe, KeyframeType keyframeType)
	{
		// set camera pose
		this->camToWorld = keyframe->camToWorld().matrix();
		this->type = keyframeType;

		// set point cloud
		const auto& points = keyframe->activePoints();
		int numPoints = (int)points.size();

		this->pointCloud.resize(numPoints);
		for (int i = 0; i < numPoints; ++i)
		{
			this->pointCloud[i].u = (float)points[i]->u(0);
			this->pointCloud[i].v = (float)points[i]->v(0);
			this->pointCloud[i].iDepth = points[i]->iDepth();
			this->pointCloud[i].iDepthHessian = points[i]->iDepthHessian();
			this->pointCloud[i].parallax = points[i]->parallax();

			this->pointCloud[i].color.resize(dsm::Pattern::size());

			const Eigen::VecXf& colors = points[i]->colors(0);
			for (int32_t idx = 0; idx < dsm::Pattern::size(); ++idx)
			{
				this->pointCloud[i].color[idx] = (unsigned char)colors[idx];
			}
		}

		this->needUpdate = true;
	}

	void KeyframeVisualizer::computeType(KeyframeType keyframeType)
	{
		this->type = keyframeType;
	}

	bool KeyframeVisualizer::update(float varThreshold, float parallaxTreshold)
	{
		this->needUpdate =	this->needUpdate || 
							(this->varThr != varThreshold) ||
							(this->parThr != parallaxTreshold);

		// return if not needed
		if (!this->needUpdate)
		{
			return false;
		}

		// calibration
		const Eigen::Matrix3f& K = GlobalCalibration::getInstance().matrix3f(0);

		// change variance threshold
		this->varThr = varThreshold;
		this->parThr = parallaxTreshold;

		// update buffers
		const int numPoints = (int)this->pointCloud.size();
		const int numPixels = dsm::Pattern::size();

		this->vertexBuffer.resize(numPoints*numPixels);
		this->colorBuffer.resize(numPoints*numPixels);
		this->numBufferPoints = 0;

		for (int i = 0; i < numPoints; ++i)
		{
			// discard negative inverse depths
			if (this->pointCloud[i].iDepth < 0.f) continue;

			float depth = 1.f / this->pointCloud[i].iDepth;
			float depth4 = depth*depth;
			depth4 *= depth4;

			float var = 1.f / (this->pointCloud[i].iDepthHessian + 1e-03f);

			// check variance threshold
			if (var * depth4 > this->varThr) continue;
			if (var > this->varThr) continue;

			// check parallax threshold
			if (this->pointCloud[i].parallax > this->parThr) continue;

			for (int32_t idx = 0; idx < numPixels; ++idx)
			{
				float uj = this->pointCloud[i].u + dsm::Pattern::at(idx, 0);
				float vj = this->pointCloud[i].v + dsm::Pattern::at(idx, 1);

				this->vertexBuffer[this->numBufferPoints] = depth*Eigen::Vector3f((uj - K(0, 2)) / K(0, 0),
																				  (vj - K(1, 2)) / K(1, 1), 
																				   1.f);

				this->colorBuffer[this->numBufferPoints][0] = this->pointCloud[i].color[idx];
				this->colorBuffer[this->numBufferPoints][1] = this->pointCloud[i].color[idx];
				this->colorBuffer[this->numBufferPoints][2] = this->pointCloud[i].color[idx];

				this->numBufferPoints++;
				assert(this->numBufferPoints <= numPoints*numPixels);
			}
		}

		// shrink to fit
		this->vertexBuffer.resize(this->numBufferPoints);
		this->colorBuffer.resize(this->numBufferPoints);

		this->needUpdate = false;

		return true;
	}

	void KeyframeVisualizer::drawPointCloud(float size, bool useType)
	{
		// make sure update() was called first
		if (this->numBufferPoints <= 0) return;

		glDisable(GL_LIGHTING);

		glPointSize(size);

		glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			glMultMatrixf(this->camToWorld.data());

			if (useType && (this->type == dsm::COVISIBILITY || this->type == dsm::TEMPORAL))
			{
				glPointSize(1.5f*size);

				if (this->type == dsm::COVISIBILITY)
				{
					// orange
					glColor3f(1.f, 0.5f, 0.f);
				}
				else
				{
					// blue
					glColor3f(0.f, 0.f, 1.f);
				}
			}
			else if (useType)
			{
				// transparent
				glColor4f(0.f, 0.f, 0.f, 0.2f);
			}
			else
			{
				// normal
				glEnableClientState(GL_COLOR_ARRAY);
				glColorPointer(3, GL_UNSIGNED_BYTE, 0, &this->colorBuffer[0]);
			}					

			// points
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, 0, &this->vertexBuffer[0]);

			// draw
			glDrawArrays(GL_POINTS, 0, this->numBufferPoints);
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_COLOR_ARRAY);

		glPopMatrix();

		glEnable(GL_LIGHTING);
	}

	void KeyframeVisualizer::drawCamera(float size, bool useType)
	{
		KeyframeType t = KeyframeType::NONE;
		if (useType)
		{
			t = this->type;
		}

		// camera params
		float sizeFactor, lineWidth, alpha;
		Eigen::Vector3f color;

		switch (t)
		{
		case NONE:
			sizeFactor = 0.07f*size;
			lineWidth = 1.f;
			alpha = 0.3f;
			color = Eigen::Vector3f(0.f, 0.f, 0.f);
			break;
		case TEMPORAL:
			sizeFactor = 0.1f*size;
			lineWidth = 1.5f;
			alpha = 1.f;
			color = Eigen::Vector3f(0.f, 0.f, 1.f);
			break;
		case COVISIBILITY:
			sizeFactor = 0.1f*size;
			lineWidth = 1.5f;
			alpha = 1.f;
			color = Eigen::Vector3f(0.8f, 0.4f, 0.f);
			break;
		case FIXED:
			sizeFactor = 0.1f*size;
			lineWidth = 1.5f;
			alpha = 1.f;
			color = Eigen::Vector3f(0.f, 0.f, 0.f);
			break;
		default:
			sizeFactor = 0.07f*size;
			lineWidth = 1.f;
			alpha = 0.3f;
			color = Eigen::Vector3f(0.f, 0.f, 0.f);
			break;
		}

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

			// pose
			glMultMatrixf(this->camToWorld.data());

			// render camera
			dsm::drawCamera(sizeFactor, lineWidth, color, alpha);

		glPopMatrix();
	}
}
