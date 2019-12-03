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

#include <vector>
#include <unordered_map>
#include <memory>

#include <Eigen/Core>

namespace dsm
{
	class Frame;

	struct CovisibilityNode
	{
		CovisibilityNode(const std::shared_ptr<Frame>& frame);

		int idxInGraph;

		Frame* node;

		// <target node, weight>
		std::unordered_map<CovisibilityNode*, int> edges;
	};

	class CovisibilityGraph
	{
	public:
		CovisibilityGraph();
		~CovisibilityGraph();

		void clear();

		void addNode(const std::shared_ptr<Frame>& frame);
		void removeNode(const std::shared_ptr<Frame>& frame);

		void connect(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2, int weight);
		void disconnect(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2);

		void getAllConnections(const std::shared_ptr<Frame>& frame, std::vector<Frame*>& connections);
		void getConnectionsByWeight(const std::shared_ptr<Frame>& frame, int weight, 
									std::vector<Frame*>& connections);

		// adjacency matrix ordered by keyframe id
		Eigen::MatrixXi adjacencyMatrix();

	private:

		std::vector<std::unique_ptr<CovisibilityNode>> nodes;
	};
}