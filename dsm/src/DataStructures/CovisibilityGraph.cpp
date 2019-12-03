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

#include "CovisibilityGraph.h"
#include "Frame.h"

namespace dsm
{
	CovisibilityNode::CovisibilityNode(const std::shared_ptr<Frame>& frame)
	{
		this->node = frame.get();
	}

	CovisibilityGraph::CovisibilityGraph()
	{}

	CovisibilityGraph::~CovisibilityGraph()
	{}

	void CovisibilityGraph::clear()
	{
		this->nodes.clear();
	}

	void CovisibilityGraph::addNode(const std::shared_ptr<Frame>& frame)
	{
		if (frame->graphNode == nullptr)
		{
			// create node
			std::unique_ptr<CovisibilityNode> node = std::make_unique<CovisibilityNode>(frame);
			frame->graphNode = node.get();

			// insert to list
			node->idxInGraph = (int)this->nodes.size();
			this->nodes.push_back(std::move(node));
		}
	}

	void CovisibilityGraph::removeNode(const std::shared_ptr<Frame>& frame)
	{
		if (frame->graphNode)
		{
			CovisibilityNode* node = frame->graphNode;

			// remove all the edges in the connected nodes
			for (auto it = node->edges.begin(); it != node->edges.end(); ++it)
			{
				it->first->edges.erase(node);
			}

			// remove node
			int idx = frame->graphNode->idxInGraph;
			this->nodes[idx] = std::move(this->nodes.back());
			this->nodes[idx]->idxInGraph = idx;
			this->nodes.pop_back();

			frame->graphNode = nullptr;
		}
	}

	void CovisibilityGraph::connect(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2, int weight)
	{
		assert(frame1->graphNode);
		assert(frame2->graphNode);

		// connect them in both directions
		// do not find, if already exists replace
		frame1->graphNode->edges[frame2->graphNode] = weight;
		frame2->graphNode->edges[frame1->graphNode] = weight;
	}

	void CovisibilityGraph::disconnect(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2)
	{
		assert(frame1->graphNode);
		assert(frame2->graphNode);

		// disconnect them in both directions
		auto it = frame1->graphNode->edges.find(frame2->graphNode);
		if (it != frame1->graphNode->edges.end())
		{
			frame1->graphNode->edges.erase(frame2->graphNode);
			frame2->graphNode->edges.erase(frame1->graphNode);
		}
	}

	void CovisibilityGraph::getAllConnections(const std::shared_ptr<Frame>& frame, std::vector<Frame*>& connections)
	{
		assert(frame->graphNode);

		connections.clear();
		connections.reserve(frame->graphNode->edges.size());

		for (auto it = frame->graphNode->edges.begin(); it != frame->graphNode->edges.end(); ++it)
		{
			connections.push_back(it->first->node);
		}
	}

	void CovisibilityGraph::getConnectionsByWeight(const std::shared_ptr<Frame>& frame, int weight, 
												   std::vector<Frame*>& connections)
	{
		assert(frame->graphNode);

		connections.clear();
		connections.reserve(frame->graphNode->edges.size());

		for (auto it = frame->graphNode->edges.begin(); it != frame->graphNode->edges.end(); ++it)
		{
			if (it->second < weight) continue;

			connections.push_back(it->first->node);
		}
	}

	Eigen::MatrixXi CovisibilityGraph::adjacencyMatrix()
	{
		int numNodes = (int)this->nodes.size();

		Eigen::MatrixXi adj;
		adj.resize(numNodes, numNodes);
		adj.setZero();

		for (const auto& node : this->nodes)
		{
			int currentNodeID = node->node->keyframeID();
			for (const auto& edge : node->edges)
			{
				int otherNodeID = edge.first->node->keyframeID();

				adj(currentNodeID, otherNodeID) = edge.second;
				adj(otherNodeID, currentNodeID) = edge.second;
			}
		}

		return adj;
	}
}