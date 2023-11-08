/* -*-c++-*- osgBrep - Copyright (C) 2023 Rocco Martino
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#include <osgBrep/EdgeLoop>

#include <algorithm>


osgBrep::EdgeLoop::EdgeLoop()
{
}


osgBrep::EdgeLoop::EdgeLoop(const EdgeLoop& other, const osg::CopyOp& copyop) :
	osg::Object(other, copyop),
	_orientedEdges(other._orientedEdges)
{
	if (copyop.getCopyFlags() & osg::CopyOp::DEEP_COPY_OBJECTS)
	{
		std::vector<Vertex*> vertexSource;
		std::vector<Vertex*> vertexDestination;
		{
			auto add_if_not_present = [&vertexSource, &vertexDestination, &copyop](Vertex* vertex)
			{
				auto position = std::find(std::begin(vertexSource), std::end(vertexSource), vertex);

				if (position == std::end(vertexSource))
				{
					auto clonedVertex = (Vertex*)vertex->clone(copyop);

					vertexSource.push_back(vertex);
					vertexDestination.push_back(clonedVertex);
				}
			};


			for (auto edge : other._orientedEdges)
			{
				add_if_not_present(edge->getEdge()->getStart());
				add_if_not_present(edge->getEdge()->getEnd());
			}
		}



		_orientedEdges.clear();


		for (auto edge : other._orientedEdges)
		{
			auto clonedEdge = (OrientedEdge*)edge->clone(copyop);

			auto startPosition = std::find(std::begin(vertexSource), std::end(vertexSource), edge->getEdge()->getStart());
			auto endPosition = std::find(std::begin(vertexSource), std::end(vertexSource), edge->getEdge()->getEnd());

			auto startIdx = startPosition - std::begin(vertexSource);
			auto endIdx = endPosition - std::begin(vertexSource);

			clonedEdge->getEdge()->setStart(vertexDestination[startIdx]);
			clonedEdge->getEdge()->setEnd(vertexDestination[endIdx]);

			_orientedEdges.push_back(clonedEdge);
		}
	}
}


osgBrep::EdgeLoop::~EdgeLoop()
{
}



void osgBrep::EdgeLoop::addOrientedEdge(OrientedEdge* orientedEdge)
{
	_orientedEdges.push_back(orientedEdge);
}

void osgBrep::EdgeLoop::removeOrientedEdge(OrientedEdge* orientedEdge)
{
	auto itr = std::find(std::begin(_orientedEdges), std::end(_orientedEdges), orientedEdge);

	if (itr != std::end(_orientedEdges))
		_orientedEdges.erase(itr);
}

const std::vector<osg::ref_ptr<osgBrep::OrientedEdge>>& osgBrep::EdgeLoop::getOrientedEdges() const
{
	return _orientedEdges;
}

void osgBrep::EdgeLoop::clear()
{
	_orientedEdges.clear();
}

bool osgBrep::EdgeLoop::isLoop() const
{
	auto numEdges = _orientedEdges.size();

	for (int i = 0u; i < numEdges; i++)
	{
		auto currentOriented = _orientedEdges[i].get();
		auto nextOriented = _orientedEdges[(i + 1) % numEdges].get();

		auto vertexA = currentOriented->getOrientedEnd();
		auto vertexB = nextOriented->getOrientedStart();

		if (vertexA != vertexB)
			return false;
	}

	return true;
}
