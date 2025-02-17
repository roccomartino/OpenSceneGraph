/* -*-c++-*- osgEditable - Copyright (C) 2023 Rocco Martino
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

#include <osgEditable/EdgeLoop>

#include <algorithm>


osgEditable::EdgeLoop::EdgeLoop()
{
}


osgEditable::EdgeLoop::EdgeLoop(const EdgeLoop& other, const osg::CopyOp& copyop) :
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


osgEditable::EdgeLoop::~EdgeLoop()
{
}



void osgEditable::EdgeLoop::addOrientedEdge(OrientedEdge* orientedEdge)
{
	_orientedEdges.push_back(orientedEdge);
}

void osgEditable::EdgeLoop::removeOrientedEdge(OrientedEdge* orientedEdge)
{
	auto itr = std::find(std::begin(_orientedEdges), std::end(_orientedEdges), orientedEdge);

	if (itr != std::end(_orientedEdges))
		_orientedEdges.erase(itr);
}

const std::vector<osg::ref_ptr<osgEditable::OrientedEdge>>& osgEditable::EdgeLoop::getOrientedEdges() const
{
	return _orientedEdges;
}

void osgEditable::EdgeLoop::clear()
{
	_orientedEdges.clear();
}

bool osgEditable::EdgeLoop::isLoop() const
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

bool osgEditable::EdgeLoop::isCcw(const osg::Vec3& normal) const
{
	auto numEdges = _orientedEdges.size();

	double A = 0;

	for (int i = 0u; i < numEdges; i++)
	{
		auto currentOriented = _orientedEdges[i].get();
		auto nextOriented = _orientedEdges[(i + 1) % numEdges].get();

		auto d1 = currentOriented->getOrientedEnd()->getPosition() - currentOriented->getOrientedStart()->getPosition();
		auto d2 = nextOriented->getOrientedEnd()->getPosition() - nextOriented->getOrientedStart()->getPosition();

		d1.normalize();
		d2.normalize();

		A += (d1 ^ d2) * normal;
	}

	return A > 0;
}
