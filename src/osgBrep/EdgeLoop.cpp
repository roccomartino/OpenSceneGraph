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


			for each (auto edge in other._orientedEdges)
			{
				add_if_not_present(edge->getEdge()->getStart());
				add_if_not_present(edge->getEdge()->getEnd());
			}
		}



		_orientedEdges.clear();


		for each (auto edge in other._orientedEdges)
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
