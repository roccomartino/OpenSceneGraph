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

#include <osgBrep/Brep>


osgBrep::Brep::Brep()
{
}


osgBrep::Brep::Brep(const Brep& other, const osg::CopyOp& copyop) :
	osg::Geode(other, copyop),
	_faces(other._faces)
{
	if (copyop.getCopyFlags() & osg::CopyOp::DEEP_COPY_OBJECTS)
	{
		//
		// Collect the vertices
		//
		std::vector<Vertex*> vertexSource;
		std::vector<Vertex*> vertexDestination;
		{
			auto add_if_not_present = [&vertexSource, &vertexDestination, &copyop](Vertex* vertex)
			{
				auto position = std::find(std::begin(vertexSource), std::end(vertexSource), vertex);

				if (position == std::end(vertexSource))
				{
					auto clonedVertex = (Vertex*)vertex->clone(osg::CopyOp::SHALLOW_COPY);

					vertexSource.push_back(vertex);
					vertexDestination.push_back(clonedVertex);
				}
			};


			for each (auto face in other._faces)
			{
				for each (auto edge in face->getEdgeLoop()->getOrientedEdges())
				{
					add_if_not_present(edge->getEdge()->getStart());
					add_if_not_present(edge->getEdge()->getEnd());
				}
			}
		}


		//
		// Collect the edges
		//
		std::vector<Edge*> edgeSource;
		std::vector<Edge*> edgeDestination;
		{
			auto add_if_not_present = [&edgeSource, &edgeDestination, &copyop](Edge* edge)
			{
				auto position = std::find(std::begin(edgeSource), std::end(edgeSource), edge);

				if (position == std::end(edgeSource))
				{
					auto clonedEdge = (Edge*)edge->clone(osg::CopyOp::SHALLOW_COPY);

					edgeSource.push_back(edge);
					edgeDestination.push_back(clonedEdge);
				}
			};


			for each (auto face in other._faces)
			{
				for each (auto edge in face->getEdgeLoop()->getOrientedEdges())
				{
					add_if_not_present(edge->getEdge());
				}
			}
		}


		//
		// Collect the oriented edges
		//
		std::vector<OrientedEdge*> orientedEdgeSource;
		std::vector<OrientedEdge*> orientedEdgeDestination;
		{
			auto add_if_not_present = [&orientedEdgeSource, &orientedEdgeDestination, &copyop](OrientedEdge* orientedEdge)
			{
				auto position = std::find(std::begin(orientedEdgeSource), std::end(orientedEdgeSource), orientedEdge);

				if (position == std::end(orientedEdgeSource))
				{
					auto clonedOrientedEdge = (OrientedEdge*)orientedEdge->clone(osg::CopyOp::SHALLOW_COPY);

					orientedEdgeSource.push_back(orientedEdge);
					orientedEdgeDestination.push_back(clonedOrientedEdge);
				}
			};


			for each (auto face in other._faces)
			{
				for each (auto orientedEdge in face->getEdgeLoop()->getOrientedEdges())
				{
					add_if_not_present(orientedEdge);
				}
			}
		}


		//
		// Collect the loops
		//
		std::vector<EdgeLoop*> edgeLoopSource;
		std::vector<EdgeLoop*> edgeLoopDestination;
		{
			auto add_if_not_present = [&edgeLoopSource, &edgeLoopDestination, &copyop](EdgeLoop* edgeLoop)
			{
				auto position = std::find(std::begin(edgeLoopSource), std::end(edgeLoopSource), edgeLoop);

				if (position == std::end(edgeLoopSource))
				{
					auto clonedEdgeLoop = (EdgeLoop*)edgeLoop->clone(osg::CopyOp::SHALLOW_COPY);

					edgeLoopSource.push_back(edgeLoop);
					edgeLoopDestination.push_back(clonedEdgeLoop);
				}
			};


			for each (auto face in other._faces)
			{
				add_if_not_present(face->getEdgeLoop());
			}
		}


		//
		// Collect the faces
		//
		std::vector<Face*> faceSource;
		std::vector<Face*> faceDestination;
		{
			auto add_if_not_present = [&faceSource, &faceDestination, &copyop](Face* face)
			{
				auto position = std::find(std::begin(faceSource), std::end(faceSource), face);

				if (position == std::end(faceSource))
				{
					auto clonedFace = (Face*)face->clone(osg::CopyOp::SHALLOW_COPY);

					faceSource.push_back(face);
					faceDestination.push_back(clonedFace);
				}
			};


			for each (auto face in other._faces)
			{
				add_if_not_present(face);
			}
		}



		//
		// Find shared vertices
		//
		{
			auto find_clone = [&vertexSource, &vertexDestination](Vertex* vertex) -> Vertex*
			{
				auto position = std::find(std::begin(vertexSource), std::end(vertexSource), vertex);
				auto idx = position - std::begin(vertexSource);

				return vertexDestination[idx];
			};

			for (auto i = 0; i < edgeSource.size(); i++)
			{
				auto source = edgeSource[i];
				auto destination = edgeDestination[i];

				auto clonedStart = find_clone(source->getStart());
				auto clonedEnd = find_clone(source->getEnd());

				destination->setStart(clonedStart);
				destination->setEnd(clonedEnd);
			}
		}



		//
		// Find shared edges
		//
		{
			auto find_clone = [&edgeSource, &edgeDestination](Edge* edge) -> Edge*
			{
				auto position = std::find(std::begin(edgeSource), std::end(edgeSource), edge);
				auto idx = position - std::begin(edgeSource);

				return edgeDestination[idx];
			};

			for (auto i = 0; i < orientedEdgeSource.size(); i++)
			{
				auto source = orientedEdgeSource[i];
				auto destination = orientedEdgeDestination[i];

				auto cloned = find_clone(source->getEdge());

				destination->setEdge(cloned);
			}
		}



		//
		// Find shared oriented edges
		//
		{
			auto find_clone = [&orientedEdgeSource, &orientedEdgeDestination](OrientedEdge* orientedEdge) -> OrientedEdge*
			{
				auto position = std::find(std::begin(orientedEdgeSource), std::end(orientedEdgeSource), orientedEdge);
				auto idx = position - std::begin(orientedEdgeSource);

				return orientedEdgeDestination[idx];
			};

			for (auto i = 0; i < edgeLoopSource.size(); i++)
			{
				auto source = edgeLoopSource[i];
				auto destination = edgeLoopDestination[i];

				destination->Clear();

				for each (auto orientedEdge in source->getOrientedEdges())
				{
					auto cloned = find_clone(orientedEdge);
					destination->addOrientedEdge(cloned);
				}
			}
		}



		//
		// Find shared edge loops
		//
		{
			auto find_clone = [&edgeLoopSource, &edgeLoopDestination](EdgeLoop* edgeLoop) -> EdgeLoop*
			{
				auto position = std::find(std::begin(edgeLoopSource), std::end(edgeLoopSource), edgeLoop);
				auto idx = position - std::begin(edgeLoopSource);

				return edgeLoopDestination[idx];
			};

			for (auto i = 0; i < faceSource.size(); i++)
			{
				auto source = faceSource[i];
				auto destination = faceDestination[i];

				auto cloned = find_clone(source->getEdgeLoop());
				destination->setEdgeLoop(cloned);
			}
		}



		_faces.clear();


		for each (auto face in faceDestination)
			_faces.push_back(face);
	}
}


osgBrep::Brep::~Brep()
{
}
