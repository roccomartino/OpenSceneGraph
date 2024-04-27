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

#include <osgUtil/Tessellator>

#include <algorithm>


osgBrep::Brep::Brep()
{
	createVertexGeometry();
	createEdgeGeometry();
	createFaceGeometry();
}


osgBrep::Brep::Brep(const Brep& other, const osg::CopyOp& copyop) :
	osg::Geode(other, copyop),
	_faces(other._faces)
{
	createVertexGeometry();
	createEdgeGeometry();
	createFaceGeometry();

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


			for (auto vertex : _vertices)
			{
				add_if_not_present(vertex);
			}


			for (auto edge : _edges)
			{
				add_if_not_present(edge->getStart());
				add_if_not_present(edge->getEnd());
			}


			for (auto face : other._faces)
			{
				for (auto edge : face->getEdgeLoop()->getOrientedEdges())
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


			for (auto face : other._faces)
			{
				for (auto edge : face->getEdgeLoop()->getOrientedEdges())
				{
					add_if_not_present(edge->getEdge());
				}
			}


			for (auto edge : _edges)
			{
				add_if_not_present(edge);
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


			for (auto face : other._faces)
			{
				for (auto orientedEdge : face->getEdgeLoop()->getOrientedEdges())
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


			for (auto face : other._faces)
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


			for (auto face : other._faces)
			{
				add_if_not_present(face);
			}
		}




		auto find_vertex_clone = [&vertexSource, &vertexDestination](Vertex* vertex) -> Vertex*
		{
			auto position = std::find(std::begin(vertexSource), std::end(vertexSource), vertex);
			auto idx = position - std::begin(vertexSource);

			return vertexDestination[idx];
		};

		auto find_edge_clone = [&edgeSource, &edgeDestination](Edge* edge) -> Edge*
		{
			auto position = std::find(std::begin(edgeSource), std::end(edgeSource), edge);
			auto idx = position - std::begin(edgeSource);

			return edgeDestination[idx];
		};

		auto find_oriented_edge_clone = [&orientedEdgeSource, &orientedEdgeDestination](OrientedEdge* orientedEdge) -> OrientedEdge*
		{
			auto position = std::find(std::begin(orientedEdgeSource), std::end(orientedEdgeSource), orientedEdge);
			auto idx = position - std::begin(orientedEdgeSource);

			return orientedEdgeDestination[idx];
		};

		auto find_edge_loop_clone = [&edgeLoopSource, &edgeLoopDestination](EdgeLoop* edgeLoop) -> EdgeLoop*
		{
			auto position = std::find(std::begin(edgeLoopSource), std::end(edgeLoopSource), edgeLoop);
			auto idx = position - std::begin(edgeLoopSource);

			return edgeLoopDestination[idx];
		};


		//
		// Find shared vertices
		//
		for (auto i = 0u; i < edgeSource.size(); i++)
		{
			auto source = edgeSource[i];
			auto destination = edgeDestination[i];

			auto clonedStart = find_vertex_clone(source->getStart());
			auto clonedEnd = find_vertex_clone(source->getEnd());

			destination->setStart(clonedStart);
			destination->setEnd(clonedEnd);
		}



		//
		// Find shared edges
		//
		for (auto i = 0u; i < orientedEdgeSource.size(); i++)
		{
			auto source = orientedEdgeSource[i];
			auto destination = orientedEdgeDestination[i];

			auto cloned = find_edge_clone(source->getEdge());

			destination->setEdge(cloned);
		}



		//
		// Find shared oriented edges
		//
		{
			for (auto i = 0u; i < edgeLoopSource.size(); i++)
			{
				auto source = edgeLoopSource[i];
				auto destination = edgeLoopDestination[i];

				destination->clear();

				for (auto orientedEdge : source->getOrientedEdges())
				{
					auto cloned = find_oriented_edge_clone(orientedEdge);
					destination->addOrientedEdge(cloned);
				}
			}
		}



		//
		// Find shared edge loops
		//
		{
			for (auto i = 0u; i < faceSource.size(); i++)
			{
				auto source = faceSource[i];
				auto destination = faceDestination[i];

				auto cloned = find_edge_loop_clone(source->getEdgeLoop());
				destination->setEdgeLoop(cloned);
			}
		}



		_vertices.clear();
		_edges.clear();
		_faces.clear();

		for (auto vertex : other._vertices)
			addVertex(find_vertex_clone(vertex));

		for (auto edge : other._edges)
			addEdge(find_edge_clone(edge));

		for (auto face : faceDestination)
			addFace(face);
	}
}


osgBrep::Brep::~Brep()
{
}


void
osgBrep::Brep::compile()
{
	compileFaces();
	compileEdges();
	compileVertices();
}


void
osgBrep::Brep::compileVertices()
{
	auto geometry = _vertexGeometry.get();


	auto vertexArray = new osg::Vec3Array();
	auto colorArray = new osg::Vec4Array();

	geometry->setVertexArray(vertexArray);
	geometry->setColorArray(colorArray, osg::Array::BIND_PER_VERTEX);


	for (auto vertex : _vertices)
	{
		auto position = vertex->getPosition();
		
		vertexArray->push_back(position);
		colorArray->push_back(vertex->getColor());
	}


	geometry->removePrimitiveSet(0, geometry->getNumPrimitiveSets());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertexArray->size()));
}


void
osgBrep::Brep::compileEdges()
{
	auto geometry = _edgeGeometry.get();



	const osg::Vec4 selsectedColor(1, 1, 0, 1);
	const osg::Vec4 regularColor(0, 0, 0, 1);


	auto vertexArray = new osg::Vec3Array();
	auto colorArray = new osg::Vec4Array();

	geometry->setVertexArray(vertexArray);
	geometry->setColorArray(colorArray, osg::Array::BIND_PER_VERTEX);


	for (auto edge : _edges)
	{
		auto start = edge->getStart();
		auto end = edge->getEnd();
;

		vertexArray->push_back(start->getPosition());
		vertexArray->push_back(end->getPosition());

		colorArray->push_back(start->getColor());
		colorArray->push_back(end->getColor());

	}

	geometry->removePrimitiveSet(0, geometry->getNumPrimitiveSets());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertexArray->size()));
}


void
osgBrep::Brep::compileFaces()
{
	auto geometry = _faceGeometry.get();


	auto vertexArray = new osg::Vec3Array();
	auto normalArray = new osg::Vec3Array();
	auto colorArray = new osg::Vec4Array();

	geometry->setVertexArray(vertexArray);
	geometry->setNormalArray(normalArray, osg::Array::BIND_PER_VERTEX);

	geometry->setColorArray(colorArray, osg::Array::BIND_OVERALL);
	colorArray->push_back(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));


	auto triangleVertexArray = new osg::Vec3Array();
	auto triangleNormalArray = new osg::Vec3Array();

	geometry->removePrimitiveSet(0, geometry->getNumPrimitiveSets());

	for (const auto& face : _faces)
	{
		auto loop = face->getEdgeLoop();


		if (!loop->isLoop())
			continue;

		auto startingIndex = vertexArray->size();

		auto edges = face->getEdgeLoop()->getOrientedEdges();
		auto normal = face->getNormal();

		if (edges.size() == 3)
		{
			triangleVertexArray->push_back(edges[0]->getOrientedStart()->getPosition());
			triangleVertexArray->push_back(edges[1]->getOrientedStart()->getPosition());
			triangleVertexArray->push_back(edges[2]->getOrientedStart()->getPosition());
			triangleNormalArray->push_back(normal);
			triangleNormalArray->push_back(normal);
			triangleNormalArray->push_back(normal);
		}

		else if (edges.size() == 4)
		{
			triangleVertexArray->push_back(edges[0]->getOrientedStart()->getPosition());
			triangleVertexArray->push_back(edges[1]->getOrientedStart()->getPosition());
			triangleVertexArray->push_back(edges[2]->getOrientedStart()->getPosition());
			triangleNormalArray->push_back(normal);
			triangleNormalArray->push_back(normal);
			triangleNormalArray->push_back(normal);
			triangleVertexArray->push_back(edges[0]->getOrientedStart()->getPosition());
			triangleVertexArray->push_back(edges[2]->getOrientedStart()->getPosition());
			triangleVertexArray->push_back(edges[3]->getOrientedStart()->getPosition());
			triangleNormalArray->push_back(normal);
			triangleNormalArray->push_back(normal);
			triangleNormalArray->push_back(normal);
		}

		else
		{
			for (const auto& edge : edges)
			{
				vertexArray->push_back(edge->getOrientedStart()->getPosition());
				normalArray->push_back(normal);
			}

			geometry->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, startingIndex, vertexArray->size() - startingIndex));
		}
	}

	if (!triangleVertexArray->empty())
	{
		auto firstTriangleIndex = vertexArray->size();

		vertexArray->insert(vertexArray->end(), triangleVertexArray->begin(), triangleVertexArray->end());
		normalArray->insert(normalArray->end(), triangleNormalArray->begin(), triangleNormalArray->end());

		geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, firstTriangleIndex, triangleVertexArray->size()));
	}



	osgUtil::Tessellator tsv;

	tsv.setTessellationType(osgUtil::Tessellator::TESS_TYPE_POLYGONS);
	tsv.setBoundaryOnly(false);
	tsv.setWindingType(osgUtil::Tessellator::TESS_WINDING_NONZERO);

	tsv.retessellatePolygons(*geometry);
}


void osgBrep::Brep::createVertexGeometry()
{
	_vertexGeometry = new osg::Geometry();

	_vertexGeometry->setUseDisplayList(_useDisplayListForVertices);
	_vertexGeometry->setUseVertexBufferObjects(_useVertexBufferObjectForVertices);
	_vertexGeometry->setUseVertexArrayObject(_useVertexArrayObjectForVertices);

	if (_showVertices)
		addDrawable(_vertexGeometry);


	auto stateSet = _vertexGeometry->getOrCreateStateSet();

	stateSet->setAttributeAndModes(new osg::Point(_vertexSize), osg::StateAttribute::ON);

	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	stateSet->getOrCreateUniform("uMaterial", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(1, 0, 0, 0));
}


void osgBrep::Brep::createEdgeGeometry()
{
	_edgeGeometry = new osg::Geometry();

	_edgeGeometry->setUseDisplayList(_useDisplayListForEdges);
	_edgeGeometry->setUseVertexBufferObjects(_useVertexBufferObjectForEdges);
	_edgeGeometry->setUseVertexArrayObject(_useVertexArrayObjectForEdges);

	if (_showEdges)
		addDrawable(_edgeGeometry);


	auto stateSet = _edgeGeometry->getOrCreateStateSet();

	stateSet->setAttributeAndModes(new osg::LineWidth(_edgeWidth), osg::StateAttribute::ON);

	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	stateSet->getOrCreateUniform("uMaterial", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(1, 0, 0, 0));
}


void osgBrep::Brep::createFaceGeometry()
{
	_faceGeometry = new osg::Geometry();

	_faceGeometry->setUseDisplayList(_useDisplayListForFaces);
	_faceGeometry->setUseVertexBufferObjects(_useVertexBufferObjectForFaces);
	_faceGeometry->setUseVertexArrayObject(_useVertexArrayObjectForFaces);

	addDrawable(_faceGeometry);

	auto stateSet = _faceGeometry->getOrCreateStateSet();

	stateSet->setAttributeAndModes(new osg::PolygonOffset(_faceOffset, 0), osg::StateAttribute::ON);

	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateSet->setMode(GL_POLYGON_OFFSET_FILL, osg::StateAttribute::ON);

	stateSet->getOrCreateUniform("uMaterial", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(0, 1, 0, 0));
}



void osgBrep::Brep::addFace(Face* face)
{
	_faces.push_back(face);
}

void osgBrep::Brep::removeFace(Face* face)
{
	auto itr = std::find(std::begin(_faces), std::end(_faces), face);

	if (itr != std::end(_faces))
		_faces.erase(itr);
}

void osgBrep::Brep::clearFaces()
{
	_faces.clear();
}


void osgBrep::Brep::addEdge(Edge* edge)
{
	_edges.push_back(edge);
}

void osgBrep::Brep::removeEdge(Edge* edge)
{
	auto itr = std::find(std::begin(_edges), std::end(_edges), edge);

	if (itr != std::end(_edges))
		_edges.erase(itr);
}

void osgBrep::Brep::clearEdges()
{
	_edges.clear();
}


void osgBrep::Brep::addVertex(Vertex* vertex)
{
	_vertices.push_back(vertex);
}

void osgBrep::Brep::removeVertex(Vertex* vertex)
{
	auto itr = std::find(std::begin(_vertices), std::end(_vertices), vertex);

	if (itr != std::end(_vertices))
		_vertices.erase(itr);
}

void osgBrep::Brep::clearVertices()
{
	_vertices.clear();
}




void osgBrep::Brep::showVerticesChanged()
{
	if (_vertexGeometry.valid())
	{
		if (_showVertices)
			addDrawable(_vertexGeometry);
		else
			removeDrawable(_vertexGeometry);
	}
}


void osgBrep::Brep::showEdgesChanged()
{
	if (_edgeGeometry.valid())
	{
		if (_showEdges)
			addDrawable(_edgeGeometry);
		else
			removeDrawable(_edgeGeometry);
	}
}