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

#include <osgEditable/Editable>

#include <algorithm>


osgEditable::Editable::Editable()
{
	_vertexArray = new osg::Vec3Array();
	_colorArray = new osg::Vec4Array();

	createVertexGeometry();
	createEdgeGeometry();
	createFaceGeometry();
}


osgEditable::Editable::Editable(const Editable& other, const osg::CopyOp& copyop) :
	osg::Geode(other, copyop),
	_faces(other._faces)
{
	_vertexArray = new osg::Vec3Array();
	_colorArray = new osg::Vec4Array();

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


osgEditable::Editable::~Editable()
{
}


void
osgEditable::Editable::compile()
{
	_vertexArray->clear();
	_colorArray->clear();

	for (const auto& item : _vertices)
	{
		item->setIndexInternal(_vertexArray->size());

		_vertexArray->push_back(item->getPosition());
		_colorArray->push_back(item->getColor());
	}

	compileFaces();
	compileEdges();
	compileVertices();

	_vertexArray->dirty();
	_colorArray->dirty();
}


void
osgEditable::Editable::compileVertices()
{
	auto geometry = _vertexGeometry.get();

	geometry->removePrimitiveSet(0, geometry->getNumPrimitiveSets());

	if (_vertices.size() == 0)
		return;

	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, _vertexArray->size()));
}


void
osgEditable::Editable::compileEdges()
{
	auto geometry = _edgeGeometry.get();

	geometry->removePrimitiveSet(0, geometry->getNumPrimitiveSets());

	if (_edges.size() == 0)
		return;


	std::vector<unsigned int> indices;

	for (auto edge : _edges)
	{
		auto start = edge->getStart();
		auto end = edge->getEnd();

		indices.push_back(start->getIndexInternal());
		indices.push_back(end->getIndexInternal());

	}

	geometry->addPrimitiveSet(new osg::DrawElementsUInt(GL_LINES, indices.size(), &indices.front()));
}


void
osgEditable::Editable::compileFaces()
{
	auto geometry = _faceGeometry.get();

	geometry->removePrimitiveSet(0, geometry->getNumPrimitiveSets());

	if (_faces.size() == 0)
	{
		geometry->setVertexArray(NULL);
		geometry->setNormalArray(NULL);
		geometry->setColorArray(NULL);

		return;
	}


	auto vertexArray = new osg::Vec3Array();
	auto normalArray = new osg::Vec3Array();
	auto colorArray = new osg::Vec4Array();

	geometry->setVertexArray(vertexArray);
	geometry->setNormalArray(normalArray, osg::Array::BIND_PER_VERTEX);

	geometry->setColorArray(colorArray, osg::Array::BIND_OVERALL);
	colorArray->push_back(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));


	auto triangleVertexArray = new osg::Vec3Array();
	auto triangleNormalArray = new osg::Vec3Array();

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

			std::vector<int> indices;

			for (auto i = startingIndex; i < vertexArray->size(); i++)
				indices.push_back(i);

			
			geometry->addPrimitiveSet(triangulate(vertexArray, indices, normal));
		}
	}

	if (!triangleVertexArray->empty())
	{
		auto firstTriangleIndex = vertexArray->size();

		vertexArray->insert(vertexArray->end(), triangleVertexArray->begin(), triangleVertexArray->end());
		normalArray->insert(normalArray->end(), triangleNormalArray->begin(), triangleNormalArray->end());

		geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, firstTriangleIndex, triangleVertexArray->size()));
	}
}


void osgEditable::Editable::createVertexGeometry()
{
	_vertexGeometry = new osg::Geometry();

	_vertexGeometry->setVertexArray(_vertexArray);
	_vertexGeometry->setColorArray(_colorArray, osg::Array::BIND_PER_VERTEX);
	_vertexGeometry->setUseDisplayList(_useDisplayListForVertices);
	_vertexGeometry->setUseVertexBufferObjects(_useVertexBufferObjectForVertices);
	_vertexGeometry->setUseVertexArrayObject(_useVertexArrayObjectForVertices);

	if (_showVertices)
		addDrawable(_vertexGeometry);
}


void osgEditable::Editable::createEdgeGeometry()
{
	_edgeGeometry = new osg::Geometry();

	_edgeGeometry->setVertexArray(_vertexArray);
	_edgeGeometry->setColorArray(_colorArray, osg::Array::BIND_PER_VERTEX);
	_edgeGeometry->setUseDisplayList(_useDisplayListForEdges);
	_edgeGeometry->setUseVertexBufferObjects(_useVertexBufferObjectForEdges);
	_edgeGeometry->setUseVertexArrayObject(_useVertexArrayObjectForEdges);

	if (_showEdges)
		addDrawable(_edgeGeometry);
}


void osgEditable::Editable::createFaceGeometry()
{
	_faceGeometry = new osg::Geometry();

	_faceGeometry->setUseDisplayList(_useDisplayListForFaces);
	_faceGeometry->setUseVertexBufferObjects(_useVertexBufferObjectForFaces);
	_faceGeometry->setUseVertexArrayObject(_useVertexArrayObjectForFaces);

	addDrawable(_faceGeometry);
}



void osgEditable::Editable::addFace(Face* face)
{
	_faces.push_back(face);
}

void osgEditable::Editable::removeFace(Face* face)
{
	auto itr = std::find(std::begin(_faces), std::end(_faces), face);

	if (itr != std::end(_faces))
		_faces.erase(itr);
}

void osgEditable::Editable::clearFaces()
{
	_faces.clear();
}


void osgEditable::Editable::addEdge(Edge* edge)
{
	_edges.push_back(edge);
}

void osgEditable::Editable::removeEdge(Edge* edge)
{
	auto itr = std::find(std::begin(_edges), std::end(_edges), edge);

	if (itr != std::end(_edges))
		_edges.erase(itr);
}

void osgEditable::Editable::clearEdges()
{
	_edges.clear();
}


void osgEditable::Editable::addVertex(Vertex* vertex)
{
	_vertices.push_back(vertex);
}

void osgEditable::Editable::removeVertex(Vertex* vertex)
{
	auto itr = std::find(std::begin(_vertices), std::end(_vertices), vertex);

	if (itr != std::end(_vertices))
		_vertices.erase(itr);
}

void osgEditable::Editable::clearVertices()
{
	_vertices.clear();
}




void osgEditable::Editable::showVerticesChanged()
{
	if (_vertexGeometry.valid())
	{
		if (_showVertices)
			addDrawable(_vertexGeometry);
		else
			removeDrawable(_vertexGeometry);
	}
}


void osgEditable::Editable::showEdgesChanged()
{
	if (_edgeGeometry.valid())
	{
		if (_showEdges)
			addDrawable(_edgeGeometry);
		else
			removeDrawable(_edgeGeometry);
	}
}


osg::PrimitiveSet* osgEditable::Editable::triangulate(osg::Vec3Array* vertices, std::vector<int> indices, const osg::Vec3& normal)
{
	const float eps = 1e-5;

	auto counter = 0;

	auto collide = [eps](std::vector<osg::Vec2f> vertices, int start0, int end0, int start1, int end1)
		{
			auto p0 = vertices[start0];
			auto p1 = vertices[start1];

			auto u = vertices[end0] - p0;
			auto ul = u.normalize();

			auto v = vertices[end1] - p1;
			auto vl = v.normalize();

			if (osg::absolute(u * v) > 1 - eps)
				return false;

			auto A =
				osg::Matrixf::inverse(
					osg::Matrixf(
						u.x(), -v.x(), 0, 0,
						u.y(), -v.y(), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1));

			auto x = osg::Vec4(p1.x() - p0.x(), p1.y() - p0.y(), 0, 1);

			auto result = A * x;

			auto s = result.x();
			auto t = result.y();

			return
				s > 0 &&
				t > 0 &&
				s < ul &&
				t < vl;
		};



	auto axis2 = -normal;

	auto axis0 = osg::absolute(axis2 * osg::Z_AXIS) < 1 - eps ?
		axis2 ^ osg::Z_AXIS :
		axis2 ^ osg::X_AXIS;

	auto axis1 = axis2 ^ axis0;

	axis0.normalize();
	axis1.normalize();
	axis2.normalize();

	auto viewProjection = osg::Matrix(
		axis0.x(), axis1.x(), 0, 0,
		axis0.y(), axis1.y(), 0, 0,
		axis0.z(), axis1.z(), 0, 0,
		0, 0, 0, 1);

	std::vector<osg::Vec2f> projected(vertices->size());

	for (auto idx : indices)
	{
		auto v = vertices->at(idx) * viewProjection;
		projected[idx] = osg::Vec2f(v.x(), v.y());
	}


	auto numIndices = indices.size();

	std::vector<unsigned int> triangles;

	while (numIndices >= 3)
	{
		for (unsigned int i = 0; i < numIndices; i++)
		{
			unsigned int i0 = indices[i];
			unsigned int i1 = indices[(i + 1) % numIndices];
			unsigned int i2 = indices[(i + 2) % numIndices];

			bool ok = true;

			for (int j = (i + 2) % numIndices; j != i; j = (j + 1) % numIndices)
			{
				if (collide(projected, i2, i0, indices[j], indices[(j + 1) % numIndices]))
				{
					ok = false;
					break;
				}
			}

			if (ok)
			{
				triangles.push_back(i0);
				triangles.push_back(i1);
				triangles.push_back(i2);
				indices.erase(indices.begin() + (i + 1) % numIndices);
				numIndices--;
				break;
			}
		}
	}

	auto triangleArray = new unsigned int[triangles.size()];

	std::copy(triangles.begin(), triangles.end(), triangleArray);

	return new osg::DrawElementsUInt(GL_TRIANGLES, triangles.size(), triangleArray);
}
