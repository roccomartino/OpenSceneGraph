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

#include <osgEditable/OrientedEdge>


osgEditable::OrientedEdge::OrientedEdge():
	_orientation(true)
{
}


osgEditable::OrientedEdge::OrientedEdge(const OrientedEdge& other, const osg::CopyOp& copyop) :
	osg::Object(other, copyop),
	_edge(other._edge),
	_orientation(other._orientation)
{
	if (copyop.getCopyFlags() & osg::CopyOp::DEEP_COPY_OBJECTS)
	{
		_edge = osg::clone(other._edge.get(), copyop);
	}
}


osgEditable::OrientedEdge::~OrientedEdge()
{
}



void osgEditable::OrientedEdge::setEdge(Edge* edge)
{
    _edge = edge;
}

osgEditable::Edge* osgEditable::OrientedEdge::getEdge()
{
    return _edge;
}

const osgEditable::Edge* osgEditable::OrientedEdge::getEdge() const
{
    return _edge;
}


void osgEditable::OrientedEdge::setOrientation(bool orientation)
{
    _orientation = orientation;
}

bool osgEditable::OrientedEdge::getOrientation() const
{
    return _orientation;
}

const osgEditable::Vertex* osgEditable::OrientedEdge::getOrientedStart() const
{
	return _orientation ? _edge->getStart() : _edge->getEnd();
}

const osgEditable::Vertex* osgEditable::OrientedEdge::getOrientedEnd() const
{
	return _orientation ? _edge->getEnd() : _edge->getStart();
}

osgEditable::Vertex* osgEditable::OrientedEdge::getOrientedStart()
{
	return _orientation ? _edge->getStart() : _edge->getEnd();
}

osgEditable::Vertex* osgEditable::OrientedEdge::getOrientedEnd()
{
	return _orientation ? _edge->getEnd() : _edge->getStart();
}
