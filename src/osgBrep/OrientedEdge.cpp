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

#include <osgBrep/OrientedEdge>


osgBrep::OrientedEdge::OrientedEdge():
	_orientation(true)
{
}


osgBrep::OrientedEdge::OrientedEdge(const OrientedEdge& other, const osg::CopyOp& copyop) :
	osg::Object(other, copyop),
	_edge(other._edge),
	_orientation(other._orientation)
{
	if (copyop.getCopyFlags() & osg::CopyOp::DEEP_COPY_OBJECTS)
	{
		_edge = osg::clone(other._edge.get(), copyop);
	}
}


osgBrep::OrientedEdge::~OrientedEdge()
{
}
