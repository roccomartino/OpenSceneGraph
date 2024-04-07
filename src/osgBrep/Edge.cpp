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

#include <osgBrep/Edge>


osgBrep::Edge::Edge()
{
}


osgBrep::Edge::Edge(const Edge& other, const osg::CopyOp& copyop) :
	osg::Object(other, copyop),
	_start(other._start),
	_end(other._end),
	_selected(other._selected)
{
	if (copyop.getCopyFlags() & osg::CopyOp::DEEP_COPY_OBJECTS)
	{
		_start = osg::clone(other._start.get(), copyop);
		_end = osg::clone(other._end.get(), copyop);
	}
}


osgBrep::Edge::~Edge()
{
}
