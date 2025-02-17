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

#include <osgEditable/Vertex>


osgEditable::Vertex::Vertex()
{
}


osgEditable::Vertex::Vertex(const Vertex& other, const osg::CopyOp& copyop):
	osg::Object(other, copyop),
	_position(other._position),
	_color(other._color)
{
}


osgEditable::Vertex::~Vertex()
{
}
