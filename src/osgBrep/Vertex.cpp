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

#include <osgBrep/Vertex>


osgBrep::Vertex::Vertex()
{
}


osgBrep::Vertex::Vertex(const Vertex& other, const osg::CopyOp& copyop):
	osg::Object(other, copyop),
	_position(other._position)
{
}


osgBrep::Vertex::~Vertex()
{
}


void osgBrep::Vertex::setPosition(const osg::Vec3d& position)
{
    _position = position;
}

const osg::Vec3& osgBrep::Vertex::getPosition() const
{
    return _position;
}
