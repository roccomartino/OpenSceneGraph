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

#include <osgEditable/Face>


osgEditable::Face::Face():
	_normal(osg::Z_AXIS)
{
}


osgEditable::Face::Face(const Face& other, const osg::CopyOp& copyop) :
	osg::Object(other, copyop),
	_edgeLoop(other._edgeLoop),
	_normal(other._normal)
{
	if (copyop.getCopyFlags() & osg::CopyOp::DEEP_COPY_OBJECTS)
		_edgeLoop = (EdgeLoop*)other._edgeLoop->clone(copyop);
}


osgEditable::Face::~Face()
{
}


void osgEditable::Face::setEdgeLoop(EdgeLoop* edgeLoop)
{
    _edgeLoop = edgeLoop;
}

osgEditable::EdgeLoop* osgEditable::Face::getEdgeLoop()
{
    return _edgeLoop;
}

const osgEditable::EdgeLoop* osgEditable::Face::getEdgeLoop() const
{
    return _edgeLoop;
}

void osgEditable::Face::setNormal(const osg::Vec3& normal)
{
    _normal = normal;
}

const osg::Vec3& osgEditable::Face::getNormal() const
{
    return _normal;
}
