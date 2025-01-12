
IF(DYNAMIC_OPENSCENEGRAPH)
    ADD_DEFINITIONS(-DOSGEDITABLE_LIBRARY)
ENDIF()

SET(LIB_NAME osgEditable)
SET(HEADER_PATH ${OpenSceneGraph_SOURCE_DIR}/include/${LIB_NAME})
SET(TARGET_H
    ${HEADER_PATH}/Editable
    ${HEADER_PATH}/Face
    ${HEADER_PATH}/Edge
    ${HEADER_PATH}/EdgeLoop
    ${HEADER_PATH}/OrientedEdge
    ${HEADER_PATH}/Vertex
    ${HEADER_PATH}/Export
    ${HEADER_PATH}/Version
)

# FIXME: For OS X, need flag for Framework or dylib
SET(TARGET_SRC
    Editable.cpp
    Face.cpp
    Edge.cpp
    EdgeLoop.cpp
    OrientedEdge.cpp
    Vertex.cpp
    Version.cpp
    ${OPENSCENEGRAPH_VERSIONINFO_RC}
)

SET(TARGET_LIBRARIES
    osg
    OpenThreads
)

SET(COMPONENT_PKGCONFIG_DESCRIPTION "A library for creating BRep with OpenSceneGraph")

SETUP_LIBRARY(${LIB_NAME})
