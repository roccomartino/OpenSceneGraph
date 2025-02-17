#---------------------------------------------------
# OSG CMAKE SUPPORT
# (C) by Michael Wagner, mtw@shared-reality.com 2005
# (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
#---------------------------------------------------

PROJECT(OSG_PLUGINS_MASTER)

IF(NOT MSVC)
    SET(LIBRARY_OUTPUT_PATH "${LIBRARY_OUTPUT_PATH}/${OSG_PLUGINS}")
    SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${OSG_PLUGINS}")
ENDIF()

SET(CMAKE_SHARED_MODULE_PREFIX ${OSG_PLUGIN_PREFIX})

IF(MSVC80 OR MSVC90)
    IF(NOT OSG_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
        SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
    ENDIF()
ENDIF()

IF(MSVC)
    # Avoid C4121 warnings
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zp16")
ENDIF()

################################################################################
## Quieten warnings that a due to optional code paths

IF(CMAKE_COMPILER_IS_GNUCXX)
    # Remove -Wextra flag as it barfs on ffmoeg headers
   STRING(REGEX REPLACE "-Wextra" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
ENDIF()

SET(TARGET_DEFAULT_PREFIX "osgdb_serializers_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Plugins osg serializer")
SET(TARGET_COMMON_LIBRARIES
    OpenThreads
    osg
    osgDB
    osgUtil
)

ADD_SUBDIRECTORY(osg)
ADD_SUBDIRECTORY(osgUtil)
ADD_SUBDIRECTORY(osgAnimation)
ADD_SUBDIRECTORY(osgFX)
ADD_SUBDIRECTORY(osgManipulator)
ADD_SUBDIRECTORY(osgParticle)
ADD_SUBDIRECTORY(osgSim)
ADD_SUBDIRECTORY(osgShadow)
ADD_SUBDIRECTORY(osgGA)
ADD_SUBDIRECTORY(osgTerrain)
ADD_SUBDIRECTORY(osgText)
ADD_SUBDIRECTORY(osgVolume)
ADD_SUBDIRECTORY(osgViewer)
ADD_SUBDIRECTORY(osgUI)
