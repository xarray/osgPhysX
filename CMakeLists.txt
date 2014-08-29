PROJECT(osgPhysX)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4.7)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
    #CMAKE_POLICY(SET CMP0022 NEW)
ENDIF(COMMAND cmake_policy)

IF(WIN32)
    IF(MSVC)
        OPTION(WIN32_USE_MP "Set to ON to build with the /MP option (Visual Studio 2005 and above)" ON)
        MARK_AS_ADVANCED(WIN32_USE_MP)
        IF(WIN32_USE_MP)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
        ENDIF(WIN32_USE_MP)

        # More MSVC specific compilation flags
        ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
        ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /wd4996 /wd4355 /wd4244 /wd4985 /wd4267 /wd4619 /wd4290")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996 /wd4355 /wd4244 /wd4985 /wd4267 /wd4619 /wd4290")
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ignore:4099")
        
        # /bigobj is required for windows builds because of the size of some object files
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /bigobj")
        
        IF(NOT CMAKE_CL_64)
            # /LARGEADDRESSAWARE enables 32-bit applications to use more than 2 GB RAM
            SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
        ENDIF(NOT CMAKE_CL_64)
        
        # Set for FFmpeg & libVLC under Windows
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /OPT:NOREF")
    ENDIF(MSVC)
ENDIF(WIN32)

IF(NOT WIN32)
    SET(CMAKE_CXX_FLAGS "-W -Wall -Wno-unused")
ENDIF(NOT WIN32)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/;${CMAKE_MODULE_PATH}")
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "add a postfix, usually d on windows")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")
INCLUDE(CheckIncludeFiles)

# Dependencies

FIND_PACKAGE(OpenGL)
FIND_PACKAGE(PhysX3)

FIND_PATH(OSG_INCLUDE_DIR osg/Referenced
    PATHS
    $ENV{OSG_ROOT}/include
    $ENV{OSG_DIR}/include
    /usr/include 
    /usr/local/include
)

FIND_PATH(OSG_LIB_DIR libosg.so osg.lib
    PATHS
    $ENV{OSG_ROOT}/lib
    $ENV{OSG_DIR}/lib
    /usr/lib 
    /usr/local/lib
)

SET(OSG_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d on windows")
SET(OSG_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/thirdparty/prebuilt/include
    ${OPENGL_INCLUDE_DIR}
    ${OSG_INCLUDE_DIR}
)
LINK_DIRECTORIES(
    ${OSG_LIB_DIR}
)

SET(THIRD_PARTY_INCLUDES ${SPEEDTREE_THIRD_PARTY_INCLUDES} ${PHYSX_INCLUDE_DIR})
SET(THIRD_PARTY_LIBRARIES ${SPEEDTREE_THIRD_PARTY_LIBRARIES} ${PHYSX_LIBRARIES})

MACRO(START_EXECUTABLE)

    INCLUDE_DIRECTORIES(${EXTERNAL_INCLUDE_DIR} ${THIRD_PARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR})
    ADD_EXECUTABLE(${EXECUTABLE_NAME} ${EXECUTABLE_FILES})
    SET_TARGET_PROPERTIES(${EXECUTABLE_NAME} PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
    SET_TARGET_PROPERTIES(${EXECUTABLE_NAME} PROPERTIES FOLDER "Executables")
    
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
        debug osg${OSG_DEBUG_POSTFIX}          optimized osg${OSG_RELEASE_POSTFIX}
        debug osgAnimation${OSG_DEBUG_POSTFIX} optimized osgAnimation${OSG_RELEASE_POSTFIX}
        debug osgParticle${OSG_DEBUG_POSTFIX}  optimized osgParticle${OSG_RELEASE_POSTFIX}
        debug osgShadow${OSG_DEBUG_POSTFIX}    optimized osgShadow${OSG_RELEASE_POSTFIX}
        debug osgDB${OSG_DEBUG_POSTFIX}        optimized osgDB${OSG_RELEASE_POSTFIX}
        debug osgGA${OSG_DEBUG_POSTFIX}        optimized osgGA${OSG_RELEASE_POSTFIX}
        debug osgText${OSG_DEBUG_POSTFIX}      optimized osgText${OSG_RELEASE_POSTFIX}
        debug osgTerrain${OSG_DEBUG_POSTFIX}   optimized osgTerrain${OSG_RELEASE_POSTFIX}
        debug osgUtil${OSG_DEBUG_POSTFIX}      optimized osgUtil${OSG_RELEASE_POSTFIX}
        debug osgViewer${OSG_DEBUG_POSTFIX}    optimized osgViewer${OSG_RELEASE_POSTFIX}
        debug OpenThreads${OSG_DEBUG_POSTFIX}  optimized OpenThreads${OSG_RELEASE_POSTFIX}
        ${OPENGL_LIBRARIES} ${THIRD_PARTY_LIBRARIES} ${EXTERNAL_LIBRARIES}
    )
    INSTALL(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

ENDMACRO(START_EXECUTABLE)

MACRO(START_LIBRARY LIBTYPE)

    INCLUDE_DIRECTORIES(${EXTERNAL_INCLUDE_DIR} ${THIRD_PARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR})
    ADD_LIBRARY(${LIBRARY_NAME} ${LIBTYPE} ${LIBRARY_FILES})
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES FOLDER "Libraries")
    
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
        debug osg${OSG_DEBUG_POSTFIX}          optimized osg${OSG_RELEASE_POSTFIX}
        debug osgAnimation${OSG_DEBUG_POSTFIX} optimized osgAnimation${OSG_RELEASE_POSTFIX}
        debug osgParticle${OSG_DEBUG_POSTFIX}  optimized osgParticle${OSG_RELEASE_POSTFIX}
        debug osgShadow${OSG_DEBUG_POSTFIX}    optimized osgShadow${OSG_RELEASE_POSTFIX}
        debug osgDB${OSG_DEBUG_POSTFIX}        optimized osgDB${OSG_RELEASE_POSTFIX}
        debug osgGA${OSG_DEBUG_POSTFIX}        optimized osgGA${OSG_RELEASE_POSTFIX}
        debug osgText${OSG_DEBUG_POSTFIX}      optimized osgText${OSG_RELEASE_POSTFIX}
        debug osgTerrain${OSG_DEBUG_POSTFIX}   optimized osgTerrain${OSG_RELEASE_POSTFIX}
        debug osgUtil${OSG_DEBUG_POSTFIX}      optimized osgUtil${OSG_RELEASE_POSTFIX}
        debug osgViewer${OSG_DEBUG_POSTFIX}    optimized osgViewer${OSG_RELEASE_POSTFIX}
        debug OpenThreads${OSG_DEBUG_POSTFIX}  optimized OpenThreads${OSG_RELEASE_POSTFIX}
        ${OPENGL_LIBRARIES} ${THIRD_PARTY_LIBRARIES} ${EXTERNAL_LIBRARIES}
    )
    INSTALL(TARGETS ${LIBRARY_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                                    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                                    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )

ENDMACRO(START_LIBRARY)

# Perform main components build

ADD_SUBDIRECTORY(physics)
ADD_SUBDIRECTORY(utils)
ADD_SUBDIRECTORY(tests)
