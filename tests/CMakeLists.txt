LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
IF(WIN32)
    IF(CMAKE_CL_64)
        LINK_DIRECTORIES(x64)
    ELSE(CMAKE_CL_64)
        LINK_DIRECTORIES(x86)
    ENDIF(CMAKE_CL_64)
    SET(THIRD_PARTY_LIBRARIES ${THIRD_PARTY_LIBRARIES} ws2_32 dinput8 dxguid)
ENDIF(WIN32)

# Basic rigid bodies
SET(EXECUTABLE_NAME hello_physics_world)
SET(EXECUTABLE_FILES hello_physics_world_test.cpp)
SET(EXTERNAL_LIBRARIES osgPhysics osgPhysicsUtils ${THIRD_PARTY_LIBRARIES})
START_EXECUTABLE()

# Height field with materials
SET(EXECUTABLE_NAME physics_heightfield)
SET(EXECUTABLE_FILES physics_heightfield_test.cpp)
SET(EXTERNAL_LIBRARIES osgPhysics osgPhysicsUtils ${THIRD_PARTY_LIBRARIES})
START_EXECUTABLE()

# Character control
SET(EXECUTABLE_NAME character_controller)
SET(EXECUTABLE_FILES character_controller_test.cpp)
SET(EXTERNAL_LIBRARIES osgPhysics osgPhysicsUtils ${THIRD_PARTY_LIBRARIES})
START_EXECUTABLE()

SET(EXECUTABLE_NAME character_animation)
SET(EXECUTABLE_FILES character_animation_test.cpp)
SET(EXTERNAL_LIBRARIES osgPhysics osgPhysicsUtils ${THIRD_PARTY_LIBRARIES})
START_EXECUTABLE()

# Wheeled vehicle control
SET(EXECUTABLE_NAME physics_vehicle)
SET(EXECUTABLE_FILES physics_vehicle_test.cpp)
SET(EXTERNAL_LIBRARIES osgPhysics osgPhysicsUtils ${THIRD_PARTY_LIBRARIES})
START_EXECUTABLE()
