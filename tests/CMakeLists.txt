LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
IF(WIN32)
    SET(THIRD_PARTY_LIBRARIES ws2_32 dinput8 dxguid)
ENDIF(WIN32)

# Character control
SET(EXECUTABLE_NAME character_controller)
SET(EXECUTABLE_FILES character_controller_test.cpp)
SET(EXTERNAL_LIBRARIES osgPhysics osgPhysicsUtils ${THIRD_PARTY_LIBRARIES})
START_EXECUTABLE()

# Wheeled vehicle control
SET(EXECUTABLE_NAME physics_vehicle)
SET(EXECUTABLE_FILES physics_vehicle_test.cpp)
SET(EXTERNAL_LIBRARIES osgPhysics osgPhysicsUtils ${THIRD_PARTY_LIBRARIES})
START_EXECUTABLE()
