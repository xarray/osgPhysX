SET(LIBRARY_NAME osgPhysics)

SET(HEADER_FILES
    Callbacks.h
    CharacterController.h
    Engine.h
    ParticleUpdater.h
    PhysicsUtil.h
    Vehicle.h
    VehicleManager.h
)

SET(LIBRARY_FILES
    Callbacks.cpp
    CharacterController.cpp
    Engine.cpp
    ParticleUpdater.cpp
    PhysicsUtil.cpp
    Vehicle.cpp
    VehicleManager.cpp
    ${HEADER_FILES}
)

START_LIBRARY(STATIC)
