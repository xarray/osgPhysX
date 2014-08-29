SET(LIBRARY_NAME osgPhysicsUtils)

SET(HEADER_FILES
    InputManager.h
    FollowNodeManipulator.h
    SceneUtil.h
)

SET(LIBRARY_FILES
    InputManager.cpp
    FollowNodeManipulator.cpp
    SceneUtil.cpp
    ${HEADER_FILES}
)

SET(LIBRARY_FILES
    ois/OISEffect.cpp
    ois/OISException.cpp
    ois/OISForceFeedback.cpp
    ois/OISInputManager.cpp
    ois/OISJoyStick.cpp
    ois/OISKeyboard.cpp
    ois/OISObject.cpp
    ois/extras/LIRC/OISLIRC.cpp
    ois/extras/LIRC/OISLIRCFactoryCreator.cpp
    ${LIBRARY_FILES}
)

IF(WIN32)
    SET(LIBRARY_FILES ${LIBRARY_FILES}
        ois/win32/Win32ForceFeedback.cpp
        ois/win32/Win32InputManager.cpp
        ois/win32/Win32JoyStick.cpp
        ois/win32/Win32KeyBoard.cpp
        ois/win32/Win32Mouse.cpp
        ois/win32/extras/WiiMote/hiddevice.cpp
        ois/win32/extras/WiiMote/OISWiiMote.cpp
        ois/win32/extras/WiiMote/OISWiiMoteFactoryCreator.cpp
        ois/win32/extras/WiiMote/OISWiiMoteForceFeedback.cpp
        ois/win32/extras/WiiMote/OISWiiMoteRingBuffer.cpp
        ois/win32/extras/WiiMote/wiimote.cpp
    )
ELSEIF(APPLE)
    SET(LIBRARY_FILES ${LIBRARY_FILES}
        ois/mac/CocoaInputManager.mm
        ois/mac/CocoaJoyStick.mm
        ois/mac/CocoaKeyBoard.mm
        ois/mac/CocoaMouse.mm
        ois/mac/MacHelpers.cpp
        ois/mac/MacHIDManager.cpp
        ois/mac/MacInputManager.cpp
        ois/mac/MacJoyStick.cpp
        ois/mac/MacKeyboard.cpp
        ois/mac/MacMouse.cpp
    )
ELSE()
    SET(LIBRARY_FILES ${LIBRARY_FILES}
        ois/linux/EventHelpers.cpp
        ois/linux/LinuxForceFeedback.cpp
        ois/linux/LinuxInputManager.cpp
        ois/linux/LinuxJoyStickEvents.cpp
        ois/linux/LinuxKeyboard.cpp
        ois/linux/LinuxMouse.cpp
    )
ENDIF()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/ois)
#SET(EXTERNAL_LIBRARIES osgPhysics)
START_LIBRARY(STATIC)
