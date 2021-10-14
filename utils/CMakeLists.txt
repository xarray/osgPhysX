SET(LIBRARY_NAME osgPhysicsUtils)

SET(HEADER_FILES
    InputManager.h
    FollowNodeManipulator.h
	PlayerAnimation.h
    SceneUtil.h
)

SET(LIBRARY_FILES
    InputManager.cpp
    FollowNodeManipulator.cpp
	PlayerAnimation.cpp
    SceneUtil.cpp
    ${HEADER_FILES}
)

SET(OIS_LIBRARY_FILES
    ois/OISEffect.cpp
    ois/OISException.cpp
    ois/OISForceFeedback.cpp
    ois/OISInputManager.cpp
    ois/OISJoyStick.cpp
    ois/OISKeyboard.cpp
    ois/OISObject.cpp
    ois/extras/LIRC/OISLIRC.cpp
    ois/extras/LIRC/OISLIRCFactoryCreator.cpp
)

SET(OZZ_LIBRARY_FILES
    ozz/base/memory/allocator.cc
    ozz/base/platform.cc
    ozz/base/log.cc
    ozz/base/containers/string_archive.cc
    ozz/base/io/archive.cc
    ozz/base/io/stream.cc
    ozz/base/maths/box.cc
    ozz/base/maths/simd_math.cc
    ozz/base/maths/math_archive.cc
    ozz/base/maths/soa_math_archive.cc
    ozz/base/maths/simd_math_archive.cc
    ozz/animation/runtime/animation.cc
    ozz/animation/runtime/animation_keyframe.h
    ozz/animation/runtime/animation_utils.cc
    ozz/animation/runtime/blending_job.cc
    ozz/animation/runtime/ik_aim_job.cc
    ozz/animation/runtime/ik_two_bone_job.cc
    ozz/animation/runtime/local_to_model_job.cc
    ozz/animation/runtime/sampling_job.cc
    ozz/animation/runtime/skeleton.cc
    ozz/animation/runtime/skeleton_utils.cc
    ozz/animation/runtime/track.cc
    ozz/animation/runtime/track_sampling_job.cc
    ozz/animation/runtime/track_triggering_job.cc
    ozz/geometry/runtime/skinning_job.cc
    ozz/options/options.cc ozz/mesh.cc
)

IF(WIN32)
    SET(OIS_LIBRARY_FILES ${OIS_LIBRARY_FILES}
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
    SET(OIS_LIBRARY_FILES ${OIS_LIBRARY_FILES}
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
    SET(OIS_LIBRARY_FILES ${OIS_LIBRARY_FILES}
        ois/linux/EventHelpers.cpp
        ois/linux/LinuxForceFeedback.cpp
        ois/linux/LinuxInputManager.cpp
        ois/linux/LinuxJoyStickEvents.cpp
        ois/linux/LinuxKeyboard.cpp
        ois/linux/LinuxMouse.cpp
    )
ENDIF()

SET(LIBRARY_FILES ${LIBRARY_FILES} ${OIS_LIBRARY_FILES} ${OZZ_LIBRARY_FILES})
SOURCE_GROUP("OIS Source Files" FILES ${OIS_LIBRARY_FILES})
SOURCE_GROUP("OZZ Source Files" FILES ${OZZ_LIBRARY_FILES})

INCLUDE_DIRECTORIES(ois ozz)
#SET(EXTERNAL_LIBRARIES osgPhysics)
START_LIBRARY(STATIC)
