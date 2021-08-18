#ifndef CORE_INPUTMANAGER
#define CORE_INPUTMANAGER

#include <osgGA/GUIEventHandler>
#include <set>

namespace OIS
{

    class InputManager;
    class Keyboard;
    class Mouse;
    class JoyStick;

}

namespace osgPhysicsUtils
{

    class InputManager : public osg::Referenced
    {
    public:
        InputManager();
        virtual ~InputManager();

        bool initialize(osg::GraphicsContext* gc, const std::vector<std::string>& params);
        void frame();
        void resize(int width, int height);
        void destroy();

        unsigned int getNumKeyboards() const { return _keyboards.size(); }
        unsigned int getNumMouses() const { return _mouses.size(); }
        unsigned int getNumJoySticks() const { return _joysticks.size(); }

        bool isKeyDown(int key, unsigned int index = 0) const;
        bool isModKeyDown(int modkey, unsigned int index = 0) const;

        osg::Vec3 getMouseCoordinate(unsigned int index = 0) const;
        int getMouseButton(unsigned int index = 0) const;

        // TODO: joystick handlers

    protected:
        OIS::InputManager* _inputManager;
        std::vector<OIS::Keyboard*> _keyboards;
        std::vector<OIS::Mouse*> _mouses;
        std::vector<OIS::JoyStick*> _joysticks;
    };

}

#endif
