#include <osgViewer/api/win32/GraphicsWindowWin32>
#include <ois/OISInputManager.h>
#include <ois/OISKeyboard.h>
#include <ois/OISMouse.h>
#include <ois/OISJoyStick.h>
#include <ois/OISEvents.h>
#include "InputManager.h"
#include <iostream>
#include <sstream>

using namespace osgPhysicsUtils;

static std::map<int, OIS::KeyCode> s_keyCodeMap;

/* InputManager */

InputManager::InputManager()
:   _inputManager(NULL)
{
    if ( !s_keyCodeMap.size() )
    {
        #define ADD_KEYPAIR(k1, k2) s_keyCodeMap[osgGA::GUIEventAdapter:: ## k1] = OIS::k2
        #define ADD_RAWKEYPAIR(k1, k2) s_keyCodeMap[k1] = OIS::k2
        ADD_KEYPAIR(KEY_A, KC_A); ADD_KEYPAIR(KEY_B, KC_B); ADD_KEYPAIR(KEY_C, KC_C); ADD_KEYPAIR(KEY_D, KC_D);
        ADD_KEYPAIR(KEY_E, KC_E); ADD_KEYPAIR(KEY_F, KC_F); ADD_KEYPAIR(KEY_G, KC_G); ADD_KEYPAIR(KEY_H, KC_H);
        ADD_KEYPAIR(KEY_I, KC_I); ADD_KEYPAIR(KEY_J, KC_J); ADD_KEYPAIR(KEY_K, KC_K); ADD_KEYPAIR(KEY_L, KC_L);
        ADD_KEYPAIR(KEY_M, KC_M); ADD_KEYPAIR(KEY_N, KC_N); ADD_KEYPAIR(KEY_O, KC_O); ADD_KEYPAIR(KEY_P, KC_P);
        ADD_KEYPAIR(KEY_Q, KC_Q); ADD_KEYPAIR(KEY_R, KC_R); ADD_KEYPAIR(KEY_S, KC_S); ADD_KEYPAIR(KEY_T, KC_T);
        ADD_KEYPAIR(KEY_U, KC_U); ADD_KEYPAIR(KEY_V, KC_V); ADD_KEYPAIR(KEY_W, KC_W); ADD_KEYPAIR(KEY_X, KC_X);
        ADD_KEYPAIR(KEY_Y, KC_Y); ADD_KEYPAIR(KEY_Z, KC_Z);
        ADD_RAWKEYPAIR('A', KC_A); ADD_RAWKEYPAIR('B', KC_B); ADD_RAWKEYPAIR('C', KC_C); ADD_RAWKEYPAIR('D', KC_D);
        ADD_RAWKEYPAIR('E', KC_E); ADD_RAWKEYPAIR('F', KC_F); ADD_RAWKEYPAIR('G', KC_G); ADD_RAWKEYPAIR('H', KC_H);
        ADD_RAWKEYPAIR('I', KC_I); ADD_RAWKEYPAIR('J', KC_J); ADD_RAWKEYPAIR('K', KC_K); ADD_RAWKEYPAIR('L', KC_L);
        ADD_RAWKEYPAIR('M', KC_M); ADD_RAWKEYPAIR('N', KC_N); ADD_RAWKEYPAIR('O', KC_O); ADD_RAWKEYPAIR('P', KC_P);
        ADD_RAWKEYPAIR('Q', KC_Q); ADD_RAWKEYPAIR('R', KC_R); ADD_RAWKEYPAIR('S', KC_S); ADD_RAWKEYPAIR('T', KC_T);
        ADD_RAWKEYPAIR('U', KC_U); ADD_RAWKEYPAIR('V', KC_V); ADD_RAWKEYPAIR('W', KC_W); ADD_RAWKEYPAIR('X', KC_X);
        ADD_RAWKEYPAIR('Y', KC_Y); ADD_RAWKEYPAIR('Z', KC_Z);
        
        ADD_KEYPAIR(KEY_1, KC_1); ADD_KEYPAIR(KEY_2, KC_2); ADD_KEYPAIR(KEY_3, KC_3); ADD_KEYPAIR(KEY_4, KC_4);
        ADD_KEYPAIR(KEY_5, KC_5); ADD_KEYPAIR(KEY_6, KC_6); ADD_KEYPAIR(KEY_7, KC_7); ADD_KEYPAIR(KEY_8, KC_8);
        ADD_KEYPAIR(KEY_9, KC_9); ADD_KEYPAIR(KEY_0, KC_0);
        ADD_RAWKEYPAIR('!', KC_1); ADD_RAWKEYPAIR('@', KC_2); ADD_RAWKEYPAIR('#', KC_3); ADD_RAWKEYPAIR('$', KC_4);
        ADD_RAWKEYPAIR('%', KC_5); ADD_RAWKEYPAIR('^', KC_6); ADD_RAWKEYPAIR('&', KC_7); ADD_RAWKEYPAIR('*', KC_8);
        ADD_RAWKEYPAIR('(', KC_9); ADD_RAWKEYPAIR(')', KC_0);
        
        ADD_KEYPAIR(KEY_F1, KC_F1); ADD_KEYPAIR(KEY_F2, KC_F2); ADD_KEYPAIR(KEY_F3, KC_F3); ADD_KEYPAIR(KEY_F4, KC_F4);
        ADD_KEYPAIR(KEY_F5, KC_F5); ADD_KEYPAIR(KEY_F6, KC_F6); ADD_KEYPAIR(KEY_F7, KC_F7); ADD_KEYPAIR(KEY_F8, KC_F8);
        ADD_KEYPAIR(KEY_F9, KC_F9); ADD_KEYPAIR(KEY_F10, KC_F10); ADD_KEYPAIR(KEY_F11, KC_F11);
        ADD_KEYPAIR(KEY_F12, KC_F12); ADD_KEYPAIR(KEY_F13, KC_F13);
        ADD_KEYPAIR(KEY_F14, KC_F14); ADD_KEYPAIR(KEY_F15, KC_F15);
        
        ADD_KEYPAIR(KEY_KP_1, KC_NUMPAD1); ADD_KEYPAIR(KEY_KP_2, KC_NUMPAD2); ADD_KEYPAIR(KEY_KP_3, KC_NUMPAD3);
        ADD_KEYPAIR(KEY_KP_4, KC_NUMPAD4); ADD_KEYPAIR(KEY_KP_5, KC_NUMPAD5); ADD_KEYPAIR(KEY_KP_6, KC_NUMPAD6);
        ADD_KEYPAIR(KEY_KP_7, KC_NUMPAD7); ADD_KEYPAIR(KEY_KP_8, KC_NUMPAD8); ADD_KEYPAIR(KEY_KP_9, KC_NUMPAD9);
        ADD_KEYPAIR(KEY_KP_0, KC_NUMPAD0); ADD_KEYPAIR(KEY_KP_Decimal, KC_DECIMAL); ADD_KEYPAIR(KEY_KP_Enter, KC_NUMPADENTER);
        ADD_KEYPAIR(KEY_KP_Divide, KC_DIVIDE); ADD_KEYPAIR(KEY_KP_Multiply, KC_MULTIPLY);
        ADD_KEYPAIR(KEY_KP_Subtract, KC_SUBTRACT); ADD_KEYPAIR(KEY_KP_Add, KC_ADD);
        
        ADD_KEYPAIR(KEY_Escape, KC_ESCAPE); ADD_KEYPAIR(KEY_Tab, KC_TAB); ADD_KEYPAIR(KEY_BackSpace, KC_BACK);
        ADD_KEYPAIR(KEY_Space, KC_SPACE); ADD_KEYPAIR(KEY_Return, KC_RETURN);
        ADD_KEYPAIR(KEY_Caps_Lock, KC_CAPITAL); ADD_KEYPAIR(KEY_Num_Lock, KC_NUMLOCK);
        ADD_KEYPAIR(KEY_Sys_Req, KC_SYSRQ); ADD_KEYPAIR(KEY_Scroll_Lock, KC_SCROLL); ADD_KEYPAIR(KEY_Pause, KC_PAUSE);
        ADD_KEYPAIR(KEY_Up, KC_UP); ADD_KEYPAIR(KEY_Down, KC_DOWN);
        ADD_KEYPAIR(KEY_Left, KC_LEFT); ADD_KEYPAIR(KEY_Right, KC_RIGHT);
        ADD_KEYPAIR(KEY_Insert, KC_INSERT); ADD_KEYPAIR(KEY_Delete, KC_DELETE);
        ADD_KEYPAIR(KEY_Home, KC_HOME); ADD_KEYPAIR(KEY_End, KC_END);
        ADD_KEYPAIR(KEY_Page_Up, KC_PGUP); ADD_KEYPAIR(KEY_Page_Down, KC_PGDOWN);
        ADD_KEYPAIR(KEY_Meta_L, KC_LWIN); ADD_KEYPAIR(KEY_Meta_R, KC_RWIN);
        
        ADD_KEYPAIR(KEY_Minus, KC_MINUS); ADD_KEYPAIR(KEY_Equals, KC_EQUALS);
        ADD_RAWKEYPAIR('-', KC_MINUS); ADD_RAWKEYPAIR('=', KC_EQUALS);
        ADD_RAWKEYPAIR('_', KC_MINUS); ADD_RAWKEYPAIR('+', KC_EQUALS);
        ADD_KEYPAIR(KEY_Leftbracket, KC_LBRACKET); ADD_KEYPAIR(KEY_Rightbracket, KC_RBRACKET);
        ADD_RAWKEYPAIR('[', KC_LBRACKET); ADD_RAWKEYPAIR(']', KC_RBRACKET);
        ADD_RAWKEYPAIR('{', KC_LBRACKET); ADD_RAWKEYPAIR('}', KC_RBRACKET);
        ADD_KEYPAIR(KEY_Semicolon, KC_SEMICOLON); ADD_KEYPAIR(KEY_Quote, KC_APOSTROPHE);
        ADD_RAWKEYPAIR(';', KC_SEMICOLON); ADD_RAWKEYPAIR('\'', KC_APOSTROPHE);
        ADD_RAWKEYPAIR(':', KC_SEMICOLON); ADD_RAWKEYPAIR('"', KC_APOSTROPHE);
        ADD_KEYPAIR(KEY_Linefeed, KC_GRAVE); ADD_KEYPAIR(KEY_Backslash, KC_BACKSLASH);
        ADD_RAWKEYPAIR('`', KC_GRAVE); ADD_RAWKEYPAIR('\\', KC_BACKSLASH);
        ADD_RAWKEYPAIR('~', KC_GRAVE); ADD_RAWKEYPAIR('|', KC_BACKSLASH);
        ADD_KEYPAIR(KEY_Comma, KC_COMMA); ADD_KEYPAIR(KEY_Period, KC_PERIOD); ADD_KEYPAIR(KEY_Slash, KC_SLASH);
        ADD_RAWKEYPAIR(',', KC_COMMA); ADD_RAWKEYPAIR('.', KC_PERIOD); ADD_RAWKEYPAIR('/', KC_SLASH);
        ADD_RAWKEYPAIR('<', KC_COMMA); ADD_RAWKEYPAIR('>', KC_PERIOD); ADD_RAWKEYPAIR('?', KC_SLASH);
    }
}

InputManager::~InputManager()
{
    destroy();
}

bool InputManager::initialize( osg::GraphicsContext* gc, const std::vector<std::string>& userParams )
{
    osgViewer::GraphicsWindow* window = dynamic_cast<osgViewer::GraphicsWindow*>( gc );
    if ( !window ) return false;
    OIS::ParamList params;
    std::stringstream ss;
    
#ifdef WIN32
    osgViewer::GraphicsWindowWin32* windowWin = dynamic_cast<osgViewer::GraphicsWindowWin32*>( window );
    if ( !windowWin ) return false;
    ss << (size_t)windowWin->getHWND();
#else
    // TODO
    return false;
#endif
    
    params.insert( std::make_pair(std::string("WINDOW"), ss.str()) );
    if ( !userParams.size() )
    {
        params.insert( std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND" )) );
        params.insert( std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")) );
    }
    else
    {
        for ( unsigned int i=0; i<userParams.size(); i+=2 )
            params.insert( std::make_pair(userParams[i], userParams[i+1]) );
    }
    _inputManager = OIS::InputManager::createInputSystem( params );
    _inputManager->enableAddOnFactory( OIS::InputManager::AddOn_All );
    
    OIS::DeviceList devices = _inputManager->listFreeDevices();
    for ( OIS::DeviceList::iterator itr=devices.begin(); itr!=devices.end(); ++itr )
    {
        switch ( itr->first )
        {
        case OIS::OISKeyboard:
            _keyboards.push_back( static_cast<OIS::Keyboard*>(
                _inputManager->createInputObject(OIS::OISKeyboard, true)) );
            break;
        case OIS::OISMouse:
            _mouses.push_back( static_cast<OIS::Mouse*>(
                _inputManager->createInputObject(OIS::OISMouse, true)) );
            break;
        case OIS::OISJoyStick:
            _joysticks.push_back( static_cast<OIS::JoyStick*>(
                _inputManager->createInputObject(OIS::OISJoyStick, true)) );
            break;
        default: break;  // TODO: tablet & multitouch
        }
    }
    
    int x=0, y=0, width=800, height=600;
    window->getWindowRectangle( x, y, width, height );
    resize( width, height );
    return true;
}

void InputManager::frame()
{
    for ( unsigned int i=0; i<_keyboards.size(); ++i )
        _keyboards[i]->capture();
    for ( unsigned int i=0; i<_mouses.size(); ++i )
        _mouses[i]->capture();
    for ( unsigned int i=0; i<_joysticks.size(); ++i )
        _joysticks[i]->capture();
}

void InputManager::resize( int width, int height )
{
    for ( unsigned int i=0; i<_mouses.size(); ++i )
    {
        const OIS::MouseState& state = _mouses[i]->getMouseState();
        state.width = width;
        state.height = height;
    }
}

void InputManager::destroy()
{
    if ( _inputManager )
        OIS::InputManager::destroyInputSystem( _inputManager );
}

bool InputManager::isKeyDown( int key, unsigned int i ) const
{
    return _keyboards[i]->isKeyDown( s_keyCodeMap[key] );
}

bool InputManager::isModKeyDown( int modkey, unsigned int i ) const
{
    if ( modkey&osgGA::GUIEventAdapter::MODKEY_CTRL )
    {
        return _keyboards[i]->isModifierDown(OIS::Keyboard::Ctrl);
    }
    else if ( modkey&osgGA::GUIEventAdapter::MODKEY_ALT )
    {
        return _keyboards[i]->isModifierDown(OIS::Keyboard::Alt);
    }
    else if ( modkey&osgGA::GUIEventAdapter::MODKEY_SHIFT )
    {
        return _keyboards[i]->isModifierDown(OIS::Keyboard::Shift);
    }
    return false;
}

osg::Vec3 InputManager::getMouseCoordinate( unsigned int i ) const
{
    const OIS::MouseState& state = _mouses[i]->getMouseState();
    return osg::Vec3(state.X.abs, state.Y.abs, state.Z.abs);
}

int InputManager::getMouseButton( unsigned int i ) const
{
    const OIS::MouseState& state = _mouses[i]->getMouseState();
    int buttonMask = 0;
    if ( state.buttons&OIS::MB_Left ) buttonMask |= osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
    if ( state.buttons&OIS::MB_Middle ) buttonMask |= osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
    if ( state.buttons&OIS::MB_Right ) buttonMask |= osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON;
    return buttonMask;
}
