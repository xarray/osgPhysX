#include <utils/SceneUtil.h>
#include <utils/InputManager.h>
#include <utils/PlayerAnimation.h>

#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>

std::string animations[] = {
    "idle", "walking", "running", "jumping", "crouch", "say_hello",
    "be_hit", "dying", "dying_backwards", "pistol_aiming", "shooting"
};

struct PartialBlenderCreator
{
    std::map<int, int> jointMap; int spineID;
    osgPhysicsUtils::PlayerAnimation* animator;

    PartialBlenderCreator(osgPhysicsUtils::PlayerAnimation* p) : animator(p)
    {
        auto jList = animator->getSkeletonIndices();
        for (size_t i = 0; i < jList.size(); ++i)
        {
            osgPhysicsUtils::PlayerAnimation::ThisAndParent j = jList[i];
            jointMap[j.first] = j.second;
        }
        spineID = animator->getSkeletonJointIndex("Spine");
    }

    bool isAncestor(int joint, int ancestor)
    {
        int parent = jointMap[joint];
        if (joint < 0 || parent < 0) return false;
        else if (parent == joint || parent == ancestor) return true;
        else return isAncestor(parent, ancestor);
    }

    static float SetLowerBody(int joint, int parent, void* user)
    {
        PartialBlenderCreator* pbc = (PartialBlenderCreator*)user;
        return pbc->isAncestor(joint, pbc->spineID) ? 0.0f : 1.0f;
    }

    static float SetUpperBody(int joint, int parent, void* user)
    {
        PartialBlenderCreator* pbc = (PartialBlenderCreator*)user;
        return pbc->isAncestor(joint, pbc->spineID) ? 1.0f : 0.0f;
    }
};

class PlayerHandler : public osgGA::GUIEventHandler
{
public:
    enum ActionType
    {
        IDLE, WALKING, RUNNING, JUMPING, CROUCH, SAY_HELLO, BE_HIT,
        DYING, DYING_BACKWARDS, PISTOL_AIMING, SHOOTING, MAX_ANIMATIONS
    };

    PlayerHandler(osgPhysicsUtils::PlayerAnimation* pa, osg::MatrixTransform* p)
    :   _animator(pa), _playerRoot(p), _type(IDLE), _lastType(IDLE),
        _blendTime(0.0f), _blendTimeMax(0.0f), _rotation(0.0f), _pressingAction(0)
    {
        _loopFlags.resize(MAX_ANIMATIONS, false);
        _loopFlags[IDLE] = true; _loopFlags[WALKING] = true; _loopFlags[RUNNING] = true;
        _loopFlags[CROUCH] = true; _loopFlags[PISTOL_AIMING] = true;
    }
    
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(&aa);
        if (_pressingAction > 1)
        {
            // Turn left/right, changing player matrix
            _rotation += (_pressingAction == 2) ? -0.01f : 0.01f;
            _playerRoot->setMatrix(osg::Matrix::rotate(_rotation, osg::Y_AXIS));
        }

        if (_blendTime > 0.0f)
        {
            // Blend animations
            if (_blendTime < 0.02f) _blendTime = 0.0f;
            else _blendTime -= 0.02f;

            float blendFactor = _blendTime / _blendTimeMax;
            _animator->select(animations[_lastType], blendFactor, _loopFlags[_lastType]);
            _animator->select(animations[_type], 1.0f - blendFactor, _loopFlags[_type]);
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
            bool running = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT;
            switch (ea.getKey())
            {
            case 's': case 'S': case 'w': case 'W':
                switchAnimation(running ? RUNNING : WALKING, 0.2f);
                _pressingAction = 1; break;
            case 'a': case 'A': case 'd': case 'D':
                switchAnimation(running ? RUNNING : WALKING, 0.2f);
                _pressingAction = (ea.getKey() == 'a' || ea.getKey() == 'A') ? 2 : 3; break;
            case osgGA::GUIEventAdapter::KEY_Control_L:
                switchAnimation(CROUCH, 0.2f); _pressingAction = 1; break;
            case 'z': case 'Z':
                switchAnimation(JUMPING, 0.2f); _pressingAction = 1; break;
            case 'x': case 'X':
                switchAnimation(PISTOL_AIMING, 0.2f);
                _pressingAction = 1; break;
            case 'c': case 'C':
                switchAnimation(SHOOTING, 0.2f);
                _pressingAction = 1; break;
            }
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
        {
            if (_pressingAction > 0)
            {
                switchAnimation(IDLE, 0.2f);
                _pressingAction = 0;
            }

            switch (ea.getKey())
            {
            case osgGA::GUIEventAdapter::KEY_Page_Up:
                switchAnimation(_type == IDLE ? SHOOTING : (ActionType)((int)_type - 1));
                break;
            case osgGA::GUIEventAdapter::KEY_Page_Down:
                switchAnimation(_type == SHOOTING ? IDLE : (ActionType)((int)_type + 1));
                break;
            case osgGA::GUIEventAdapter::KEY_Home: setupPartialBlending(true); break;
            case osgGA::GUIEventAdapter::KEY_End: setupPartialBlending(false); break;
            }
        }
        return false;
    }

    void switchAnimation(ActionType newType, float blendTime = 0.5f)
    {
        if (_type == newType) return;
        OSG_NOTICE << "Switch animation from " << animations[_type]
                   << " to " << animations[newType] << std::endl;

        if (blendTime < 0.02f)
        {
            _lastType = _type; _type = newType;
            _animator->select(animations[_lastType], 0.0f, _loopFlags[_lastType]);
            _animator->select(animations[_type], 1.0f, _loopFlags[_type]);
        }
        else
        {
            _lastType = _type; _type = newType;
            _blendTime = blendTime; _blendTimeMax = blendTime;
        }
        _animator->seek(animations[_type], 0.0f);
    }

    void setupPartialBlending(bool started)
    {
        if (started)
        {
            PartialBlenderCreator pbc(_animator.get());
            _animator->selectPartial(animations[_type], 1.0f, _loopFlags[_type],
                                     PartialBlenderCreator::SetLowerBody, &pbc);
            _animator->selectPartial("pistol_aiming", 1.0f, true,
                                     PartialBlenderCreator::SetUpperBody, &pbc);
        }
        else
        {
            _animator->select(animations[_type], 1.0f, _loopFlags[_type]);
            _animator->select("pistol_aiming", 0.0f, true);
        }
        _animator->seek("idle", 0.0f);
        _animator->seek("pistol_aiming", 0.0f);
    }

protected:
    osg::observer_ptr<osgPhysicsUtils::PlayerAnimation> _animator;
    osg::observer_ptr<osg::MatrixTransform> _playerRoot;
    std::vector<bool> _loopFlags;
    ActionType _type, _lastType;
    float _blendTime, _blendTimeMax, _rotation;
    int _pressingAction;  // 1: any, 2: turning left, 3 - right
};

void loadAnimations(osgPhysicsUtils::PlayerAnimation* anim, const std::string& prefix)
{
    for (int i = 0; i < PlayerHandler::MAX_ANIMATIONS; ++i)
    {
        std::string name = animations[i];
        if (!anim->loadAnimation(name, prefix + name + ".ozz"))
        { OSG_WARN << "Unable to load animation " << name << std::endl; }
    }
}

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer;
    osg::ref_ptr<osg::Geode> player = new osg::Geode;

    osg::ref_ptr<osg::MatrixTransform> playerRoot = new osg::MatrixTransform;
    playerRoot->addChild(player.get());

    osg::ref_ptr<osg::MatrixTransform> root = new osg::MatrixTransform;
    root->setMatrix(osg::Matrix::rotate(osg::PI_2, osg::X_AXIS));
    root->addChild(playerRoot.get());

    osg::ref_ptr<osgPhysicsUtils::PlayerAnimation> animManager = new osgPhysicsUtils::PlayerAnimation;
    if (!animManager->initialize("exo_gray.skeleton.ozz", "exo_gray.mesh.ozz")) return 1;
    loadAnimations(animManager.get(), "animations/exo_gray@");
    animManager->select("idle", 1.0f, true);
    animManager->seek("idle", 0.0f);

#if 0
    std::vector<osgPhysicsUtils::PlayerAnimation::ThisAndParent> joints =
        animManager->getSkeletonIndices();
    for (size_t i = 0; i < joints.size(); ++i)
    {
        osgPhysicsUtils::PlayerAnimation::ThisAndParent p = joints[i];
        OSG_NOTICE << p.first << ": " << animManager->getSkeletonJointName(p.first)
                   << ", parent ID = " << p.second << std::endl;
    }
#endif

    viewer.setSceneData(root.get());
    viewer.addEventHandler(new PlayerHandler(animManager.get(), playerRoot.get()));
    viewer.setCameraManipulator(new osgGA::TrackballManipulator);
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.setUpViewInWindow(50, 50, 800, 600);

    bool createdPlayer = false;
    while (!viewer.done())
    {
        viewer.frame();
        if (viewer.getFrameStamp())
        {
            animManager->update(*viewer.getFrameStamp(), false);
            animManager->applyMeshes(*player, true);
            if (!createdPlayer)
            {
                viewer.getCameraManipulator()->home(0.0);
                createdPlayer = true;
            }
        }
    }
    return 0;
}