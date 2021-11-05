#include <utils/SceneUtil.h>
#include <utils/InputManager.h>
#include <utils/PlayerAnimation.h>

#include <osg/io_utils>
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
        _blendTime(0.0f), _blendTimeMax(0.0f), _rotation(0.0f), _pressingAction(0),
        _createdPlayer(false)
    {
        _loopFlags.resize(MAX_ANIMATIONS, false);
        _loopFlags[IDLE] = true; _loopFlags[WALKING] = true; _loopFlags[RUNNING] = true;
        _loopFlags[CROUCH] = true; _loopFlags[PISTOL_AIMING] = true;

        _attachmentOffset = osg::Matrix::rotate(osg::PI_2, osg::Y_AXIS)
                          * osg::Matrix::translate(-0.02f, -0.02f, 0.05f);
        _attachmentID = _animator->getSkeletonJointIndex("RightHandMiddle1");
        _ikTarget = osg::Vec3(0.0f, 2.0f, 1.0f); _ikOffset = osg::Vec3();
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
            case osgGA::GUIEventAdapter::KEY_Insert: setupIK(true, 0); break;
            case osgGA::GUIEventAdapter::KEY_Delete: setupIK(false, 0); break;
            case '0': if (!_ikChain.empty()) setupIK(true, 0); break;
            case '1': if (!_ikChain.empty()) setupIK(true, 1); break;
            case '2': if (!_ikChain.empty()) setupIK(true, 2); break;
            case '3': if (!_ikChain.empty()) setupIK(true, 3); break;
            case '4': if (!_ikChain.empty()) setupIK(true, 4); break;
            case osgGA::GUIEventAdapter::KEY_Tab: setupAttachment(); break;
            }
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
        {
            _ikTarget.x() = ea.getXnormalized() * 2.0f;  // Model-space targetX in [-2, 2]
            _ikTarget.y() = ea.getYnormalized() + 1.0f;  // Model-space targetY in [0, 2]
            if (_target.valid()) _target->setMatrix(osg::Matrix::translate(_ikTarget));
        }

        if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
        {
            // Update player data
            osg::Geode* player = _playerRoot->getChild(0)->asGeode();
            if (player && view->getFrameStamp())
            {
                _animator->update(*view->getFrameStamp(), false);
                if (!_ikChain.empty())  // Update IK after model-space data calculated
                {
                    if (_ikAction > 0 && _ikChain.size() > 2)
                    {
                        bool reached = false; _ikTarget.z() = 0.3f;  // Try to reach the target
                        osg::Vec3 midAxis = -osg::X_AXIS;  // local-space knee
                        if (_ikAction == 1) midAxis = osg::Z_AXIS;
                        else if (_ikAction == 2) midAxis = -osg::Z_AXIS;
                        _animator->updateTwoBoneIK(
                            _ikTarget, _ikChain[0].joint, _ikChain[1].joint, _ikChain[2].joint,
                            reached, 1.0f, 1.0f, 0.0f, midAxis);
                    }
                    else
                    {
                        _ikTarget.z() = 1.0f;  // Not too close while looking at the target
                        _animator->updateAimIK(_ikTarget, _ikChain, _ikOffset);
                    }
                }

                if (_attachment.valid())  // Update attachment after model-space data calculated
                {
                    osg::Matrix m = _animator->getModelSpaceJointMatrix(_attachmentID);
                    _attachment->setMatrix(_attachmentOffset * m);
                }

                _animator->applyMeshes(*player, true);
                if (!_createdPlayer)
                {
                    view->getCameraManipulator()->home(0.0);
                    _createdPlayer = true;
                }
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

    void setupIK(bool started, int ikAction)
    {
        _ikAction = ikAction;
        _ikChain.clear();
        if (started)
        {
            switch (_ikAction)
            {
            case 0:
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("Head"), 0.5f, osg::Y_AXIS, osg::Z_AXIS });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("Neck"), 0.5f, osg::Y_AXIS, osg::Z_AXIS });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("Spine2"), 0.5f, osg::Y_AXIS, osg::Z_AXIS });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("Spine"), 0.5f, osg::Y_AXIS, osg::Z_AXIS });
                break;
            case 1:
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("LeftShoulder") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("LeftForeArm") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("LeftHandMiddle1") });
                break;
            case 2:
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("RightShoulder") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("RightForeArm") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("RightHandMiddle1") });
                break;
            case 3:
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("LeftUpLeg") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("LeftLeg") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("LeftToeBase") });
                break;
            case 4:
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("RightUpLeg") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("RightLeg") });
                _ikChain.push_back(osgPhysicsUtils::PlayerAnimation::JointIkData{
                    _animator->getSkeletonJointIndex("RightToeBase") });
                break;
            }

            if (!_target)
            {
                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 0.05f)));
                _target = new osg::MatrixTransform;
                _target->addChild(geode.get());
                _playerRoot->addChild(_target.get());
            }
        }
        else
        {
            _playerRoot->removeChild(_target.get());
            _target = NULL;
        }
    }

    void setupAttachment()
    {
        if (_attachment.valid())
        {
            _playerRoot->removeChild(_attachment.get());
            _attachment = NULL;
        }
        else if (_attachmentID >= 0)
        {
            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 0.2f, 0.03f, 0.03f)));
            _attachment = new osg::MatrixTransform;
            _attachment->addChild(geode.get());
            _playerRoot->addChild(_attachment.get());
        }
    }

protected:
    osg::observer_ptr<osgPhysicsUtils::PlayerAnimation> _animator;
    osg::observer_ptr<osg::MatrixTransform> _playerRoot;
    osg::ref_ptr<osg::MatrixTransform> _attachment, _target;
    osg::Matrix _attachmentOffset;
    osg::Vec3 _ikTarget, _ikOffset;

    std::vector<osgPhysicsUtils::PlayerAnimation::JointIkData> _ikChain;
    std::vector<bool> _loopFlags;
    ActionType _type, _lastType;
    float _blendTime, _blendTimeMax, _rotation;
    int _attachmentID, _ikAction;  // 0: head, 1.2 - left/right arm, 3/4 - left/right leg
    int _pressingAction;  // 0, none, 1: any, 2: turning left, 3 - right
    bool _createdPlayer;
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

#if 1
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

    while (!viewer.done()) viewer.frame();
    return 0;
}