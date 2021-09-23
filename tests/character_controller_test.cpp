#include <physics/CharacterController.h>
#include <physics/PhysicsUtil.h>
#include <physics/Callbacks.h>
#include <utils/FollowNodeManipulator.h>
#include <utils/SceneUtil.h>
#include <utils/InputManager.h>

#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>

class WalkHandler : public osgGA::GUIEventHandler
{
public:
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(&aa);
        if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
        {
            _inputManager->frame();
            if (_inputManager->getNumKeyboards() > 0)
            {
                if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Up)) _inputValue |= 0x1;
                else _inputValue &= (~0x1);
                if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Down)) _inputValue |= 0x2;
                else _inputValue &= (~0x2);
                if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Left)) _inputValue |= 0x4;
                else _inputValue &= (~0x4);
                if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Right)) _inputValue |= 0x8;
                else _inputValue &= (~0x8);
            }
            else _inputValue = 0;

            doWalkerMotion(view, _inputValue);
            _controller->move(_offset, _speed);
            return false;
        }
        return false;
    }

    void doWalkerMotion(osgViewer::View* view, int value)
    {
        if (value > 0)
        {
            osg::Vec3 eye, dir, up, side;
            view->getCamera()->getViewMatrixAsLookAt(eye, dir, up);
            dir = dir - eye; side = up ^ dir;

            osg::Vec3 horizontalDir;
            if (value & 0x1) horizontalDir += osg::Vec3(dir.x(), dir.y(), 0.0f);
            if (value & 0x2) horizontalDir += osg::Vec3(-dir.x(), -dir.y(), 0.0f);
            if (value & 0x4) horizontalDir += osg::Vec3(side.x(), side.y(), 0.0f);
            if (value & 0x8) horizontalDir += osg::Vec3(-side.x(), -side.y(), 0.0f);
            
            horizontalDir.normalize();
            _offset.set(horizontalDir * _speed);
        }
        else
            _offset.set(0.0f, 0.0f, 0.0f);
    }

    WalkHandler(osgPhysicsUtils::InputManager* im, osgPhysics::CharacterController* c, float s)
        : _inputManager(im), _controller(c), _speed(s), _inputValue(0) {}

    osg::observer_ptr<osgPhysicsUtils::InputManager> _inputManager;
    osg::observer_ptr<osgPhysics::CharacterController> _controller;
    osg::Vec3 _offset;
    float _speed;
    int _inputValue;
};

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer;

    osg::ref_ptr<osg::MatrixTransform> terrain = new osg::MatrixTransform;
    terrain->addChild(osgDB::readNodeFile("lz.osg"));

    // Add the ground
    osg::ComputeBoundsVisitor cbv;
    terrain->accept(cbv);

    osg::BoundingBox bb = cbv.getBoundingBox();
    float w = 2.0f * (bb.xMax() - bb.xMin()), h = 2.0f * (bb.yMax() - bb.yMin());
    terrain->addChild(osgPhysicsUtils::createQuad(
        osg::Vec3(bb.center().x() - w * 0.5f, bb.center().y() - h * 0.5f, bb.zMin()), w, h));

    // The scene and scene updater
    osgPhysics::Engine::instance()->addScene(
        "def", osgPhysics::createScene(osg::Vec3(0.0f, 0.0f, -9.8f)));
    osg::ref_ptr<osgPhysics::UpdatePhysicsSystemCallback> physicsUpdater =
        new osgPhysics::UpdatePhysicsSystemCallback("def");
    physicsUpdater->setMaxSimuationDeltaTime(0.005);  // to fix jitter in slower system

    // The terrain mesh
    physx::PxTriangleMesh* terrainMesh = osgPhysics::createTriangleMesh(*terrain);
    physx::PxRigidActor* terrainActor = osgPhysics::createTriangleMeshActor(terrainMesh);
    osgPhysics::Engine::instance()->addActor("def", terrainActor);

    // The actor
    osg::Vec3 initialPosition(0.0f, 0.0f, 100.0f);
    float radius = 0.5f, height = 2.0f, speed = 0.1f;

    osgPhysics::CharacterController::ControllerData controllerData(1.0f, initialPosition, osg::Z_AXIS);
    controllerData.stepOffset = 0.2f;  // to climb up stairs

    osg::ref_ptr<osgPhysics::CharacterController> controller = new osgPhysics::CharacterController;
    controller->createCapsule("def", radius, height, true, controllerData);
    
    physx::PxBoxObstacle airWalls[4];
    airWalls[0].mPos = osgPhysics::toPxVec3d(osg::Vec3d(0.0, -100.0, 0.0));
    airWalls[0].mHalfExtents = osgPhysics::toPxVec3(osg::Vec3(100.0f, 0.1f, 100.0f));
    airWalls[1].mPos = osgPhysics::toPxVec3d(osg::Vec3d(0.0, 100.0, 0.0));
    airWalls[1].mHalfExtents = osgPhysics::toPxVec3(osg::Vec3(100.0f, 0.1f, 100.0f));
    airWalls[2].mPos = osgPhysics::toPxVec3d(osg::Vec3d(-100.0, 0.0, 0.0));
    airWalls[2].mHalfExtents = osgPhysics::toPxVec3(osg::Vec3(0.1f, 100.0f, 100.0f));
    airWalls[3].mPos = osgPhysics::toPxVec3d(osg::Vec3d(100.0, 0.0, 0.0));
    airWalls[3].mHalfExtents = osgPhysics::toPxVec3(osg::Vec3(0.1f, 100.0f, 100.0f));
    for (int i = 0; i < 4; ++i) controller->updateObstacle(0/*new obstacle*/, airWalls[i]);

    osg::ref_ptr<osg::ShapeDrawable> capsule = new osg::ShapeDrawable(
        new osg::Capsule(osg::Vec3(), radius, height*0.5f));

    osg::ref_ptr<osg::Geode> characterGeode = new osg::Geode;
    characterGeode->addDrawable(capsule.get());

    osg::ref_ptr<osg::MatrixTransform> character = new osg::MatrixTransform;
    character->addChild(characterGeode.get());
    character->addUpdateCallback(new osgPhysics::UpdateCharacterCallback(controller.get()));

    // Build the scene graph
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addUpdateCallback(physicsUpdater.get());
    root->addChild(terrain.get());
    root->addChild(character.get());

    // Start the viewer
    osg::ref_ptr<osgPhysicsUtils::FollowNodeManipulator> follower = new osgPhysicsUtils::FollowNodeManipulator;
    follower->setLocalOffset(osg::Vec3(0.0f, -radius * 15.0f, height));
    follower->setTrackNode(character.get());

    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitch = new osgGA::KeySwitchMatrixManipulator;
    keyswitch->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator);
    keyswitch->addMatrixManipulator('2', "Follower", follower.get());

    osg::ref_ptr<osgPhysicsUtils::InputManager> inputManager = new osgPhysicsUtils::InputManager;
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
    viewer.addEventHandler(new WalkHandler(inputManager.get(), controller.get(), speed));
    viewer.setCameraManipulator(keyswitch.get());
    viewer.setSceneData(root.get());
    viewer.setUpViewOnSingleScreen(0);

    osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(
        viewer.getCamera()->getGraphicsContext());
    if (gw)
    {
        // Send window size event for scene objects to initialize
        int x = 0, y = 0, w = 1920, h = 1080;
        gw->getWindowRectangle(x, y, w, h);
        viewer.getEventQueue()->windowResize(x, y, w, h);

        // Initialize the buffered input manager
        std::vector<std::string> inputParameters;
        inputManager->initialize(gw, inputParameters);
    }
    return viewer.run();
}
