#include <physics/CharacterController.h>
#include <physics/PhysicsUtil.h>
#include <physics/Callbacks.h>
#include <utils/FollowNodeManipulator.h>
#include <utils/SceneUtil.h>

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
            if (_isDown) doWalkerMotion(view, true);
            _controller->move(_offset, _speed);
            return false;
        }

        switch (ea.getEventType())
        {
        case osgGA::GUIEventAdapter::PUSH:
            if (ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL)
            {
                doWalkerMotion(view, true);
                _isDown = true;
            }
            break;
        case osgGA::GUIEventAdapter::RELEASE:
            doWalkerMotion(view, false);
            _isDown = false;
            break;
        }
        return false;
    }

    void doWalkerMotion(osgViewer::View* view, bool isDown)
    {
        if (isDown)
        {
            osg::Vec3 eye, dir, up;
            view->getCamera()->getViewMatrixAsLookAt(eye, dir, up);
            dir = dir - eye;

            osg::Vec3 horizontalDir(dir.x(), dir.y(), 0.0f); horizontalDir.normalize();
            _offset.set(horizontalDir * _speed);
        }
        else
        {
            _offset.set(0.0f, 0.0f, 0.0f);
        }
    }

    WalkHandler(osgPhysics::CharacterController* c, float s)
        : _controller(c), _speed(s), _isDown(false) {}

    osgPhysics::CharacterController* _controller;
    osg::Vec3 _offset;
    float _speed;
    bool _isDown;
};

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer;

    // Test data
#if 0
    osg::Vec3 initialPosition(2.0f, 0.0f, 0.5f);
    float radius = 0.2f, height = 1.0f;
    float speed = 0.02f;

    osg::ref_ptr<osg::MatrixTransform> terrain = new osg::MatrixTransform;
    terrain->addChild(osgDB::readNodeFile("E:/NewWorld/resource/dam_models/in_room/718House.osg"));
#else
    osg::Vec3 initialPosition(0.0f, 0.0f, 100.0f);
    float radius = 0.5f, height = 2.0f;
    float speed = 0.2f;

    osg::ref_ptr<osg::MatrixTransform> terrain = new osg::MatrixTransform;
    terrain->addChild(osgDB::readNodeFile("lz.osg"));
#endif

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

    // Some obstacles
    /*osg::ref_ptr<osg::Group> obstacles = new osg::Group;
    for ( unsigned int i=0; i<10; ++i )
    {
        osg::Vec3 extent = osgPhysicsUtils::randomVector(osg::Vec3(1.0f,1.0f,1.0f), osg::Vec3(5.0f,5.0f,5.0f));
        osg::Vec3 pos = osgPhysicsUtils::randomVector(osg::Vec3(-50.0f,-50.0f,100.0f), osg::Vec3(50.0f,50.0f,100.0f));
        osg::ref_ptr<osg::ShapeDrawable> box = new osg::ShapeDrawable(
            new osg::Box(osg::Vec3(), extent[0], extent[1], extent[2]) );

        osg::ref_ptr<osg::Geode> boxGeode = new osg::Geode;
        boxGeode->addDrawable( box.get() );

        osg::ref_ptr<osg::MatrixTransform> boxTrans = new osg::MatrixTransform;
        boxTrans->addChild( boxGeode.get() );
        obstacles->addChild( boxTrans.get() );

        physx::PxRigidActor* boxActor = osgPhysics::createBoxActor( extent, 10.0 );
        boxActor->setGlobalPose( physx::PxTransform(
            osgPhysics::toPxMatrix(osg::Matrix::translate(pos))) );
        osgPhysics::Engine::instance()->addActor( "def", boxActor );
        boxTrans->addUpdateCallback( new osgPhysics::UpdateActorCallback(boxActor) );
    }*/

    // The actor
    osgPhysics::CharacterController::ControllerData controllerData(1.0f, initialPosition, osg::Z_AXIS);
    controllerData.stepOffset = 0.2f;  // to climb up stairs

    osg::ref_ptr<osgPhysics::CharacterController> controller = new osgPhysics::CharacterController;
    controller->createCapsule("def", radius, height, true, controllerData);

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
    //root->addChild( obstacles.get() );
    root->addChild(character.get());

    // Start the viewer
    osg::ref_ptr<osgPhysicsUtils::FollowNodeManipulator> follower = new osgPhysicsUtils::FollowNodeManipulator;
    follower->setLocalOffset(osg::Vec3(0.0f, 0.0f, height));
    follower->setTrackNode(character.get());

    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitch = new osgGA::KeySwitchMatrixManipulator;
    keyswitch->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator);
    keyswitch->addMatrixManipulator('2', "Follower", follower.get());

    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
    viewer.addEventHandler(new WalkHandler(controller.get(), speed));
    viewer.setCameraManipulator(keyswitch.get());
    viewer.setSceneData(root.get());
    viewer.setUpViewOnSingleScreen(0);
    return viewer.run();
}
