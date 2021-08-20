#include <physics/PhysicsUtil.h>
#include <physics/Callbacks.h>
#include <utils/SceneUtil.h>

#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>

class ShootBoxHandler : public osgGA::GUIEventHandler
{
public:
    ShootBoxHandler(osg::Group* r) : _root(r) {}
    osg::observer_ptr<osg::Group> _root;
    
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(&aa);
        osg::Matrix invMV = view->getCamera()->getInverseViewMatrix();
        osg::Vec3 start = osg::Vec3() * invMV, target = -osg::Z_AXIS * invMV;
        target = (target - start); target.normalize(); target *= 50.0f;

        switch (ea.getEventType())
        {
        case osgGA::GUIEventAdapter::KEYUP:
            if (ea.getKey() == '1' && _root.valid())
            {
                physx::PxRigidActor* actor = osgPhysics::createBoxActor(osg::Vec3(1.0f, 1.0f, 1.0f), 5.0);
                actor->setGlobalPose(physx::PxTransform(osgPhysics::toPxMatrix(
                    osg::Matrix::translate(start))));
                
                physx::PxRigidDynamic* dynActor = actor->is<physx::PxRigidDynamic>();
                dynActor->setLinearVelocity(physx::PxVec3(target[0], target[1], target[2]));
                osgPhysics::Engine::instance()->addActor("def", actor);

                osg::ref_ptr<osg::MatrixTransform> mt = dynamic_cast<osg::MatrixTransform*>(
                    osgPhysics::createNodeForActor(actor));
                mt->addUpdateCallback(new osgPhysics::UpdateActorCallback(actor));
                _root->addChild(mt.get());
            }
            break;
        }
        return false;
    }
};

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer;

    // The scene and scene updater
    osgPhysics::Engine::instance()->addScene(
        "def", osgPhysics::createScene(osg::Vec3(0.0f, 0.0f, -9.8f)));
    osg::ref_ptr<osgPhysics::UpdatePhysicsSystemCallback> physicsUpdater =
        new osgPhysics::UpdatePhysicsSystemCallback("def");
    physicsUpdater->setMaxSimuationDeltaTime(0.005);  // to fix jitter in slower system

    // Build the scene graph
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addUpdateCallback(physicsUpdater.get());

    // Create the ground
    physx::PxRigidActor* planeActor = osgPhysics::createPlaneActor(osg::Plane(osg::Z_AXIS, 0.0));
    osgPhysics::Engine::instance()->addActor("def", planeActor);

    // Create many cubes
    for (int y = 0; y < 10; ++y)
        for (int x = 0; x < 10; ++x)
        {
            physx::PxRigidActor* actor = osgPhysics::createBoxActor(osg::Vec3(1.0f, 1.0f, 1.0f), 1.0);
            actor->setGlobalPose(physx::PxTransform(osgPhysics::toPxMatrix(
                osg::Matrix::translate(1.05f * (float)x, 0.0f, 1.5f * (float)y))));
            osgPhysics::Engine::instance()->addActor("def", actor);

            osg::ref_ptr<osg::MatrixTransform> mt = dynamic_cast<osg::MatrixTransform*>(
                osgPhysics::createNodeForActor(actor));
            mt->addUpdateCallback(new osgPhysics::UpdateActorCallback(actor));
            root->addChild(mt.get());
        }

    // Start the viewer
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
    viewer.addEventHandler(new ShootBoxHandler(root.get()));
    viewer.setSceneData(root.get());
    viewer.setUpViewOnSingleScreen(0);
    return viewer.run();
}
