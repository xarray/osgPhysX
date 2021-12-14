#include <physics/Vehicle.h>
#include <physics/PhysicsUtil.h>
#include <physics/ParticleUpdater.h>
#include <physics/Callbacks.h>
#include <utils/InputManager.h>
#include <utils/FollowNodeManipulator.h>
#include <utils/SceneUtil.h>

#include <osg/ComputeBoundsVisitor>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ModularProgram>
#include <osgParticle/ParticleSystemUpdater>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>

class UpdateVehicleEventsCallback : public osg::NodeCallback
{
public:
    UpdateVehicleEventsCallback(osgPhysicsUtils::InputManager* m, osgPhysics::WheeledVehicle* veh,
        const osg::Matrix& pose)
        : _inputManager(m), _vehicle(veh), _initMatrix(pose), _lastSimulationTime(0.0f)
    {
        _vehicle->setAllowControllers(true);
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
        const osgGA::EventQueue::Events& events = ev->getEvents();
        float dt = nv->getFrameStamp()->getSimulationTime() - _lastSimulationTime;
        for (osgGA::EventQueue::Events::const_iterator itr = events.begin();
            itr != events.end(); ++itr)
        {
            osgGA::GUIEventAdapter* e = (*itr)->asGUIEventAdapter();
            handleEvent(node, *e, dt);
        }
        _lastSimulationTime = nv->getFrameStamp()->getSimulationTime();
        traverse(node, nv);
    }

    void handleEvent(osg::Node* node, const osgGA::GUIEventAdapter& ea, float dt)
    {
        osg::MatrixTransform* car = static_cast<osg::MatrixTransform*>(node);
        if (!car || !_vehicle) return;

        switch (ea.getEventType())
        {
        case osgGA::GUIEventAdapter::FRAME:
            controlByKeyboard(car, dt);
            break;
        }
        return;
    }

    void controlByKeyboard(osg::Group* car, float dt)
    {
        // Data from keyboard
        _inputManager->frame();
        if (_inputManager->getNumKeyboards() > 0)
        {
            if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Up)) _vehicle->accelerate(true);
            else _vehicle->accelerate(false);
            if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Down)) _vehicle->brake(true);
            else _vehicle->brake(false);
            if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Left)) _vehicle->steerRight(true);
            else _vehicle->steerRight(false);
            if (_inputManager->isKeyDown(osgGA::GUIEventAdapter::KEY_Right)) _vehicle->steerLeft(true);
            else _vehicle->steerLeft(false);

            if (_inputManager->isKeyDown('r'))
                _vehicle->resetPose(physx::PxTransform(osgPhysics::toPxMatrix(_initMatrix)));
        }
    }

protected:
    osg::observer_ptr<osgPhysicsUtils::InputManager> _inputManager;
    osg::ref_ptr<osgPhysics::WheeledVehicle> _vehicle;
    osg::Matrix _initMatrix;
    float _lastSimulationTime;
};

#define USE_6W_VEHICLE 0

osg::Group* createCar(osgPhysicsUtils::InputManager* manager, osgPhysics::UpdatePhysicsSystemCallback* physicsUpdater,
    const osg::Matrix& pose)
{
    float carMass = 1800.0f, wheelMass = 20.0f;
    float carWidth = 2.0f, carHeight = 0.5f, carLowerHeight = 0.2f, carLength = 3.0f;
    float wheelOffsetH = -0.3f, wheelOffsetW = 0.1f, wheelIndent = 0.3f;
    float wheelRadius = 0.2f, wheelWidth = 0.4f;

#if USE_6W_VEHICLE
    const unsigned int numWheels = 6;
#else
    const unsigned int numWheels = 4;
#endif

    osg::Vec3 wheelOffset[numWheels];
    wheelOffset[0].set(-(carWidth*0.5f - wheelOffsetW), wheelOffsetH, carLength*0.5f - wheelIndent);  // Front-left
    wheelOffset[1].set(carWidth*0.5f - wheelOffsetW, wheelOffsetH, carLength*0.5f - wheelIndent);  // Front-right
    wheelOffset[2].set(-(carWidth*0.5f - wheelOffsetW), wheelOffsetH, -(carLength*0.5f - wheelIndent));  // Rear-left
    wheelOffset[3].set(carWidth*0.5f - wheelOffsetW, wheelOffsetH, -(carLength*0.5f - wheelIndent));  // Rear-right
#if USE_6W_VEHICLE
    wheelOffset[4].set(-(carWidth*0.5f - wheelOffsetW), wheelOffsetH, 0.0f);  // Extra-left
    wheelOffset[5].set(carWidth*0.5f - wheelOffsetW, wheelOffsetH, 0.0f);  // Extra-left
#endif

    // Scene graph data
    osg::ref_ptr<osg::MatrixTransform> car = new osg::MatrixTransform;

    osg::ref_ptr<osg::MatrixTransform> wheel[numWheels];
    for (int i = 0; i < numWheels; ++i)
    {
        osg::ref_ptr<osg::Cylinder> wheelCylinder = new osg::Cylinder(osg::Vec3(), wheelRadius, wheelWidth);
        wheelCylinder->setRotation(osg::Quat(osg::PI_2, osg::Y_AXIS));  // care about different axes

        osg::ref_ptr<osg::Geode> wheelGeode = new osg::Geode;
        wheelGeode->addDrawable(new osg::ShapeDrawable(wheelCylinder.get()));

        // No need to set matrix of the wheel, it will be updated in the callback
        wheel[i] = new osg::MatrixTransform;
        wheel[i]->addChild(wheelGeode.get());
        car->addChild(wheel[i].get());
    }

    osg::ref_ptr<osg::Geode> chassisGeode = new osg::Geode;
    chassisGeode->addDrawable(new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(0.0f, carHeight*0.5f - carLowerHeight, 0.0f), carWidth, carHeight, carLength)));

    osg::ref_ptr<osg::MatrixTransform> chassis = new osg::MatrixTransform;
    chassis->addChild(chassisGeode.get());
    car->addChild(chassis.get());

    // Physics data
    osgPhysics::WheeledVehicle::MotorData motorData;
    motorData.engine.mPeakTorque = 1000.0f;
    motorData.engine.mMaxOmega = 1200.0f;
    motorData.gears.mFinalRatio = 1.0f;

    osgPhysics::WheeledVehicle::ChassisData chassisData;
    chassisData.mass = carMass;
    chassisData.mesh = osgPhysics::createBoxMesh(
        osg::Vec3(0.0f, carHeight*0.5f - carLowerHeight, 0.0f), osg::Vec3(carWidth, carHeight, carLength));
    chassisData.computeParameters();

    osgPhysics::WheeledVehicle::WheelData wheelData[numWheels];
    for (int i = 0; i < numWheels; ++i)
    {
        wheelData[i].offset = physx::PxVec3(wheelOffset[i][0], wheelOffset[i][1], wheelOffset[i][2]);
        wheelData[i].mass = wheelMass;
        wheelData[i].mesh = osgPhysics::createCylinderMesh(osg::Vec3(), wheelRadius, wheelWidth, 12);
        wheelData[i].computeParameters(chassisData, i < 2);
    }

#if USE_6W_VEHICLE
    osg::ref_ptr<osgPhysics::TruckVehicle> vehicle = new osgPhysics::TruckVehicle;
#else
    osg::ref_ptr<osgPhysics::CarVehicle> vehicle = new osgPhysics::CarVehicle;
#endif
    vehicle->create(motorData, chassisData, wheelData, NULL, NULL, true);
    osgPhysics::Engine::instance()->addActor("def", vehicle->getActor());
    physicsUpdater->addVehicle(vehicle.get());
    vehicle->resetPose(physx::PxTransform(osgPhysics::toPxMatrix(pose)));

    // Connect scene and physics object
    osg::ref_ptr<UpdateVehicleEventsCallback> updater = new UpdateVehicleEventsCallback(manager, vehicle.get(), pose);
    car->addUpdateCallback(new osgPhysics::UpdateVehicleCallback(vehicle.get()));
    car->addEventCallback(updater.get());
    //car->setMatrix( pose );  // No need to set matrix of the whole car
    vehicle->getActor()->setMaxAngularVelocity(12.0);
    return car.release();
}

#if !(PX_PHYSICS_VERSION_MAJOR > 3)
osgParticle::ParticleSystem* createParticleSystem(osg::Group* parent)
{
    // Set up the particle appearance
    osg::ref_ptr<osgParticle::ParticleSystem> ps = new osgParticle::ParticleSystem;
    ps->getDefaultParticleTemplate().setShape(osgParticle::Particle::POINT);

    osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
    blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
    texture->setImage(osgDB::readImageFile("Images/smoke.rgb"));

    osg::StateSet* ss = ps->getOrCreateStateSet();
    ss->setAttributeAndModes(blendFunc.get());
    ss->setTextureAttributeAndModes(0, texture.get());

    ss->setAttribute(new osg::Point(20.0f));
    ss->setTextureAttributeAndModes(0, new osg::PointSprite);

    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    osg::ref_ptr<osgParticle::RandomRateCounter> rrc = new osgParticle::RandomRateCounter;
    rrc->setRateRange(500, 800);

    // Set up the particle updater & native emitter/operator
    osgPhysics::ParticleUpdater::ParticleAttributes attributes;
    osg::ref_ptr<osgPhysics::ParticleUpdater> physicsParticleUpdater = new osgPhysics::ParticleUpdater;
    physicsParticleUpdater->create(attributes);
    //physicsParticleUpdater->setDataAccess( osgPhysics::ParticleUpdater::FLAGS_DATA|
    //    osgPhysics::ParticleUpdater::POSITION_DATA|osgPhysics::ParticleUpdater::VELOCITY_DATA );

    osgPhysics::VehicleManager::SurfaceType surface = osgPhysics::VehicleManager::SURFACE_TARMAC;
    osgPhysics::VehicleManager::FilterType filter = osgPhysics::VehicleManager::FILTER_OBSTACLE;
    osgPhysics::VehicleManager::instance()->addActor(
        "def", physicsParticleUpdater->getActor(), surface, filter, false);

    osg::ref_ptr<osgPhysics::NativeParticleEmitter> nativeEmitter =
        new osgPhysics::NativeParticleEmitter(physicsParticleUpdater.get());
    nativeEmitter->setParticleLifeTime(3.0f);
    nativeEmitter->setParticleSystem(ps.get());
    nativeEmitter->setCounter(rrc.get());

    osg::ref_ptr<osgPhysics::NativeParticleOperator> nativeOperator =
        new osgPhysics::NativeParticleOperator(physicsParticleUpdater.get());

    // Set up native particle/operator containers
    osg::ref_ptr<osgParticle::ModularProgram> program = new osgParticle::ModularProgram;
    program->setParticleSystem(ps.get());
    program->addOperator(nativeOperator.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(ps.get());

    parent->addChild(nativeEmitter.get());
    parent->addChild(program.get());
    parent->addChild(geode.get());
    return ps.get();
}
#endif

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer;

    // The scene and scene updater
    osgPhysics::Engine::instance()->addScene(
        "def", osgPhysics::VehicleManager::createScene(osg::Vec3(0.0f, 0.0f, -9.8f)));
    osg::ref_ptr<osgPhysics::UpdatePhysicsSystemCallback> physicsUpdater =
        new osgPhysics::UpdatePhysicsSystemCallback("def");
    physicsUpdater->setMaxSimuationDeltaTime(0.005);  // to fix jitter in slower system

    // The terrain mesh
    osg::ref_ptr<osg::MatrixTransform> terrain = new osg::MatrixTransform;
    terrain->addChild(osgDB::readNodeFile("lz.osg"));

    physx::PxTriangleMesh* terrainMesh = osgPhysics::createTriangleMesh(*terrain);
    physx::PxRigidActor* terrainActor = osgPhysics::createTriangleMeshActor(terrainMesh);

    osgPhysics::VehicleManager::SurfaceType surface = osgPhysics::VehicleManager::SURFACE_TARMAC;
    osgPhysics::VehicleManager::FilterType filter = osgPhysics::VehicleManager::FILTER_GROUND;
    osgPhysics::VehicleManager::instance()->addActor("def", terrainActor, surface, filter, true);

    // The actor
    osg::Vec3 initialPosition(0.0f, 0.0f, 93.0f);
    osg::ref_ptr<osgPhysicsUtils::InputManager> inputManager = new osgPhysicsUtils::InputManager;
    osg::ref_ptr<osg::Group> vehicle = createCar(inputManager.get(), physicsUpdater.get(),
        osg::Matrix::rotate(osg::PI_2, osg::X_AXIS) * osg::Matrix::translate(initialPosition));

    // A ball in the scene
    osg::ref_ptr<osg::MatrixTransform> ball = new osg::MatrixTransform;
    ball->addChild(osgPhysicsUtils::createGeode(osgPhysicsUtils::createEllipsoid(osg::Vec3(), 1.0f, 1.0f, 1.0f, 8)));

    physx::PxRigidActor* ballActor = (physx::PxRigidActor*)osgPhysics::createConvexMeshActor(
        osgPhysics::createConvexMesh(*ball, physx::PxConvexFlag::eCOMPUTE_CONVEX), 1.0f);
    ballActor->setGlobalPose(physx::PxTransform(
        osgPhysics::toPxMatrix(osg::Matrix::translate(initialPosition + osg::Z_AXIS * 80.0f))));
    ball->addUpdateCallback(new osgPhysics::UpdateActorCallback(ballActor));

    surface = osgPhysics::VehicleManager::SURFACE_TARMAC;
    filter = osgPhysics::VehicleManager::FILTER_OBSTACLE;
    osgPhysics::VehicleManager::instance()->addActor("def", ballActor, surface, filter, false);

    // The smoke particle
    osg::ref_ptr<osg::MatrixTransform> particleGroup = new osg::MatrixTransform;
    particleGroup->setMatrix(osg::Matrix::translate(initialPosition + osg::Z_AXIS * 5.0f));

#if !(PX_PHYSICS_VERSION_MAJOR > 3)
    osgParticle::ParticleSystem* ps = createParticleSystem(particleGroup.get());
    osg::ref_ptr<osgParticle::ParticleSystemUpdater> particleUpdater = new osgParticle::ParticleSystemUpdater;
    particleUpdater->addParticleSystem(ps);
#endif

    // Build the scene graph
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addUpdateCallback(physicsUpdater.get());
    root->addChild(terrain.get());
    root->addChild(vehicle.get());
    root->addChild(ball.get());
#if !(PX_PHYSICS_VERSION_MAJOR > 3)
    root->addChild(particleUpdater.get());
#endif
    root->addChild(particleGroup.get());

    // Start the viewer
    osg::ref_ptr<osgPhysicsUtils::FollowNodeManipulator> follower = new osgPhysicsUtils::FollowNodeManipulator;
    follower->setFixedRotation(true);
    follower->setLocalOffset(osg::Vec3(0.0f, 2.0f, -10.0f));  // in the coordinate of car chassis
    follower->setTrackNode(vehicle->getChild(vehicle->getNumChildren() - 1));

    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitch = new osgGA::KeySwitchMatrixManipulator;
    keyswitch->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator);
    keyswitch->addMatrixManipulator('2', "Follower", follower.get());

    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
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
