#include <osg/io_utils>
#include "Vehicle.h"
#include <algorithm>
#include <iostream>

#include <vehicle/PxVehicleUtil.h>
using namespace osgPhysics;
using namespace physx;

#define SDK_OBJ (Engine::instance()->getPhysicsSDK())
#define SDK_COOK (Engine::instance()->getOrCreateCooking())

// The default speed table
PxF32 g_speedDataTable[2 * 8] =
{
    0.0f,       0.75f,
    5.0f,       0.75f,
    30.0f,      0.125f,
    120.0f,     0.1f,
    PX_MAX_F32, PX_MAX_F32,
    PX_MAX_F32, PX_MAX_F32,
    PX_MAX_F32, PX_MAX_F32,
    PX_MAX_F32, PX_MAX_F32
};

// The default control smoothing data
PxVehicleKeySmoothingData g_smoothingData =
{
    {
        3.0f,    //rise rate eANALOG_INPUT_ACCEL        
        3.0f,    //rise rate eANALOG_INPUT_BRAKE        
        10.0f,   //rise rate eANALOG_INPUT_HANDBRAKE    
        2.5f,    //rise rate eANALOG_INPUT_STEER_LEFT    
        2.5f,    //rise rate eANALOG_INPUT_STEER_RIGHT    
    },
    {
        5.0f,    //fall rate eANALOG_INPUT_ACCEL        
        5.0f,    //fall rate eANALOG_INPUT_BRAKE        
        10.0f,   //fall rate eANALOG_INPUT_HANDBRAKE    
        5.0f,    //fall rate eANALOG_INPUT_STEER_LEFT    
        5.0f     //fall rate eANALOG_INPUT_STEER_RIGHT    
    }
};

PxVehiclePadSmoothingData g_smoothingDataPad =
{
    {
        6.0f,    //rise rate eANALOG_INPUT_ACCEL        
        6.0f,    //rise rate eANALOG_INPUT_BRAKE        
        12.0f,   //rise rate eANALOG_INPUT_HANDBRAKE    
        2.5f,    //rise rate eANALOG_INPUT_STEER_LEFT    
        2.5f,    //rise rate eANALOG_INPUT_STEER_RIGHT    
    },
    {
        10.0f,    //fall rate eANALOG_INPUT_ACCEL        
        10.0f,    //fall rate eANALOG_INPUT_BRAKE        
        12.0f,    //fall rate eANALOG_INPUT_HANDBRAKE    
        5.0f,     //fall rate eANALOG_INPUT_STEER_LEFT    
        5.0f      //fall rate eANALOG_INPUT_STEER_RIGHT    
    }
};

/* WheeledVehicle::ChassisData */

void WheeledVehicle::ChassisData::computeParameters()
{
    const PxU32 numVerts = mesh->getNbVertices();
    const PxVec3* verts = mesh->getVertices();
    PxVec3 min(PX_MAX_F32, PX_MAX_F32, PX_MAX_F32);
    PxVec3 max(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32);
    for (PxU32 i = 0; i < numVerts; ++i)
    {
        min.x = PxMin(min.x, verts[i].x);
        min.y = PxMin(min.y, verts[i].y);
        min.z = PxMin(min.z, verts[i].z);
        max.x = PxMax(max.x, verts[i].x);
        max.y = PxMax(max.y, verts[i].y);
        max.z = PxMax(max.z, verts[i].z);
    }

    dims = max - min;
    moi = PxVec3((dims.y * dims.y + dims.z * dims.z) * mass / 12.0f,
        (dims.x * dims.x + dims.z * dims.z) * mass / 12.0f,
        (dims.x * dims.x + dims.y * dims.y) * mass / 12.0f);
    moi.y *= 0.8f;  // The car will have more responsive turning if we reduce the y-component

    // Set mass center to be below the mesh center and a little towards the front
    massOffsetCenter = PxVec3(0.0f, -dims.y * 0.5f + 0.65f, 0.25f);
}

/* WheeledVehicle::WheelData */

void WheeledVehicle::WheelData::computeParameters(const ChassisData& chassis, bool asFrontWheel)
{
    const PxU32 numVerts = mesh->getNbVertices();
    const PxVec3* verts = mesh->getVertices();
    PxVec3 min(PX_MAX_F32, PX_MAX_F32, PX_MAX_F32);
    PxVec3 max(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32);
    for (PxU32 i = 0; i < numVerts; ++i)
    {
        min.x = PxMin(min.x, verts[i].x);
        min.y = PxMin(min.y, verts[i].y);
        min.z = PxMin(min.z, verts[i].z);
        max.x = PxMax(max.x, verts[i].x);
        max.y = PxMax(max.y, verts[i].y);
        max.z = PxMax(max.z, verts[i].z);
    }

    width = max.x - min.x;
    radius = PxMax(max.y, max.z) * 0.975f;
    moi = 0.5f * mass * radius * radius;

    // Work out the front/rear mass split from the center of mass offset
    PxF32 massRear = 0.5f * chassis.mass * (chassis.dims.z - 3.0f * chassis.massOffsetCenter.z) / chassis.dims.z;
    PxF32 massFront = chassis.mass - massRear;
    if (asFrontWheel)
    {
        maxHandBrakeTorque = 0.0f;
        maxSteer = PxPi * 0.3333f;
        tireData.mType = VehicleManager::TIRE_SLICKS;
        suspensionData.mSprungMass = massFront * 0.5f;
    }
    else
    {
        maxHandBrakeTorque = 4000.0f;
        maxSteer = 0.0f;
        tireData.mType = VehicleManager::TIRE_WETS;
        suspensionData.mSprungMass = massRear * 0.5f;
    }

    maxBrakeTorque = 1500.0f;
    dampingRate = 0.25f;
    toeAngle = 0.0f;
    suspensionData.mMaxCompression = 0.3f;
    suspensionData.mMaxDroop = 0.1f;
    suspensionData.mSpringStrength = 85000.0f;
    suspensionData.mSpringDamperRate = 4500.0f;

    wheelCenterCMOffsets = offset - chassis.massOffsetCenter;
    suspensionForceAppCMOffsets = PxVec3(wheelCenterCMOffsets.x, -0.3f, wheelCenterCMOffsets.z);
    tireForceAppCMOffsets = PxVec3(wheelCenterCMOffsets.x, -0.3f, wheelCenterCMOffsets.z);
}

/* WheeledVehicle */

WheeledVehicle::WheeledVehicle(int numWheels)
    : _chassisShape(NULL), _actor(NULL), _reverseMode(false), _movingForwardSlowly(false),
    _analogMode(false), _allowControllers(true)
{
    VehicleManager::instance();
    _speedTable = PxFixedSizeLookupTable<8>(g_speedDataTable, 4);
    _wheelShapes.resize(numWheels);

    _wheelQueryResult.resize(numWheels);
    _vehicleQueryResult.nbWheelQueryResults = numWheels;
    _vehicleQueryResult.wheelQueryResults = &(_wheelQueryResult[0]);
}

void WheeledVehicle::resetPose(const PxTransform& startTransform)
{
    // Set the car's transform to be the start transform
    PxRigidDynamic* actor = _drive->getRigidDynamicActor();
    actor->setGlobalPose(startTransform);
    _reverseMode = false;
    _movingForwardSlowly = false;
}

void WheeledVehicle::setSpeedTable(const std::vector<float>& table, unsigned int numUsed)
{
    _speedTable = PxFixedSizeLookupTable<8>(&(table[0]), numUsed);
}

unsigned int WheeledVehicle::getComponentTransforms(std::vector<PxTransform>& transforms) const
{
    std::vector<PxShape*> shapes(_actor->getNbShapes());
    unsigned int size = _actor->getShapes(&(shapes[0]), _actor->getNbShapes());
    for (unsigned int i = 0; i < size; ++i)
    {
        PxTransform t = PxShapeExt::getGlobalPose(*(shapes[i]), *_actor);
        transforms.push_back(t);
    }
    return size;
}

double WheeledVehicle::computeRotationSpeed() const
{
    PxVehicleDriveDynData& dynamicData = _drive->mDriveDynData;
    return dynamicData.getEngineRotationSpeed();
}

const char* WheeledVehicle::getCurrentGear() const
{
    unsigned int gear = _drive->mDriveDynData.getCurrentGear();
    switch (gear)
    {
    case PxVehicleGearsData::eREVERSE: return "R";
    case PxVehicleGearsData::eNEUTRAL: return "N";
    case PxVehicleGearsData::eFIRST: return "1";
    case PxVehicleGearsData::eSECOND: return "2";
    case PxVehicleGearsData::eTHIRD: return "3";
    case PxVehicleGearsData::eFOURTH: return "4";
    case PxVehicleGearsData::eFIFTH: return "5";
    case PxVehicleGearsData::eSIXTH: return "6";
    case PxVehicleGearsData::eSEVENTH: return "7";
    case PxVehicleGearsData::eEIGHTH: return "8";
    default: return "";
    }
}

bool WheeledVehicle::inAir(int i) const
{
    return i < 0 ? physx::PxVehicleIsInAir(_vehicleQueryResult)
        : _vehicleQueryResult.wheelQueryResults[i].isInAir;
}

PxVehicleWheelsSimData* WheeledVehicle::createWheelsSimData(WheeledVehicle::WheelData* wheels,
    PxVec3* suspensionTravelDirs, int numWheels)
{
    PxVehicleWheelsSimData* wheelsSimData = PxVehicleWheelsSimData::allocate(numWheels);
    std::vector<PxVehicleWheelData> wheelData(numWheels);
    for (int i = 0; i < numWheels; ++i)
    {
        wheelData[i].mRadius = wheels[i].radius;
        wheelData[i].mMass = wheels[i].mass;
        wheelData[i].mMOI = wheels[i].moi;
        wheelData[i].mWidth = wheels[i].width;
        wheelData[i].mDampingRate = wheels[i].dampingRate;
        wheelData[i].mMaxBrakeTorque = wheels[i].maxBrakeTorque;
        wheelData[i].mMaxHandBrakeTorque = wheels[i].maxHandBrakeTorque;
        wheelData[i].mMaxSteer = wheels[i].maxSteer;
        wheelData[i].mToeAngle = wheels[i].toeAngle;

        wheelsSimData->setWheelData(i, wheelData[i]);
        wheelsSimData->setTireData(i, wheels[i].tireData);
        wheelsSimData->setSuspensionData(i, wheels[i].suspensionData);
        wheelsSimData->setSuspTravelDirection(i, suspensionTravelDirs[i]);
        wheelsSimData->setWheelCentreOffset(i, wheels[i].wheelCenterCMOffsets);
        wheelsSimData->setSuspForceAppPointOffset(i, wheels[i].suspensionForceAppCMOffsets);
        wheelsSimData->setTireForceAppPointOffset(i, wheels[i].tireForceAppCMOffsets);
    }

    // Set the car to perform 3 sub-steps when it moves with a forwards speed of less than 5.0
    // and with a single step when it moves at speed greater than or equal to 5.0.
    // These are the default settings and we won't change it at the moment.
    wheelsSimData->setSubStepCount(5.0f, 3, 1);
    return wheelsSimData;
}

physx::PxRigidDynamic* WheeledVehicle::createActorAndFilters(
    WheeledVehicle::ChassisData& chassis, PxMaterial* chassisMtl,
    const PxGeometry** wheelGeometries, PxMaterial* wheelMtl, int numWheels)
{
    // Create the actor
    physx::PxRigidDynamic* actor = SDK_OBJ->createRigidDynamic(PxTransform(PxIdentity));
    if (!actor) return false;

    actor->setMass(chassis.mass);
    actor->setMassSpaceInertiaTensor(chassis.moi);
    actor->setCMassLocalPose(PxTransform(chassis.massOffsetCenter, PxQuat(PxIdentity)));

    // Set up the physx rigid body actor with shapes, local poses, and filters
    PxMaterial* defMaterial = VehicleManager::instance()->getSurfaceMaterial(VehicleManager::SURFACE_TARMAC);
    for (int i = 0; i < numWheels; ++i)
    {

#if (PX_PHYSICS_VERSION_MAJOR > 3)
        _wheelShapes[i] = PxRigidActorExt::createExclusiveShape(*actor, *(wheelGeometries[i]), wheelMtl ? *wheelMtl : *defMaterial);
        actor->attachShape(*_wheelShapes[i]);
#else
        _wheelShapes[i] = actor->createShape(*(wheelGeometries[i]), wheelMtl ? *wheelMtl : *defMaterial);
#endif
        
        _wheelShapes[i]->setLocalPose(PxTransform(PxIdentity));

        // Create a query filter data for the car to ensure that cars do not attempt to drive on themselves
        VehicleManager::createFilter(VehicleManager::FILTER_UNDRIVABLE_SURFACE, _wheelShapes[i]);
        VehicleManager::createFilter(VehicleManager::FILTER_WHEEL, _wheelShapes[i]);
    }

    // Must first create wheels and then chassis because PhysX will treat the first shape as the wheel
#if (PX_PHYSICS_VERSION_MAJOR > 3)
    _chassisShape = PxRigidActorExt::createExclusiveShape(*actor, PxConvexMeshGeometry(chassis.mesh), chassisMtl ? *chassisMtl : *defMaterial);
    actor->attachShape(*_chassisShape);
#else
    _chassisShape = actor->createShape(PxConvexMeshGeometry(chassis.mesh), chassisMtl ? *chassisMtl : *defMaterial);
#endif
    _chassisShape->setLocalPose(PxTransform(PxIdentity));
    VehicleManager::createFilter(VehicleManager::FILTER_UNDRIVABLE_SURFACE, _chassisShape);
    VehicleManager::createFilter(VehicleManager::FILTER_CHASSIS, _chassisShape);
    return actor;
}

void WheeledVehicle::handleAutoGearMode(double step, bool accelRaw, bool brakeRaw, bool handbrakeRaw)
{
    // Handle auto gears
    static const PxF32 s_forwardSpeedThreshold = 0.4f;
    static const PxF32 s_sideSpeedThreshold = 0.6f;
    static const PxF32 s_backwardSpeedThreshold = 0.4f;
    PxVehicleDriveDynData& dynamicData = _drive->mDriveDynData;

    bool toggleAutoReverse = false, currentMovingForwardSlowly = false, currentMovingBackward = false;
    if (!inAir())
    {
        PxF32 forwardSpeed = _drive->computeForwardSpeed(), sideSpeed = _drive->computeSidewaysSpeed();
        PxF32 forwardSpeedABS = PxAbs(forwardSpeed), sideSpeedABS = PxAbs(sideSpeed);
        PxU32 currentGear = dynamicData.getCurrentGear(), targetGear = dynamicData.getTargetGear();

        // Check if the car is rolling against the gear
        // (backwards in forward gear or forwards in reverse gear)
        if (currentGear == PxVehicleGearsData::eFIRST && forwardSpeed < -s_backwardSpeedThreshold)
            currentMovingBackward = true;
        else if (currentGear == PxVehicleGearsData::eREVERSE && forwardSpeed > s_backwardSpeedThreshold)
            currentMovingBackward = true;

        // Check if the car is moving slowly
        if (forwardSpeedABS < s_forwardSpeedThreshold && sideSpeedABS < s_sideSpeedThreshold)
            currentMovingForwardSlowly = true;

        // Now find if we need to toggle from forwards gear to reverse gear or vice versa
        if (currentMovingBackward)
        {
            if (!accelRaw && !brakeRaw && !handbrakeRaw && currentGear == targetGear)
                toggleAutoReverse = true;
        }
        else if (_movingForwardSlowly && currentMovingForwardSlowly)
        {
            // The car was moving slowly in forward gear without player input,
            // and is now moving slowly with player input (or vice versa)
            // This indicates the player wants to switch to reverse gear
            if (currentGear > PxVehicleGearsData::eNEUTRAL && brakeRaw && !accelRaw && currentGear == targetGear)
                toggleAutoReverse = true;
            else if (currentGear == PxVehicleGearsData::eREVERSE && accelRaw && !brakeRaw && currentGear == targetGear)
                toggleAutoReverse = true;
        }

        // If the car was brought to rest through braking then the player needs to release the brake,
        // then reapply to indicate that the gears should toggle between reverse and forward
        if (accelRaw || brakeRaw || handbrakeRaw)
            currentMovingForwardSlowly = false;
    }
    _movingForwardSlowly = currentMovingForwardSlowly;

    if (toggleAutoReverse)
    {
        _reverseMode = !_reverseMode;
        if (_reverseMode) dynamicData.forceGearChange(PxVehicleGearsData::eREVERSE);
        else dynamicData.forceGearChange(PxVehicleGearsData::eFIRST);
    }
}

/* CarVehicle */

CarVehicle::CarVehicle()
    : WheeledVehicle(4)
{
    _drive = PxVehicleDrive4W::allocate(4);
}

CarVehicle::~CarVehicle()
{
    PxVehicleDrive4W* drive4W = (PxVehicleDrive4W*)_drive;
    drive4W->free();
}

void CarVehicle::resetPose(const physx::PxTransform& startTransform)
{
    // Set the car back to its rest state and first gear
    PxVehicleDrive4W* drive4W = (PxVehicleDrive4W*)_drive;
    if (!drive4W) return;

    drive4W->setToRestState();
    drive4W->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);
    WheeledVehicle::resetPose(startTransform);
}

void CarVehicle::handleInputs(double step)
{
    PxVehicleDriveDynData& dynamicData = _drive->mDriveDynData;
    if (dynamicData.getUseAutoGears())
    {
        if (_controlData.getGearUp() || _controlData.getGearDown())
        {
            _controlData.setGearUp(false);
            _controlData.setGearDown(false);
        }

        bool accelRaw = _analogMode ? (_controlData.getAnalogAccel() > 0.0f) : _controlData.getDigitalAccel();
        bool brakeRaw = _analogMode ? (_controlData.getAnalogBrake() > 0.0f) : _controlData.getDigitalBrake();
        bool handbrakeRaw = _analogMode ? (_controlData.getAnalogHandbrake() > 0.0f) : _controlData.getDigitalHandbrake();
        handleAutoGearMode(step, accelRaw, brakeRaw, handbrakeRaw);
    }

    PxVehicleDrive4W* drive4W = (PxVehicleDrive4W*)_drive;
    if (!drive4W || !_allowControllers) return;

    if (_reverseMode) swapAccelerationAndBrake();
    if (_analogMode)
    {
        PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs(
            g_smoothingDataPad, _speedTable, _controlData, step, inAir(), *drive4W);
    }
    else
    {
        PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs(
            g_smoothingData, _speedTable, _controlData, step, inAir(), *drive4W);
    }
    if (_reverseMode) swapAccelerationAndBrake();
}

void CarVehicle::clearInputs()
{
    _controlData.setDigitalAccel(false);
    _controlData.setAnalogAccel(0.0f);
    _controlData.setDigitalBrake(false);
    _controlData.setAnalogBrake(0.0f);
    _controlData.setDigitalHandbrake(false);
    _controlData.setAnalogHandbrake(0.0f);
    _controlData.setGearUp(false);
    _controlData.setGearDown(false);
    _controlData.setDigitalSteerLeft(false);
    _controlData.setDigitalSteerRight(false);
    _controlData.setAnalogSteer(0.0f);
    _reverseMode = false;
    _movingForwardSlowly = false;
}

bool CarVehicle::create(WheeledVehicle::MotorData& motor, WheeledVehicle::ChassisData& chassis,
    WheeledVehicle::WheelData wheels[4],
    PxMaterial* chassisMtl, PxMaterial* wheelMtl, bool autoGears)
{
    if (_actor)
    {
        OSG_NOTICE << "[CarVehicle] Vehicle already created" << std::endl;
        return false;
    }

    // Setup necessary chassis, wheel and motor parameters
    PxVehicleChassisData chassisData;
    chassisData.mMass = chassis.mass;
    chassisData.mMOI = chassis.moi;
    chassisData.mCMOffset = chassis.massOffsetCenter;

    PxVec3 suspensionTravelDirs[4] = { PxVec3(0,-1,0), PxVec3(0,-1,0), PxVec3(0,-1,0), PxVec3(0,-1,0) };
    PxVehicleWheelsSimData* wheelsSimData = createWheelsSimData(wheels, suspensionTravelDirs, 4);

    PxVehicleDriveSimData4W driveSimData;
    driveSimData.setDiffData(motor.differential4W);
    driveSimData.setEngineData(motor.engine);
    driveSimData.setGearsData(motor.gears);
    driveSimData.setClutchData(motor.clutch);
    driveSimData.setAutoBoxData(motor.autoBox);

    motor.ackermann4W.mAxleSeparation = wheels[PxVehicleDrive4WWheelOrder::eFRONT_LEFT].wheelCenterCMOffsets.z
        - wheels[PxVehicleDrive4WWheelOrder::eREAR_LEFT].wheelCenterCMOffsets.z;
    motor.ackermann4W.mFrontWidth = wheels[PxVehicleDrive4WWheelOrder::eFRONT_RIGHT].wheelCenterCMOffsets.x
        - wheels[PxVehicleDrive4WWheelOrder::eFRONT_LEFT].wheelCenterCMOffsets.x;
    motor.ackermann4W.mRearWidth = wheels[PxVehicleDrive4WWheelOrder::eREAR_RIGHT].wheelCenterCMOffsets.x
        - wheels[PxVehicleDrive4WWheelOrder::eREAR_LEFT].wheelCenterCMOffsets.x;
    driveSimData.setAckermannGeometryData(motor.ackermann4W);

    // Add wheel collision shapes, local poses, material and a simulation filter for the wheels.
    PxConvexMeshGeometry frontLeftWheelGeom(wheels[PxVehicleDrive4WWheelOrder::eFRONT_LEFT].mesh);
    PxConvexMeshGeometry frontRightWheelGeom(wheels[PxVehicleDrive4WWheelOrder::eFRONT_RIGHT].mesh);
    PxConvexMeshGeometry rearLeftWheelGeom(wheels[PxVehicleDrive4WWheelOrder::eREAR_LEFT].mesh);
    PxConvexMeshGeometry rearRightWheelGeom(wheels[PxVehicleDrive4WWheelOrder::eREAR_RIGHT].mesh);

    const PxGeometry* wheelGeometries[4] = {
        &frontLeftWheelGeom, &frontRightWheelGeom,
        &rearLeftWheelGeom, &rearRightWheelGeom
    };

    _actor = createActorAndFilters(chassis, chassisMtl, wheelGeometries, wheelMtl, 4);
    if (!_actor)
    {
        wheelsSimData->free();
        return false;
    }

    // Setup the drive object
    PxVehicleDrive4W* drive4W = (PxVehicleDrive4W*)_drive;
    drive4W->setup(SDK_OBJ, _actor, *wheelsSimData, driveSimData, 0);
    drive4W->mDriveDynData.setUseAutoGears(autoGears);
    wheelsSimData->free();

    // Set up the mapping between wheel and actor shape
    for (int i = 0; i < 4; ++i) drive4W->mWheelsSimData.setWheelShapeMapping(i, i);

    // Set up the scene query filter data for each suspension line
    PxFilterData queryFilterData = VehicleManager::createFilter(VehicleManager::FILTER_UNDRIVABLE_SURFACE);
    for (int i = 0; i < 4; ++i) drive4W->mWheelsSimData.setSceneQueryFilterData(i, queryFilterData);
    return true;
}

void CarVehicle::swapAccelerationAndBrake()
{
    if (_analogMode)
    {
        float accelRaw = _controlData.getAnalogAccel();
        float brakeRaw = _controlData.getAnalogBrake();
        _controlData.setAnalogAccel(brakeRaw);
        _controlData.setAnalogBrake(accelRaw);
    }
    else
    {
        bool accelRaw = _controlData.getDigitalAccel();
        bool brakeRaw = _controlData.getDigitalBrake();
        _controlData.setDigitalAccel(brakeRaw);
        _controlData.setDigitalBrake(accelRaw);
    }
}

/* TruckVehicle */
TruckVehicle::TruckVehicle()
    : WheeledVehicle(6)
{
    _drive = PxVehicleDriveNW::allocate(6);
}

TruckVehicle::~TruckVehicle()
{
    PxVehicleDriveNW* driveNW = (PxVehicleDriveNW*)_drive;
    driveNW->free();
}

void TruckVehicle::resetPose(const physx::PxTransform& startTransform)
{
    // Set the car back to its rest state and first gear
    PxVehicleDriveNW* driveNW = (PxVehicleDriveNW*)_drive;
    if (!driveNW) return;

    driveNW->setToRestState();
    driveNW->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);
    WheeledVehicle::resetPose(startTransform);
}

void TruckVehicle::handleInputs(double step)
{
    PxVehicleDriveDynData& dynamicData = _drive->mDriveDynData;
    if (dynamicData.getUseAutoGears())
    {
        if (_controlData.getGearUp() || _controlData.getGearDown())
        {
            _controlData.setGearUp(false);
            _controlData.setGearDown(false);
        }

        bool accelRaw = _analogMode ? (_controlData.getAnalogAccel() > 0.0f) : _controlData.getDigitalAccel();
        bool brakeRaw = _analogMode ? (_controlData.getAnalogBrake() > 0.0f) : _controlData.getDigitalBrake();
        bool handbrakeRaw = _analogMode ? (_controlData.getAnalogHandbrake() > 0.0f) : _controlData.getDigitalHandbrake();
        handleAutoGearMode(step, accelRaw, brakeRaw, handbrakeRaw);
    }

    PxVehicleDriveNW* driveNW = (PxVehicleDriveNW*)_drive;
    if (!driveNW || !_allowControllers) return;

    if (_reverseMode) swapAccelerationAndBrake();
    if (_analogMode)
    {
        PxVehicleDriveNWSmoothAnalogRawInputsAndSetAnalogInputs(
            g_smoothingDataPad, _speedTable, _controlData, step, inAir(), *driveNW);
    }
    else
    {
        PxVehicleDriveNWSmoothDigitalRawInputsAndSetAnalogInputs(
            g_smoothingData, _speedTable, _controlData, step, inAir(), *driveNW);
    }
    if (_reverseMode) swapAccelerationAndBrake();
}

void TruckVehicle::clearInputs()
{
    _controlData.setDigitalAccel(false);
    _controlData.setAnalogAccel(0.0f);
    _controlData.setDigitalBrake(false);
    _controlData.setAnalogBrake(0.0f);
    _controlData.setDigitalHandbrake(false);
    _controlData.setAnalogHandbrake(0.0f);
    _controlData.setGearUp(false);
    _controlData.setGearDown(false);
    _controlData.setDigitalSteerLeft(false);
    _controlData.setDigitalSteerRight(false);
    _controlData.setAnalogSteer(0.0f);
    _reverseMode = false;
    _movingForwardSlowly = false;
}

bool TruckVehicle::create(WheeledVehicle::MotorData& motor, WheeledVehicle::ChassisData& chassis,
    WheeledVehicle::WheelData wheels[6],
    PxMaterial* chassisMtl, PxMaterial* wheelMtl, bool autoGears)
{
    if (_actor)
    {
        OSG_NOTICE << "[TruckVehicle] Vehicle already created" << std::endl;
        return false;
    }

    // Setup necessary chassis, wheel and motor parameters
    PxVehicleChassisData chassisData;
    chassisData.mMass = chassis.mass;
    chassisData.mMOI = chassis.moi;
    chassisData.mCMOffset = chassis.massOffsetCenter;

    PxVec3 suspensionTravelDirs[4] = { PxVec3(0,-1,0), PxVec3(0,-1,0), PxVec3(0,-1,0), PxVec3(0,-1,0) };
    PxVehicleWheelsSimData* wheelsSimData4W = createWheelsSimData(wheels, suspensionTravelDirs, 4);

    // Copy 4-wheel vehicle wheel data to 6-wheel truck
    PxVehicleWheelsSimData* wheelsSimData = PxVehicleWheelsSimData::allocate(6);
    for (int i = 0; i < 4; ++i) wheelsSimData->copy(*wheelsSimData4W, i, i);
    setupExtraWheels(wheelsSimData, wheelsSimData4W);
    wheelsSimData4W->free();

    for (int i = 0; i < 6; ++i)
        motor.differentialNW.setDrivenWheel(i, true);

    PxVehicleDriveSimDataNW driveSimData;
    driveSimData.setDiffData(motor.differentialNW);
    driveSimData.setEngineData(motor.engine);
    driveSimData.setGearsData(motor.gears);
    driveSimData.setClutchData(motor.clutch);
    driveSimData.setAutoBoxData(motor.autoBox);

    // Add wheel collision shapes, local poses, material and a simulation filter for the wheels.
    PxConvexMeshGeometry frontLeftWheelGeom(wheels[0].mesh);
    PxConvexMeshGeometry frontRightWheelGeom(wheels[1].mesh);
    PxConvexMeshGeometry rearLeftWheelGeom(wheels[2].mesh);
    PxConvexMeshGeometry rearRightWheelGeom(wheels[3].mesh);
    PxConvexMeshGeometry extraWheelGeom0(wheels[4].mesh);
    PxConvexMeshGeometry extraWheelGeom1(wheels[5].mesh);

    const PxGeometry* wheelGeometries[6] = {
        &frontLeftWheelGeom, &frontRightWheelGeom,
        &rearLeftWheelGeom, &rearRightWheelGeom,
        &extraWheelGeom0, &extraWheelGeom1
    };

    _actor = createActorAndFilters(chassis, chassisMtl, wheelGeometries, wheelMtl, 6);
    if (!_actor)
    {
        wheelsSimData->free();
        return false;
    }

    // Setup the drive object
    PxVehicleDriveNW* driveNW = (PxVehicleDriveNW*)_drive;
    driveNW->setup(SDK_OBJ, _actor, *wheelsSimData, driveSimData, 6);
    driveNW->mDriveDynData.setUseAutoGears(autoGears);
    wheelsSimData->free();

    // Set up the mapping between wheel and actor shape
    for (int i = 0; i < 6; ++i)
        driveNW->mWheelsSimData.setWheelShapeMapping(i, i);

    // Set up the scene query filter data for each suspension line
    PxFilterData queryFilterData = VehicleManager::createFilter(VehicleManager::FILTER_UNDRIVABLE_SURFACE);
    for (int i = 0; i < 6; ++i)
        driveNW->mWheelsSimData.setSceneQueryFilterData(i, queryFilterData);
    return true;
}

void TruckVehicle::swapAccelerationAndBrake()
{
    if (_analogMode)
    {
        float accelRaw = _controlData.getAnalogAccel();
        float brakeRaw = _controlData.getAnalogBrake();
        _controlData.setAnalogAccel(brakeRaw);
        _controlData.setAnalogBrake(accelRaw);
    }
    else
    {
        bool accelRaw = _controlData.getDigitalAccel();
        bool brakeRaw = _controlData.getDigitalBrake();
        _controlData.setDigitalAccel(brakeRaw);
        _controlData.setDigitalBrake(accelRaw);
    }
}

void TruckVehicle::setupExtraWheels(physx::PxVehicleWheelsSimData* wheelsSimData,
    physx::PxVehicleWheelsSimData* wheelsSimData4W)
{
    wheelsSimData->copy(*wheelsSimData4W, 0, 4);
    wheelsSimData->copy(*wheelsSimData4W, 1, 5);
    wheelsSimData->setTireLoadFilterData(wheelsSimData4W->getTireLoadFilterData());

    // Make sure that the two non-driven wheels don't respond to the handbrake
    PxVehicleWheelData wd = wheelsSimData->getWheelData(4);
    wd.mMaxHandBrakeTorque = 0.0f;
    wheelsSimData->setWheelData(4, wd);

    wd = wheelsSimData->getWheelData(5);
    wd.mMaxHandBrakeTorque = 0.0f;
    wheelsSimData->setWheelData(5, wd);

    // We've now got a 6-wheeled vehicle but the offsets of the 2 extra wheels are still incorrect
    // Set up the 2 extra wheels to lie on an axle that goes through the centre of the car
    // and is parallel to the front and rear axles
    PxVec3 w = wheelsSimData4W->getWheelCentreOffset(0);
    w.z = 0.5f * (wheelsSimData->getWheelCentreOffset(0).z + wheelsSimData->getWheelCentreOffset(2).z);
    wheelsSimData->setWheelCentreOffset(4, w);

    w = wheelsSimData4W->getWheelCentreOffset(1);
    w.z = 0.5f * (wheelsSimData->getWheelCentreOffset(1).z + wheelsSimData->getWheelCentreOffset(3).z);
    wheelsSimData->setWheelCentreOffset(5, w);

    w = wheelsSimData4W->getSuspForceAppPointOffset(0);
    w.z = 0.5f * (wheelsSimData->getSuspForceAppPointOffset(0).z + wheelsSimData->getSuspForceAppPointOffset(2).z);
    wheelsSimData->setSuspForceAppPointOffset(4, w);

    w = wheelsSimData4W->getSuspForceAppPointOffset(1);
    w.z = 0.5f * (wheelsSimData->getSuspForceAppPointOffset(1).z + wheelsSimData->getSuspForceAppPointOffset(3).z);
    wheelsSimData->setSuspForceAppPointOffset(5, w);

    w = wheelsSimData4W->getTireForceAppPointOffset(0);
    w.z = 0.5f * (wheelsSimData->getTireForceAppPointOffset(0).z + wheelsSimData->getTireForceAppPointOffset(2).z);
    wheelsSimData->setTireForceAppPointOffset(4, w);

    w = wheelsSimData4W->getTireForceAppPointOffset(1);
    w.z = 0.5f * (wheelsSimData->getTireForceAppPointOffset(1).z + wheelsSimData->getTireForceAppPointOffset(3).z);
    wheelsSimData->setTireForceAppPointOffset(5, w);

    // The first 4 wheels were all set up to support a mass M but now that mass is
    // distributed between 6 wheels. Adjust the suspension springs accordingly
    // and try to preserve the natural frequency and damping ratio of the springs
    for (int i = 0; i < 4; ++i)
    {
        PxVehicleSuspensionData suspData = wheelsSimData->getSuspensionData(i);
        suspData.mSprungMass *= 0.6666f;
        suspData.mSpringStrength *= 0.666f;
        suspData.mSpringDamperRate *= 0.666f;
        wheelsSimData->setSuspensionData(i, suspData);
    }

    const PxF32 sprungMass = wheelsSimData->getSuspensionData(0).mSprungMass
        + wheelsSimData->getSuspensionData(3).mSprungMass;
    const PxF32 strength = wheelsSimData->getSuspensionData(0).mSpringStrength
        + wheelsSimData->getSuspensionData(3).mSpringStrength;
    const PxF32 damperRate = wheelsSimData->getSuspensionData(0).mSpringDamperRate
        + wheelsSimData->getSuspensionData(3).mSpringDamperRate;

    PxVehicleSuspensionData suspData4 = wheelsSimData->getSuspensionData(4);
    suspData4.mSprungMass = sprungMass * 0.5f;
    suspData4.mSpringStrength = strength * 0.5f;
    suspData4.mSpringDamperRate = damperRate * 0.5f;
    wheelsSimData->setSuspensionData(4, suspData4);

    PxVehicleSuspensionData suspData5 = wheelsSimData->getSuspensionData(5);
    suspData5.mSprungMass = sprungMass * 0.5f;
    suspData5.mSpringStrength = strength * 0.5f;
    suspData5.mSpringDamperRate = damperRate * 0.5f;
    wheelsSimData->setSuspensionData(5, suspData5);
}
