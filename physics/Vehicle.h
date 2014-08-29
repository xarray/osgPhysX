#ifndef PHYSICS_VEHICLE
#define PHYSICS_VEHICLE

#include <osg/Referenced>
#include "VehicleManager.h"

namespace osgPhysics
{

/** The wheeled vehicle object
    The wheel mesh must be set at the origin, with radii in YOZ, and width along X.
    The first wheel is always the left-front one, and the last is at right-rear.
*/
class WheeledVehicle : public osg::Referenced
{
public:
    struct ChassisData
    {
        physx::PxF32 mass;  // mass, must be set
        physx::PxConvexMesh* mesh;  // mesh data, must be set
        
        physx::PxVec3 dims;  // AABB box of the mesh, will be computed
        physx::PxVec3 moi;  // moment of inertia, will be computed
        physx::PxVec3 massOffsetCenter;  // center of mass offset, will be computed
        
        // Call this to compute specific parameters, or manually set them
        void computeParameters();
    };
    
    struct WheelData
    {
        physx::PxF32 mass;  // mass, must be set
        physx::PxVec3 offset;  // offset from center, must be set
        physx::PxConvexMesh* mesh;  // mesh data, must be set
        
        physx::PxReal moi;  // moment of inertia, will be computed
        physx::PxReal radius, width;  // wheel size, will be computed
        physx::PxReal dampingRate;  // damping rate, will be set to default while computing
        physx::PxReal maxBrakeTorque;  // max brake torque in Nm, will be set to default while computing
        physx::PxReal maxHandBrakeTorque;  // max handbrake torque in Nm, will be set to default while computing
        physx::PxReal maxSteer;  // max steer angle in radians, will be set to default while computing
        physx::PxReal toeAngle;  // wheel toe angle in radians [0, PI/2], will be set to default while computing
        physx::PxVec3 wheelCenterCMOffsets;  // will be set to default while computing
        physx::PxVec3 suspensionForceAppCMOffsets;  // will be set to default while computing
        physx::PxVec3 tireForceAppCMOffsets;  // will be set to default while computing
        physx::PxVehicleTireData tireData;  // tire data, will be set to default while computing
        physx::PxVehicleSuspensionData suspensionData;  // suspension data, will be set to default while computing
        
        // Call this to compute specific parameters, or manually set them
        void computeParameters( const ChassisData& chassis, bool asFrontWheel );
    };
    
    struct MotorData
    {
        physx::PxVehicleEngineData engine;
        physx::PxVehicleGearsData gears;
        physx::PxVehicleClutchData clutch;
        physx::PxVehicleAutoBoxData autoBox;
        physx::PxVehicleAckermannGeometryData ackermann4W;
        physx::PxVehicleDifferential4WData differential4W;
#if USE_PHYSX_33
        physx::PxVehicleDifferentialNWData differentialNW;
#endif
    };
    
    WheeledVehicle( int numWheels );
    
    virtual physx::PxShape* getWheelShape( int i ) { return _wheelShapes[i]; }
    virtual const physx::PxShape* getWheelShape( int i ) const { return _wheelShapes[i]; }
    
    physx::PxShape* getChassisShape() { return _chassisShape; }
    const physx::PxShape* getChassisShape() const { return _chassisShape; }
    
    physx::PxRigidDynamic* getActor() { return _actor; }
    const physx::PxRigidDynamic* getActor() const { return _actor; }
    
    physx::PxVehicleDrive* getDriveEngine() { return _drive; }
    const physx::PxVehicleDrive* getDriveEngine() const { return _drive; }
    
#if USE_PHYSX_33
    physx::PxVehicleWheelQueryResult& getQueryResult() { return _vehicleQueryResult; }
    const physx::PxVehicleWheelQueryResult& getQueryResult() const { return _vehicleQueryResult; }
#endif

    /** Reset the car pose */
    virtual void resetPose( const physx::PxTransform& startTransform );
    
    /** Handle control inputs every frame */
    virtual void handleInputs( double step ) = 0;
    
    /** Clear all inputs */
    virtual void clearInputs() = 0;
    
    /** Accelerate */
    virtual void accelerate( bool b ) = 0;
    virtual void accelerate( float value ) = 0;
    
    /** Brake */
    virtual void brake( bool b ) = 0;
    virtual void brake( float value ) = 0;
    
    /** Hand brake */
    virtual void handBrake( bool b ) = 0;
    virtual void handBrake( float value ) = 0;
    
    /** Gear up/down */
    virtual void gearUp( bool b ) = 0;
    virtual void gearDown( bool b ) = 0;
    
    /** Steer left/right */
    virtual void steerLeft( bool b ) = 0;
    virtual void steerRight( bool b ) = 0;
    virtual void steer( float value ) = 0;
    
    /** Set the steer speed table, must be sized 2 * 8 */
    void setSpeedTable( const std::vector<float>& table, unsigned int numUsed );
    
    /** Set allow user controllers (digital/analog) or not */
    void setAllowControllers( bool b ) { _allowControllers = b; }
    bool getAllowControllers() const { return _allowControllers; }
    
    /** Obtain global pose of every part of the car (usually 4 wheels and the chassis) */
    unsigned int getComponentTransforms( std::vector<physx::PxTransform>& transforms ) const;
    
    /** Compute the speed of the car */
    double computeForwardSpeed() const { return _drive->computeForwardSpeed(); }
    double computeSideSpeed() const { return _drive->computeSidewaysSpeed(); }
    
    /** Compute engine rotation speed */
    double computeRotationSpeed() const;
    
    /** Get current gear level */
    const char* getCurrentGear() const;
    
    /** Check if the car (index < 0) or one of its wheel [0, numWheels-1] is in the air or not */
    bool inAir( int i=-1 ) const;
    
protected:
    virtual physx::PxVehicleWheelsSimData* createWheelsSimData(
        WheelData* wheels, physx::PxVec3* suspensionTravelDirs, int numWheels );
    virtual physx::PxRigidDynamic* createActorAndFilters(
        ChassisData& chassis, physx::PxMaterial* chassisMtl,
        const physx::PxGeometry** wheelGeometries, physx::PxMaterial* wheelMtl, int numWheels );
    virtual void swapAccelerationAndBrake() {}
    virtual void handleAutoGearMode( double step, bool accelRaw, bool brakeRaw, bool handbrakeRaw );
    
    physx::PxShape* _chassisShape;
    physx::PxRigidDynamic* _actor;
    physx::PxVehicleDrive* _drive;
    std::vector<physx::PxShape*> _wheelShapes;
    
#if USE_PHYSX_33
    std::vector<physx::PxWheelQueryResult> _wheelQueryResult;
    physx::PxVehicleWheelQueryResult _vehicleQueryResult;
#endif
    
    physx::PxFixedSizeLookupTable<8> _speedTable;
    bool _reverseMode, _movingForwardSlowly;
    bool _analogMode, _allowControllers;
};

/** The 4-wheeled car vehicle */
class CarVehicle : public WheeledVehicle
{
public:
    CarVehicle();
    
    virtual void resetPose( const physx::PxTransform& startTransform );
    virtual void handleInputs( double step );
    virtual void clearInputs();
    
    virtual void accelerate( bool b ) { _controlData.setDigitalAccel(b); _analogMode = false; }
    virtual void accelerate( float value ) { _controlData.setAnalogAccel(value); _analogMode = true; }
    
    virtual void brake( bool b ) { _controlData.setDigitalBrake(b); _analogMode = false; }
    virtual void brake( float value ) { _controlData.setAnalogBrake(value); _analogMode = true; }
    
    virtual void handBrake( bool b ) { _controlData.setDigitalHandbrake(b); _analogMode = false; }
    virtual void handBrake( float value ) { _controlData.setAnalogHandbrake(value); _analogMode = true; }
    
    virtual void gearUp( bool b ) { _controlData.setGearUp(b); }
    virtual void gearDown( bool b ) { _controlData.setGearDown(b); }
    
    virtual void steerLeft( bool b ) { _controlData.setDigitalSteerLeft(b); _analogMode = false; }
    virtual void steerRight( bool b ) { _controlData.setDigitalSteerRight(b); _analogMode = false; }
    virtual void steer( float value ) { _controlData.setAnalogSteer(value); _analogMode = true; }
    
    /** Create the car using user-defined chassis and wheel data */
    bool create( MotorData& motor, ChassisData& chassis, WheelData wheels[4],
                 physx::PxMaterial* chassisMtl=0, physx::PxMaterial* wheelMtl=0, bool autoGears=true );
    
protected:
    virtual ~CarVehicle();
    virtual void swapAccelerationAndBrake();
    
    physx::PxVehicleDrive4WRawInputData _controlData;
};

#if USE_PHYSX_33

/** The 6-wheeled car vehicle */
class TruckVehicle : public WheeledVehicle
{
public:
    TruckVehicle();
    
    virtual void resetPose( const physx::PxTransform& startTransform );
    virtual void handleInputs( double step );
    virtual void clearInputs();
    
    virtual void accelerate( bool b ) { _controlData.setDigitalAccel(b); _analogMode = false; }
    virtual void accelerate( float value ) { _controlData.setAnalogAccel(value); _analogMode = true; }
    
    virtual void brake( bool b ) { _controlData.setDigitalBrake(b); _analogMode = false; }
    virtual void brake( float value ) { _controlData.setAnalogBrake(value); _analogMode = true; }
    
    virtual void handBrake( bool b ) { _controlData.setDigitalHandbrake(b); _analogMode = false; }
    virtual void handBrake( float value ) { _controlData.setAnalogHandbrake(value); _analogMode = true; }
    
    virtual void gearUp( bool b ) { _controlData.setGearUp(b); }
    virtual void gearDown( bool b ) { _controlData.setGearDown(b); }
    
    virtual void steerLeft( bool b ) { _controlData.setDigitalSteerLeft(b); _analogMode = false; }
    virtual void steerRight( bool b ) { _controlData.setDigitalSteerRight(b); _analogMode = false; }
    virtual void steer( float value ) { _controlData.setAnalogSteer(value); _analogMode = true; }
    
    /** Create the car using user-defined chassis and wheel data */
    bool create( MotorData& motor, ChassisData& chassis, WheelData wheels[6],
                 physx::PxMaterial* chassisMtl=0, physx::PxMaterial* wheelMtl=0, bool autoGears=true );
    
protected:
    virtual ~TruckVehicle();
    virtual void swapAccelerationAndBrake();
    virtual void setupExtraWheels( physx::PxVehicleWheelsSimData* dataNW, physx::PxVehicleWheelsSimData* data4W );
    
    physx::PxVehicleDriveNWRawInputData _controlData;
};

#endif

}

#endif
