#ifndef PHYSICS_CALLBACKS
#define PHYSICS_CALLBACKS

#include <osg/observer_ptr>
#include <osg/NodeCallback>
#include "Engine.h"

namespace physx
{

class PxVehicleWheels;
class PxRigidActor;

};

namespace osgPhysics
{

class CharacterController;
class WheeledVehicle;

/** The callback to update the entire physics system, should be applied to the root node */
class UpdatePhysicsSystemCallback : public osg::NodeCallback
{
public:
    UpdatePhysicsSystemCallback( const std::string& sceneName="" )
    : _sceneName(sceneName), _numTotalWheels(0), _maxSimulationDelta(0.0), _lastSimulationTime(0.0), _frameTime(0.02) {}
    
    UpdatePhysicsSystemCallback( const UpdatePhysicsSystemCallback& copy, const osg::CopyOp& op=osg::CopyOp::SHALLOW_COPY )
    :   osg::NodeCallback(copy, op), _sceneName(copy._sceneName), _numTotalWheels(copy._numTotalWheels),
        _maxSimulationDelta(copy._maxSimulationDelta), _frameTime(copy._frameTime) {}
    
    META_Object( osgPhysics, UpdatePhysicsSystemCallback );
    
    void addVehicle( WheeledVehicle* vehicle );
    void computeTotalWheels();
    
    std::vector<WheeledVehicle*>& getVehicles() { return _vehicles; }
    const std::vector<WheeledVehicle*>& getVehicles() const { return _vehicles; }
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );
    
    void setMaxSimuationDeltaTime( double t ) { _maxSimulationDelta = t; }
    double getMaxSimuationDeltaTime() const { return _maxSimulationDelta; }
    
    void setFrameTime( double t ) { _frameTime = t; }
    double getFrameTime() const { return _frameTime; }
    
protected:
    std::vector<WheeledVehicle*> _vehicles;
    std::vector<physx::PxVehicleWheels*> _vehicleEngines;
#if USE_PHYSX_33
    std::vector<physx::PxVehicleWheelQueryResult> _queryResults;
#endif
    
    std::string _sceneName;
    unsigned int _numTotalWheels;
    double _maxSimulationDelta;
    double _lastSimulationTime;
    double _frameTime;
};

/** The callback to update the actor, should be applied to a matrix transform node */
class UpdateActorCallback : public osg::NodeCallback
{
public:
    UpdateActorCallback( physx::PxRigidActor* a=0 ) : _actor(a) {}
    
    UpdateActorCallback( const UpdateActorCallback& copy, const osg::CopyOp& op=osg::CopyOp::SHALLOW_COPY )
    : osg::NodeCallback(copy, op), _actor(copy._actor) {}
    
    META_Object( osgPhysics, UpdateActorCallback );
    
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );
    
protected:
    physx::PxRigidActor* _actor;
};

/** The callback to update the character controller */
class UpdateCharacterCallback : public osg::NodeCallback
{
public:
    UpdateCharacterCallback( CharacterController* c=0 )
    : _character(c), _lastSimulationTime(0.0), _frameTime(0.02) {}
    
    UpdateCharacterCallback( const UpdateCharacterCallback& copy, const osg::CopyOp& op=osg::CopyOp::SHALLOW_COPY )
    : osg::NodeCallback(copy, op), _character(copy._character), _frameTime(copy._frameTime) {}
    
    META_Object( osgPhysics, UpdateCharacterCallback );
    
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );
    
    void setFrameTime( double t ) { _frameTime = t; }
    double getFrameTime() const { return _frameTime; }
    
protected:
    CharacterController* _character;
    double _lastSimulationTime;
    double _frameTime;
};

/** The callback to update the vehicle */
class UpdateVehicleCallback : public osg::NodeCallback
{
public:
    UpdateVehicleCallback( WheeledVehicle* car=0 )
    : _physicsVehicle(car), _lastSimulationTime(0.0), _frameTime(0.02) {}
    
    UpdateVehicleCallback( const UpdateVehicleCallback& copy, const osg::CopyOp& op=osg::CopyOp::SHALLOW_COPY )
    : osg::NodeCallback(copy, op), _physicsVehicle(copy._physicsVehicle), _frameTime(copy._frameTime) {}
    
    META_Object( osgPhysics, UpdateVehicleCallback );
    
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );
    
    void setFrameTime( double t ) { _frameTime = t; }
    double getFrameTime() const { return _frameTime; }
    
protected:
    WheeledVehicle* _physicsVehicle;
    double _lastSimulationTime;
    double _frameTime;
};

}

#endif
