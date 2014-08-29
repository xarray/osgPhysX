#ifndef PHYSICS_CHARACTERCONTROLLER
#define PHYSICS_CHARACTERCONTROLLER

#include <osg/Referenced>
#include <osg/Vec3>
#include "Engine.h"

namespace osgPhysics
{

/** The character controller manager */
class CharacterControlManager : public osg::Referenced
{
public:
    static CharacterControlManager* instance();
    physx::PxControllerManager* getOrCreateManager( physx::PxScene* scene );
    
protected:
    CharacterControlManager();
    virtual ~CharacterControlManager();
    std::map<physx::PxScene*, physx::PxControllerManager*> _managers;
};

/** The character controller */
class CharacterController : public osg::Referenced, public physx::PxUserControllerHitReport,
                            public physx::PxControllerBehaviorCallback, public physx::PxSceneQueryFilterCallback
{
public:
    struct ControllerData
    {
        physx::PxExtendedVec3 position;  // the initial position
        physx::PxVec3 up;  // the up direction
        physx::PxF32 density;  // density of the character
        physx::PxF32 maxSlopeAngle;  // max slope angle we can reach (or 0 for any)
        physx::PxF32 stepOffset;  // max height of an obstacle which we can climb
        physx::PxF32 contactOffset;  // a skin around the object for creating contacts
        physx::PxF32 scaleCoeffcient;  // scale coeffcient for underlying kinematic actor
        ControllerData( float d=10.0f, const osg::Vec3& pos=osg::Vec3(), const osg::Vec3& upDir=osg::Z_AXIS );
    };
    
    CharacterController();
    
    physx::PxRigidDynamic* getControlActor() { return _actor; }
    const physx::PxRigidDynamic* getControlActor() const { return _actor; }
    
    physx::PxController* getController() { return _controller; }
    const physx::PxController* getController() const { return _controller; }
    
    /** Create a box controller */
    bool createBox( const std::string& scene, physx::PxF32 halfForward, physx::PxF32 halfSide, physx::PxF32 halfHeight,
                    const ControllerData& data, physx::PxMaterial* mtl=0 );
    
    /** Create a capsule controller */
    bool createCapsule( const std::string& scene, physx::PxF32 radius, physx::PxF32 height, bool easyMode,
                        const ControllerData& data, physx::PxMaterial* mtl=0 );
    
    /** Create the controller using generic description */
    bool create( const std::string& scene, physx::PxControllerDesc* description,
                 const ControllerData& data, physx::PxMaterial* mtl=0 );
    
    /** Set the gravity the character should suffer from */
    void setGravity( osg::Vec3& g ) { _gravity = g; }
    const osg::Vec3& getGravity() const { return _gravity; }
    
    /** Set the character position immediately */
    void setPosition( physx::PxExtendedVec3& p ) { _controller->setPosition(p); }
    physx::PxExtendedVec3 getPosition() const { return _controller->getPosition(); }
    
    /** Move the character */
    void move( const osg::Vec3& offset, float gravityScale=1.0f );
    
    /** Update the controller movement every frame */
    void updateMovement( double step );
    
    // Implements PxUserControllerHitReport
    virtual void onShapeHit( const physx::PxControllerShapeHit& hit );
    virtual void onControllerHit( const physx::PxControllersHit& hit ) {}
    virtual void onObstacleHit( const physx::PxControllerObstacleHit& hit ) {}

    // Implements PxControllerBehaviorCallback
#if USE_PHYSX_33
    virtual physx::PxControllerBehaviorFlags getBehaviorFlags( const physx::PxShape& shape, const physx::PxActor& actor );
    virtual physx::PxControllerBehaviorFlags getBehaviorFlags( const physx::PxController& controller );
    virtual physx::PxControllerBehaviorFlags getBehaviorFlags( const physx::PxObstacle& obstacle );
#else
    virtual physx::PxU32 getBehaviorFlags( const physx::PxShape& shape );
    virtual physx::PxU32 getBehaviorFlags( const physx::PxController& controller );
    virtual physx::PxU32 getBehaviorFlags( const physx::PxObstacle& obstacle );
#endif

    // Implements PxSceneQueryFilterCallback
#if USE_PHYSX_33
    virtual physx::PxQueryHitType::Enum preFilter(
        const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxSceneQueryFlags& flags );
    virtual physx::PxQueryHitType::Enum postFilter(
        const physx::PxFilterData& filterData, const physx::PxSceneQueryHit& hit );
#else
    virtual physx::PxSceneQueryHitType::Enum preFilter(
        const physx::PxFilterData& filterData, physx::PxShape* shape, physx::PxSceneQueryFilterFlags& flags );
    virtual physx::PxSceneQueryHitType::Enum postFilter(
        const physx::PxFilterData& filterData, const physx::PxSceneQueryHit& hit );
#endif

protected:
    virtual ~CharacterController();
    void addForceAtLocalPos( physx::PxRigidBody& body, const physx::PxVec3& force, const physx::PxVec3& pos,
                             physx::PxForceMode::Enum mode, bool wakeup=true );
    
    physx::PxRigidDynamic* _actor;
    physx::PxController* _controller;
    osg::Vec3 _offset, _gravity;
};

}

#endif
