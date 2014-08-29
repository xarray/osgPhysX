#include <osg/io_utils>
#include "PhysicsUtil.h"
#include "CharacterController.h"
#include <algorithm>
#include <iostream>

using namespace osgPhysics;
using namespace physx;

#define SDK_OBJ (Engine::instance()->getPhysicsSDK())
#define DEF_MTL (Engine::instance()->getDefaultMaterial())

/* CharacterControlManager */

CharacterControlManager* CharacterControlManager::instance()
{
    static osg::ref_ptr<CharacterControlManager> s_registry = new CharacterControlManager;
    return s_registry.get();
}

CharacterControlManager::CharacterControlManager()
{
}

CharacterControlManager::~CharacterControlManager()
{
    for ( std::map<physx::PxScene*, physx::PxControllerManager*>::iterator itr=_managers.begin();
          itr!=_managers.end(); ++itr )
    {
        itr->second->release();
    }
}

physx::PxControllerManager* CharacterControlManager::getOrCreateManager( physx::PxScene* scene )
{
    physx::PxControllerManager* manager = _managers[scene];
    if ( !manager )
    {
#if USE_PHYSX_33
        manager = PxCreateControllerManager( *scene );
#else
        manager = PxCreateControllerManager( SDK_OBJ->getFoundation() );
#endif
        _managers[scene] = manager;
    }
    return manager;
}

/* CharacterController */

CharacterController::ControllerData::ControllerData( float d, const osg::Vec3& p, const osg::Vec3& u )
{
    position = PxExtendedVec3(p[0], p[1], p[2]);
    up = PxVec3(u[0], u[1], u[2]);
    density = d;
    maxSlopeAngle = 0.0f;
    stepOffset = 0.05f;
    contactOffset = 0.01f;
    scaleCoeffcient = 0.8f;
}

CharacterController::CharacterController()
:   _actor(NULL), _controller(NULL)
{
}

CharacterController::~CharacterController()
{
    if ( _controller ) _controller->release();
}

bool CharacterController::createBox( const std::string& scene, PxF32 halfForward, PxF32 halfSide, PxF32 halfHeight,
                                     const ControllerData& data, PxMaterial* mtl )
{
    PxBoxControllerDesc desc;
    desc.halfForwardExtent = halfForward;
    desc.halfSideExtent = halfSide;
    desc.halfHeight = halfHeight;
    return create( scene, &desc, data, mtl );
}

bool CharacterController::createCapsule( const std::string& scene, PxF32 radius, PxF32 height, bool easyMode,
                                         const ControllerData& data, PxMaterial* mtl )
{
    PxCapsuleControllerDesc desc;
    desc.radius = radius;
    desc.height = height;
    desc.climbingMode = easyMode ? PxCapsuleClimbingMode::eEASY : PxCapsuleClimbingMode::eCONSTRAINED;
    return create( scene, &desc, data, mtl );
}

bool CharacterController::create( const std::string& s, PxControllerDesc* description,
                                  const ControllerData& data, PxMaterial* mtl )
{
    PxScene* scene = Engine::instance()->getScene( s );
    if ( !scene || !description ) return false;
    
    description->position = data.position;
    description->upDirection = data.up;
    description->density = data.density;
    description->slopeLimit = data.maxSlopeAngle;
    description->contactOffset = data.contactOffset;
    description->stepOffset = data.stepOffset;
    description->scaleCoeff = data.scaleCoeffcient;
    description->material = mtl ? mtl : DEF_MTL;
    
    physx::PxControllerManager* manager = CharacterControlManager::instance()->getOrCreateManager(scene);
#if USE_PHYSX_33
    _controller = manager->createController( *description );
#else
    _controller = manager->createController( *SDK_OBJ, scene, *description );
#endif
    if ( !_controller ) return false;
    
    _actor = _controller->getActor();
    if ( _actor && _actor->getNbShapes() )
    {
        std::vector<PxShape*> shapes(1);
        _actor->getShapes( &(shapes[0]), 1 );
        if ( shapes[0] ) shapes[0]->setFlag( PxShapeFlag::eSCENE_QUERY_SHAPE, false );
    }
    PxVec3 g = scene->getGravity();
    _gravity.set( g.x, g.y, g.z );
    return true;
}

void CharacterController::move( const osg::Vec3& offset, float gravityScale )
{
    _offset = offset + _gravity * gravityScale;
}

void CharacterController::updateMovement( double step )
{
    PxVec3 offset(_offset[0], _offset[1], _offset[2]);
    const static float minOffsetDistance = 0.001f;
    const PxU32 flags = _controller->move( offset, minOffsetDistance, step, PxControllerFilters(), NULL);
}

void CharacterController::onShapeHit( const PxControllerShapeHit& hit )
{
#if USE_PHYSX_33
    PxRigidDynamic* actor = hit.shape->getActor()->is<PxRigidDynamic>();
#else
    PxRigidDynamic* actor = hit.shape->getActor().isRigidDynamic();
#endif
    if ( !actor ) return;
    if ( actor->getRigidDynamicFlags() & PxRigidDynamicFlag::eKINEMATIC ) return;
    
    // We only allow horizontal pushes. Vertical pushes when we stand on dynamic objects creates
    // useless stress on the solver. It would be possible to enable/disable vertical pushes on
    // particular objects, if the gameplay requires it
    const PxVec3 upVector = hit.controller->getUpDirection();
    const PxF32 dp = hit.dir.dot(upVector);
    if ( fabsf(dp)<1e-3f )
    {
        const PxTransform globalPose = actor->getGlobalPose();
        const PxVec3 localPos = globalPose.transformInv( toVec3(hit.worldPos) );
        addForceAtLocalPos( *actor, hit.dir*hit.length*1000.0f, localPos, PxForceMode::eACCELERATION );
    }
}

#if USE_PHYSX_33

PxControllerBehaviorFlags CharacterController::getBehaviorFlags( const PxShape& shape, const physx::PxActor& actor )
{ return PxControllerBehaviorFlags(0); }

PxControllerBehaviorFlags CharacterController::getBehaviorFlags( const PxController& controller )
{ return PxControllerBehaviorFlags(0); }

PxControllerBehaviorFlags CharacterController::getBehaviorFlags( const PxObstacle& obstacle )
{ return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT|PxControllerBehaviorFlag::eCCT_SLIDE; }

PxQueryHitType::Enum CharacterController::preFilter(
    const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxSceneQueryFlags& flags )
{ return PxQueryHitType::eBLOCK; }

PxQueryHitType::Enum CharacterController::postFilter(
    const PxFilterData& filterData, const PxSceneQueryHit& hit )
{ return PxSceneQueryHitType::eBLOCK; }

#else

PxU32 CharacterController::getBehaviorFlags( const PxShape& shape )
{ return 0; }

PxU32 CharacterController::getBehaviorFlags( const PxController& controller )
{ return 0; }

PxU32 CharacterController::getBehaviorFlags( const PxObstacle& obstacle )
{ return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT|PxControllerBehaviorFlag::eCCT_SLIDE; }

PxSceneQueryHitType::Enum CharacterController::preFilter(
    const PxFilterData& filterData, PxShape* shape, PxSceneQueryFilterFlags& flags )
{ return PxSceneQueryHitType::eBLOCK; }

PxSceneQueryHitType::Enum CharacterController::postFilter(
    const PxFilterData& filterData, const PxSceneQueryHit& hit )
{ return PxSceneQueryHitType::eBLOCK; }

#endif

void CharacterController::addForceAtLocalPos( PxRigidBody& body, const PxVec3& force, const PxVec3& pos,
                                              PxForceMode::Enum mode, bool wakeup )
{
    const PxVec3 globalForcePos = body.getGlobalPose().transform(pos);  // to world space
    const PxTransform globalPose = body.getGlobalPose();
    const PxVec3 centerOfMass = globalPose.transform( body.getCMassLocalPose().p );
    const PxVec3 torque = (globalForcePos - centerOfMass).cross(force);
    body.addForce( force, mode, wakeup );
    body.addTorque( torque, mode, wakeup );
}
