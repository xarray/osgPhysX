#ifndef PHYSICS_VEHICLEMANAGER
#define PHYSICS_VEHICLEMANAGER

#include <osg/Referenced>
#include <osg/Vec3>
#include "Engine.h"

namespace osgPhysics
{

/** The global vehicle manager */
class VehicleManager : public osg::Referenced
{
public:
    static VehicleManager* instance();
    
    // Collision types and flags describing collision interactions of each collision type
    enum CollisionFlag
    {
        COLLISION_FLAG_GROUND = 1 << 0,
        COLLISION_FLAG_WHEEL = 1 << 1,
        COLLISION_FLAG_CHASSIS = 1 << 2,
        COLLISION_FLAG_OBSTACLE = 1 << 3,
        COLLISION_FLAG_DRIVABLE_OBSTACLE = 1 << 4,
        COLLISION_FLAG_GROUND_AGAINST = COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
        COLLISION_FLAG_WHEEL_AGAINST = COLLISION_FLAG_WHEEL | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE,
        COLLISION_FLAG_CHASSIS_AGAINST = COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
        COLLISION_FLAG_OBSTACLE_AGAINST = COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
        COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST = COLLISION_FLAG_GROUND | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
        COLLISION_FLAG_DRIVABLE_SURFACE = 0xffff0000,
        COLLISION_FLAG_UNDRIVABLE_SURFACE = 0x0000ffff
    };
    
    /** Filter type */
    enum FilterType
    {
        FILTER_GROUND, FILTER_WHEEL, FILTER_CHASSIS, FILTER_OBSTACLE, FILTER_DRIVABLE_OBSTACLE,
        FILTER_DRIVABLE_SURFACE, FILTER_UNDRIVABLE_SURFACE
    };
    
    /** Drivable surface types */
    enum SurfaceType { SURFACE_MUD=0, SURFACE_TARMAC, SURFACE_SNOW, SURFACE_GRASS, MAX_NUM_SURFACE_TYPES };
    
    /** Tire types of cars */
    enum TireType { TIRE_WETS=0, TIRE_SLICKS, TIRE_ICE, TIRE_MUD, MAX_NUM_TIRE_TYPES };
    
    /** Create filter for vehicle components */
    static physx::PxFilterData createFilter( FilterType t, physx::PxShape* shape=NULL );
    
    /** Create specified filted scene for vehicles instead of default */
    static physx::PxScene* createScene( const osg::Vec3& gravity );
    
    /** Add actor object of specified type to scene for vehicles */
    static bool addActor( const std::string& scene, physx::PxActor* actor, SurfaceType st,
                          FilterType ft, bool drivable );
    
    /** Get material for specified surface */
    physx::PxMaterial* getSurfaceMaterial( SurfaceType t ) { return _surfaceMaterials[t]; }
    
    /** Set tire/surface friction */
    void setSurfaceToTireFriction( SurfaceType s, TireType t, double value );
    double getSurfaceToTireFriction( SurfaceType s, TireType t ) const;
    
    /** Update car vehicles of specified scene every frame */
#if USE_PHYSX_33
    virtual void update( double step, const std::string& scene, std::vector<physx::PxVehicleWheels*>& vehicles,
                         std::vector<physx::PxVehicleWheelQueryResult>& queryResults, unsigned int numWheels );
#else
    virtual void update( double step, const std::string& scene, std::vector<physx::PxVehicleWheels*>& vehicles, unsigned int numWheels );
#endif

protected:
    VehicleManager();
    virtual ~VehicleManager();
    
    virtual void initialize();
    virtual void updateQueryData( physx::PxScene* scene, unsigned int numWheels );
    
    physx::PxMaterial* _surfaceMaterials[MAX_NUM_SURFACE_TYPES];
    physx::PxVehicleDrivableSurfaceType _surfaceTypes[MAX_NUM_SURFACE_TYPES];
    physx::PxVehicleDrivableSurfaceToTireFrictionPairs* _surfaceTirePairs;
    
    physx::PxBatchQuery* _query;
    physx::PxRaycastQueryResult* _queryResults;
    physx::PxRaycastHit* _queryHitBuffer;
    physx::PxU32 _numQueries, _numMaxWheels;
};

}

#endif
