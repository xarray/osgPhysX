#include <osg/io_utils>
#include "PhysicsUtil.h"
#include "VehicleManager.h"
#include <algorithm>
#include <iostream>

using namespace osgPhysics;
using namespace physx;

#define SDK_OBJ (Engine::instance()->getPhysicsSDK())
#define SDK_COOK (Engine::instance()->getOrCreateCooking())

// Tire model friction for each combination of drivable surface type and tire type
static PxF32 g_tireFrictionMultipliers[VehicleManager::MAX_NUM_SURFACE_TYPES][VehicleManager::MAX_NUM_TIRE_TYPES] =
{
    //WETS    SLICKS    ICE        MUD
    {1.70f,    1.85f,    1.70f,    1.70f},       //MUD
    {2.00f,    2.15f,    1.90f,    1.90f},       //TARMAC
    {0.70f,    0.70f,    1.20f,    0.90f},       //ICE
    {1.20f,    0.90f,    1.30f,    1.40f}        //GRASS
};

// Scene filter shader
static PxFilterFlags vehicleSceneFilter(
    PxFilterObjectAttributes attributes0, PxFilterData filterData0,
    PxFilterObjectAttributes attributes1, PxFilterData filterData1,
    PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
    //PX_FORCE_PARAMETER_REFERENCE( constantBlock );
    //PX_FORCE_PARAMETER_REFERENCE( constantBlockSize );

    // Let triggers through
    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
    {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
        return physx::PxFilterFlag::eDEFAULT;
    }

    // Use a group-based mechanism for all other pairs:
    // - Objects within the default group (mask 0) always collide
    // - By default, objects of the default group do not collide
    //   with any other group. If they should collide with another
    //   group then this can only be specified through the filter
    //   data of the default group objects (objects of a different
    //   group can not choose to do so)
    // - For objects that are not in the default group, a bitmask
    //   is used to define the groups they should collide with
    if ((filterData0.word0 != 0 || filterData1.word0 != 0) &&
        !(filterData0.word0&filterData1.word1 || filterData1.word0&filterData0.word1))
    {
        return PxFilterFlag::eSUPPRESS;
    }
    pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT;

    // Enable CCD stuff - for now just for everything or nothing
    const int CCD_LINEAR = 1;
    if ((filterData0.word3 | filterData1.word3) & CCD_LINEAR)
        pairFlags |= PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_CCD_CONTACT;

    // The pairFlags for each object are stored in word2 of the filter data, so combine them
    pairFlags |= PxPairFlags(PxU16(filterData0.word2 | filterData1.word2));
    return physx::PxFilterFlag::eDEFAULT;
}

// Raycast filter shader
static PxQueryHitType::Enum wheelRaycastPreFilter(
    PxFilterData filterData0, PxFilterData filterData1,
    const void* constantBlock, PxU32 constantBlockSize,
    PxSceneQueryFlags& queryFlags)
{
    //filterData0 is the vehicle suspension raycast.
    //filterData1 is the shape potentially hit by the raycast.
    PX_UNUSED(queryFlags);
    PX_UNUSED(constantBlockSize);
    PX_UNUSED(constantBlock);
    PX_UNUSED(filterData0);
    return ((filterData1.word3 & VehicleManager::COLLISION_FLAG_DRIVABLE_SURFACE) == 0) ?
        PxQueryHitType::eNONE : PxQueryHitType::eBLOCK;
}

/* VehicleManager */

VehicleManager* VehicleManager::instance()
{
    static osg::ref_ptr<VehicleManager> s_registry = new VehicleManager;
    return s_registry.get();
}

VehicleManager::VehicleManager()
    : _query(NULL), _queryResults(NULL), _queryHitBuffer(NULL), _numQueries(0), _numMaxWheels(0)
{
    PxInitVehicleSDK(*SDK_OBJ);
    initialize();
}

VehicleManager::~VehicleManager()
{
    if (_query) _query->release();
    if (_queryResults) delete[] _queryResults;
    if (_queryHitBuffer) delete[] _queryHitBuffer;
    _surfaceTirePairs->release();
    PxCloseVehicleSDK();
}

PxFilterData VehicleManager::createFilter(VehicleManager::FilterType type, PxShape* shape)
{
    PxFilterData filter;
    switch (type)
    {
    case FILTER_GROUND:
        filter.word0 = COLLISION_FLAG_GROUND;
        filter.word1 = COLLISION_FLAG_GROUND_AGAINST;
        if (shape) shape->setSimulationFilterData(filter);
        break;
    case FILTER_WHEEL:
        filter.word0 = COLLISION_FLAG_WHEEL;
        filter.word1 = COLLISION_FLAG_WHEEL_AGAINST;
        if (shape) shape->setSimulationFilterData(filter);
        break;
    case FILTER_CHASSIS:
        filter.word0 = COLLISION_FLAG_CHASSIS;
        filter.word1 = COLLISION_FLAG_CHASSIS_AGAINST;
        if (shape) shape->setSimulationFilterData(filter);
        break;
    case FILTER_OBSTACLE:
        filter.word0 = COLLISION_FLAG_OBSTACLE;
        filter.word1 = COLLISION_FLAG_OBSTACLE_AGAINST;
        if (shape) shape->setSimulationFilterData(filter);
        break;
    case FILTER_DRIVABLE_OBSTACLE:
        filter.word0 = COLLISION_FLAG_DRIVABLE_OBSTACLE;
        filter.word1 = COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST;
        if (shape) shape->setSimulationFilterData(filter);
        break;
    case FILTER_DRIVABLE_SURFACE:
        filter.word3 = COLLISION_FLAG_DRIVABLE_SURFACE;
        if (shape) shape->setQueryFilterData(filter);
        break;
    case FILTER_UNDRIVABLE_SURFACE:
        filter.word3 = COLLISION_FLAG_UNDRIVABLE_SURFACE;
        if (shape) shape->setQueryFilterData(filter);
        break;
    default: break;
    }
    return filter;
}

PxScene* VehicleManager::createScene(const osg::Vec3& gravity)
{
    return osgPhysics::createScene(gravity, vehicleSceneFilter);
}

bool VehicleManager::addActor(const std::string& scene, PxActor* actor,
    VehicleManager::SurfaceType st, VehicleManager::FilterType ft, bool drivable)
{
    PxRigidActor* rigidActor = actor ? actor->is<PxRigidActor>() : NULL;
    if (rigidActor)
    {
        std::vector<PxShape*> shapes(rigidActor->getNbShapes());
        unsigned int size = rigidActor->getShapes(&(shapes[0]), rigidActor->getNbShapes());

        FilterType surfaceFilter = drivable ? FILTER_DRIVABLE_SURFACE : FILTER_UNDRIVABLE_SURFACE;
        PxMaterial* material = VehicleManager::instance()->getSurfaceMaterial(st);
        for (unsigned int i = 0; i < size; ++i)
        {
            PxShape* shape = shapes[i];
            if (material) shape->setMaterials(&material, 1);
            createFilter(ft, shape);
            createFilter(surfaceFilter, shape);
        }
    }
    else if (actor)
    {
        PxParticleBase* particleBase = actor->is<PxParticleBase>();
        if (particleBase)
        {
            PxFilterData filter;
            filter.word0 = COLLISION_FLAG_OBSTACLE;
            filter.word1 = COLLISION_FLAG_OBSTACLE_AGAINST;
            particleBase->setSimulationFilterData(filter);
        }
    }
    return osgPhysics::Engine::instance()->addActor(scene, actor);
}

void VehicleManager::setSurfaceToTireFriction(SurfaceType s, TireType t, double value)
{
    _surfaceTirePairs->setTypePairFriction(s, t, value);
}

double VehicleManager::getSurfaceToTireFriction(SurfaceType s, TireType t) const
{
    return _surfaceTirePairs->getTypePairFriction(s, t);
}

void VehicleManager::update(double step, const std::string& s, std::vector<PxVehicleWheels*>& vehicles,
    std::vector<physx::PxVehicleWheelQueryResult>& queryResults, unsigned int numWheels)
{
    PxScene* scene = Engine::instance()->getScene(s);
    if (!scene || step <= 0.0) return;
    updateQueryData(scene, numWheels);

    unsigned int size = vehicles.size();
    PxVehicleSuspensionRaycasts(_query, size, &(vehicles[0]), _numQueries, _queryResults);
    PxVehicleUpdates(step, scene->getGravity(), *_surfaceTirePairs, size, &(vehicles[0]), &(queryResults[0]));
}

void VehicleManager::initialize()
{
    PxVehicleSetBasisVectors(PxVec3(0, 1, 0), PxVec3(0, 0, 1));
    for (int i = 0; i < MAX_NUM_SURFACE_TYPES; ++i)
    {
        _surfaceMaterials[i] = SDK_OBJ->createMaterial(0.2f, 0.5f, 0.5f);
        _surfaceTypes[i].mType = (PxU32)SURFACE_MUD + i;
    }

    _surfaceTirePairs = PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES);
    _surfaceTirePairs->setup(MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES,
        (const PxMaterial**)_surfaceMaterials, _surfaceTypes);

    for (int i = 0; i < MAX_NUM_SURFACE_TYPES; ++i)
    {
        for (int j = 0; j < MAX_NUM_TIRE_TYPES; ++j)
            _surfaceTirePairs->setTypePairFriction(i, j, g_tireFrictionMultipliers[i][j]);
    }
}

void VehicleManager::updateQueryData(PxScene* scene, unsigned int numWheels)
{
    if (!_query || _numMaxWheels < numWheels)
    {
        _numMaxWheels = numWheels + 4 * (4 + 6);  // Allocate for another 8 vehicles (4W / 6W) besides current needs
        _numQueries = numWheels;
        if (_query) _query->release();
        if (_queryResults) delete[] _queryResults;
        if (_queryHitBuffer) delete[] _queryHitBuffer;

        _queryResults = new PxRaycastQueryResult[numWheels];
        _queryHitBuffer = new PxRaycastHit[numWheels];

        PxBatchQueryDesc queryDesc(_numMaxWheels, 0, 0);
        queryDesc.queryMemory.userRaycastResultBuffer = _queryResults;
        queryDesc.queryMemory.userRaycastTouchBuffer = _queryHitBuffer;
        queryDesc.queryMemory.raycastTouchBufferSize = _numQueries;
        queryDesc.preFilterShader = wheelRaycastPreFilter;
        _query = scene->createBatchQuery(queryDesc);
    }
}
