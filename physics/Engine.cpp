#include <osg/io_utils>
#include "PhysicsUtil.h"
#include <algorithm>
#include <iostream>

using namespace osgPhysics;
using namespace physx;

class ErrorCallback : public PxErrorCallback
{
public:
    virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
    {
        OSG_WARN << "Error " << code << " was found in (" << file << ", " << line << "): "
            << message << std::endl;
    }
};

ErrorCallback errorHandler;
PxDefaultAllocator defaultAllocator;
bool Engine::startWithPVD = false;

Engine* Engine::instance()
{
    static osg::ref_ptr<Engine> s_registry = new Engine;
    return s_registry.get();
}

Engine::Engine()
    : _cooking(NULL), _cudaManager(NULL), _pvdTransport(NULL), _pvd(NULL)
{
#if (PX_PHYSICS_VERSION_MAJOR > 3)
    PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, defaultAllocator, errorHandler);
#else
    PxFoundation* foundation = PxCreateFoundation(PX_FOUNDATION_VERSION, defaultAllocator, errorHandler);
#endif
    
    if (!foundation)
    {
        OSG_WARN << "Unable to initialize PhysX foundation." << std::endl;
        return;
    }

    if (startWithPVD)
    {
#if _DEBUG
        _pvd = PxCreatePvd(*foundation);
        _pvdTransport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
        _pvd->connect(*_pvdTransport, PxPvdInstrumentationFlag::eALL);
        OSG_NOTICE << "Initializing PVD support." << std::endl;
#else
        OSG_WARN << "PVD only works with debug, checked and profiling configurations." << std::endl;
#endif
    }

    _physicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(),
                                  (_pvd != NULL), _pvd);
    if (!_physicsSDK)
    {
        OSG_WARN << "Unable to initialize PhysX SDK." << std::endl;
        return;
    }
    _defaultMaterial = _physicsSDK->createMaterial(0.5, 0.5, 0.5);

    if (!PxInitExtensions(*_physicsSDK, NULL))
    {
        OSG_WARN << "Unable to initialize PhysX extensions." << std::endl;
    }
}

Engine::~Engine()
{
    clear();
    PxCloseExtensions();
    _defaultMaterial->release();
    _physicsSDK->release();
    if (_pvd) _pvd->release();
    if (_pvdTransport) _pvdTransport->release();
    if (_cooking) _cooking->release();
    if (_cudaManager) _cudaManager->release();
}

bool Engine::addScene(const std::string& name, PxScene* s)
{
    if (!s || _sceneMap.find(name) != _sceneMap.end()) return false;
    _sceneMap[name] = s;
    return true;
}

bool Engine::removeScene(const std::string& name, bool doRelease)
{
    SceneMap::iterator itr = _sceneMap.find(name);
    if (itr == _sceneMap.end()) return false;

    if (doRelease)
    {
        releaseActors(itr->second);
        itr->second->release();
    }
    _sceneMap.erase(itr);
    return true;
}

PxScene* Engine::getScene(const std::string& name)
{
    SceneMap::iterator itr = _sceneMap.find(name);
    if (itr == _sceneMap.end()) return NULL;
    return itr->second;
}

const PxScene* Engine::getScene(const std::string& name) const
{
    SceneMap::const_iterator itr = _sceneMap.find(name);
    if (itr == _sceneMap.end()) return NULL;
    return itr->second;
}

bool Engine::addActor(const std::string& s, PxActor* actor)
{
    PxScene* scene = getScene(s);
    if (!scene || !actor) return false;
    scene->addActor(*actor);
    _actorMap[scene].push_back(actor);
    return true;
}

bool Engine::addActor(const std::string& s, PxRigidActor* actor, const PxFilterData& filter)
{
    if (addActor(s, actor))
        return createSimulationFilter(actor, filter);
    else
        return false;
}

#if !(PX_PHYSICS_VERSION_MAJOR > 3)
bool Engine::addActor(const std::string& s, physx::PxParticleBase* ps, const physx::PxFilterData& filter)
{
    if (addActor(s, ps))
    {
        ps->setSimulationFilterData(filter);
        return true;
    }
    else
        return false;
}
#endif

bool Engine::removeActor(const std::string& s, PxActor* actor)
{
    PxScene* scene = getScene(s);
    if (!scene || !actor) return false;

    ActorMap::iterator itr = _actorMap.find(scene);
    if (itr == _actorMap.end()) return false;

    ActorList& actors = itr->second;
    ActorList::iterator fitr = std::find(actors.begin(), actors.end(), actor);
    if (fitr == actors.end()) return false;

    scene->removeActor(*actor);
    actors.erase(fitr);
    if (!actors.size()) _actorMap.erase(itr);
    return true;
}

PxCooking* Engine::getOrCreateCooking(PxCookingParams* params, bool forceCreating)
{
    if (forceCreating && _cooking)
    {
        _cooking->release();
        _cooking = NULL;
    }

    if (!_cooking)
    {
        if (params)
            _cooking = PxCreateCooking(PX_PHYSICS_VERSION, _physicsSDK->getFoundation(), *params);
        else
        {
            physx::PxTolerancesScale sc;
            physx::PxCookingParams defParams(sc);
            _cooking = PxCreateCooking(PX_PHYSICS_VERSION, _physicsSDK->getFoundation(), defParams);
        }
    }
    return _cooking;
}

PxCudaContextManager* Engine::getOrCreateCudaContextManager(PxCudaContextManagerDesc* desc, bool forceCreating)
{
    if (forceCreating && _cudaManager)
    {
        _cudaManager->release();
        _cudaManager = NULL;
    }

    if (!_cudaManager)
    {
        if (desc)
            _cudaManager = PxCreateCudaContextManager(_physicsSDK->getFoundation(), *desc);
        else
        {
            physx::PxCudaContextManagerDesc defDesc;
            defDesc.interopMode = PxCudaInteropMode::OGL_INTEROP;
            _cudaManager = PxCreateCudaContextManager(_physicsSDK->getFoundation(), defDesc);
        }
    }
    return _cudaManager;
}

void Engine::update(double step)
{
    for (SceneMap::iterator itr = _sceneMap.begin(); itr != _sceneMap.end(); ++itr)
    {
        PxScene* scene = itr->second;
        scene->simulate(step);
        while (!scene->fetchResults()) { /* do nothing but wait */ }
    }
}

void Engine::clear()
{
    for (SceneMap::iterator itr = _sceneMap.begin(); itr != _sceneMap.end(); ++itr)
    {
        PxScene* scene = itr->second;
        releaseActors(scene);
        scene->release();
    }
    _sceneMap.clear();
    _actorMap.clear();
}

void Engine::releaseActors(PxScene* scene)
{
    ActorMap::iterator itr = _actorMap.find(scene);
    if (itr == _actorMap.end()) return;

    ActorList& actors = itr->second;
    for (unsigned int i = 0; i < actors.size(); ++i)
    {
        scene->removeActor(*(actors[i]));
    }
    _actorMap.erase(itr);
}
