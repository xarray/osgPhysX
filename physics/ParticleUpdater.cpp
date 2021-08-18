#include <osg/io_utils>
#include "PhysicsUtil.h"
#include "ParticleUpdater.h"
#include <algorithm>
#include <iostream>

#include <intrin.h>
#pragma intrinsic(_BitScanForward)
#pragma intrinsic(_BitScanReverse)

static unsigned int lowestSetBit(unsigned int index)
{
    unsigned long value;
    _BitScanForward(&value, index);
    return value;
}

static unsigned int highestSetBit(unsigned int index)
{
    unsigned long value;
    _BitScanReverse(&value, index);
    return value;
}

using namespace osgPhysics;
using namespace physx;

#define SDK_OBJ (Engine::instance()->getPhysicsSDK())
#define DEF_MTL (Engine::instance()->getDefaultMaterial())

/* ParticleUpdater::ParticleAttributes */

void ParticleUpdater::ParticleAttributes::setDefaults(unsigned int maxP)
{
    useGPU = true;
    maxParticles = maxP;
    gridSize = 3.0f;
    maxMotionDistance = 0.43f;
    restOffset = 0.0143f;
    contactOffset = 0.0286f;
    damping = 0.0f;
    restitution = 0.2f;
    dynamicFriction = 0.05f;
    restParticleDistance = 0.0f;
    viscosity = 0.0f;
    stiffness = 0.0f;
}

void ParticleUpdater::ParticleAttributes::setFluidDefaults(unsigned int maxP)
{
    useGPU = true;
    maxParticles = maxP;
    gridSize = 5.0f;
    maxMotionDistance = 0.3f;
    restOffset = 0.24f;
    contactOffset = 0.48f;
    damping = 0.0f;
    restitution = 0.3f;
    dynamicFriction = 0.001f;
    restParticleDistance = 0.8f;
    viscosity = 60.0f;
    stiffness = 45.0f;
}

/* ParticleUpdater */

ParticleUpdater::ParticleUpdater()
    : _particleSystem(NULL), _indexPool(NULL), _timelessLife(true), _isFluidSystem(false)
{
    _dataAccesses = POSITION_DATA | FLAGS_DATA;
}

ParticleUpdater::~ParticleUpdater()
{
    if (_particleSystem)
    {
        //_particleSystem->release();
        _particleSystem = NULL;
    }

    if (_indexPool)
    {
        _indexPool->release();
        _indexPool = NULL;
    }
}

void ParticleUpdater::setDataAccess(int da)
{
    _particleSystem->setParticleReadDataFlag(PxParticleReadDataFlag::ePOSITION_BUFFER, (da&POSITION_DATA) != 0);
    _particleSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eVELOCITY_BUFFER, (da&VELOCITY_DATA) != 0);
    _particleSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eREST_OFFSET_BUFFER, (da&REST_OFFSET_DATA) != 0);
    _particleSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eDENSITY_BUFFER, (da&DENSITY_DATA) != 0);
    _particleSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eCOLLISION_NORMAL_BUFFER, (da&NORMAL_DATA) != 0);
    _particleSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eFLAGS_BUFFER, (da&FLAGS_DATA) != 0);
    _dataAccesses = da;
}

bool ParticleUpdater::setAsDrain(physx::PxRigidActor* actor, bool b)
{
    if (!actor) return false;
    std::vector<PxShape*> shapes(actor->getNbShapes());

    PxU32 num = actor->getShapes(&(shapes[0]), actor->getNbShapes());
    for (PxU32 i = 0; i < num; ++i)
        shapes[i]->setFlag(PxShapeFlag::ePARTICLE_DRAIN, b);
    return true;
}

bool ParticleUpdater::create(const ParticleAttributes& attr, bool supportRestOffset)
{
    if (_particleSystem)
    {
        OSG_NOTICE << "[ParticleUpdater] Particle system already created" << std::endl;
        return false;
    }

    _particleSystem = SDK_OBJ->createParticleSystem(attr.maxParticles, supportRestOffset);
    _indexPool = PxParticleExt::createIndexPool(attr.maxParticles);
    _particleAttributes.resize(attr.maxParticles);
    setupAttributes(attr, false);
    return true;
}

bool ParticleUpdater::createFluid(const ParticleAttributes& attr, bool supportRestOffset)
{
    if (_particleSystem)
    {
        OSG_NOTICE << "[ParticleUpdater] Particle fluid system already created" << std::endl;
        return false;
    }

    _particleSystem = SDK_OBJ->createParticleFluid(attr.maxParticles, supportRestOffset);
    _indexPool = PxParticleExt::createIndexPool(attr.maxParticles);
    _particleAttributes.resize(attr.maxParticles);
    setupAttributes(attr, true);
    return true;
}

unsigned int ParticleUpdater::generate(const ParticleData& particles)
{
    if (!_particleSystem) return 0;
    if (!particles.numParticles || !particles.positions.size())
        return 0;

    PxParticleCreationData creationData;
    creationData.numParticles = particles.numParticles;
    creationData.positionBuffer = PxStrideIterator<const PxVec3>(&(particles.positions[0]));
    if (particles.velocities.size() > 0)
        creationData.velocityBuffer = PxStrideIterator<const PxVec3>(&(particles.velocities[0]));
    if (particles.restOffsets.size() > 0)
        creationData.restOffsetBuffer = PxStrideIterator<const PxF32>(&(particles.restOffsets[0]));
    if (particles.indices.size() > 0)
        creationData.indexBuffer = PxStrideIterator<const PxU32>(&(particles.indices[0]));

    if (particles.lifeTime > 0.0f)
    {
        for (unsigned int i = 0; i < particles.indices.size(); ++i)
        {
            ParticleObjectAttr& attr = _particleAttributes[particles.indices[i]];
            attr.second = particles.lifeTime;
        }
        _timelessLife = false;
    }
    else _timelessLife = true;

    std::vector<physx::PxU32> allocatedIndices(particles.numParticles);
    PxU32 numIndices = _indexPool->allocateIndices(
        particles.numParticles, PxStrideIterator<PxU32>(&(allocatedIndices[0])));
    bool ok = _particleSystem->createParticles(creationData);
    return ok ? numIndices : 0;
}

void ParticleUpdater::kill(const std::vector<physx::PxU32>& indices)
{
    unsigned int size = indices.size();
    if (!size || !_particleSystem) return;

    PxStrideIterator<const PxU32> indexData(&(indices[0]));
    _particleSystem->releaseParticles(size, indexData);
    _indexPool->freeIndices(size, indexData);
}

void ParticleUpdater::killAll()
{
    if (!_particleSystem) return;
    _particleSystem->releaseParticles();
    _indexPool->freeIndices();
}

void ParticleUpdater::update(float dt, ParticleDataEx* particles)
{
    if (!_particleSystem)
    {
        OSG_NOTICE << "[ParticleUpdater] Particle system is not created" << std::endl;
        return;
    }

    std::vector<physx::PxU32> toRemoveList;
    PxParticleReadData* readData = _particleSystem->lockParticleReadData();
    if (readData && readData->validParticleRange > 0)
    {
        PxStrideIterator<const PxVec3> positions(readData->positionBuffer);
        PxStrideIterator<const PxVec3> velocities(readData->velocityBuffer);
        PxStrideIterator<const PxVec3> collisionNormals(readData->collisionNormalBuffer);
        PxStrideIterator<const PxF32> restOffsets(readData->restOffsetBuffer);
        PxStrideIterator<const PxParticleFlags> flags(readData->flagsBuffer);

        PxStrideIterator<const PxF32>* densities = NULL;
        if (_isFluidSystem && _dataAccesses&DENSITY_DATA)
        {
            PxParticleFluidReadData* fluidReadData = static_cast<PxParticleFluidReadData*>(readData);
            densities = new PxStrideIterator<const PxF32>(fluidReadData->densityBuffer);
        }
        if (particles) particles->numParticles = readData->nbValidParticles;

        PxU32 bitmapRange = (readData->validParticleRange - 1) >> 5;
        for (PxU32 w = 0; w <= bitmapRange; ++w)
        {
            for (PxU32 b = readData->validParticleBitmap[w]; b > 0; b &= b - 1)
            {
                PxU32 index = (w << 5 | lowestSetBit(b));
                bool shouldBeRemoved = false;

                // Check if a particle is drained or dead
                if (flags[index] & PxParticleFlag::eCOLLISION_WITH_DRAIN ||
                    flags[index] & PxParticleFlag::eSPATIAL_DATA_STRUCTURE_OVERFLOW)
                {
                    toRemoveList.push_back(index);
                    shouldBeRemoved = true;
                }
                else if (!_timelessLife)
                {
                    _particleAttributes[index].second -= dt;
                    if (_particleAttributes[index].second <= 0.0f)
                    {
                        toRemoveList.push_back(index);
                        shouldBeRemoved = true;
                    }
                }

                if (shouldBeRemoved || !particles) continue;
                particles->indices.push_back(index);
                if (positions.ptr()) particles->positions.push_back(positions[index]);
                if (velocities.ptr()) particles->velocities.push_back(velocities[index]);
                if (collisionNormals.ptr()) particles->normals.push_back(collisionNormals[index]);
                if (restOffsets.ptr()) particles->restOffsets.push_back(restOffsets[index]);
                if (densities) particles->densities.push_back((*densities)[index]);
            }
        }
        if (densities) delete densities;
    }
    readData->unlock();

    // Kill outdated particles
    if (toRemoveList.size() > 0)
    {
        if (particles) particles->toRemoveIndices = toRemoveList;
        else kill(toRemoveList);
    }
}

void ParticleUpdater::setupAttributes(const ParticleAttributes& attr, bool isFluid)
{
    _particleSystem->setGridSize(attr.gridSize);
    _particleSystem->setMaxMotionDistance(attr.maxMotionDistance);
    _particleSystem->setRestOffset(attr.restOffset);
    _particleSystem->setContactOffset(attr.contactOffset);
    _particleSystem->setDamping(attr.damping);
    _particleSystem->setRestitution(attr.restitution);
    _particleSystem->setDynamicFriction(attr.dynamicFriction);
    if (isFluid)
    {
        PxParticleFluid* fluid = static_cast<PxParticleFluid*>(_particleSystem);
        fluid->setRestParticleDistance(attr.restParticleDistance);
        fluid->setViscosity(attr.viscosity);
        fluid->setStiffness(attr.stiffness);
    }

#if PX_SUPPORT_GPU_PHYSX
    _particleSystem->setParticleBaseFlag(PxParticleBaseFlag::eGPU, attr.useGPU);
#endif
    _isFluidSystem = isFluid;
}

unsigned int ParticleUpdater::reuseParticleAttributeIndex() const
{
    // FIXME: very foolish method to get unused index
    for (unsigned int i = 0; i < _particleAttributes.size(); ++i)
    {
        if (!_particleAttributes[i].first)
            return i;
    }
    return _particleAttributes.size();
}

#ifdef USE_OSGPARTICLE

/* NativeParticleEmitter */

NativeParticleEmitter::NativeParticleEmitter(ParticleUpdater* updater)
    : osgParticle::ModularEmitter(), _updater(updater), _totalLifeTime(-1.0f)
{
}

void NativeParticleEmitter::emitParticles(double dt)
{
    ParticleUpdater::ParticleData newParticles;
    if (!_updater)
    {
        osgParticle::ModularEmitter::emitParticles(dt);
        return;
    }

    unsigned int maxParticles = _updater->getNumParticleAttributes();
    unsigned int aliveParticles = (unsigned int)
        (getParticleSystem()->numParticles() - getParticleSystem()->numDeadParticles());
    if (maxParticles <= aliveParticles)
    {
        // don't generate more particles, the index pool is full
        return;
    }

    osg::Matrix psToWorld;
    osg::MatrixList worldMats = getParticleSystem()->getWorldMatrices();
    if (!worldMats.empty()) psToWorld = worldMats[0];

    //if ( getReferenceFrame()==RELATIVE_RF )  // TODO: how to handle reference frame?
    {
        int n = getCounter()->numParticlesToCreate(dt);
        for (int i = 0; i < n; ++i)
        {
            osgParticle::Particle* P = getParticleSystem()->createParticle(
                getUseDefaultTemplate() ? 0 : &getParticleTemplate());
            if (!P) continue;

            getPlacer()->place(P);
            getShooter()->shoot(P);
            P->setLifeTime(_totalLifeTime);

            unsigned int index = _updater->reuseParticleAttributeIndex();
            if (index >= maxParticles)
            {
                P->kill();
                continue;
            }

            newParticles.positions.push_back(toPhysicsVec3(P->getPosition() * psToWorld));
            newParticles.velocities.push_back(toPhysicsVec3(
                osg::Matrix::transform3x3(P->getVelocity(), psToWorld)));
            newParticles.indices.push_back(index);
            _updater->setParticleObject(index, this);  // FIXME: only to make the index increase
        }
    }

    newParticles.numParticles = newParticles.positions.size();
    newParticles.lifeTime = _totalLifeTime;
    _updater->generate(newParticles);
}

/* NativeParticleOperator */

NativeParticleOperator::NativeParticleOperator(ParticleUpdater* updater)
    : osgParticle::Operator(), _updater(updater)
{
}

void NativeParticleOperator::operateParticles(osgParticle::ParticleSystem* ps, double dt)
{
    ParticleUpdater::ParticleDataEx particles;
    if (!_updater) return;
    _updater->update(dt, &particles);

    // Update displaying particles
    unsigned int numNativeParticles = ps->numParticles();
    unsigned int numParticles = particles.indices.size();
    if (numParticles <= numNativeParticles)
        numNativeParticles -= numParticles;
    else
    {
        numParticles = numNativeParticles;
        numNativeParticles = 0;
    }

    osg::Matrix worldToPs;
    osg::MatrixList worldMats = ps->getWorldMatrices();
    if (!worldMats.empty()) worldToPs = osg::Matrix::inverse(worldMats[0]);

    unsigned int index = 0;
    //if ( getReferenceFrame()==RELATIVE_RF )  // TODO: how to handle reference frame?
    {
        for (; index < numParticles; ++index)
        {
            unsigned int particleID = particles.indices[index];
            osgParticle::Particle* P = ps->getParticle(particleID);
            if (!P) continue;
            if (!P->isAlive()) continue;
            P->setPosition(toVec3(particles.positions[index]) * worldToPs);
            //P->setVelocity( osg::Matrix::transform3x3(toVec3(particles.velocities[index]), worldToPs) );
            P->setVelocity(osg::Vec3());  // let PhysX update the position instead
        }
    }

    // Update removed particles
    numParticles = particles.toRemoveIndices.size();
    if (numParticles > 0 && numNativeParticles > 0)
    {
        if (numParticles > numNativeParticles) numParticles = numNativeParticles;
        for (unsigned int i = 0; i < numParticles; ++i)
        {
            unsigned int particleID = particles.toRemoveIndices[i];
            osgParticle::Particle* P = ps->getParticle(particleID);
            if (P) P->kill();
            _updater->setParticleObject(particleID, NULL);
        }
        _updater->kill(particles.toRemoveIndices);
    }
    else if (numNativeParticles > 0)
    {
        // FIXME: are unhandled particles all dead?
    }
}

#endif
