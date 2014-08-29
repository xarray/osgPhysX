#ifndef PHYSICS_PARTICLEUPDATER
#define PHYSICS_PARTICLEUPDATER

#define USE_OSGPARTICLE 1

#ifdef USE_OSGPARTICLE
#   include <osgParticle/ModularEmitter>
#   include <osgParticle/Operator>
#endif

#include <osg/Referenced>
#include "Engine.h"

namespace osgPhysics
{

/** The physics based particle updater which provides convenient particle functions */
class ParticleUpdater : public osg::Referenced
{
public:
    /** Particle system attributes for creation */
    struct ParticleAttributes
    {
        bool useGPU;
        unsigned int maxParticles;
        float gridSize;
        float maxMotionDistance;
        float restOffset;
        float contactOffset;
        float damping;
        float restitution;
        float dynamicFriction;
        
        // Fluid related attributes
        float restParticleDistance;
        float viscosity;
        float stiffness;
        
        ParticleAttributes( unsigned int maxP=1000 ) { setDefaults(maxP); }
        void setDefaults( unsigned int maxP );
        void setFluidDefaults( unsigned int maxP );
    };
    
    /** Particle data for generation on-the-fly, numParticles and positions and indices are required */
    struct ParticleData
    {
        physx::PxU32 numParticles;
        physx::PxReal lifeTime;  // negative value means unlimited life
        std::vector<physx::PxVec3> positions;
        std::vector<physx::PxVec3> velocities;
        std::vector<physx::PxF32> restOffsets;
        std::vector<physx::PxU32> indices;
        
        ParticleData() : numParticles(0), lifeTime(-1.0f) {}
        ParticleData( physx::PxU32 num, physx::PxReal life=-1.0f )
        { numParticles = num; lifeTime = life; positions.resize(num); }
    };
    
    /** Particle data for storing fetched particles */
    struct ParticleDataEx : public ParticleData
    {
        std::vector<physx::PxVec3> normals;
        std::vector<physx::PxF32> densities;
        std::vector<physx::PxU32> toRemoveIndices;
        ParticleDataEx() : ParticleData() {}
    };
    
    ParticleUpdater();
    
    physx::PxParticleBase* getParticleBase() { return _particleSystem; }
    const physx::PxParticleBase* getParticleBase() const { return _particleSystem; }
    
    physx::PxParticleExt::IndexPool* getIndexPool() { return _indexPool; }
    const physx::PxParticleExt::IndexPool* getIndexPool() const { return _indexPool; }
    
    enum DataAccess
    {
        POSITION_DATA = 0x1,
        VELOCITY_DATA = 0x2,
        REST_OFFSET_DATA = 0x4,
        DENSITY_DATA = 0x8,
        NORMAL_DATA = 0x10,
        FLAGS_DATA = 0x20
    };
    
    /** Set what kinds of data are enabled to be read back, should be called after created
        Default accesses include position & flags as indicated by PhysX
    */
    void setDataAccess( int dataAccesses );
    
    /** Set an actor as the particle drain or not */
    bool setAsDrain( physx::PxRigidActor* actor, bool b );
    
    /** Create a normal particle system */
    bool create( const ParticleAttributes& attr, bool supportRestOffset=false );
    
    /** Create a particle fluid system */
    bool createFluid( const ParticleAttributes& attr, bool supportRestOffset=false );
    
    /** Generate new particles */
    unsigned int generate( const ParticleData& particles );
    
    /** Kill some particles */
    void kill( const std::vector<physx::PxU32>& indices );
    void killAll();
    
    /** Update particles and fetch result if required
        If particles is set to NULL, deletion of outdated particles will be automatically done;
        otherwise you will have to manually decide whether to kill them
    */
    virtual void update( float dt, ParticleDataEx* particles );
    
    /** Set/get specific particle attributes */
    void setParticleObject( unsigned int i, void* p ) { _particleAttributes[i].first = p; }
    void setParticleLifeTime( unsigned int i, float life ) { _particleAttributes[i].second = life; }
    void* getParticleObject( unsigned int i, float* life=NULL ) const
    {
        const ParticleObjectAttr& attr = _particleAttributes[i];
        if (life) *life = attr.second; return attr.first;
    }
    
    /** Get a unused particle attribute for storing information */
    unsigned int reuseParticleAttributeIndex() const;
    unsigned int getNumParticleAttributes() const { return _particleAttributes.size(); }
    
    /** Get the actor to be added to scene */
    physx::PxParticleBase* getActor() { return _particleSystem; }
    const physx::PxParticleBase* getActor() const { return _particleSystem; }
    
protected:
    virtual ~ParticleUpdater();
    void setupAttributes( const ParticleAttributes& attr, bool isFluid );
    
    typedef std::pair<void*, physx::PxReal> ParticleObjectAttr;
    std::vector<ParticleObjectAttr> _particleAttributes;
    
    physx::PxParticleBase* _particleSystem;
    physx::PxParticleExt::IndexPool* _indexPool;
    int _dataAccesses;
    bool _timelessLife;
    bool _isFluidSystem;
};

#ifdef USE_OSGPARTICLE

class NativeParticleEmitter : public osgParticle::ModularEmitter
{
public:
    NativeParticleEmitter( ParticleUpdater* updater=NULL );
    
    void setParticleLifeTime( float life ) { _totalLifeTime = life; }
    float getParticleLifeTime() const { return _totalLifeTime; }
    
    void setParticleUpdater( ParticleUpdater* pu ) { _updater = pu; }
    ParticleUpdater* getParticleUpdater() { return _updater.get(); }
    const ParticleUpdater* getParticleUpdater() const { return _updater.get(); }
    
protected:
    virtual void emitParticles( double dt );
    osg::ref_ptr<ParticleUpdater> _updater;
    float _totalLifeTime;
};

class NativeParticleOperator : public osgParticle::Operator
{
public:
    NativeParticleOperator( ParticleUpdater* updater=NULL );
    NativeParticleOperator( const NativeParticleOperator& copy,
                            const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY )
    :   osgParticle::Operator(copy, copyop), _updater(copy._updater) {}
    
    META_Object( osgPhysics, NativeParticleOperator );
    
    virtual void operateParticles( osgParticle::ParticleSystem* ps, double dt );
    virtual void operate( osgParticle::Particle* P, double dt ) {}
    
    void setParticleUpdater( ParticleUpdater* pu ) { _updater = pu; }
    ParticleUpdater* getParticleUpdater() { return _updater.get(); }
    const ParticleUpdater* getParticleUpdater() const { return _updater.get(); }
    
protected:
    osg::ref_ptr<ParticleUpdater> _updater;
};

#endif

}

#endif
