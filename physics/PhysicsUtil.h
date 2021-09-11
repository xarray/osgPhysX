#ifndef PHYSICS_PHYSICSUTIL
#define PHYSICS_PHYSICSUTIL

#include <osg/Plane>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Transform>
#include "Engine.h"

namespace osgPhysics
{

    inline physx::PxVec3 toPhysicsVec3(const osg::Vec3& v)
    {
        return physx::PxVec3(v[0], v[1], v[2]);
    }

    inline osg::Vec3 toVec3(const physx::PxVec3& v)
    {
        return osg::Vec3(v[0], v[1], v[2]);
    }

    /** Convert OpenSceneGraph matrix to Physics matrix */
    extern physx::PxMat44 toPxMatrix(const osg::Matrix& matrix);

    /** Convert Physics matrix to OpenSceneGraph matrix */
    extern osg::Matrix toMatrix(const physx::PxMat44& pmatrix);

    /** Cook and create new convex mesh */
    extern physx::PxConvexMesh* createConvexMesh(
        const std::vector<physx::PxVec3>& verts,
        physx::PxConvexFlags flags = physx::PxConvexFlag::eCOMPUTE_CONVEX | physx::PxConvexFlag::eINFLATE_CONVEX);

    /** Cook and create new convex mesh from node */
    extern physx::PxConvexMesh* createConvexMesh(
        osg::Node& node,
        physx::PxConvexFlags flags = physx::PxConvexFlag::eCOMPUTE_CONVEX | physx::PxConvexFlag::eINFLATE_CONVEX);

    /** Cook and create new height field, where lowerTriangleData & upperTriangleData contain material index data
        and hole flag of the lower/upper triangle of each height field cell
    */
    extern physx::PxHeightField* createHeightField(
        unsigned int numRows, unsigned int numColumns, const float* heightData,
        const char* lowerTriangleData = 0, const char* upperTriangleData = 0, float thickness = -1.0f);

    /** Cook and create new convex mesh as a box */
    extern physx::PxConvexMesh* createBoxMesh(const osg::Vec3& c, const osg::Vec3& dim);

    /** Cook and create new convex mesh as a cylinder */
    extern physx::PxConvexMesh* createCylinderMesh(const osg::Vec3& c, float radius, float width, unsigned int samples);

    /** Cook and create new physics triangle mesh */
    extern physx::PxTriangleMesh* createTriangleMesh(const std::vector<physx::PxVec3>& verts,
        const std::vector<physx::PxU32>& indices);

    /** Cook and create new physics triangle mesh from node */
    extern physx::PxTriangleMesh* createTriangleMesh(osg::Node& node);

    /** Cook and create new physics cloth fabric */
    extern physx::PxClothFabric* createClothFabric(const std::vector<physx::PxVec3>& verts,
        const std::vector<physx::PxU32>& indices,
        const osg::Vec3& gravity);

    /** Create a physics scene */
    extern physx::PxScene* createScene(const osg::Vec3& gravity,
        const physx::PxSimulationFilterShader& filter = &physx::PxDefaultSimulationFilterShader,
        physx::PxSceneFlags flags = physx::PxSceneFlags(), unsigned int numThreads = 1, bool useGPU = false);

    /** Create a physics actor (static if density is 0) */
    extern physx::PxRigidActor* createActor(const physx::PxGeometry& geom, double density, physx::PxMaterial* mtl = 0);

    /** Create a physics actor with a materials list */
    extern physx::PxRigidActor* createActor(const physx::PxGeometry& geom, const std::vector<physx::PxMaterial*>& mtlList);

    /** Create a physics box actor */
    extern physx::PxRigidActor* createBoxActor(const osg::Vec3& dim, double density, physx::PxMaterial* mtl = 0);

    /** Create a physics sphere actor */
    extern physx::PxRigidActor* createSphereActor(double radius, double density, physx::PxMaterial* mtl = 0);

    /** Create a physics capsule actor */
    extern physx::PxRigidActor* createCapsuleActor(double radius, double height, double density, physx::PxMaterial* mtl = 0);

    /** Create a physics convex mesh actor */
    extern physx::PxRigidActor* createConvexMeshActor(physx::PxConvexMesh* mesh, double density, physx::PxMaterial* mtl = 0);

    /** Create a physics height field actor */
    extern physx::PxRigidActor* createHeightFieldActor(
        physx::PxHeightField* hf, float xScale, float yScale, float zScale, physx::PxMaterial* mtl = 0);
    extern physx::PxRigidActor* createHeightFieldActor(
        physx::PxHeightField* hf, float xScale, float yScale, float zScale, const std::vector<physx::PxMaterial*>& mtlList);

    /** Create a static triangle mesh actor */
    extern physx::PxRigidActor* createTriangleMeshActor(physx::PxTriangleMesh* mesh, physx::PxMaterial* mtl = 0);

    /** Create a static ground plane actor */
    extern physx::PxRigidActor* createPlaneActor(const osg::Plane& plane, physx::PxMaterial* mtl = 0);

    /** Set simulation filter for a certain actor */
    extern bool createSimulationFilter(physx::PxRigidActor* actor, const physx::PxFilterData& filter);

    /** Create a node for specified actor */
    extern osg::Node* createNodeForActor(physx::PxRigidActor* actor);

    /** Convenient function to create geometry */
    extern osg::Geometry* createGeometry(osg::Vec3Array* va, osg::Vec3Array* na, osg::Vec2Array* ta,
        osg::PrimitiveSet* p, bool useVBO = true);

    /** A visitor for collecting vital geometry data (vertices and indices) in the node subgraph */
    struct GeometryDataCollector : public osg::NodeVisitor
    {
        GeometryDataCollector();
        virtual void apply(osg::Transform& transform);
        virtual void apply(osg::Geode& node);

        inline void pushMatrix(osg::Matrix& matrix) { matrixStack.push_back(matrix); }
        inline void popMatrix() { matrixStack.pop_back(); }

        typedef std::map<osg::Vec3, unsigned int> VertexMap;
        VertexMap vertexMap;
        std::vector<osg::Vec3> vertices;

        struct GeometryFace { unsigned int indices[4]; };
        std::vector<GeometryFace> faces;

        osg::BoundingBox bound;
        int numTotalVertices;

        typedef std::vector<osg::Matrix> MatrixStack;
        MatrixStack matrixStack;
    };

    /** The memory output stream class */
    class MemoryOutputStream : public physx::PxOutputStream
    {
    public:
        MemoryOutputStream();
        virtual ~MemoryOutputStream();

        virtual physx::PxU32 write(const void* src, physx::PxU32 count);
        physx::PxU32 getSize() const { return _size; }
        physx::PxU8* getData() const { return _data; }

    private:
        physx::PxU8* _data;
        physx::PxU32 _size;
        physx::PxU32 _capacity;
    };

    /** The memory input data class */
    class MemoryInputData : public physx::PxInputData
    {
    public:
        MemoryInputData(physx::PxU8* data, physx::PxU32 length);
        virtual ~MemoryInputData();

        virtual physx::PxU32 read(void* dest, physx::PxU32 count);
        virtual void seek(physx::PxU32 pos);
        virtual physx::PxU32 getLength() const { return _size; }
        virtual physx::PxU32 tell() const { return _pos; }

    private:
        const physx::PxU8* _data;
        physx::PxU32 _size;
        physx::PxU32 _pos;
    };

}

#endif
