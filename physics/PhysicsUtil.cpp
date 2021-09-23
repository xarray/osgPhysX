#include <osg/io_utils>
#include <osg/TriangleFunctor>
#include <osg/MatrixTransform>
#include "PhysicsUtil.h"
#include "Vehicle.h"
#include "CharacterController.h"
#include <algorithm>
#include <iostream>

using namespace physx;
using namespace osgPhysics;

#define SDK_OBJ (Engine::instance()->getPhysicsSDK())
#define SDK_COOK (Engine::instance()->getOrCreateCooking())
#define DEF_MTL (Engine::instance()->getDefaultMaterial())

/* GeometryDataCollector */

struct CollectFaceOperator
{
    void operator()(const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3)
    {
        if (v1 == v2 || v2 == v3 || v3 == v1) return;
        GeometryDataCollector::GeometryFace face;
        face.indices[0] = getOrCreateVertex(v1 * matrix);
        face.indices[1] = getOrCreateVertex(v2 * matrix);
        face.indices[2] = getOrCreateVertex(v3 * matrix);
        face.indices[3] = 0;  // 0 for triangle
        collector->faces.push_back(face);
    }

    unsigned int getOrCreateVertex(const osg::Vec3& p)
    {
        unsigned int index = 0;
        if (collector->vertexMap.find(p) == collector->vertexMap.end())
        {
            index = collector->vertexMap.size();
            collector->vertexMap[p] = index;
            collector->vertices.push_back(p);
            collector->bound.expandBy(p);
        }
        else
            index = collector->vertexMap[p];
        return index;
    }

    GeometryDataCollector* collector;
    osg::Matrix matrix;
};

GeometryDataCollector::GeometryDataCollector()
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), numTotalVertices(0)
{}

void GeometryDataCollector::apply(osg::Transform& transform)
{
    osg::Matrix matrix;
    if (!matrixStack.empty()) matrix = matrixStack.back();
    transform.computeLocalToWorldMatrix(matrix, this);

    pushMatrix(matrix);
    traverse(transform);
    popMatrix();
}

void GeometryDataCollector::apply(osg::Geode& node)
{
    //osg::Matrix matrix = osg::computeLocalToWorld( node.getParentalNodePaths()[0] );
    osg::Matrix matrix;
    if (matrixStack.size() > 0) matrix = matrixStack.back();
    for (unsigned int i = 0; i < node.getNumDrawables(); ++i)
    {
        osg::Geometry* geom = node.getDrawable(i)->asGeometry();
        if (geom)
        {
            osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
            numTotalVertices += (va ? va->size() : 0);

            osg::TriangleFunctor<CollectFaceOperator> functor;
            functor.collector = this;
            functor.matrix = matrix;
            geom->accept(functor);
        }
        else
        {
            osg::TriangleFunctor<CollectFaceOperator> functor;
            functor.collector = this;
            functor.matrix = matrix;
            node.getDrawable(i)->accept(functor);
        }
    }
    traverse(node);
}

MemoryOutputStream::MemoryOutputStream()
    : _data(NULL), _size(0), _capacity(0)
{
}

/* MemoryOutputStream */

MemoryOutputStream::~MemoryOutputStream()
{
    if (_data) delete[] _data;
}

PxU32 MemoryOutputStream::write(const void* src, PxU32 count)
{
    PxU32 expectedSize = _size + count;
    if (expectedSize > _capacity)
    {
        _capacity = expectedSize + 4096;
        PxU8* newData = new PxU8[_capacity];
        if (newData)
        {
            memcpy(newData, _data, _size);
            delete[] _data;
        }
        _data = newData;
    }
    memcpy(_data + _size, src, count);
    _size += count;
    return count;
}

/* MemoryInputData */

MemoryInputData::MemoryInputData(PxU8* data, PxU32 length)
    : _data(data), _size(length), _pos(0)
{
}

MemoryInputData::~MemoryInputData()
{
}

PxU32 MemoryInputData::read(void* dest, PxU32 count)
{
    PxU32 length = PxMin<PxU32>(count, _size - _pos);
    memcpy(dest, _data + _pos, length);
    _pos += length;
    return length;
}

void MemoryInputData::seek(PxU32 pos)
{
    _pos = PxMin<PxU32>(_size, pos);
}

namespace osgPhysics
{

    PxMat44 toPxMatrix(const osg::Matrix& matrix)
    {
        PxReal d[16];
        for (int i = 0; i < 16; ++i) d[i] = *(matrix.ptr() + i);
        return PxMat44(d);
    }

    osg::Matrix toMatrix(const PxMat44& pmatrix)
    {
        double m[16];
        for (int i = 0; i < 16; ++i) m[i] = *(pmatrix.front() + i);
        return osg::Matrix(&m[0]);
    }

    PxConvexMesh* createConvexMesh(const std::vector<PxVec3>& verts, PxConvexFlags flags)
    {
        PxConvexMeshDesc convexDesc;
        convexDesc.points.count = verts.size();
        convexDesc.points.stride = sizeof(PxVec3);
        convexDesc.points.data = &(verts[0]);
        convexDesc.flags = flags;

        MemoryOutputStream writeBuffer;
        if (!SDK_COOK->cookConvexMesh(convexDesc, writeBuffer)) return NULL;

        MemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        return SDK_OBJ->createConvexMesh(readBuffer);
    }

    PxConvexMesh* createConvexMesh(osg::Node& node, PxConvexFlags flags)
    {
        GeometryDataCollector collector;
        node.accept(collector);

        std::vector<PxVec3> verts;
        for (unsigned int i = 0; i < collector.vertices.size(); ++i)
        {
            const osg::Vec3& v = collector.vertices[i];
            verts.push_back(PxVec3(v[0], v[1], v[2]));
        }

        std::vector<PxU32> indices;
        for (unsigned int i = 0; i < collector.faces.size(); ++i)
        {
            const GeometryDataCollector::GeometryFace& f = collector.faces[i];
            indices.push_back(f.indices[0]);
            indices.push_back(f.indices[1]);
            indices.push_back(f.indices[2]);
        }
        if (!verts.size() || !indices.size()) return NULL;

        PxConvexMeshDesc convexDesc;
        convexDesc.points.count = verts.size();
        convexDesc.points.stride = sizeof(PxVec3);
        convexDesc.points.data = &(verts[0]);
        convexDesc.indices.count = indices.size() / 3;
        convexDesc.indices.stride = 3 * sizeof(PxU32);
        convexDesc.indices.data = &(indices[0]);
        convexDesc.flags = flags;

        MemoryOutputStream writeBuffer;
        if (!SDK_COOK->cookConvexMesh(convexDesc, writeBuffer)) return NULL;

        MemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        return SDK_OBJ->createConvexMesh(readBuffer);
    }

    PxHeightField* createHeightField(unsigned int numRows, unsigned int numColumns, const float* heightData,
        const char* lowerTriangleData, const char* upperTriangleData, float thickness)
    {
        PxHeightFieldDesc heightFieldDesc;
        heightFieldDesc.nbColumns = numColumns;
        heightFieldDesc.nbRows = numRows;
        heightFieldDesc.format = PxHeightFieldFormat::eS16_TM;
        heightFieldDesc.thickness = thickness;

        PxHeightFieldSample* samplesData = (PxHeightFieldSample*)malloc(
            sizeof(PxHeightFieldSample) * numColumns * numRows);
        heightFieldDesc.samples.data = samplesData;
        heightFieldDesc.samples.stride = sizeof(PxHeightFieldSample);

        PxU8* ptr = (PxU8*)heightFieldDesc.samples.data;
        for (PxU32 row = 0; row < numRows; ++row)
        {
            for (PxU32 col = 0; col < numColumns; ++col)
            {
                unsigned int index = row * numColumns + col;
                PxHeightFieldSample* sample = (PxHeightFieldSample*)ptr;
                sample->height = (physx::PxI16)*(heightData + index);
                sample->materialIndex0 = lowerTriangleData ? *(lowerTriangleData + index) : 0;
                sample->materialIndex1 = upperTriangleData ? *(upperTriangleData + index) : 0;
                ptr += heightFieldDesc.samples.stride;
            }
        }

        PxHeightField* heightField = SDK_COOK->createHeightField(
            heightFieldDesc, SDK_OBJ->getPhysicsInsertionCallback());
        free(samplesData);
        return heightField;
    }

    PxConvexMesh* createBoxMesh(const osg::Vec3& c, const osg::Vec3& dim)
    {
        PxVec3 center(c[0], c[1], c[2]), halfDim(dim[0] * 0.5f, dim[1] * 0.5f, dim[2] * 0.5f);
        std::vector<PxVec3> verts;
        verts.push_back(center + PxVec3(-halfDim.x, -halfDim.y, -halfDim.z));
        verts.push_back(center + PxVec3(-halfDim.x, -halfDim.y, +halfDim.z));
        verts.push_back(center + PxVec3(-halfDim.x, +halfDim.y, -halfDim.z));
        verts.push_back(center + PxVec3(-halfDim.x, +halfDim.y, +halfDim.z));
        verts.push_back(center + PxVec3(+halfDim.x, -halfDim.y, -halfDim.z));
        verts.push_back(center + PxVec3(+halfDim.x, -halfDim.y, +halfDim.z));
        verts.push_back(center + PxVec3(+halfDim.x, +halfDim.y, -halfDim.z));
        verts.push_back(center + PxVec3(+halfDim.x, +halfDim.y, +halfDim.z));
        return createConvexMesh(verts);
    }

    PxConvexMesh* createCylinderMesh(const osg::Vec3& c, float radius, float width, unsigned int samples)
    {
        PxVec3 center(c[0], c[1], c[2]);
        std::vector<PxVec3> verts;
        float theta = 2.0f * osg::PI / (float)samples;
        for (unsigned int i = 0; i < samples; ++i)
        {
            float cosine = radius * cos(theta * (float)i);
            float sine = radius * sin(theta * (float)i);
            verts.push_back(center + PxVec3(-0.5f * width, cosine, sine));
            verts.push_back(center + PxVec3(+0.5f * width, cosine, sine));
        }
        return createConvexMesh(verts);
    }

    PxTriangleMesh* createTriangleMesh(const std::vector<PxVec3>& verts, const std::vector<PxU32>& indices)
    {
        PxTriangleMeshDesc meshDesc;
        meshDesc.points.count = verts.size();
        meshDesc.points.stride = sizeof(PxVec3);
        meshDesc.points.data = &(verts[0]);
        meshDesc.triangles.count = indices.size() / 3;
        meshDesc.triangles.stride = 3 * sizeof(PxU32);
        meshDesc.triangles.data = &(indices[0]);

        MemoryOutputStream writeBuffer;
        if (!SDK_COOK->cookTriangleMesh(meshDesc, writeBuffer)) return NULL;

        MemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        return SDK_OBJ->createTriangleMesh(readBuffer);
    }

    PxTriangleMesh* createTriangleMesh(osg::Node& node)
    {
        GeometryDataCollector collector;
        node.accept(collector);

        std::vector<PxVec3> verts;
        for (unsigned int i = 0; i < collector.vertices.size(); ++i)
        {
            const osg::Vec3& v = collector.vertices[i];
            verts.push_back(PxVec3(v[0], v[1], v[2]));
        }

        std::vector<PxU32> indices;
        for (unsigned int i = 0; i < collector.faces.size(); ++i)
        {
            const GeometryDataCollector::GeometryFace& f = collector.faces[i];
            indices.push_back(f.indices[0]);
            indices.push_back(f.indices[1]);
            indices.push_back(f.indices[2]);
        }
        if (!verts.size() || !indices.size()) return NULL;
        return createTriangleMesh(verts, indices);
    }

    PxClothFabric* createClothFabric(const std::vector<PxVec3>& verts, const std::vector<PxU32>& indices,
        const osg::Vec3& gravity)
    {
        PxClothMeshDesc meshDesc;
        meshDesc.points.count = verts.size();
        meshDesc.points.stride = sizeof(PxVec3);
        meshDesc.points.data = &(verts[0]);
        meshDesc.triangles.count = indices.size() / 3;
        meshDesc.triangles.stride = 3 * sizeof(PxU32);
        meshDesc.triangles.data = &(indices[0]);

        PxVec3 g(gravity[0], gravity[1], gravity[2]);
        return PxClothFabricCreate(*SDK_OBJ, meshDesc, g);
    }

    PxScene* createScene(const osg::Vec3& gravity, const PxSimulationFilterShader& filter,
        physx::PxSceneFlags flags, unsigned int numThreads, bool useGPU)
    {
        PxSceneDesc sceneDesc(SDK_OBJ->getTolerancesScale());
        sceneDesc.gravity = PxVec3(gravity[0], gravity[1], gravity[2]);
        sceneDesc.filterShader = filter;
        sceneDesc.flags |= flags;

        if (useGPU)
        {
            PxCudaContextManager* cudaManager = Engine::instance()->getOrCreateCudaContextManager();
            if (cudaManager) sceneDesc.gpuDispatcher = cudaManager->getGpuDispatcher();
        }

        if (!sceneDesc.gpuDispatcher && !sceneDesc.cpuDispatcher)
        {
            PxDefaultCpuDispatcher* defCpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
            if (!defCpuDispatcher)
                OSG_WARN << "Failed to create default Cpu dispatcher." << std::endl;
            sceneDesc.cpuDispatcher = defCpuDispatcher;
        }

        PxScene* scene = SDK_OBJ->createScene(sceneDesc);
        if (!scene)
        {
            OSG_WARN << "Failed to create the physics world." << std::endl;
            return NULL;
        }
        scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
        scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
        return scene;
    }

    PxRigidActor* createActor(const PxGeometry& geom, double density, PxMaterial* mtl)
    {
        if (density > 0.0)
        {
            PxRigidDynamic* actor = PxCreateDynamic(*SDK_OBJ, PxTransform(PxIdentity), geom,
                mtl ? *mtl : *DEF_MTL, density);
            return actor;
        }
        else
        {
            PxRigidStatic* actor = PxCreateStatic(*SDK_OBJ, PxTransform(PxIdentity), geom,
                mtl ? *mtl : *DEF_MTL);
            return actor;
        }
    }

    PxRigidActor* createActor(const PxGeometry& geom, const std::vector<PxMaterial*>& mtlList)
    {
        PxShape* shape = NULL;
        if (mtlList.empty())
            shape = SDK_OBJ->createShape(geom, *DEF_MTL, true);
        else
            shape = SDK_OBJ->createShape(geom, &mtlList[0], mtlList.size(), true);

        PxRigidStatic* actor = SDK_OBJ->createRigidStatic(PxTransform(PxIdentity));
        actor->attachShape(*shape); shape->release();
        return actor;
    }

    PxRigidActor* createBoxActor(const osg::Vec3& dim, double density, PxMaterial* mtl)
    {
        PxBoxGeometry geometry(PxVec3(dim[0] * 0.5f, dim[1] * 0.5f, dim[2] * 0.5f));
        return createActor(geometry, density, mtl);
    }

    PxRigidActor* createSphereActor(double radius, double density, PxMaterial* mtl)
    {
        PxSphereGeometry geometry(radius);
        return createActor(geometry, density, mtl);
    }

    PxRigidActor* createCapsuleActor(double radius, double height, double density, PxMaterial* mtl)
    {
        PxCapsuleGeometry geometry(radius, height*0.5);
        return createActor(geometry, density, mtl);
    }

    PxRigidActor* createConvexMeshActor(PxConvexMesh* mesh, double density, PxMaterial* mtl)
    {
        PxConvexMeshGeometry geometry(mesh);
        return createActor(geometry, density, mtl);
    }

    PxRigidActor* createHeightFieldActor(PxHeightField* hf, float xScale, float yScale,
                                         float zScale, PxMaterial* mtl)
    {
        PxHeightFieldGeometry geometry;
        geometry.heightField = hf;
        geometry.rowScale = xScale;
        geometry.columnScale = yScale;
        geometry.heightScale = zScale;
        PxRigidStatic* actor = PxCreateStatic(
            *SDK_OBJ, PxTransform(PxQuat((float)osg::PI_2, PxVec3(1.0f, 0.0f, 0.0f))), geometry,
            mtl ? *mtl : *DEF_MTL);
        return actor;
    }

    PxRigidActor* createHeightFieldActor(PxHeightField* hf, float xScale, float yScale, float zScale,
                                         const std::vector<PxMaterial*>& mtlList)
    {
        PxHeightFieldGeometry geometry;
        geometry.heightField = hf;
        geometry.rowScale = xScale;
        geometry.columnScale = yScale;
        geometry.heightScale = zScale;

        PxShape* shape = SDK_OBJ->createShape(geometry, &mtlList[0], mtlList.size(), true);
        PxRigidStatic* actor = SDK_OBJ->createRigidStatic(
            PxTransform(PxQuat((float)osg::PI_2, PxVec3(1.0f, 0.0f, 0.0f))));
        actor->attachShape(*shape); shape->release();
        return actor;
    }

    PxRigidActor* createTriangleMeshActor(PxTriangleMesh* mesh, PxMaterial* mtl)
    {
        PxTriangleMeshGeometry geometry(mesh);
        PxRigidStatic* actor = PxCreateStatic(*SDK_OBJ, PxTransform(PxIdentity), geometry,
            mtl ? *mtl : *DEF_MTL);
        return actor;
    }

    PxRigidActor* createPlaneActor(const osg::Plane& plane, PxMaterial* mtl)
    {
        osg::Quat q;
        q.makeRotate(osg::Vec3f(1.0f, 0.0f, 0.0f), osg::Vec3f(plane[0], plane[1], plane[2]));
        PxTransform pose(PxVec3(0.0f, 0.0f, -plane[3]), PxQuat(q[0], q[1], q[2], q[3]));
        PxRigidStatic* actor = PxCreateStatic(*SDK_OBJ, pose, PxPlaneGeometry(), mtl ? *mtl : *DEF_MTL);
        return actor;
    }

    bool createSimulationFilter(PxRigidActor* actor, const PxFilterData& filter)
    {
        if (!actor) return false;
        std::vector<PxShape*> shapes(actor->getNbShapes());

        PxU32 num = actor->getShapes(&(shapes[0]), actor->getNbShapes());
        for (PxU32 i = 0; i < num; ++i)
            shapes[i]->setSimulationFilterData(filter);
        return true;
    }

    osg::Node* createNodeForActor(PxRigidActor* actor)
    {
        if (!actor) return NULL;
        std::vector<PxShape*> shapes(actor->getNbShapes());

        osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
        transform->setMatrix(toMatrix(PxMat44(actor->getGlobalPose())));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        transform->addChild(geode.get());

        PxU32 num = actor->getShapes(&(shapes[0]), actor->getNbShapes());
        for (PxU32 i = 0; i < num; ++i)
        {
            PxShape* shape = shapes[i];
            osg::Matrix localMatrix = toMatrix(PxMat44(actor->getGlobalPose()));
            osg::Vec3 localPos = toVec3(shape->getLocalPose().p);
            osg::Quat localQuat(shape->getLocalPose().q.x, shape->getLocalPose().q.y,
                shape->getLocalPose().q.z, shape->getLocalPose().q.w);

            switch (shape->getGeometryType())
            {
            case PxGeometryType::eSPHERE:
                {
                    PxSphereGeometry sphere;
                    shape->getSphereGeometry(sphere);

                    osg::Sphere* sphereShape = new osg::Sphere(localPos, sphere.radius);
                    geode->addDrawable(new osg::ShapeDrawable(sphereShape));
                }
                break;
            case PxGeometryType::ePLANE:
                // TODO
                break;
            case PxGeometryType::eCAPSULE:
                {
                    PxCapsuleGeometry capsule;
                    shape->getCapsuleGeometry(capsule);

                    osg::Capsule* capsuleShape = new osg::Capsule(
                        localPos, capsule.radius, capsule.halfHeight * 2.0f);
                    capsuleShape->setRotation(localQuat);
                    geode->addDrawable(new osg::ShapeDrawable(capsuleShape));
                }
                break;
            case PxGeometryType::eBOX:
                {
                    PxBoxGeometry box;
                    shape->getBoxGeometry(box);

                    osg::Box* boxShape = new osg::Box(localPos,
                        box.halfExtents[0] * 2.0f, box.halfExtents[1] * 2.0f, box.halfExtents[2] * 2.0f);
                    boxShape->setRotation(localQuat);
                    geode->addDrawable(new osg::ShapeDrawable(boxShape));
                }
                break;
            case PxGeometryType::eCONVEXMESH:
                {
                    PxConvexMeshGeometry convexMeshGeom;
                    shape->getConvexMeshGeometry(convexMeshGeom);
                    // TODO: consider convexMeshGeom.scale

                    PxConvexMesh* convexMesh = convexMeshGeom.convexMesh;
                    if (convexMesh)
                    {
                        /*for ( unsigned int i=0; i<convexMesh->getNbPolygons(); ++i )
                        {

                        }*/
                        // TODO
                    }
                }
                break;
            case PxGeometryType::eTRIANGLEMESH:
                {
                    PxTriangleMeshGeometry triangleMeshGeom;
                    shape->getTriangleMeshGeometry(triangleMeshGeom);
                    // TODO: consider triangleMeshGeom.scale

                    PxTriangleMesh* triangleMesh = triangleMeshGeom.triangleMesh;
                    if (triangleMesh)
                    {
                        osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array(triangleMesh->getNbVertices());
                        for (unsigned int i = 0; i < va->size(); ++i)
                            (*va)[i] = toVec3(*(triangleMesh->getVertices() + i)) * localMatrix;

                        osg::ref_ptr<osg::DrawElements> de;
                        if (triangleMesh->getTriangleMeshFlags()&PxTriangleMeshFlag::e16_BIT_INDICES)
                        {
                            osg::DrawElementsUShort* de16 = new osg::DrawElementsUShort(GL_TRIANGLES);
                            de = de16;

                            const PxU16* indices = (const PxU16*)triangleMesh->getTriangles();
                            for (unsigned int i = 0; i < triangleMesh->getNbTriangles(); ++i)
                            {
                                de16->push_back(indices[3 * i + 0]);
                                de16->push_back(indices[3 * i + 1]);
                                de16->push_back(indices[3 * i + 2]);
                            }
                        }
                        else
                        {
                            osg::DrawElementsUInt* de32 = new osg::DrawElementsUInt(GL_TRIANGLES);
                            de = de32;

                            const PxU32* indices = (const PxU32*)triangleMesh->getTriangles();
                            for (unsigned int i = 0; i < triangleMesh->getNbTriangles(); ++i)
                            {
                                de32->push_back(indices[3 * i + 0]);
                                de32->push_back(indices[3 * i + 1]);
                                de32->push_back(indices[3 * i + 2]);
                            }
                        }
                        geode->addDrawable(createGeometry(va.get(), NULL, NULL, de.get()));
                    }
                }
                break;
            case PxGeometryType::eHEIGHTFIELD:
                {
                    PxHeightFieldGeometry hfGeom;
                    shape->getHeightFieldGeometry(hfGeom);

                    PxHeightField* hf = hfGeom.heightField;
                    if (hf)
                    {
                        osg::HeightField* grid = new osg::HeightField;
                        grid->allocate(hf->getNbColumns(), hf->getNbRows());
                        grid->setOrigin(osg::Vec3(0.0f, 0.0f, 0.0f));
                        grid->setXInterval(hfGeom.columnScale);
                        grid->setYInterval(hfGeom.rowScale);

                        float zScale = hfGeom.heightScale;
                        for (unsigned int r = 0; r < hf->getNbRows(); ++r)
                            for (unsigned int c = 0; c < hf->getNbColumns(); ++c)
                            {
                                const PxHeightFieldSample& s = hf->getSample(r, c);
                                grid->setHeight(c, r, (float)s.height * zScale);
                            }
                        
                        geode->addDrawable(new osg::ShapeDrawable(grid));
                        transform->setMatrix(osg::Matrix::rotate(-osg::PI_2, osg::Y_AXIS)
                                           * osg::Matrix::rotate(-osg::PI_2, osg::Z_AXIS)
                                           * transform->getMatrix());
                    }
                }
                break;
            }
        }
        return transform.release();
    }

    osg::Geometry* createGeometry(osg::Vec3Array* va, osg::Vec3Array* na, osg::Vec2Array* ta,
                                  osg::PrimitiveSet* p, bool useVBO)
    {
        if (!va || !p)
        {
            OSG_NOTICE << "createGeometry: invalid parameters" << std::endl;
            return NULL;
        }

        osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array(1);
        (*ca)[0] = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setVertexArray(va);
        geom->setTexCoordArray(0, ta);
        geom->setColorArray(ca.get());
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->addPrimitiveSet(p);
        if (useVBO)
        {
            geom->setUseDisplayList(false);
            geom->setUseVertexBufferObjects(true);
        }

        if (na)
        {
            unsigned int normalSize = na->size();
            if (normalSize == va->size())
            {
                geom->setNormalArray(na);
                geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
            }
            else if (normalSize == 1)
            {
                geom->setNormalArray(na);
                geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
            }
        }
        return geom.release();
    }

}
