#include <osg/io_utils>
#include <osg/MatrixTransform>
#include "Callbacks.h"
#include "CharacterController.h"
#include "Vehicle.h"
#include "VehicleManager.h"
#include "PhysicsUtil.h"
#include <algorithm>
#include <iostream>

using namespace osgPhysics;
using namespace physx;

/* UpdatePhysicsSystemCallback */

void UpdatePhysicsSystemCallback::addVehicle(WheeledVehicle* vehicle)
{
    _vehicles.push_back(vehicle);
    _vehicleEngines.push_back(vehicle->getDriveEngine());
    _queryResults.push_back(vehicle->getQueryResult());
    computeTotalWheels();
}

void UpdatePhysicsSystemCallback::computeTotalWheels()
{
    _numTotalWheels = 0;
    for (unsigned int i = 0; i < _vehicleEngines.size(); ++i)
        _numTotalWheels += _vehicleEngines[i]->mWheelsSimData.getNbWheels();
}

void UpdatePhysicsSystemCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    double step = _frameTime;
    if (step <= 0.0)
    {
        const osg::FrameStamp* fs = nv->getFrameStamp();
        step = fs->getSimulationTime() - _lastSimulationTime;
        _lastSimulationTime = fs->getSimulationTime();
    }

    if (_maxSimulationDelta > 0.0)
    {
        for (; step > 0.0; step -= _maxSimulationDelta)
        {
            double s = std::min(step, _maxSimulationDelta);
            if (_vehicleEngines.size() > 0)
                VehicleManager::instance()->update(s, _sceneName, _vehicleEngines, _queryResults, _numTotalWheels);
            Engine::instance()->update(s);
        }
    }
    else
    {
        if (_vehicleEngines.size() > 0)
            VehicleManager::instance()->update(step, _sceneName, _vehicleEngines, _queryResults, _numTotalWheels);
        Engine::instance()->update(step);
    }

    if (node) traverse(node, nv);
}

/* UpdateActorCallback */

void UpdateActorCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::MatrixTransform* mt = (node->asTransform() ? node->asTransform()->asMatrixTransform() : NULL);
    if (mt && _actor)
    {
        PxMat44 matrix(_actor->getGlobalPose());
        mt->setMatrix(toMatrix(matrix));
    }
    traverse(node, nv);
}

/* UpdateCharacterCallback */

void UpdateCharacterCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::MatrixTransform* mt = (node->asTransform() ? node->asTransform()->asMatrixTransform() : NULL);
    if (!mt || !_character) return;

    const PxExtendedVec3& pos = _character->getPosition();
    mt->setMatrix(osg::Matrix::translate(pos.x, pos.y, pos.z));

    double step = _frameTime;
    if (step <= 0.0)
    {
        const osg::FrameStamp* fs = nv->getFrameStamp();
        step = fs->getSimulationTime() - _lastSimulationTime;
        _lastSimulationTime = fs->getSimulationTime();
    }
    _character->updateMovement(step);
    traverse(node, nv);
}

/* UpdateVehicleCallback */

void UpdateVehicleCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::Group* car = node->asGroup();
    if (!car || !_physicsVehicle) return;

    // No need to update the whole car's matrix, all parts will obtain own transforms
    //PxMat44 carMatrix( _physicsVehicle->getActor()->getGlobalPose() );
    //car->setMatrix( toMatrix(carMatrix) );

    // Update poses of car parts, note they are all world matrices
    std::vector<PxTransform> transforms;
    unsigned int size = _physicsVehicle->getComponentTransforms(transforms);
    size = std::min<unsigned int>(car->getNumChildren(), size);
    for (unsigned int i = 0; i < size; ++i)
    {
        // We must ensure all children are transforms
        osg::MatrixTransform* component = static_cast<osg::MatrixTransform*>(car->getChild(i));
        component->setMatrix(toMatrix(PxMat44(transforms[i])));
    }

    // Handle inputs
    double step = _frameTime;
    if (step <= 0.0)
    {
        const osg::FrameStamp* fs = nv->getFrameStamp();
        step = fs->getSimulationTime() - _lastSimulationTime;
        _lastSimulationTime = fs->getSimulationTime();
    }
    _physicsVehicle->handleInputs(step);
    traverse(node, nv);
}
