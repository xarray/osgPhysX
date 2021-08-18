#include "FollowNodeManipulator.h"
#include <osg/Notify>
#include <osg/io_utils>

using namespace osgPhysicsUtils;

FollowNodeManipulator::FollowNodeManipulator(int flags)
    : osgGA::FirstPersonManipulator(flags), _fixedRotation(false)
{
}

void FollowNodeManipulator::yaw(double dx)
{
    osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_eye);
    osg::Vec3d localUp = getUpVector(coordinateFrame);
    rotateYawPitch(_rotation, dx, 0.0, localUp);
}

void FollowNodeManipulator::pitch(double dy)
{
    osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_eye);
    osg::Vec3d localUp = getUpVector(coordinateFrame);
    rotateYawPitch(_rotation, 0.0, dy, localUp);
}

bool FollowNodeManipulator::handleFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    osg::Matrix matrix;
    if (_trackNode.valid())
    {
        matrix = _trackNode->getWorldMatrices()[0];
    }
    else if (_trackDrawable.valid())
    {
        osg::Node* node = _trackDrawable->getNumParents() > 0 ? _trackDrawable->getParent(0) : NULL;
        matrix = osg::Matrix::translate(_trackDrawable->getBound().center());
        if (node) matrix *= node->getWorldMatrices()[0];
    }

    if (_fixedRotation)
    {
        osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_eye);
        osg::Vec3d localUp = getUpVector(coordinateFrame);
        osg::Vec3d pos = _centerOffset * matrix;
        setTransformation(_offset * matrix, pos, localUp);
    }
    else
        _eye = _offset * matrix;
    return osgGA::FirstPersonManipulator::handleFrame(ea, aa);
}

bool FollowNodeManipulator::handleMouseMove(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    return osgGA::FirstPersonManipulator::handleMouseMove(ea, aa);
}
