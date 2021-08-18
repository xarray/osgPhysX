#ifndef CORE_FOLLOWNODEMANIPULATOR
#define CORE_FOLLOWNODEMANIPULATOR

#include <osg/Version>
#include <osg/Quat>
#include <osg/Drawable>
#include <osgGA/FirstPersonManipulator>

namespace osgPhysicsUtils
{

    /** The first-person manipulator to track a node and place the eye at the node center */
    class FollowNodeManipulator : public osgGA::FirstPersonManipulator
    {
    public:
        FollowNodeManipulator(int flags = DEFAULT_SETTINGS);

        /** Set the offset of the eye */
        void setLocalOffset(const osg::Vec3& o) { _offset = o; }
        const osg::Vec3& getLocalOffset() const { return _offset; }

        /** Set the offset of the tracker center */
        void setLocalCenterOffset(const osg::Vec3& o) { _centerOffset = o; }
        const osg::Vec3& getLocalCenterOffset() const { return _centerOffset; }

        /** Set node to track */
        void setTrackNode(osg::Node* node) { _trackNode = node; }
        osg::Node* getTrackNode() { return _trackNode.get(); }
        const osg::Node* getTrackNode() const { return _trackNode.get(); }

        /** Set drawable to track */
        void setTrackDrawable(osg::Drawable* d) { _trackDrawable = d; }
        osg::Drawable* getTrackDrawable() { return _trackDrawable.get(); }
        const osg::Drawable* getTrackDrawable() const { return _trackDrawable.get(); }

        /** Set fixed rotation for driving simulation etc. */
        void setFixedRotation(bool b) { _fixedRotation = b; }
        bool getFixedRotation() const { return _fixedRotation; }

        void yaw(double dx);
        void pitch(double dy);

        virtual bool handleFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
        virtual bool handleMouseMove(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

    protected:
        osg::observer_ptr<osg::Node> _trackNode;
        osg::observer_ptr<osg::Drawable> _trackDrawable;
        osg::Vec3 _offset, _centerOffset;
        bool _fixedRotation;
    };

}

#endif
