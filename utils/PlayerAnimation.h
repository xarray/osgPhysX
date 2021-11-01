#ifndef CORE_PLAYERANIMATION
#define CORE_PLAYERANIMATION

#include <osg/Version>
#include <osg/Geometry>

namespace osgPhysicsUtils
{

    /** The player animation support class */
    class PlayerAnimation : public osg::Referenced
    {
    public:
        typedef float (*SetJointWeightFunc)(int, int, void*);
        PlayerAnimation();

        bool initialize(const std::string& skeleton, const std::string& mesh);
        bool loadAnimation(const std::string& key, const std::string& animation);
        void unloadAnimation(const std::string& key);

        bool update(const osg::FrameStamp& fs, bool paused);
        bool applyMeshes(osg::Geode& meshDataRoot, bool withSkinning);

        typedef std::pair<int, int> ThisAndParent;
        std::vector<ThisAndParent> getSkeletonIndices(int from = -1) const;
        std::string getSkeletonJointName(int joint) const;
        int getSkeletonJointIndex(const std::string& joint) const;

        osg::BoundingBox computeSkeletonBounds() const;
        float getAnimationStartTime(const std::string& key);
        float getTimeRatio(const std::string& key) const;
        float getDuration(const std::string& key) const;

        float getPlaybackSpeed(const std::string& key) const;
        void setPlaybackSpeed(const std::string& key, float s);

        void select(const std::string& key, float weight, bool looping);
        void selectPartial(const std::string& key, float weight, bool looping,
                           SetJointWeightFunc func, void* userData);
        void seek(const std::string& key, float timeRatio);

    protected:
        osg::ref_ptr<osg::Referenced> _internal;
        float _blendingThreshold;
    };

}

#endif
