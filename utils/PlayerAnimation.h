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
        PlayerAnimation();

        bool initialize(const std::string& skeleton, const std::string& mesh);
        bool loadAnimation(const std::string& key, const std::string& animation);
        void unloadAnimation(const std::string& key);

        bool update(const osg::FrameStamp& fs, bool paused, bool looping);
        bool applyMeshes(osg::Geode& meshDataRoot, bool withSkinning);

        osg::BoundingBox computeSkeletonBounds() const;
        float getAnimationStartTime() const { return _startTime; }
        float getTimeRatio() const { return osg::clampBetween(_timeRatio, 0.0f, 1.0f); }
        float getDuration() const;

        float getPlaybackSpeed() const { return _playbackSpeed; }
        void setPlaybackSpeed(float s) { _playbackSpeed = s; }

        void selectAnimation(const std::string& key);
        void seek(float timeRatio);

    protected:
        osg::ref_ptr<osg::Referenced> _internal;
        float _playbackSpeed, _timeRatio, _startTime;
        bool _resetTimeRatio;
    };

}

#endif
