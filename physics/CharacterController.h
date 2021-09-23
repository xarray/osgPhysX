#ifndef PHYSICS_CHARACTERCONTROLLER
#define PHYSICS_CHARACTERCONTROLLER

#include <osg/Referenced>
#include <osg/Vec3>
#include "Engine.h"

namespace osgPhysics
{

    /** The character controller manager */
    class CharacterControlManager : public osg::Referenced
    {
    public:
        static CharacterControlManager* instance();
        physx::PxControllerManager* getOrCreateManager(physx::PxScene* scene);
        physx::PxObstacleContext* getOrCreateObstacle(physx::PxScene* scene);

        physx::PxObstacleContext* getObstacle(physx::PxScene* scene);
        void removeObstacle(physx::PxScene* scene);

    protected:
        CharacterControlManager();
        virtual ~CharacterControlManager();
        std::map<physx::PxScene*, physx::PxControllerManager*> _managers;
        std::map<physx::PxScene*, physx::PxObstacleContext*> _obstacleContexts;
    };

    /** The character controller */
    class CharacterController : public osg::Referenced, public physx::PxUserControllerHitReport,
        public physx::PxControllerBehaviorCallback, public physx::PxSceneQueryFilterCallback
    {
    public:
        struct ControllerData
        {
            physx::PxExtendedVec3 position;  // the initial position
            physx::PxVec3 up;  // the up direction
            physx::PxF32 density;  // density of the character
            physx::PxF32 maxSlopeAngle;  // max slope angle we can reach
            physx::PxF32 maxJumpHeight;
            physx::PxF32 invisibleWallHeight;
            physx::PxF32 stepOffset;  // max height of an obstacle which we can climb
            physx::PxF32 contactOffset;  // a skin around the object for creating contacts
            physx::PxF32 scaleCoeffcient;  // scale coeffcient for underlying kinematic actor
            ControllerData(float d = 10.0f, const osg::Vec3& pos = osg::Vec3(), const osg::Vec3& upDir = osg::Z_AXIS);
        };

        CharacterController();

        physx::PxRigidDynamic* getControlActor() { return _actor; }
        const physx::PxRigidDynamic* getControlActor() const { return _actor; }

        physx::PxController* getController() { return _controller; }
        const physx::PxController* getController() const { return _controller; }

        /** Create a box controller */
        bool createBox(const std::string& scene, physx::PxF32 halfForward, physx::PxF32 halfSide, physx::PxF32 halfHeight,
            const ControllerData& data, physx::PxMaterial* mtl = 0);

        /** Create a capsule controller */
        bool createCapsule(const std::string& scene, physx::PxF32 radius, physx::PxF32 height, bool easyMode,
            const ControllerData& data, physx::PxMaterial* mtl = 0);

        /** Create the controller using generic description */
        bool create(const std::string& scene, physx::PxControllerDesc* description,
            const ControllerData& data, physx::PxMaterial* mtl = 0);

        /** Set the gravity the character should suffer from */
        void setGravity(osg::Vec3& g) { _gravity = g; }
        const osg::Vec3& getGravity() const { return _gravity; }

        /** Set the character position immediately */
        void setPosition(physx::PxExtendedVec3& p) { _controller->setPosition(p); }
        physx::PxExtendedVec3 getPosition() const { return _controller->getPosition(); }

        /** Move the character */
        void move(const osg::Vec3& offset, float gravityScale = 1.0f);

        /** Update the controller movement every frame */
        physx::PxControllerCollisionFlags updateMovement(double step);

        /** Add/update an invisible obstacle. Set handle to 0 if you want to add new */
        void updateObstacle(physx::ObstacleHandle handle, const physx::PxObstacle& obstacle);
        void removeObstacle(physx::ObstacleHandle handle, bool removeAll);

        // Implements PxUserControllerHitReport
        virtual void onShapeHit(const physx::PxControllerShapeHit& hit);
        virtual void onControllerHit(const physx::PxControllersHit& hit) {}
        virtual void onObstacleHit(const physx::PxControllerObstacleHit& hit) {}

        // Implements PxControllerBehaviorCallback
        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxShape& shape, const physx::PxActor& actor);
        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxController& controller);
        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxObstacle& obstacle);

        // Implements PxSceneQueryFilterCallback
        virtual physx::PxQueryHitType::Enum preFilter(
            const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxSceneQueryFlags& flags);
        virtual physx::PxQueryHitType::Enum postFilter(
            const physx::PxFilterData& filterData, const physx::PxSceneQueryHit& hit);

    protected:
        virtual ~CharacterController();
        void addForceAtLocalPos(physx::PxRigidBody& body, const physx::PxVec3& force, const physx::PxVec3& pos,
            physx::PxForceMode::Enum mode, bool wakeup = true);

        physx::PxScene* _controllerScene;
        physx::PxRigidDynamic* _actor;
        physx::PxController* _controller;
        osg::Vec3 _offset, _gravity;
    };

}

#endif
