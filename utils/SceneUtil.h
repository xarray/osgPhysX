#ifndef CORE_SCENEUTIL
#define CORE_SCENEUTIL

#include <osg/Texture>
#include <osg/Camera>
#include <osg/PagedLOD>
#include <osg/Geode>
#include <osgGA/GUIEventHandler>
#include <osgUtil/IntersectionVisitor>
#include <osgText/Text>

namespace osgPhysicsUtils
{

    /** Create a polar sphere (r1 = r2 = r3) or ellipsoid */
    extern osg::Geometry* createEllipsoid(const osg::Vec3& center, float radius1, float radius2,
        float radius3, int samples = 32);

    /** Create a prism (n > 3) or cylinder (n is large enough) */
    extern osg::Geometry* createPrism(const osg::Vec3& centerBottom, float radiusBottom, float radiusTop,
        float height, int n = 4, bool capped = true);

    /** Create a pyramid (n > 3) or cone (n is large enough) */
    extern osg::Geometry* createPyramid(const osg::Vec3& centerBottom, float radius, float height,
        int n = 4, bool capped = false);

    /** Create a geode for specified geometry, just for convenience */
    extern osg::Geode* createGeode(osg::Drawable* drawable, bool transparent = false);

    /** Create a quad node */
    extern osg::Geode* createQuad(const osg::Vec3& corner, float width = 1.0f, float height = 1.0f,
        const osg::Vec4& uvRange = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));

    /** Create a textured quad node */
    extern osg::Geode* createTexturedQuad(osg::Texture* texture, const osg::Vec3& corner,
        float width = 1.0f, float height = 1.0f,
        const osg::Vec4& uvRange = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));

    /** Create a text object on XOY plane */
    extern osgText::Text* createText(const osg::Vec3& pos, const std::string& content, float size);

    /** Create an HUD camera with specified matrices */
    extern osg::Camera* createHUDCamera(const osg::Matrix& view, const osg::Matrix& proj,
        bool unlighted = true, bool asBackground = false);

    /** Create an HUD camera on XOY plane */
    inline osg::Camera* createHUDCamera(double left, double right, double bottom, double top,
        bool unlighted = true, bool asBackground = false)
    {
        return createHUDCamera(osg::Matrix(), osg::Matrix::ortho2D(left, right, bottom, top),
            unlighted, asBackground);
    }

}

#endif
