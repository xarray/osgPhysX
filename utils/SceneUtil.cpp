#include <osg/Depth>
#include <osg/FrameBufferObject>
#include <osg/PolygonMode>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgText/Font>
#include <osgUtil/SmoothingVisitor>
#include <physics/PhysicsUtil.h>
#include "SceneUtil.h"
#include <algorithm>
#include <functional>
#include <iostream>

using namespace osgPhysics;
using namespace osgPhysicsUtils;

namespace osgPhysicsUtils
{

    static osg::ref_ptr<osgText::Font> g_font;
    static osg::ref_ptr<osg::Geode> g_screenQuad = createQuad(osg::Vec3(), 1.0f, 1.0f);

    osg::Geometry* createEllipsoid(const osg::Vec3& center, float radius1, float radius2,
        float radius3, int samples)
    {
        if (samples < 2 || radius1 <= 0.0f || radius2 <= 0.0f || radius3 <= 0.0f)
        {
            OSG_NOTICE << "createEllipsoid: invalid parameters" << std::endl;
            return NULL;
        }

        int halfSamples = samples / 2;
        float samplesF = (float)samples;

        osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec2Array> ta = new osg::Vec2Array;
        for (int j = 0; j < halfSamples; ++j)
        {
            float theta1 = 2.0f * (float)j * osg::PI / samplesF - osg::PI_2;
            float theta2 = 2.0f * (float)(j + 1) * osg::PI / samplesF - osg::PI_2;
            for (int i = 0; i <= samples; ++i)
            {
                float theta3 = 2.0f * (float)i * osg::PI / samplesF;
                osg::Vec3 e;
                e.x() = cosf(theta1) * cosf(theta3) * radius1;
                e.y() = sinf(theta1) * radius2;
                e.z() = cosf(theta1) * sinf(theta3) * radius3;

                va->push_back(center + e);
                ta->push_back(osg::Vec2((float)i / samplesF, 2.0f * (float)j / samplesF));

                e.x() = cosf(theta2) * cosf(theta3) * radius1;
                e.y() = sinf(theta2) * radius2;
                e.z() = cosf(theta2) * sinf(theta3) * radius3;

                va->push_back(center + e);
                ta->push_back(osg::Vec2((float)i / samplesF, 2.0f * (float)(j + 1) / samplesF));
            }
        }

        osg::Geometry* geom = createGeometry(
            va.get(), NULL, ta.get(), new osg::DrawArrays(GL_QUAD_STRIP, 0, va->size()));
        if (geom) osgUtil::SmoothingVisitor::smooth(*geom);
        return geom;
    }

    osg::Geometry* createPrism(const osg::Vec3& centerBottom, float radiusBottom, float radiusTop,
        float height, int n, bool capped)
    {
        osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec2Array> ta = new osg::Vec2Array;
        float samplesF = (float)n;
        for (int i = 0; i <= n; ++i)
        {
            float theta = 2.0f * (float)i * osg::PI / samplesF;
            osg::Vec3 r;
            r.x() = radiusTop * cosf(theta);
            r.y() = radiusTop * sinf(theta);
            r.z() = height;
            va->push_back(centerBottom + r);
            ta->push_back(osg::Vec2((float)i / samplesF, 1.0f));

            r.x() = radiusBottom * cosf(theta);
            r.y() = radiusBottom * sinf(theta);
            r.z() = 0.0f;
            va->push_back(centerBottom + r);
            ta->push_back(osg::Vec2((float)i / samplesF, 0.0f));
        }

        osg::ref_ptr<osg::DrawArrays> de = new osg::DrawArrays(GL_QUAD_STRIP, 0, va->size());
        if (capped)  // Add center points
        {
            va->push_back(centerBottom + osg::Vec3(0.0f, 0.0f, height));
            ta->push_back(osg::Vec2(1.0f, 1.0f));
            va->push_back(centerBottom);
            ta->push_back(osg::Vec2(0.0f, 0.0f));
        }

        osg::Geometry* geom = createGeometry(va.get(), NULL, ta.get(), de.get(), !capped);
        if (geom && capped)
        {
            osg::ref_ptr<osg::DrawElementsUByte> top = new osg::DrawElementsUByte(GL_TRIANGLE_FAN);
            osg::ref_ptr<osg::DrawElementsUByte> bottom = new osg::DrawElementsUByte(GL_TRIANGLE_FAN);
            top->push_back(va->size() - 2);
            for (int i = 0; i <= n; ++i)
            {
                top->push_back(i * 2);
                bottom->insert(bottom->begin(), i * 2 + 1);
            }
            bottom->insert(bottom->begin(), va->size() - 1);

            geom->addPrimitiveSet(top.get());
            geom->addPrimitiveSet(bottom.get());
            osgUtil::SmoothingVisitor::smooth(*geom);
        }
        return geom;
    }

    osg::Geometry* createPyramid(const osg::Vec3& centerBottom, float radius, float height, int n, bool capped)
    {
        osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec2Array> ta = new osg::Vec2Array;
        va->push_back(centerBottom + osg::Vec3(0.0f, 0.0f, height));
        ta->push_back(osg::Vec2(1.0f, 1.0f));

        float samplesF = (float)n;
        for (int i = 0; i <= n; ++i)
        {
            float theta = 2.0f * (float)i * osg::PI / samplesF;
            osg::Vec3 r;
            r.x() = radius * cosf(theta);
            r.y() = radius * sinf(theta);
            va->push_back(centerBottom + r);
            ta->push_back(osg::Vec2((float)i / samplesF, 0.0f));
        }

        osg::ref_ptr<osg::DrawArrays> de = new osg::DrawArrays(GL_TRIANGLE_FAN, 0, va->size());
        if (capped)  // Add center points
        {
            va->push_back(centerBottom);
            ta->push_back(osg::Vec2(0.0f, 0.0f));
        }

        osg::Geometry* geom = createGeometry(va.get(), NULL, ta.get(), de.get(), !capped);
        if (geom && capped)
        {
            osg::ref_ptr<osg::DrawElementsUByte> bottom = new osg::DrawElementsUByte(GL_TRIANGLE_FAN);
            for (int i = 0; i <= n; ++i)
            {
                bottom->insert(bottom->begin(), i + 1);
            }
            bottom->insert(bottom->begin(), va->size() - 1);

            geom->addPrimitiveSet(bottom.get());
            osgUtil::SmoothingVisitor::smooth(*geom);
        }
        return geom;
    }

    osg::Geode* createGeode(osg::Drawable* drawable, bool transparent)
    {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(drawable);
        if (transparent)
        {
            geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
            geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        }
        return geode.release();
    }

    osg::Geode* createQuad(const osg::Vec3& corner, float width, float height, const osg::Vec4& uvRange)
    {
        osg::ref_ptr<osg::Geode> quad = new osg::Geode;
        quad->addDrawable(osg::createTexturedQuadGeometry(
            corner, osg::Vec3(width, 0.0f, 0.0f), osg::Vec3(0.0f, height, 0.0f),
            uvRange[0], uvRange[1], uvRange[2], uvRange[3]));
        return quad.release();
    }

    osg::Geode* createTexturedQuad(osg::Texture* texture, const osg::Vec3& corner,
        float width, float height, const osg::Vec4& uvRange)
    {
        osg::Geode* quad = createQuad(corner, width, height, uvRange);
        quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);

        osg::Image* image = texture->getImage(0);
        if (image && image->isImageTranslucent())
        {
            quad->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
            quad->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        }
        return quad;
    }

    osgText::Text* createText(const osg::Vec3& pos, const std::string& content, float size)
    {
        if (!g_font)
            g_font = osgText::readFontFile("../data/OpenSans.ttf");

        osg::ref_ptr<osgText::Text> text = new osgText::Text;
        text->setDataVariance(osg::Object::DYNAMIC);
        text->setFont(g_font.get());
        text->setCharacterSize(size);
        text->setAxisAlignment(osgText::TextBase::XY_PLANE);
        text->setPosition(pos);
        text->setText(content);
        return text.release();
    }

    osg::Camera* createHUDCamera(const osg::Matrix& view, const osg::Matrix& proj,
        bool unlighted, bool asBackground)
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        camera->setClearMask(GL_DEPTH_BUFFER_BIT);
        camera->setRenderOrder(osg::Camera::POST_RENDER);
        camera->setAllowEventFocus(false);
        camera->setViewMatrix(view);
        camera->setProjectionMatrix(proj);

        osg::StateSet* ss = camera->getOrCreateStateSet();
        if (unlighted)
        {
            int values = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
            ss->setAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL), values);
            ss->setMode(GL_LIGHTING, values);
        }

        if (asBackground)
        {
            camera->setClearMask(0);
            ss->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 1.0, 1.0));
        }
        return camera.release();
    }

}
