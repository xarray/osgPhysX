#include <utils/SceneUtil.h>
#include <utils/InputManager.h>
#include <utils/PlayerAnimation.h>

#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer;
    osg::ref_ptr<osg::Geode> root = new osg::Geode;

    osg::ref_ptr<osgPhysicsUtils::PlayerAnimation> animManager = new osgPhysicsUtils::PlayerAnimation;
    if (!animManager->initialize("exo_gray.skeleton.ozz", "exo_gray.mesh.ozz")) return 1;
    if (!animManager->loadAnimation("idle", "exo_gray@idle.ozz")) return 1;
    animManager->select("idle", 1.0f, true);
    animManager->seek("idle", 0.0f);

    viewer.setSceneData(root.get());
    viewer.setCameraManipulator(new osgGA::TrackballManipulator);
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.setUpViewInWindow(50, 50, 800, 600);
    while (!viewer.done())
    {
        viewer.frame();
        if (viewer.getFrameStamp())
        {
            animManager->update(*viewer.getFrameStamp(), false);
            animManager->applyMeshes(*root, true);
        }
    }
    return 0;
}