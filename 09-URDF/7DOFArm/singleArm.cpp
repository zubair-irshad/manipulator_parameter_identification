#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

class MyWindow : public dart::gui::SimWindow
{
public: 
	MyWindow(const WorldPtr& world)
	{
		setWorld(world);
		mTestURDF = world->getSkeleton("testURDF");
	}
	void timeStepping() override
  {
  		SimWindow::timeStepping();
  }

protected:

  SkeletonPtr mTestURDF;
};


SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 1;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}


SkeletonPtr createTestURDF()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr testURDF =
      loader.parseSkeleton("/home/panda/dart/09-URDF/7DOFArm/singlearm.urdf");
  testURDF->setName("urdf");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0.0, 0.3);
  testURDF->getJoint(0)->setTransformFromParentBodyNode(tf);

  // Get it into a useful configuration
  // krang->getDof(15)->setPosition(140.0 * M_PI / 180.0);

  return testURDF;
}


int main(int argc, char* argv[])
{

	SkeletonPtr testURDF = createTestURDF();
	SkeletonPtr floor = createFloor();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(testURDF);
	world->addSkeleton(floor);

	MyWindow window(world);

	glutInit(&argc, argv);
  	window.initWindow(1280,720, "Test SingleArm URDF");
  glutMainLoop();
}