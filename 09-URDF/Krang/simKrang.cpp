#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

const double default_push_force = 8.0;
const int default_countdown = 200;
const double default_torque = 50.0;

class Controller
{
public:

  Controller(const SkeletonPtr& krang)
    : mKrang(krang)
  {
  	mQDesired = mKrang->getPositions();

	Eigen::Isometry3d target_offset(Eigen::Isometry3d::Identity());
	target_offset.translation() = Eigen::Vector3d(0,0,0);

	//Target reference frame for Op Ctrl
	mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target");
	mTarget->setTransform(target_offset, Frame::World());

    mKpPD = 0;
    mKdPD = 5.0;
  }

  void setPDForces()
  {
    if(nullptr == mKrang)
      return;

    // Compute the joint position error
    Eigen::VectorXd q = mKrang->getPositions();
    Eigen::VectorXd dq = mKrang->getVelocities();
    q += dq * mKrang->getTimeStep();
  	Eigen::VectorXd q_err = mQDesired - q;

    // Compute the joint velocity error
    Eigen::VectorXd dq_err = -dq;

    // Compute the joint forces needed to compensate for Coriolis forces and
    // gravity
    const Eigen::VectorXd& Cg = mKrang->getCoriolisForces();

    // Compute the desired joint forces
    const Eigen::MatrixXd& M = mKrang->getMassMatrix();

    mForces = M * (mKpPD*q_err + mKdPD*dq_err) + Cg;

    mKrang->setForces(mForces);
  }

  /// Compute an operational space controller to extend the arms
  void setJointForces()
  {
    if(nullptr == mKrang)
      return;


    mKrang->setForces(mForces);
  }

protected:

	SkeletonPtr mKrang;

	/// The target pose for the controller
	SimpleFramePtr mTarget;

	/// End effector for the right arm of Krang
	BodyNodePtr mEndEffector;

	/// Desired joint positions when not applying the operational space controller
    Eigen::VectorXd mQDesired;

    /// The offset of the end effector from the body origin of the last BodyNode
    /// in the manipulator
    Eigen::Vector3d mOffset;

	Eigen::VectorXd mForces;

	//Control gain for proportional error in PD Controller
	double mKpPD;

	//Control gain for deriv. error in PD Controller
	double mKdPD;

	/// Control gains for the proportional error terms in the operational
    /// space controller
    double mKpOS;

    /// Control gains for the derivative error terms in the operational space
    /// controller
    double mKdOS;
};

class MyWindow : public dart::gui::SimWindow
{
public:
	MyWindow(const WorldPtr& world)
	: mHasEverRun(false),
	  mPositiveSign(true)
	{
		setWorld(world);
		mKrang = world->getSkeleton("krang");
		mController = dart::common::make_unique<Controller>(
        world->getSkeleton("krang"));

        mForceCountDown.resize(mKrang->getNumDofs(), 0);
	}

	void applyForce(std::size_t index)
	  {
	    if(index < mForceCountDown.size())
	      mForceCountDown[index] = default_countdown;
	  }

	void keyboard(unsigned char key, int x, int y) override
    {
	    switch(key)
	    {
	      case '1':
	        applyForce(10);
	        break;

	      case '2':
	        applyForce(8);
	        break;

	      default:
        	SimWindow::keyboard(key, x, y);
	    }
	}

	void timeStepping() override
  	{
  		mController->setPDForces();
  		mController->setJointForces();
  		SimWindow::timeStepping();

	    // Apply joint torques based on user input, and color the Joint shape red
	    for(std::size_t i = 0; i < mKrang->getNumDofs(); ++i)
	    {
	      if(mForceCountDown[i] > 0)
	      {
	        DegreeOfFreedom* dof = mKrang->getDof(i);
	        dof->setForce( mPositiveSign? default_torque : -default_torque );

	        BodyNode* bn = dof->getChildBodyNode();
	        auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
	        visualShapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());

	        --mForceCountDown[i];
	      }
	    }
  	}

protected:

  /// Floor of the scene
  SkeletonPtr mFloor;

  SkeletonPtr mKrang;

  /// Set to true the first time spacebar is pressed
  bool mHasEverRun;

  std::unique_ptr<Controller> mController;

  /// Number of iterations before clearing a force entry
  std::vector<int> mForceCountDown;

    /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;

};

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 7.0;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height-0.3);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createKrang()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr krang =
      loader.parseSkeleton("/home/krang/dart/09-URDF/scenes/Krang.urdf");
  krang->setName("krang");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0.0, 0.0);
  tf *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
  krang->getJoint(0)->setTransformFromParentBodyNode(tf);

  // Get it into a useful configuration
  // krang->getDof(15)->setPosition(140.0 * M_PI / 180.0);

  return krang;
}

int main(int argc, char* argv[])
{

	SkeletonPtr krang = createKrang();
	SkeletonPtr floor = createFloor();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(krang);
	world->addSkeleton(floor);

	MyWindow window(world);

	std::cout << "Output45";

	glutInit(&argc, argv);
  	window.initWindow(1280,720, "Krang Simulation");
  	glutMainLoop();
}
