#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;


const double default_speed_increment = 0.5;

const int default_ik_iterations = 4500;

const double default_force =  50.0; // N
const int default_countdown = 100;  // Number of timesteps for applying force


class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& skel)
    : mNDOF(skel),
      mSpeed(0.0)
  {
    int nDofs = mNDOF->getNumDofs();
    
    mForces = Eigen::VectorXd::Zero(nDofs);
    
    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);
  
    for(std::size_t i = 0; i < 6; ++i)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for(std::size_t i = 6; i < mNDOF->getNumDofs(); ++i)
    {
      mKp(i, i) = 1000;
      mKd(i, i) = 50;
    }
    
    setTargetPositions(mNDOF->getPositions());
  }
  
  /// Reset the desired dof position to the current position
  void setTargetPositions(const Eigen::VectorXd& pose)
  {
    mTargetPositions = pose;
  }

  /// Clear commanding forces
  void clearForces()
  {
    mForces.setZero();
  }
  
  /// Add commanding forces from PD controllers (Lesson 2 Answer)
  void addPDForces()
  {
    Eigen::VectorXd q = mNDOF->getPositions();
    Eigen::VectorXd dq = mNDOF->getVelocities();
    
    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    
    mForces += p + d;
    mNDOF->setForces(mForces);
  }

  /// Add commanind forces from Stable-PD controllers (Lesson 3 Answer)
  void addSPDForces()
  {
    Eigen::VectorXd q = mNDOF->getPositions();
    Eigen::VectorXd dq = mNDOF->getVelocities();

    Eigen::MatrixXd invM = (mNDOF->getMassMatrix()
                            + mKd * mNDOF->getTimeStep()).inverse();
    Eigen::VectorXd p =
        -mKp * (q + dq * mNDOF->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot =
        invM * (-mNDOF->getCoriolisAndGravityForces()
            + p + d + mNDOF->getConstraintForces());
    
    mForces += p + d - mKd * qddot * mNDOF->getTimeStep();
    mNDOF->setForces(mForces);
  }
  
  /// add commanding forces from ankle strategy (Lesson 4 Answer)
  void addAnkleStrategyForces()
  {
    Eigen::Vector3d COM = mNDOF->getCOM();
    // Approximated center of pressure in sagittal axis
    Eigen::Vector3d offset(0.05, 0, 0);
    Eigen::Vector3d COP = mNDOF->getBodyNode("h_heel_left")->
        getTransform() * offset;
    double diff = COM[0] - COP[0];

    Eigen::Vector3d dCOM = mNDOF->getCOMLinearVelocity();
    Eigen::Vector3d dCOP =  mNDOF->getBodyNode("h_heel_left")->
        getLinearVelocity(offset);
    double dDiff = dCOM[0] - dCOP[0];

    int lHeelIndex = mNDOF->getDof("j_heel_left_1")->getIndexInSkeleton();
    int rHeelIndex = mNDOF->getDof("j_heel_right_1")->getIndexInSkeleton();
    int lToeIndex = mNDOF->getDof("j_toe_left")->getIndexInSkeleton();
    int rToeIndex = mNDOF->getDof("j_toe_right")->getIndexInSkeleton();
    if(diff < 0.1 && diff >= 0.0) {
      // Feedback rule for recovering forward push
      double k1 = 200.0;
      double k2 = 100.0;
      double kd = 10;
      mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[lToeIndex] += -k2 * diff - kd * dDiff;
      mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[rToeIndex] += -k2 * diff - kd * dDiff;
    }else if(diff > -0.2 && diff < -0.05) {
      // Feedback rule for recovering backward push
      double k1 = 2000.0;
      double k2 = 100.0;
      double kd = 100;
      mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[lToeIndex] += -k2 * diff - kd * dDiff;
      mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[rToeIndex] += -k2 * diff - kd * dDiff;
    }  
    mNDOF->setForces(mForces);
  }

  void calculateDynamics()
  {
    // double r_w = 0.265; 
    // double m_w = 4.66; 
    // double I_wa = 0.102019; 
    // double M_g = 102.856; 
    // double I_yy = 0.102019;
    // double l_g = 0.2;
    // double I_ra = 0;%.000001; % Gear Moment of Inertia
    // double gamma = 1.0; % Gear Ratio
    // double g = 9.81;
    // double c_w = 0.1;

    // double delta = (M_g*l_g+I_yy+gamma^2*I_ra)*(M_g+m_w)*r_w^2+I_wa+I_ra*gamma^2-(M_g*r_w*l_g-I_ra*gamma^2)^2;
    // double c1 = (M_g+m_w)*r_w^2+I_wa+I_ra*gamma^2+M_g*r_w*l_g+I_ra*gamma^2;
    // double c2 = M_g*r_w*l_g+M_g*l_g^2+I_yy;  

    /// Matlab to Get A,B -> F

    /// Going to hardcode F for now

    /// Need 4 Terms: wheel angle, tilt angle, wheel speed, tilt speed

    /// Eventually
    /// τw = −F(x − xd)

  }
  
  // Send velocity commands on wheel actuators (Lesson 6 Answer)
  void setWheelCommands()
  {
    // Extract Position and Angle
    // Eigen::VectorXd q = m3DOF->getPositions();

    // cout << q << endl;


    // Set Wheel Speed from LQR Controller


    int wheelFirstIndex = mNDOF->getDof("JLWheel")->getIndexInSkeleton();
    for (std::size_t i = wheelFirstIndex; i < mNDOF->getNumDofs(); ++i)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }
    
    int index1 = mNDOF->getDof("JLWheel")->getIndexInSkeleton();
    int index2 = mNDOF->getDof("JRWheel")->getIndexInSkeleton();

    mNDOF->setCommand(index1, mSpeed);
    mNDOF->setCommand(index2, mSpeed);

  }
  
  void changeWheelSpeed(double increment)
  {
    mSpeed += increment;
    std::cout << "wheel speed = " << mSpeed << std::endl;
  }
  
protected:
  /// The biped Skeleton that we will be controlling
  SkeletonPtr mNDOF;
  
  /// Joint forces for the biped (output of the Controller)
  Eigen::VectorXd mForces;
  
  /// Control gains for the proportional error terms in the PD controller
  Eigen::MatrixXd mKp;

  /// Control gains for the derivative error terms in the PD controller
  Eigen::MatrixXd mKd;

  /// Target positions for the PD controllers
  Eigen::VectorXd mTargetPositions;
    
  /// For velocity actuator: Current speed of the skateboard
  double mSpeed;
};


class MyWindow : public dart::gui::SimWindow
{
public: 
	MyWindow(const WorldPtr& world)
	{
		setWorld(world);
		m3DOF = world->getSkeleton("m3DOF");
        qInit = m3DOF->getPositions();
        // qPrintOut.open("One.csv");

        mController = dart::common::make_unique<Controller>(m3DOF);
	}

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case ',':
        mForceCountDown = default_countdown;
        mPositiveSign = false;
        break;
      case '.':
        mForceCountDown = default_countdown;
        mPositiveSign = true;
        break;
      case 'a':
        mController->changeWheelSpeed(default_speed_increment);
        break;
      case 's':
        mController->changeWheelSpeed(-default_speed_increment);
        break;
      default:
        SimWindow::keyboard(key, x, y);
    }
  }

	void timeStepping() override
  {
      Eigen::VectorXd q = m3DOF->getPositions();
      Eigen::VectorXd qDelta = qInit - q;
      // cout << "Shift in q from Original:" << endl << qDelta << endl << endl;
      // qPrintOut << q[1] << "," << q[2] << "," << q[3] << "," <<  q[4] << 
      //       "," <<  q[5] << "," <<  q[6] << "," <<  q[7] << "," <<  q[8] << "\n";
      Eigen::Isometry3d BaseTf = m3DOF->getBodyNode("Base")->getTransform();
      Eigen::AngleAxisd BaseAngleAxis(BaseTf.matrix().block<3,3>(0,0));
      Eigen::Vector3d aa = BaseAngleAxis.axis()*BaseAngleAxis.angle();

      Eigen::MatrixXd M = m3DOF->getMassMatrix();
      Eigen::Vector3d COM = m3DOF->getCOM();
      // Eigen::Vector3d dCOM = mNDOF->getCOMLinearVelocity();

      // m3DOF->getDof

      // Use this to lock joints
      // // Set joint limits
      // for(std::size_t i = 0; i < biped->getNumJoints(); ++i)
      //   biped->getJoint(i)->setPositionLimitEnforced(true);

      BodyNodePtr Base = m3DOF->getBodyNode("Base");
      Eigen::Vector3d localCOM = Base->getCOM(Base);

    cout << "DOF,q,dq" << endl;
    for(size_t i = 0; i < m3DOF->getNumDofs(); ++i)
    {
      DegreeOfFreedom* dof = m3DOF->getDof(i);
      double q = dof->getPosition();
      double dq = dof->getVelocity();

      cout << i << " " << q << " " << dq << endl;
    }


      // for(std::size_t i = 6; i < m3DOF->getNumDofs(); ++i)
      // {
      //   mKp(i, i) = 1000;
      //   mKd(i, i) = 50;
      // }      

      // cout << "Mass Matrix: " << Base->getPositions() << endl;
      // cout << "COM Matrix: " << localCOM << endl;

      // cout << "DOFs: " << m3DOF->getNumDofs() << endl;


      cout << "Base Anlge: " << BaseAngleAxis.angle() << endl;
      cout << "Base Axis: ";
      int i;
      for(i=0; i<3; i++){
        cout << BaseAngleAxis.axis()(i) << "   ";
      }
      cout << endl;
      cout << "aa: ";
      for(i=0; i<3; i++){
        cout << aa(i) << "   ";
      }
      cout << endl;

      cout << "q: ";
      for(i=0; i<3; i++){
        cout << q(i) << "   ";
      }
      cout << endl << "=====================================================" << endl;


      mController->clearForces();
      // mController->addPDForces();
      mController->setWheelCommands();

  	  SimWindow::timeStepping();
  }


protected:

  SkeletonPtr m3DOF;

  Eigen::VectorXd qInit;

  Eigen::VectorXd dof1;




  std::unique_ptr<Controller> mController;
  
  /// Number of iterations before clearing a force entry
  int mForceCountDown;
  
  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;

  // ofstream qPrintOut;
};


// Set the actuator type for four wheel joints to "VELOCITY" (Lesson 6 Answer)
void setVelocityAccuators(SkeletonPtr skel)
{  
  Joint* wheel1 = skel->getJoint("LWheel");
  Joint* wheel2 = skel->getJoint("RWheel");

  wheel1->setActuatorType(Joint::VELOCITY);
  wheel2->setActuatorType(Joint::VELOCITY);
}


SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 15;
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


SkeletonPtr create3DOF_URDF()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr threeDOF =
      loader.parseSkeleton("/home/n8k9/Projects/GaTech/CS8803/Final/dart/09-URDF/3DOF-WIP/3dof.urdf");
  threeDOF->setName("m3DOF");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0.0, .275);
  /*tf *= Eigen::AngleAxisd(3.14159,Eigen::Vector3d::UnitZ());*/
  tf *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
  tf *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitY());
  threeDOF->getJoint(0)->setTransformFromParentBodyNode(tf);

  // Get it into a useful configuration
  // krang->getDof(15)->setPosition(140.0 * M_PI / 180.0);

  return threeDOF;
}


int main(int argc, char* argv[])
{

	SkeletonPtr threeDOF = create3DOF_URDF();
	SkeletonPtr floor = createFloor();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(threeDOF);
	world->addSkeleton(floor);

  // Move the base 
  //threeDOF->getDof(0)->setPosition(M_PI/6);
  /*threeDOF->getDof(1)->setPosition(M_PI/6);
  Eigen::Isometry3d BaseTf = threeDOF->getBodyNode("Base")->getTransform();

  // Angle and Axis of orientation of our base
  Eigen::AngleAxisd BaseAngleAxis(BaseTf.matrix().block<3,3>(0,0));*/
  
  // Iterate through the following possibilities
  // 1st Tf: prex, noprex, prey, noprey
  // 2nd Tf: with x first: prey, noprey, prelocalycol, noprelocalycol, prelocalyrow, noprelocalyrow, 
  //         with y first: prex, noprex, prelocalxcol, noprelocalxcol, prelocalxrow, noprelocalxrow
  /*int count = 0;
  for(int i=0; i<4; i++){
    int jbegin = (i<2? 0 : 6);
    for(int j=jbegin; j<jbegin+6; j++){
      count++;
      Eigen::Transform<double, 3, Eigen::Affine> OurTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
      switch(i){
        case 0:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitX()));
          break;
        case 1:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitX()));
          break;
        case 2:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitY()));
          break;
        case 3:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitY()));
          break;
      }
      switch(j){
        case 0:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitY()));
          break;
        case 1:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitY()));
          break;
        case 2:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<3,1>(0,1)));
          break;
        case 3:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<3,1>(0,1)));
          break;
        case 4:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<1,3>(1,0)));
          break;
        case 5:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<1,3>(1,0)));
          break;
        case 6:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitX()));
          break;
        case 7:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitX()));
          break;
        case 8:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<3,1>(0,0)));
          break;
        case 9:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<3,1>(0,0)));
          break;
        case 10:
          OurTf.prerotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<1,3>(0,0)));
          break;
        case 11:
          OurTf.rotate(Eigen::AngleAxisd(M_PI/6, OurTf.matrix().block<1,3>(0,0)));
          break;
      }
      // Angle Axis representation of our Tf
      Eigen::AngleAxisd OurAngleAxis(OurTf.matrix().block<3,3>(0,0));
      Eigen::Matrix3d OurNewTfMat;
      OurNewTfMat = Eigen::AngleAxisd(OurAngleAxis.angle(), OurAngleAxis.axis());


      // Print out the values
      cout << "count=" << count << ", i=" << i << ", j=" << j << endl;
      cout << "BaseLink Transform: " << endl;
      for (int i=0; i<3; i++){
        for(int j=0; j<3; j++){
          cout << BaseTf(i,j) << "   ";
        }
        cout << endl;
      }
      cout << "Base Angle: " << BaseAngleAxis.angle() << endl;
      cout << "Base Axis:  " ;
      for(int i=0; i<3; i++){
        cout << BaseAngleAxis.axis()(i) << "   ";
      }
      cout << endl << "Our Tf: " << endl;
      Eigen::Matrix<double, 4, 4> OurTfMat = OurTf.matrix();
      for (int i=0; i<3; i++){
        for(int j=0; j<3; j++){
          cout << OurTfMat(i,j) << "   ";
        }
        cout << endl;
      }
      cout << "Our Angle: " << OurAngleAxis.angle() << endl;
      cout << "Our Axis:  " ;
      for(int i=0; i<3; i++){
        cout << OurAngleAxis.axis()(i) << "   ";
      }
      cout << endl << "Our New Tf: " << endl;
      for (int i=0; i<3; i++){
        for(int j=0; j<3; j++){
          cout << OurNewTfMat(i,j) << "   ";
        }
        cout << endl;
      }
      cout << endl << "Difference: " << endl;
      for (int i=0; i<3; i++){
        for(int j=0; j<3; j++){
          cout << OurTfMat(i,j)-BaseTf(i,j) << "   ";
        }
        cout << endl;
      }
      cout << "===============================================================" << endl;
    }
  }*/

  MyWindow window(world);
	glutInit(&argc, argv);
  	window.initWindow(1280,720, "3DOF URDF");
  glutMainLoop();
}
