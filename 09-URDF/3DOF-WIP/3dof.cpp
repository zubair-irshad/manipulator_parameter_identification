#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <boost/circular_buffer.hpp>

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



class filter {
  public:
    filter(const int dim, const int n)
    {
      samples.set_capacity(n);
      total = Eigen::VectorXd::Zero(dim,1);
    }
    void AddSample(Eigen::VectorXd v)
    {
      if(samples.full()) 
      {
        total -= samples.front();
      }
      samples.push_back(v);
      total += v;
      average = total/samples.size();
    }
  
    boost::circular_buffer<Eigen::VectorXd> samples;
    Eigen::VectorXd total;
    Eigen::VectorXd average;
    
};

class MyWindow : public dart::gui::SimWindow
{
  public: 
    MyWindow(const WorldPtr& world)
    {
      setWorld(world);
      m3DOF = world->getSkeleton("m3DOF");
      qInit = m3DOF->getPositions();
      psi = 0; // Heading Angle
      steps = 0;
      outFile.open("constraints.csv");
      dqFilt = new filter(8, 100);
      cFilt = new filter(5, 100);
      R = 0.25;
      L = 0.68;//*6;
    }


  // /// Handle keyboard input
  // void keyboard(unsigned char key, int x, int y) override
  // {
  //   switch(key)
  //   {
  //     case ',':
  //       mForceCountDown = default_countdown;
  //       mPositiveSign = false;
  //       break;
  //     case '.':
  //       mForceCountDown = default_countdown;
  //       mPositiveSign = true;
  //       break;
  //     case 'a':
  //       mController->changeWheelSpeed(default_speed_increment);
  //       break;
  //     case 's':
  //       mController->changeWheelSpeed(-default_speed_increment);
  //       break;
  //     default:
  //       SimWindow::keyboard(key, x, y);
  //   }
  // }


    void timeStepping() override
    {
      // Read Positions, Speeds, Transform speeds to world coordinates and filter the speeds
      Eigen::Matrix<double, 4, 4> Tf = m3DOF->getBodyNode(0)->getTransform().matrix();
      psi =  atan2(Tf(0,0),-Tf(1,0));
      qBody1 = atan2(Tf(0,1)*cos(psi) + Tf(1,1)*sin(psi), Tf(2,1));
      Eigen::VectorXd q = m3DOF->getPositions();
      Eigen::VectorXd xPlane(3);
      xPlane << q(0),q(1),0;

      dist = xPlane.norm();

      Eigen::VectorXd dq_orig = m3DOF->getVelocities();
      Eigen::Matrix<double, 8, 1> dq;
      dq << (Tf.block<3,3>(0,0) * dq_orig.head(3)) , (Tf.block<3,3>(0,0) * dq_orig.segment(3,3)), dq_orig(6), dq_orig(7);
      dqFilt->AddSample(dq);

      // Wheel Rotation (World Frame)
      thL = q(6) - std::abs(qBody1);
      thR = q(7) - std::abs(qBody1);

      // Corresponding Velocities (Filtered)
      dpsi = dq(2);
      dpsiFilt = dqFilt->average(2);
      dqBody1 = -dq_orig(0);
      dqBody1Filt = (-dqFilt->average(0)*sin(psi) + dqFilt->average(1)*cos(psi));
      dthL = dq(6) + dqBody1;
      dthLFilt = dqFilt->average(6) + dqBody1Filt;
      dthR = dq(7) + dqBody1;
      dthRFilt = dqFilt->average(7) + dqBody1Filt;


      cout << dist << endl;

      if(true){//steps == 1 || steps == 500 || steps == 5000){

        cout << "thL/R: " << thL << " " << thR << " | dthL/R: " << dthL << " " << dthR;
        cout << "| qBody1: " << qBody1 << "| dqBody1: " << dqBody1 << endl;

      //   cout << "Sum: " << q(6) + qBody1 << endl;
      //   cout << "Difference: " << q(6) - qBody1 << endl;
      //   cout << "Absolute: " << q(6) - std::abs (qBody1)<< endl;
      }

      // thR = 
      
      // Constraints
      // 1. dZ0 = 0                                               => dq_orig(4)*cos(qBody1) + dq_orig(5)*sin(qBody1) = 0
      // 2. da3 + R/L*(dthL - dthR) = 0                           => dq_orig(1)*cos(qBody1) + dq_orig(2)*sin(qBody1) + R/L*(dq_orig(6) - dq_orig(7)) = 0 
      // 3. da1*cos(psii) + da2*sin(psii) = 0                     => dq_orig(1)*sin(qBody1) - dq_orig(2)*cos(qBody1) = 0
      // 4. dX0*sin(psii) - dY0*cos(psii) = 0                     => dq_orig(3) = 0
      // 5. dX0*cos(psii) + dY0*sin(psii) - R/2*(dthL + dthR) = 0 => dq_orig(4)*sin(qBody1) - dq_orig(5)*cos(qBody1) - R/2*(dq_orig(6) + dq_orig(7) - 2*dq_orig(0)) = 0
      Eigen::Matrix<double, 5, 1> c;
      c << (dq_orig(4)*cos(qBody1) + dq_orig(5)*sin(qBody1)), (dq_orig(1)*cos(qBody1) + dq_orig(2)*sin(qBody1) + R/L*(dq_orig(6) - dq_orig(7))), (dq_orig(1)*sin(qBody1) - dq_orig(2)*cos(qBody1)), dq_orig(3), (dq_orig(4)*sin(qBody1) - dq_orig(5)*cos(qBody1) - R/2*(dq_orig(6) + dq_orig(7) - 2*dq_orig(0)));
      cFilt->AddSample(c);
      //if(Tf(2,1) > 0)
      {
      for(int i=0; i<8; i++) outFile << dq(i) << ", ";
      for(int i=0; i<8; i++) outFile << dqFilt->average(i) << ", ";
      outFile << psi << ", " << dpsi << ", " << qBody1 << ", " << dqBody1 << ", " <<  dthL << ", " << dthR << ", ";
      outFile << psiFilt << ", " << dpsiFilt << ", " << qBody1Filt << ", " << dqBody1Filt << ", " <<  dthLFilt << ", " << dthRFilt << ", ";
      for(int i=0; i<5; i++) outFile << c(i) << ", ";
      for(int i=0; i<5; i++) outFile << cFilt->average(i) << ", ";
      for(int i=0; i<8; i++) outFile << q(i) << ", "; 
      outFile << std::endl;
      }

      // Calculate the quantities we are interested in
      // cout << psi << endl;
      // cout <<  q(6) << "  " << q(7) << endl;
      // dpsiFilt = dqFilt->average(2);
      // dqBody1 = -dq_orig(0);
      // dqBody1Filt = (-dqFilt->average(0)*sin(psi) + dqFilt->average(1)*cos(psi));
      // dthL = dq(6) + dqBody1;
      // dthLFilt = dqFilt->average(6) + dqBody1Filt;
      // dthR = dq(7) + dqBody1;
      // dthRFilt = dqFilt->average(7) + dqBody1Filt;


      /// Decoupled Control
      // Eigen::VectorXd xL(4);
      // xL << qBody1, thL, dqBody1, dthL;

      // Eigen::VectorXd xR(4);
      // xR << qBody1, thR, dqBody1, dthR;

      // Eigen::VectorXd xDesired(4);
      // xDesired << 0,0,0,0;

      // Eigen::VectorXd F(4);
      // F << -444.232838220247,   -13.8564064605507,   -111.681669536162,   -26.3837189119712;
      // // F << -1848.03012979190,    -13.8564064605509,   -269.597433286135,   -18.3661533292315;
      // // F << -2443.80123228903,    -21.9089023002056,   -366.198685841371,   -28.8559866982220;
      // // F << -3617.30655560822,    -69.2820323027686,   -676.022987559394,   -95.4128736402033;
      
      // tauL = -F.transpose()*(xL-xDesired);
      // tauR = -F.transpose()*(xR-xDesired);



      Eigen::VectorXd xL(4);
      xL << qBody1, thL, dqBody1, dthL;
      // xL << qBody1, thR, dqBody1, dthR;

      Eigen::VectorXd xR(4);
      xR << qBody1, thR, dqBody1, dthR;


      Eigen::VectorXd xDesired(4);
      xDesired << 0,0,0,0;

      Eigen::VectorXd F(4);
      // F << -444.232838220247,   -13.8564064605507,   -111.681669536162,   -26.3837189119712;
      // F << -1848.03012979190,    -13.8564064605509,   -269.597433286135,   -18.3661533292315;
      // F << -2443.80123228903,    -21.9089023002056,   -366.198685841371,   -28.8559866982220;
      // F << -3617.30655560822,    -69.2820323027686,   -676.022987559394,   -95.4128736402033;
      // F << -6802.85419804756, -69.2820323027654,   -928.328202601923,   -86.9485793475169;
      // F << -2669.67918242245,   -21.9089023002072,   -371.814613562732,   -28.2110280320200;
      F << -2549.11511950484,   -21.9089023002060,   -368.094666786882,   -28.5125368751267;

      tauL = -F.transpose()*(xL-xDesired);
      tauR = -F.transpose()*(xR-xDesired);

      // tauL *= .001;
      // tauR *= .001;
      tauL *= .0015;
      tauR *= .0015;


      cout << tauL << " " << tauR << endl;

      // Eigen::Vector3d f = -mKp*(x - _targetPosition) - mKv*dx;

      

      // arbitrary control inputs
      steps++;
      // double headSign = (cos(2*3.14/80*steps*0.01) > 0) ? 1 : -1;
      // double spinSign = (sin(2*3.14/20*steps*0.01) > 0) ? 1 : -1;
      // double uL = 1;//((steps < 1000) ? 0 : 5*headSign-5*spinSign);
      // double uR = 1;//((steps < 1000) ? 0 : 5*headSign+5*spinSign);
      /*double head = cos(2*3.14/10*steps*0.01);
      double spin = ( (sin(2*3.14/40*steps*0.01)>0) ? -1 : 1 );
      double uL = ((steps < 1000) ? 0 : 60*head-200*spin);
      double uR = ((steps < 1000) ? 0 : 60*head+200*spin);*/

      mForces << 0, 0, 0, 0, 0, 0, tauL, tauR;
      m3DOF->setForces(mForces);
      
      SimWindow::timeStepping();
    }
    ~MyWindow() {
      outFile.close();     
    }
    

  protected:

    SkeletonPtr m3DOF;

    Eigen::VectorXd qInit;

    Eigen::VectorXd dof1;

    double dist;
    double psi, dpsi, thL, dthL, thR, dthR, qBody1, dqBody1;
    double psiFilt, dpsiFilt, qBody1Filt, dqBody1Filt, dthLFilt, dthRFilt;

    double R;
    double L;

    double tauL, tauR;
    
    int steps;

    Eigen::Matrix<double, 8, 1> mForces;
   
    ofstream outFile; 

    filter *dqFilt, *cFilt;
};


SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
//  body->setFrictionCoeff(1e16);

  // Give the body a shape
  double floor_width = 50;
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
//  threeDOF->getBodyNode("LWheel")->setFrictionCoeff(1e16);
//  threeDOF->getBodyNode("RWheel")->setFrictionCoeff(1e16);
//  threeDOF->getJoint(0)->setDampingCoefficient(0, 1.0);
//  threeDOF->getJoint(1)->setDampingCoefficient(0, 1.0);
  
  // Get it into a useful configuration
  double psiInit = M_PI/4, qBody1Init = 0;//M_PI;//0;
  Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  // RotX(pi/2)*RotY(-pi/2+psi)*RotX(-qBody1)
  baseTf.prerotate(Eigen::AngleAxisd(-qBody1Init,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+psiInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));
  Eigen::Matrix<double, 8, 1> q;
//  q << 1.2092, -1.2092, -1.2092, 0, 0, 0.28, 0, 0;
  q << aa.angle()*aa.axis(), 0, 0, 0.28, 0, 0;
  threeDOF->setPositions(q);

  return threeDOF;
}


int main(int argc, char* argv[])
{

  SkeletonPtr threeDOF = create3DOF_URDF();
  SkeletonPtr floor = createFloor();

  WorldPtr world = std::make_shared<World>();
  world->addSkeleton(threeDOF);
  world->addSkeleton(floor);

  MyWindow window(world);
  glutInit(&argc, argv);
  window.initWindow(1280,720, "3DOF URDF");
  glutMainLoop();
}


