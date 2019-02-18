/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"
#include <nlopt.hpp>
#include <string>
#include <iostream>

//==========================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _LeftendEffector,
                       dart::dynamics::BodyNode* _RightendEffector)
  : mRobot(_robot),
    mLeftEndEffector(_LeftendEffector),
    mRightEndEffector(_RightendEffector)
   {
  assert(_robot != nullptr);
  assert(_LeftendEffector != nullptr);
  assert(_RightendEffector != nullptr);

  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;

  mForces.setZero(dof);
  mKp.setZero();
  mKv.setZero();

  for (int i = 0; i < 3; ++i) {
    mKp(i, i) = 750.0;
    mKv(i, i) = 250.0;
  }

  // Remove position limits
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);

  // Set joint damping
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);
}

//=========================================================================
Controller::~Controller() {}
//=========================================================================
struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

//=========================================================================
void printMatrix(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      std::cout << A(i,j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

//========================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, 18, 1> X(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 18, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5 * pow((optParams->P*X - optParams->b).norm(), 2));
}

//=========================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition) {
  using namespace dart;
  using namespace std;

  // Get the stuff that we need for left arm
  Eigen::Vector3d xLft    = mLeftEndEffector->getTransform().translation();
  Eigen::Vector3d dxLft   = mLeftEndEffector->getLinearVelocity();
  math::LinearJacobian JvLft   = mLeftEndEffector->getLinearJacobian();       // 3 x n
  math::LinearJacobian dJvLft  = mLeftEndEffector->getLinearJacobianDeriv();  // 3 x n

  // // Get the stuff that we need for right arm
  Eigen::Vector3d xRgt    = mRightEndEffector->getTransform().translation();
  Eigen::Vector3d dxRgt   = mRightEndEffector->getLinearVelocity();
  math::LinearJacobian JvRgt   = mRightEndEffector->getLinearJacobian();       // 3 x n
  math::LinearJacobian dJvRgt  = mRightEndEffector->getLinearJacobianDeriv();  // 3 x n

  // Get the stuff that we need for the robot
  Eigen::MatrixXd M     = mRobot->getMassMatrix();                // n x n
  Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
  Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1

  // ddxref for left and right arms
  Eigen::Vector3d ddxrefLft = -mKp*(xLft - _targetPosition) - mKv*dxLft;
  Eigen::Vector3d ddxrefRgt = -mKp*(xRgt - _targetPosition) - mKv*dxRgt;

  // cout <<"Matrix M Size:   " << M.rows()  <<"x"<<    M.cols()   << endl;
  // cout <<"Coriolis C Size: " << Cg.rows() <<"x"<<   Cg.cols()   << endl;
  // cout <<"Velocities Size: " << dq.rows() <<"x"<<   dq.cols()   << endl << endl;
  // cout <<"Velocities: " << endl<< dq << endl << endl;
  //
  // cout <<"JvLft Size:       " << JvLft.rows()   <<"x"<<   JvLft.cols()  << endl;
  // cout <<"dJvLft Size:      " << dJvLft.rows()  <<"x"<<  dJvLft.cols()  << endl;
  // cout <<"JvRgt Size:       " << JvRgt.rows()   <<"x"<<   JvRgt.cols()  << endl;
  // cout <<"dJvRgt Size:      " << dJvRgt.rows()  <<"x"<<  dJvRgt.cols()  << endl;

  int dof = mRobot->getNumDofs();
  // cout << "[update] DoF: " << dof << endl << endl << endl;

  // cout << "Left Jacobian Columns: " << endl;
  // for (int i = 0; i < dof; i++) {
  //   cout << "Column: " << i << endl;
  //   cout << JvLft.col(i) << endl;
  // }
  // cout << endl;
  // cout << "Right Jacobian Columns: " << endl;
  // for (int i = 0; i < dof; i++) {
  //   cout << "Column: " << i << endl;
  //   cout << JvRgt.col(i) << endl;
  // }
  // cout << endl;

  // Weights
  double weightRight, weightLeft;
  weightRight = 1.0;
  weightLeft  = 1-weightRight;

  // Computing left and right P from Jacobians;
  Eigen::Vector3d zeroColumn(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 7> zero7Columns;
  zero7Columns << zeroColumn, zeroColumn, zeroColumn, zeroColumn, \
              zeroColumn, zeroColumn, zeroColumn;
  // cout << "Computing zero-column vector and zero-7-column matrix .." << endl;

  Eigen::Matrix<double, 3, 18> FullJacobianRight;
  FullJacobianRight <<  JvRgt.block<3,3>(0,0), zeroColumn, JvRgt.block<3,7>(0,3), zero7Columns;
  // cout <<"FullJacobianRight Size:  " << FullJacobianRight.rows()   <<"x"<<   FullJacobianRight.cols()  << endl;  //
  // cout << "Generated FullJacobianRight matrix ..." << endl;
  // cout << "Computed P-right against weight  using Full Jacobian Right ... !" << endl << endl << endl;
  // cout << PRight << endl << endl;

  Eigen::Matrix<double, 3, 18> FullJacobianLeft;
  FullJacobianLeft  << JvLft.block<3,3>(0,0), zeroColumn, zero7Columns, JvLft.block<3,7>(0,3);
  // cout <<"FullJacobianLeft Size:  " << FullJacobianLeft.rows()   <<"x"<<   FullJacobianLeft.cols()  << endl;  //
  // cout << "Generated FullJacobianLeft matrix ..." << endl;
  // cout << "Computed P-left against weight  using Full Jacobian Left ... !" << endl << endl << endl;
  // cout << PLeft << endl << endl;

  // Computing left and right b from Jacobian derivative
  Eigen::Matrix<double, 3, 18> FullJacobianDerRgt;
  FullJacobianDerRgt  <<  dJvRgt.block<3,3>(0,0), zeroColumn, dJvRgt.block<3,7>(0,3), zero7Columns;
  // cout <<"FullJacobianDerRgt Size:  " << FullJacobianDerRgt.rows()   <<"x"<<   FullJacobianDerRgt.cols()  << endl;  //
  // cout << "Generated FullJacobianDerRgt matrix ..." << endl;
  // cout <<"bRight Size:  " << bRight.rows()   <<"x"<<   bRight.cols()  << endl;  //
  // cout << "Computed b-right against weight using Full Jacobian Derivative Right ... !" << endl << endl << endl;


  Eigen::Matrix<double, 3, 18> FullJacobianDerLft;
  FullJacobianDerLft  <<  dJvLft.block<3,3>(0,0), zeroColumn, zero7Columns, dJvLft.block<3,7>(0,3);
  // cout <<"FullJacobianDerLft Size:  " << FullJacobianDerLft.rows()   <<"x"<<   FullJacobianDerLft.cols()  << endl;  //
  // cout << "Generated FullJacobianDerLft matrix ..." << endl;
  // cout <<"bLeft Size:  " << bLeft.rows()   <<"x"<<   bLeft.cols()  << endl;  //
  // cout << "Computed b-left against weight using Full Jacobian Derivative Left ... !" << endl << endl << endl;


  Eigen::MatrixXd PRight  = weightRight*FullJacobianRight;
  Eigen::MatrixXd PLeft   = weightLeft*FullJacobianLeft;

  Eigen::VectorXd bRight  = -weightRight*( FullJacobianDerRgt*dq - ddxrefRgt );
  Eigen::VectorXd bLeft   = -weightLeft* ( FullJacobianDerLft*dq - ddxrefLft );

  //
  // cout << "P Right:"  << endl << PRight << endl << endl;
  // cout << "P Left:"   << endl << PLeft  << endl << endl;
  // cout << "b Right:"  << endl << bRight << endl << endl;
  // cout << "b Left:"   << endl << bLeft  << endl << endl;

  // // Optimizer stuff
  nlopt::opt opt(nlopt::LD_MMA, dof);
  OptParams optParams;
  std::vector<double> ddq_vec(dof);
  double minf;

  // Create optimization parameters P and b
  Eigen::MatrixXd NewP(PRight.rows() + PLeft.rows(), PRight.cols() );
  NewP << PRight,
          PLeft;
  cout <<"NewP Size:  " << NewP.rows()   <<"x"<<   NewP.cols()  << endl;  //

  Eigen::VectorXd NewB(bRight.rows() + bLeft.rows(), bRight.cols() );
  NewB << bRight,
          bLeft;
  cout <<"NewB Size:  " << NewB.rows()   <<"x"<<   NewB.cols()  << endl;  //

  // Perform optimization to find joint accelerations
  cout << "Passing optimizing parameters ...";
  optParams.P = NewP;
  optParams.b = NewB;
  cout << "Success !" << endl << endl;


  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);
  opt.optimize(ddq_vec, minf);

  Eigen::Matrix<double, 18, 1> ddq(ddq_vec.data());
  // cout <<"Qdotdot Vector:      " << endl << ddq << endl;

  //torques
  double w0 = weightRight;
  double w1 = weightLeft;
  Eigen::Matrix<double, 18, 18> weightMatrix;

  // weightMatrix << w0+w1,      0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,  w0+w1,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,  w0+w1, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,   w0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,   w0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,   w0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,   w0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,   w0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w1,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w1,  0.0,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w1,  0.0,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w1,  0.0,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w1,  0.0,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w1,  0.0,
                      // 0.0,    0.0,    0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   w1;

  // cout << weightMatrix << endl << endl;
  mForces = M*ddq + Cg;
  // cout << "Forces:" << endl << mForces << endl;
  // Apply the joint space forces to the robot
  mRobot->setForces(mForces);
}

//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

//=========================================================================
dart::dynamics::BodyNode* Controller::getEndEffector(const std::string &s) const {
  if (s.compare("left")) {  return mLeftEndEffector; }
  else if (s.compare("right")) { return mRightEndEffector; }
}

//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}
