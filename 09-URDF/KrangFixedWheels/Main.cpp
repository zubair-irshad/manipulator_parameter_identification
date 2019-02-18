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

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MyWindow.hpp"


dart::dynamics::SkeletonPtr createKrang() {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang =
      loader.parseSkeleton("/home/krang/dart/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf");
  krang->setName("krang");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf  = Eigen::Isometry3d::Identity();
  // Eigen::Isometry3d tfRot  = Eigen::Isometry3d::Identity();

  tf.translation()      = Eigen::Vector3d(0.0, 0.0, 0.0);
  // tf *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
  krang->getJoint(0)->setTransformFromParentBodyNode(tf);

  // tfRot = Eigen::AngleAxisd(-0.50, Eigen::Vector3d::UnitX());
  // krang->getJoint(2)->setTransformFromParentBodyNode(tfRot);

  // Get it into a useful configuration
  // krang->getDof(4)->setPosition(50.0 * M_PI / 180.0);
  // krang->getDof(5)->setPosition(-50.0 * M_PI / 180.0);

  // krang->getDof(11)->setPosition(50.0 * M_PI / 180.0);
  // krang->getDof(12)->setPosition(-20.0 * M_PI / 180.0);


  return krang;
}

int main(int argc, char* argv[])
{
  // create and initialize the world
  dart::simulation::WorldPtr world(new dart::simulation::World);
  assert(world != nullptr);

  // load skeletons
  dart::utils::DartLoader dl;
  dart::dynamics::SkeletonPtr ground  = dl.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");
  dart::dynamics::SkeletonPtr robot = createKrang();

  world->addSkeleton(ground); //add ground and robot to the world pointer
  world->addSkeleton(robot);

  // create and initialize the world
  Eigen::Vector3d gravity(0.0,  -9.81, 0.0);
  world->setGravity(gravity);
  world->setTimeStep(1.0/1000);

  // create a window and link it to the world
  MyWindow window(new Controller(robot, robot->getBodyNode("lGripper"), robot->getBodyNode("rGripper") ) );
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(960, 720, "Forward Simulation");
  glutMainLoop();

  return 0;
}
