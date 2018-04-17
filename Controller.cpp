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
//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _endEffector)
  : mRobot(_robot),
    mEndEffector(_endEffector)
{
  assert(_robot != nullptr);
  assert(_endEffector != nullptr);

  int dof = mRobot->getNumDofs();

  mForces.setZero(dof);

  mKp.setZero();
  mKv.setZero();

  for (int i = 0; i < 3; ++i)
  {
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

//==============================================================================
Controller::~Controller()
{
}
//==============================================================================
struct OptParams{
  Eigen::Matrix<double, -1, 7> P;
  Eigen::VectorXd b;
};



//==============================================================================
void matprint(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      std::cout << A(i,j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

//==============================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, 7, 1> X(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 7, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5 * pow((optParams->P*X - optParams->b).norm(), 2));
}

//==============================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition)
{
  using namespace dart;

  // Get the stuff that we need
  Eigen::Vector3d x    = mEndEffector->getTransform().translation();
  Eigen::Vector3d dx   = mEndEffector->getLinearVelocity();
  Eigen::MatrixXd M = mRobot->getMassMatrix();                   // n x n
  Eigen::VectorXd Cg   = mRobot->getCoriolisAndGravityForces();        // n x 1
  math::LinearJacobian Jv   = mEndEffector->getLinearJacobian();       // 3 x n
  math::LinearJacobian dJv  = mEndEffector->getLinearJacobianDeriv();  // 3 x n
  Eigen::VectorXd dq        = mRobot->getVelocities();                 // n x 1

  // Optimizer stuff
  nlopt::opt opt(nlopt::LD_MMA, 7);
  OptParams optParams;
  std::vector<double> ddq_vec(7);
  double minf;

  // ddxref
  Eigen::Vector3d ddxref = -mKp*(x - _targetPosition) - mKv*dx;

  // Perform optimization to find joint accelerations 
  optParams.P = Jv;
  optParams.b = -(dJv*dq - ddxref);
  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);
  opt.optimize(ddq_vec, minf);
  Eigen::Matrix<double, 7, 1> ddq(ddq_vec.data()); 
  

  //torques
  mForces = M*ddq + Cg;

  // Apply the joint space forces to the robot
  mRobot->setForces(mForces);
}

//==============================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const
{
  return mRobot;
}

//==============================================================================
dart::dynamics::BodyNode* Controller::getEndEffector() const
{
  return mEndEffector;
}

//==============================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/)
{
}

