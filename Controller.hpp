#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <dart/dart.hpp>
using namespace std;
#define SAMPLES 500
#define DOF 7
#define DOF3 21

/// \brief Operational space controller for 6-dof manipulator
class Controller
{
public:
  /// \brief Constructor
  Controller(dart::dynamics::SkeletonPtr _robot,
             dart::dynamics::BodyNode* _endEffector);

  /// \brief Destructor
  virtual ~Controller();

  /// \brief
  void update(const Eigen::Vector3d& _targetPosition);

  // double getTime() {return mTime;}

    /// \brief Get robot
  dart::dynamics::SkeletonPtr getRobot() const;

  /// \brief Get end effector of the robot
  dart::dynamics::BodyNode* getEndEffector() const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

private:
  /// File handles for dumping data
  ofstream dataQ;
  ofstream dataQref;
  ofstream dataQdot;
  ofstream dataQdotdot;
  ofstream dataTorque;
  ofstream dataTime;
  ofstream dataM;
  ofstream dataCg;
  ofstream dataError;

  ifstream readAlpha;
  ifstream readQ;
  ifstream readQdot;
  ifstream readQdotdot;

  const int LINES = 1005;
  Eigen::Matrix<double, SAMPLES, DOF3> xp;
  Eigen::Matrix<double, SAMPLES, DOF> alpha;

  /// Variables for calculating reference (optical) trajectories
  double mTime;
  double dt;
  double wf;
  Eigen::Matrix<double, DOF, 1> qref;
  Eigen::Matrix<double, DOF, 1> dqref;
  Eigen::Matrix<double, DOF, 1> q0;
  Eigen::Matrix<double, DOF, 4> a, b;

  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobot;

  /// \brief End-effector of the robot
  dart::dynamics::BodyNode* mEndEffector;

  /// \brief Control forces
  Eigen::VectorXd mForces;
  Eigen::VectorXd mTauPred;

  /// \brief Proportional gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKp;

  /// \brief Derivative gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKv;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
