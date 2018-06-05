#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <dart/dart.hpp>
using namespace std;
#define SAMPLES 2174
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
  /// File handles for dumping online-data and reading offline data
  // States - q and qref
  ofstream onlineTime;
  ofstream onlineQ_ref;
  ofstream onlineQ_rbd;
  ofstream onlineQ_pred;
  ofstream onlinexq;
  // End-effector position
  ofstream onlineEE_ref;
  ofstream onlineEE_rbd;
  ofstream onlineEE_pred;
  // Torques
  ofstream onlineTau_ref;
  ofstream onlineTau_rbd;
  ofstream onlineTau_pred;
  ofstream onlinekAlpha;
  ofstream onlineTau_non1;
  ofstream onlineTau_non2;

  ifstream readAlpha;
  ifstream readTrainQ;
  ifstream readTrainQdot;
  ifstream readTrainQdotdot;

  // const int LINES = 1006;
  Eigen::Matrix<double, SAMPLES, DOF*2> xp;
  Eigen::Matrix<double, SAMPLES, DOF> alpha;

  /// Variables for calculating reference (optical) trajectories
  double mTime;
    int count = 0;
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

  /// Variables for data generation
  int flag = 1;

  /// \brief Control forces
  Eigen::VectorXd mTauRef;
  Eigen::VectorXd mTauRbd;
  Eigen::VectorXd mTauPred;
  Eigen::VectorXd kAlpha;
  Eigen::VectorXd mForceNon;

  /// \brief Proportional gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKp;

  /// \brief Derivative gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKv;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
