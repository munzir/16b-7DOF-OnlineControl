#include "Controller.hpp"
#include <nlopt.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <chrono>

using namespace dart;
using namespace std;
using namespace chrono;
#define SAMPLES 2174
#define DOF 7
Eigen::Matrix<double, DOF*3, DOF*3> I_21 = Eigen::Matrix<double, DOF*3, DOF*3>::Identity();
Eigen::Matrix<double, SAMPLES, 1> k;

//131720 is the stack limit - 784
//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _endEffector)
  : mRobot(_robot),
    mEndEffector(_endEffector)
{
  cout << endl << "============= Controller constructor activated =============" << endl << endl;
    assert(_robot != nullptr);
    assert(_endEffector != nullptr);

    cout << "Getting total degrees of freedom ";
    const int dof = mRobot->getNumDofs();
    cout << "| DOF = " << dof << endl;

    if (dof != DOF) {
      cout << "DOF do not match with defined constant ... exiting program!" << endl;
      exit (EXIT_FAILURE);
    }

    cout << "Initializing time ... ";
    mTime = 0;
    cout << "t(0) = " << mTime << endl;

    mForceNon.setZero(DOF);
    mTauRef.setZero(DOF);
    mKp = Eigen::Matrix<double, DOF, DOF>::Zero();
    mKv = Eigen::Matrix<double, DOF, DOF>::Zero();

    for (int i = 0; i < DOF; ++i) {
      mKp(i, i) = 750.0;
      mKv(i, i) = 250.0;
    }

    // Remove position limits
    for (int i = 0; i < DOF; ++i)
      _robot->getJoint(i)->setPositionLimitEnforced(false);

    // Set joint damping
    for (int i = 0; i < DOF; ++i)
      _robot->getJoint(i)->setDampingCoefficient(0, 0.5);

  // // Dump data
  onlineTime.open   ("./data/onlineTime.txt");
    onlineTime        << "onlineTime" << endl;
    // States - ref, rbd, gp predicted
    onlineQ_ref.open  ("./data/onlineQ_ref.txt");
    onlineQ_ref       << "onlineQ_ref" << endl;

    onlineQ_rbd.open  ("./data/onlineQ_rbd.txt");
    onlineQ_rbd       << "onlineQ_rbd" << endl;

    onlineQ_pred.open ("./data/onlineQ_pred.txt");
    onlineQ_pred      << "onlineQ_pred" << endl;

    // End-effector position - ref, rbd, gp predicted
    onlineEE_ref.open ("./data/onlineEE_ref.txt");
    onlineEE_ref      << "onlineEE_ref" << endl;

    onlineEE_rbd.open ("./data/onlineEE_rbd.txt");
    onlineEE_rbd      << "onlineEE_rbd" << endl;

    onlineEE_pred.open("./data/onlineEE_pred.txt");
    onlineEE_pred     << "onlineEE_pred" << endl;

    // Torques - ref, rbd, gp predicted
    onlineTau_ref.open("./data/onlineTau_ref.txt");
    onlineTau_ref     << "onlineTau_ref" << endl;

    onlineTau_rbd.open("./data/onlineTau_rbd.txt");
    onlineTau_rbd     << "onlineTau_rbd" << endl;

    onlineTau_pred.open ("./data/onlineTau_pred.txt");
    onlineTau_pred    << "onlineTau_pred" << endl;

    onlineTau_non1.open ("./data/onlineTau_non1.txt");
    onlineTau_non1    << "onlineTau_non1" << endl;

    onlineTau_non2.open ("./data/onlineTau_non2.txt");
    onlineTau_non2    << "onlineTau_non2" << endl;

    onlinexq.open     ("./data/onlinexq.txt");
    onlinexq          << "onlinexq" << endl;

    onlinekAlpha.open ("./data/onlinekAlpha.txt");
    onlinekAlpha      << "onlinekAlpha" << endl;

  // Read q, dq, ddq from training text files
  readAlpha.open    ("../TrajectorySingleArm/matlab/forPredictionData/alpha2174_nonddq.txt");
  readTrainQ.open   ("../TrajectorySingleArm/all_data/dataSet2/trainingData/dataQ.txt");
  readTrainQdot.open("../TrajectorySingleArm/all_data/dataSet2/trainingData/dataQdot.txt");
  readTrainQdotdot.open("../TrajectorySingleArm/all_data/dataSet2/trainingData/dataQdotdot.txt");

  // storing states q, dq, ddq for xp
  Eigen::MatrixXd xpQ   (1, DOF);
    Eigen::MatrixXd xpdQ  (1, DOF);
    Eigen::MatrixXd xpddQ (1, DOF);
    string s;

    for (int i = 0; i < 1; i++) {
      getline(readTrainQ,s);
      getline(readTrainQdot,s);
      getline(readTrainQdotdot,s);
    }

    cout << " -------------- " << endl;
    cout << "Reading states [q, dq, ddq] from training text files | ";
    if (readTrainQ.is_open()) {
        for (int row = 0; row < SAMPLES; row++) {

            for (int col = 0; col < DOF; col++) {
                float qVal = 0.0;
                float dqVal = 0.0;
                float ddqVal = 0.0;

                readTrainQ >> qVal;
                readTrainQdot >> dqVal;
                readTrainQdotdot >> ddqVal;

                xpQ(0, col) = qVal;
                xpdQ(0, col) = dqVal;
                xpddQ(0, col) = ddqVal;
            }
            xp.row(row) << xpQ, xpdQ;//, xpddQ;
        }
        readTrainQ.close();
        readTrainQdot.close();
        readTrainQdotdot.close();
        cout <<"xp size: " << xp.rows() <<"x"<<   xp.cols()   << endl;
    }
    else {
      cout << "Unable to read states from training data files ... check folders!" << endl;
      exit (EXIT_FAILURE);
    }
    cout << "Operation completed!" << endl;

    cout << " -------------- " << endl;
    cout << "Reading matrix alpha_k from text file | ";
    if (readAlpha.is_open()) {
      for (int row = 0; row < SAMPLES; row++) {
          for (int col = 0; col < DOF; col++) {
              float value = 0.0;
              readAlpha >> value;
              alpha(row, col) = value;
          }
      }
      readAlpha.close();
      cout <<"alpha size: " << alpha.rows() <<"x"<<   alpha.cols()   << endl;
    }
    else {
      cout << "Alpha-k file unable to read ... check folders!" << endl;
      exit (EXIT_FAILURE);
    }
    // cout << alpha.topLeftCorner(10,7) << endl;
    cout << "Operation completed!" << endl;

    dt = 0.001;

    // Define coefficients for sinusoidal, pulsation frequency for q and dq
    cout << " -------------- " << endl;
    cout << " Setting coefficients for optical trajectories | ";

  // // Values from actual URDF
  // wf = 0.558048373585;
  // a << -0.009, -0.36, 0.311, -0.362,
  //       0.095, -0.132, -0.363, 0.474,
  //       -0.418, -0.25, -0.12, 0.119,
  //       0.023, 0.113, 0.497, 0.213,
  //       -0.23, -0.237, 0.153, -0.147,
  //       0.366, 0.366, 0.302, -0.373,
  //       -0.247, -0.166, 0.315, 0.031;
  //
  // b <<  -0.051, 0.027, 0.003, -0.332,
  //       -0.292, 0.358, -0.056, -0.436,
  //       -0.355, 0.039, -0.397, -0.445,
  //       0.328, 0.256, -0.36, 0.143,
  //       0.428, 0.093, 0.035, -0.28,
  //       -0.39, -0.085, 0.388, 0.46,
  //       -0.046, 0.135, -0.428, 0.387;
  //
  // q0 << 0.235, -0.004, -0.071, 0.095, -0.141, 0.208, -0.182;

  // Values from random URDF
  wf = 0.433881552676;
  a << 0.127, -0.062, 0.464, -0.109,
       0.112, -0.647, -0.325, 0.417,
      -0.156, -0.345, -0.035, 0.436,
      -0.153, -0.187, 0.481, 0.831,
      -0.482, -0.377, 0.107, 0.138,
       0.074, -0.117, 0.403, -0.085,
      -0.417, 0.157, -0.354, 0.346;

  b << 0.42, -0.485, 0.196, -0.337,
      -0.394, 0.313, -0.357, -0.363,
       0.326, -0.278, -0.258, 0.066,
      -0.475, -0.296, -0.429, 0.11,
      -0.098, 0.291, -0.137, 0.183,
      -0.065, 0.069, 0.284, -0.174,
      -0.134, -0.364, 0.168, -0.301;

  q0 << -0.203, -0.178, 0.397, 0.105, 0.093, -0.274, -0.204;

  // wf = 0.933881552676;
  // a << 0.127, -0.062, 0.464, -0.109,
  //     -0.122, -0.447, -0.325, 0.417,
  //     -0.156, -0.345, -0.035, 0.436,
  //     -0.153, -0.187, 0.181, 0.231,
  //     -0.482, -0.377, 0.107, 0.138,
  //     0.074, -0.117, 0.403, -0.085,
  //     -0.417, 0.357, -0.354, 0.346;
  //
  // b << 0.42, -0.485, 0.196, -0.337,
  //     -0.394, 0.313, -0.357, -0.363,
  //     0.326, -0.278, -0.258, 0.066,
  //     -0.475, -0.296, -0.429, 0.11,
  //     -0.098, 0.291, -0.137, 0.183,
  //     -0.065, 0.069, 0.284, -0.174,
  //     -0.434, -0.364, 0.168, -0.001;
  //
  // q0 << -0.203, -0.178, 0.097, 0.105, 0.193, -0.274, -0.204;

  // Setting initial joint angles to the robot
  mRobot->setPositions(q0);
  cout << "Operation completed!" << endl;

  // Writing pulsation frequency for setting Time axis while plotting
  ofstream wf_value;
  wf_value.open  ("./data/wf_value.txt");
  wf_value        << "wf_value" << endl;
  wf_value        << wf << endl;
  wf_value.close();

  cout << endl << "============= Controller constructor successful ============" << endl << endl;
}

//==============================================================================
Controller::~Controller() {}

//==============================================================================
struct OptParams{
  Eigen::Matrix<double, -1, DOF> P;
  Eigen::VectorXd b;
};

//==============================================================================
void printMatrix(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      cout << A(i,j) << ", ";
    }
    cout << endl;
  }
  cout << endl;
}


//==============================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, DOF, 1> X(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, DOF, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
    // cout << "mGrad: " << endl << mGrad << endl << endl;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  double normSquared = pow((optParams->P*X - optParams->b).norm(), 2);
  // cout << "Norm sq: " << normSquared << endl << endl;
  return (0.5 * normSquared);
}


//==============================================================================
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//==============================================================================
Eigen::MatrixXd error(Eigen::VectorXd dq, const int dof) {
  Eigen::MatrixXd err(DOF,1);
  for (int i = 0; i < dof; i++) {
    // double sign = sgn(dq(i));
    err(i) = -( 2 / ( 1 + exp(-2*dq(i)) ) -1 ) ;
  }
  return err;
}


//==============================================================================
//=========== WRAPPER FUNCTIONS FOR COMPUTING TORQUE ==========================
void getQueryKernel(Eigen::MatrixXd xp, Eigen::MatrixXd xq,
  const int N, const double charLength, const double coeff) {
  // xp - matrix of dimension Nx21
  // xq - vector of dimension 1x21

  double sf  = exp(coeff);
  double len = exp(charLength);
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  for (int i = 0; i < N; i++) {
    Eigen::MatrixXd diff = xp.row(i) - xq;
    Eigen::MatrixXd distMatrix = diff*diff.transpose();  //Mahalanobis distance
    double distance = distMatrix(0,0)/(len*len);
    k.row(i) << sf*sf*exp( -0.5*distance );
  }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double, milli> time_span = t2 - t1;
  cout << "Elapsed time: " << time_span.count() << endl;
  // return k;
  // double expLength = exp(charLength);
    // double expCoeff  = exp(coeff);
    //
    // Eigen::Matrix<double, SAMPLES, 1> k;
    // for (int i = 0; i < N; i++) {
    //   Eigen::MatrixXd diff = xp.row(i) - xq;
    //   Eigen::MatrixXd dist = (1/(expLength*expLength))* diff*I_21*diff.transpose();
    //   double distance = dist(0,0);
    //   k.row(i) << expCoeff*expCoeff*exp( -0.5*distance );
    // }
    // return k;
}

//==============================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition) {
  // const int dof = mRobot->getNumDofs();

  // Compute joint angles & velocities using Pulsed Trajectories
  qref << 0, 0, 0, 0, 0, 0, 0;
    dqref = qref;

    // cout << "Time: " << mTime << endl;
    for (int joint = 0; joint < DOF; joint++) {
      for (int l = 1; l <= 4; l++) {
        qref(joint) = qref(joint) + (a(joint, l-1)/(wf*l))*sin(wf*l*mTime)
        - (b(joint, l-1)/(wf*l))*cos(wf*l*mTime);
        dqref(joint) = dqref(joint) + a(joint,l-1)*cos(wf*l*mTime)
        + b(joint, l-1)*sin(wf*l*mTime);
      }
    }
    qref = qref + q0;

    mTime += dt;
    count += 1;

    // Get the stuff that we need
    Eigen::MatrixXd  M    = mRobot->getMassMatrix();                // n x n
    Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
    Eigen::VectorXd  q    = mRobot->getPositions();                 // n x 1
    Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1
    Eigen::VectorXd ddq   = mRobot->getAccelerations();             // n x 1
    Eigen::VectorXd ddqref = -mKp*(q - qref) - mKv*(dq - dqref);    // n x 1

    // Get the EE's cartesian coordinate
    Eigen::VectorXd EE_pos_online = mEndEffector->getTransform().translation();

    // Optimizer stuff
    nlopt::opt opt(nlopt::LD_MMA, DOF);
    OptParams optParams;
    std::vector<double> ddqref_vec(DOF);
    double minf;
    // cout << "Initialized optimizer variables ... " << endl << endl;

    // Perform optimization to find joint accelerations
    Eigen::Matrix<double, DOF, DOF> I7 = Eigen::Matrix<double, DOF, DOF>::Identity();

    // cout << "Passing optimizing parameters ... ";
    optParams.P = I7;
    optParams.b = ddqref;
    // cout << "Success !" << endl << endl;

    opt.set_min_objective(optFunc, &optParams);
    opt.set_xtol_rel(1e-4);
    opt.set_maxtime(0.005);
    opt.optimize(ddqref_vec, minf);
    Eigen::Matrix<double, DOF, 1> ddqRef(ddqref_vec.data());


    // Computing predicted torque
    const double charLength = 1.23;    // cov [1]
    const double coeff      = 4.20;    // cov [2]
    Eigen::Matrix<double, 1, DOF*2> xq;

  // xp << q, dq, ddq;
  xq << q.transpose(), dq.transpose();//, ddq.transpose();

  Eigen::Matrix<double, DOF, DOF> errCoeff = Eigen::Matrix<double, DOF, DOF>::Identity();
    errCoeff(0,0) =  30.0;    //30.0;     30.0
    errCoeff(1,1) = 200.0;    //200.0;    10.0
    errCoeff(2,2) =   5.0;    //15.0;      5.0
    errCoeff(3,3) =   5.0;    //100.0;     5.0
    errCoeff(4,4) =   3.0;    //3.0;       3.0
    errCoeff(5,5) =   3.0;    //25.0;      5.0
    errCoeff(6,6) =   0.0;    //1.0;       1.0

  if (flag==0) {
    mTauRef  = M*ddqRef + Cg;
    mRobot->setForces(mTauRef);
  }
  else if (flag == 1) {
    mTauRbd   = M*ddqRef + Cg;
    mForceNon = mTauRbd + errCoeff*error(dq, DOF);
    mRobot->setForces(mForceNon);
  }
  else if (flag == 2) {
    getQueryKernel(xp, xq, SAMPLES, charLength, coeff); // SAMPLES x 1

    kAlpha   = (k.transpose()*alpha).transpose();

    mTauPred  = M*ddqRef + Cg + kAlpha;
    mForceNon = mTauPred + errCoeff*error(dq, DOF);
    mRobot->setForces(mForceNon);
  }


  int sample = 30;
  if (count%sample == 0) {
    if (flag == 0) {
      onlineTime      << mTime << endl;
      onlineQ_ref     << qref.transpose() << endl;
      onlineEE_ref    << EE_pos_online.transpose() << endl;
      onlineTau_ref   << mTauRef.transpose() << endl;
    }
    else if (flag == 1) {
      onlineTime      << mTime << endl;
      onlineQ_ref     << qref.transpose() << endl;
      onlineQ_rbd     << q.transpose() << endl;
      onlineEE_rbd    << EE_pos_online.transpose() << endl;
      onlineTau_rbd   << mTauRbd.transpose() << endl;
      onlineTau_non1  << mForceNon.transpose() << endl;
    }
    else if (flag == 2) {
      onlinexq        << xq << endl;
      onlineQ_pred    << q.transpose() << endl;
      onlineEE_pred   << EE_pos_online.transpose() << endl;
      onlineTau_pred  << mTauPred.transpose() << endl;
      onlineTau_non2  << mForceNon.transpose() << endl;
      onlinekAlpha    << kAlpha.transpose()   << endl;
    }
  }

  // Closing operation
  double T = 2*3.1416/wf;
  int cycle = 1;
  if (mTime >= cycle*T) {
    cout << " ------------------------------------------------------- " <<  endl;
    cout << " Model {" << flag << "} simulated. Re-setting arm to initial location!" << endl;
    cout << " ------------------------------------------------------- " <<  endl;
    mTime = 0;
    flag += 1;
    mRobot->setPositions(q0);
  }

  if (flag == 3) {
    // cout << "Time period met. Closing simulation ..." << endl;
    cout << "Data collected for all 3 types of behavior." << endl;
    cout << "Closing all files handles ... ";
    onlineTime.close();

    onlineQ_ref.close();
    onlineEE_ref.close();
    onlineTau_ref.close();

    onlineQ_rbd.close();
    onlineEE_rbd.close();
    onlineTau_rbd.close();

    onlineQ_pred.close();
    onlineEE_pred.close();
    onlineTau_pred.close();
    onlineTau_non1.close();;
    onlineTau_non2.close();;
    onlinekAlpha.close();
    cout << "File handles closed!" << endl << endl << endl;
    exit (EXIT_FAILURE);
  }
}

//==============================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

//==============================================================================
dart::dynamics::BodyNode* Controller::getEndEffector() const {
  return mEndEffector;
}

//==============================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}
