#include "MyWindow.hpp"
#include <iostream>

//==============================================================================
MyWindow::MyWindow(Controller* _controller)
  : SimWindow(),
    mController(_controller),
    mCircleTask(false)
{
  assert(_controller != nullptr);

  // Set the initial target positon to the initial position of the end effector
  mTargetPosition = mController->getEndEffector()->getTransform().translation();
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
  if (mCircleTask)
  {
    static double time = 0.0;
    const double dt = 0.0005;
    const double radius = 0.6;
    Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.1, 0.0);

    mTargetPosition = center;
    mTargetPosition[0] = radius * std::sin(time);
    mTargetPosition[1] = 0.25 * radius * std::sin(time);
    mTargetPosition[2] = radius * std::cos(time);

    time += dt;
  }

  // Update the controller and apply control force to the robot
  mController->update(mTargetPosition);

  // Step forward the simulation
  mWorld->step();
}

//==============================================================================
void MyWindow::drawWorld() const
{
  // Draw the target position
  if (mRI)
  {
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(mTargetPosition);
    mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    mRI->popMatrix();
  }

  // Draw world
  SimWindow::drawWorld();
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  double incremental = 0.01;

  switch (_key)
  {
    case 'c':  // print debug information
      if (mCircleTask)
      {
        std::cout << "Circle task [off]." << std::endl;
        mCircleTask = false;
      }
      else
      {
        std::cout << "Circle task [on]." << std::endl;
        mCircleTask = true;
      }
      break;
    case 'q':
      mTargetPosition[0] -= incremental;
      break;
    case 'w':
      mTargetPosition[0] += incremental;
      break;
    case 'a':
      mTargetPosition[1] -= incremental;
      break;
    case 's':
      mTargetPosition[1] += incremental;
      break;
    case 'z':
      mTargetPosition[2] -= incremental;
      break;
    case 'x':
      mTargetPosition[2] += incremental;
      break;
    default:
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y);

  glutPostRedisplay();
}
