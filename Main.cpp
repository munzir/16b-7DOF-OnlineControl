#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MyWindow.hpp"

int main(int argc, char* argv[])
{
  // create and initialize the world
  dart::simulation::WorldPtr world(new dart::simulation::World);
  assert(world != nullptr);

  // load skeletons
  dart::utils::DartLoader dl;
  dart::dynamics::SkeletonPtr ground
      = dl.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");
  dart::dynamics::SkeletonPtr robot
      = dl.parseSkeleton("/home/mouhyemen/desktop/research/KrangDart/OnlineControl/krangArm.urdf");
  world->addSkeleton(ground);
  world->addSkeleton(robot);

  // create and initialize the world
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  world->setGravity(gravity);
  world->setTimeStep(1.0/1000);

  // create a window and link it to the world
  MyWindow window(new Controller(robot, robot->getBodyNode("L7_EFF")));
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Forward Simulation");
  glutMainLoop();

  return 0;
}
