#include <stdio.h>
#include <webots/robot.h>

#include "map.h"
#include "robot.h"

// MAIN
// ==========================================

int main(int argc, char **argv)
{
  // 1. init robot
  Robot robot;
  robot_init(&robot);

  // tick loop
  while (wb_robot_step(robot.dev.timestep) != -1)
  {
    // 2. SENSE current state
    robot_sense(&robot);

    // 3. DECIDE actions to be done
    robot_explore(&robot);
    robot_navigate(&robot);

    // 4. CONTROL actuations
    robot_actuate(&robot);
  }

  wb_robot_cleanup();
  return 0;
}