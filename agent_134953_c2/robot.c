#include <stdio.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "map.h"
#include "robot.h"

Cell route[MAX_CELLS];
Cell explorationStack[MAX_CELLS];
int top = 0;

// ROBOT FUNCTIONS
// ==========================================
#pragma region PUBLIC METHODS

void robot_init(Robot *robot)
{
  RobotDevices devs;
  wb_robot_init(); // wb lib init

  devs.timestep = (int)wb_robot_get_basic_time_step();
  devs.left_motor = wb_robot_get_device("left wheel motor");
  devs.right_motor = wb_robot_get_device("right wheel motor");
  devs.gps = wb_robot_get_device("gps");
  devs.camera = wb_robot_get_device("camera");
  devs.compass = wb_robot_get_device("compass");
  devs.gs0 = wb_robot_get_device("gs0");
  devs.gs1 = wb_robot_get_device("gs1");
  devs.gs2 = wb_robot_get_device("gs2");

  wb_gps_enable(devs.gps, TIME_STEP_MS);
  wb_camera_enable(devs.camera, TIME_STEP_MS);
  wb_compass_enable(devs.compass, TIME_STEP_MS);
  wb_distance_sensor_enable(devs.gs0, TIME_STEP_MS);
  wb_distance_sensor_enable(devs.gs1, TIME_STEP_MS);
  wb_distance_sensor_enable(devs.gs2, TIME_STEP_MS);

  wb_motor_set_position(devs.left_motor, INFINITY);
  wb_motor_set_position(devs.right_motor, INFINITY);
  set_avelocity(&devs, 0);

  wb_robot_step(devs.timestep);
  robot->dev = devs;
  robot->now = robot_sense(robot);
}

RobotValues robot_sense(Robot *robot)
{
  RobotValues vals;

  vals.lmv = get_motor_avelocity(&robot->dev.left_motor);
  vals.rmv = get_motor_avelocity(&robot->dev.right_motor);
  vals.coord = get_coord(&robot->dev);
  vals.rcoord = get_raw_coord(&robot->dev);
  vals.img = get_image(&robot->dev);
  vals.heading = get_heading(&robot->dev);
  vals.gs0_v = get_gsvalue(&robot->dev.gs0);
  vals.gs1_v = get_gsvalue(&robot->dev.gs1);
  vals.gs2_v = get_gsvalue(&robot->dev.gs2);

  printf(
      "lmv %.2f; rmv %.2f; x %.2f; y %.2f; hdn %.2f; g0 %.2f; g1 %.2f; g2 %.2f", vals.lmv, vals.rmv, vals.coord.x,
      vals.coord.y, vals.heading, vals.gs0_v, vals.gs1_v, vals.gs2_v);

  return vals;
}

void robot_explore(Robot *robot)
{
  if (top == 0) explorationStack[top++];
}

void robot_navigate(Robot *robot) {}

#pragma endregion

#pragma region PRIVATE METHODS

int stop(RobotDevices *robot)
{
  bool stationary = false;
  set_avelocity(robot, 0);

  // check gps stationary
  while (!stationary)
  {
    double rx = 0, ry = 0;
    Coord coord = get_coord(robot);
    double v = hypot(coord.x - rx, coord.y - ry) / (TIME_STEP_MS / 1000.0);

    rx = coord.x;
    ry = coord.y;

    stationary = v < 1e-4;
  }

  return 1;
}

// int turn(Robot *robot, TurnDirection dir) {}

int spin(RobotDevices *robot, SpinDirection spinDir)
{
  while (stop(robot));

  double start_headn = get_heading(robot);
  double curr_headn = start_headn;

  do
  {
    double spinL = spinDir == CLOCKWISE ? CRUISE_VELOCITY_RAD : -CRUISE_VELOCITY_RAD;
    double spinR = spinDir == CLOCKWISE ? -CRUISE_VELOCITY_RAD : CRUISE_VELOCITY_RAD;
    wb_motor_set_velocity(robot->left_motor, spinL);
    wb_motor_set_velocity(robot->right_motor, spinR);

    curr_headn = get_heading(robot);

  } while (curr_headn != start_headn); // needs another stopping condition

  return 1;
}

#pragma endregion

#pragma region SENSING METHODS

/// @brief Calculates robot wheel angular velocity (NOT ACTUAL VELOCITY)
/// @param r
/// @return
double get_avelocity(RobotDevices *dev)
{
  double lv = wb_motor_get_velocity(dev->left_motor);
  double rv = wb_motor_get_velocity(dev->right_motor);

  return lv + rv / 2;
}

double get_motor_avelocity(WbDeviceTag *mt) { return wb_motor_get_velocity(*mt); }

/// @brief Calculates robot coordinate angular velocity (NOT ACTUAL VELOCITY)
/// @param r
/// @return
double get_pvelocity(RobotDevices *dev)
{
  double lv = wb_motor_get_velocity(dev->left_motor);
  double rv = wb_motor_get_velocity(dev->right_motor);

  return lv + rv / 2;
}

Coord get_raw_coord(RobotDevices *dev)
{
  Coord coord;

  const double *coords = wb_gps_get_values(dev->gps);
  coord.x = coords[0];
  coord.x = coords[1];

  return coord;
}

double get_raw_x(RobotDevices *dev)
{
  Coord coord = get_raw_coord(dev);
  return coord.x;
}

double get_raw_y(RobotDevices *dev)
{
  Coord coord = get_raw_coord(dev);
  return coord.y;
}

Coord get_coord(RobotDevices *dev)
{
  const double *rcoords = wb_gps_get_values(dev->gps);
  return normalize(rcoords);
}

double get_x(RobotDevices *dev)
{
  Coord coord = get_coord(dev);
  return coord.x;
}

double get_y(RobotDevices *dev)
{
  Coord coord = get_coord(dev);
  return coord.y;
}

unsigned char *get_image(RobotDevices *dev) { return wb_camera_get_image(dev->camera); }

/// @brief Gets robot heading in radians (-pi/pi]
double get_heading(RobotDevices *dev)
{
  const double *n = wb_compass_get_values(dev->compass);
  return atan2(n[1], n[0]);
}

double get_gsvalue(WbDeviceTag *gs) { return wb_distance_sensor_get_value(*gs); }

const double *get_gsvalues(RobotDevices *dev)
{
  static double g[3];

  g[0] = get_gsvalue(&dev->gs0);
  g[1] = get_gsvalue(&dev->gs1);
  g[2] = get_gsvalue(&dev->gs2);

  return g;
}

#pragma endregion

#pragma region ACTUATION METHODS

void set_avelocity(RobotDevices *dev, double v)
{
  if (v >= MAX_VELOCITY_RAD) v = MAX_VELOCITY_RAD;

  set_motor_velocity(dev->left_motor, v);
  set_motor_velocity(dev->right_motor, v);
}

void set_motor_velocity(WbDeviceTag m, double v)
{
  if (v >= MAX_VELOCITY_RAD) v = MAX_VELOCITY_RAD;
  wb_motor_set_velocity(m, v);
}

#pragma endregion
