#include <stdio.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "map.h"
#include "robot.h"

// ROBOT METHODS
// ==========================================

#pragma region PUBLIC METHODS

void robot_init(Robot *robot)
{
  RobotDevices *d = &robot->dev;
  wb_robot_init(); // wb lib init

  d->timestep = (int)wb_robot_get_basic_time_step();
  d->left_motor = wb_robot_get_device("left wheel motor");
  d->right_motor = wb_robot_get_device("right wheel motor");
  d->gps = wb_robot_get_device("gps");
  d->camera = wb_robot_get_device("camera");
  d->compass = wb_robot_get_device("compass");
  d->gs0 = wb_robot_get_device("gs0");
  d->gs1 = wb_robot_get_device("gs1");
  d->gs2 = wb_robot_get_device("gs2");

  wb_gps_enable(d->gps, d->timestep);
  wb_camera_enable(d->camera, d->timestep);
  wb_compass_enable(d->compass, d->timestep);
  wb_distance_sensor_enable(d->gs0, d->timestep);
  wb_distance_sensor_enable(d->gs1, d->timestep);
  wb_distance_sensor_enable(d->gs2, d->timestep);

  camera_init(&robot->now.img, &robot->dev);
  wb_motor_set_position(d->left_motor, INFINITY);
  wb_motor_set_position(d->right_motor, INFINITY);
  set_avelocity(&d, 0);

  wb_robot_step(d->timestep);

  robot_sense(robot);

  RobotExploration *e = &robot->exp;
  e->state = EXP_SETUP;

  RobotNavigation *n = &robot->nav;
  n->state = NAV_SETUP;

  printf("init robot finished\n");
}

int robot_kill(void)
{
  wb_robot_cleanup();
  return 0;
}

void robot_sense(Robot *robot)
{
  RobotValues *v = &robot->now;

  v->origin = get_raw_coord(&robot->dev);
  v->rcoord = get_raw_coord(&robot->dev);
  v->coord = get_coord(&robot->dev);
  v->heading = get_heading(&robot->dev);
  v->gs0_v = get_gsvalue(&robot->dev.gs0);
  v->gs1_v = get_gsvalue(&robot->dev.gs1);
  v->gs2_v = get_gsvalue(&robot->dev.gs2);
  v->speed = get_lvelocity(&robot->dev);
  v->lmv = get_motor_avelocity(&robot->dev.left_motor);
  v->rmv = get_motor_avelocity(&robot->dev.right_motor);

  printf(
      "lmv %.2f; rmv %.2f; x %.2f; y %.2f; hdn %.2f; g0 %.2f; g1 %.2f; g2 %.2f\n", v->lmv, v->rmv, v->coord.x,
      v->coord.y, v->heading, v->gs0_v, v->gs1_v, v->gs2_v);

  get_image(&robot->dev, &v->img);
}

void robot_explore(Robot *robot)
{
  // define checks for RobotExplore, basically continue if not DONE
}

void robot_navigate(Robot *robot)
{
  // define checks for RobotNavigation, try altering states by what happened over senses
}
void robot_actuate(Robot *robot) {}

#pragma endregion

// PRIVATE METHODS
// ==========================================

#pragma region PRIVATE METHODS

/// @brief Stops the robot, only returns when linear (GPS) velocity is 0.
/// @param robot
/// @return
bool stop(RobotDevices *robot)
{
  bool stationary = false;
  set_avelocity(robot, 0);

  while (!stationary)
  {
    double speed = get_lvelocity(robot);
    stationary = speed < 1e-4;
  }

  return stationary;
}

int turn(Robot *robot, TurnDirection dir) {}

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

// SENSING METHODS
// ==========================================

#pragma region SENSING METHODS

Coord get_raw_coord(RobotDevices *dev)
{
  Coord coord;

  const double *coords = wb_gps_get_values(dev->gps);
  coord.x = coords[0];
  coord.y = coords[1];

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

void camera_init(RobotDevices *dev, CameraImage *img)
{
  img->w = wb_camera_get_width(dev->camera);
  img->h = wb_camera_get_height(dev->camera);
  img->px = malloc((size_t)img->w * img->h * sizeof *img->px);
}

bool get_image(RobotDevices *dev, CameraImage *img)
{
  const unsigned char *raw = wb_camera_get_image(dev->camera);
  if (raw == NULL) return false;

  memcpy(img->px, raw, (size_t)img->w * img->h * sizeof *img->px);
  return true;
}

void camera_free(CameraImage *img)
{
  free(img->px);
  img->px = NULL;
}

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

/// @brief Calculates robot wheel angular velocity.
/// THIS IS NOT ACTUAL VELOCITY and may 0 even when robot is moving.
/// @param dev
/// @return
double get_avelocity(RobotDevices *dev)
{
  double lv = wb_motor_get_velocity(dev->left_motor);
  double rv = wb_motor_get_velocity(dev->right_motor);

  return (lv + rv) / 2;
}

double get_motor_avelocity(WbDeviceTag *mt) { return wb_motor_get_velocity(*mt); }

/// @brief Estimates robot linear velocity in m/s (GPS finite difference) coordinates.
/// @param dev
/// @return m/s
double get_lvelocity(RobotDevices *dev)
{
  static Coord prevc;
  static double prevt;
  static bool primed = false;

  Coord coord = get_raw_coord(dev);
  double inst = wb_robot_get_time();

  if (!primed)
  {
    prevc = coord;
    prevt = inst;
    primed = true;
    return 0.0;
  }

  double dt = inst - prevt;
  double v = (dt > 0) ? hypot(coord.x - prevc.x, coord.y - prevc.y) / (dev->timestep / 1000.0) : 0.0;

  prevc = coord;
  prevt = inst;
  return v;
}

#pragma endregion

// ACTUATION METHODS
// ==========================================

#pragma region ACTUATION METHODS

void set_avelocity(RobotDevices *dev, double mv)
{
  if (mv >= MAX_VELOCITY_RAD) mv = MAX_VELOCITY_RAD;

  set_motor_avelocity(dev->left_motor, mv);
  set_motor_avelocity(dev->right_motor, mv);
}

void set_motor_avelocity(WbDeviceTag mt, double mv)
{
  if (mv >= MAX_VELOCITY_RAD) mv = MAX_VELOCITY_RAD;
  wb_motor_set_velocity(mt, mv);
}

#pragma endregion

// HELPER METHODS
// ==========================================

#pragma region HELPER METHODS

double calculate_heading_rad(RobotDevices *dev, TurnDirection dir) { return 0.0; }

#pragma endregion
