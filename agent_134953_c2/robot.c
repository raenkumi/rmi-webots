#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "map.h"
#include "robot.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

// ROBOT METHODS
// ==========================================

#pragma region PUBLIC METHODS

void robot_init(Robot *robot)
{
  RobotDevices *d = &robot->dev;
  wb_robot_init(); // wb lib init

  d->timestep = (int)wb_robot_get_basic_time_step();
  d->gps = wb_robot_get_device("gps");
  d->camera = wb_robot_get_device("camera");
  d->compass = wb_robot_get_device("compass");
  d->gs0 = wb_robot_get_device("gs0");
  d->gs1 = wb_robot_get_device("gs1");
  d->gs2 = wb_robot_get_device("gs2");
  d->left_motor = wb_robot_get_device("left wheel motor");
  d->right_motor = wb_robot_get_device("right wheel motor");

  wb_gps_enable(d->gps, d->timestep);
  wb_camera_enable(d->camera, d->timestep);
  wb_compass_enable(d->compass, d->timestep);
  wb_distance_sensor_enable(d->gs0, d->timestep);
  wb_distance_sensor_enable(d->gs1, d->timestep);
  wb_distance_sensor_enable(d->gs2, d->timestep);

  camera_init(&robot->dev, &robot->now.img);
  wb_motor_set_position(d->left_motor, INFINITY);
  wb_motor_set_position(d->right_motor, INFINITY);
  set_avelocity(d, 0, 0);

  wb_robot_step(d->timestep);

  robot_sense(robot);

  RobotExploration *e = &robot->exp;
  e->state = EXP_SETUP;

  RobotNavigation *n = &robot->nav;
  n->state = NAV_SETUP;

  printf("init robot finished\n");
}

int robot_kill(Robot *robot)
{
  wb_robot_cleanup();
  camera_free(&robot->now.img);
  return 0;
}

void robot_sense(Robot *robot)
{
  RobotValues *v = &robot->now;
  static int dbg_tick = 0;

  v->rcoord = get_raw_coord(&robot->dev);
  v->coord = get_coord(&robot->dev);
  v->heading = get_heading(&robot->dev);
  v->heading_rad = get_heading_rad(&robot->dev);
  v->gs0_v = get_gsvalue(&robot->dev.gs0);
  v->gs1_v = get_gsvalue(&robot->dev.gs1);
  v->gs2_v = get_gsvalue(&robot->dev.gs2);
  v->speed = get_lvelocity(&robot->dev);
  v->lmv = get_motor_avelocity(&robot->dev.left_motor);
  v->rmv = get_motor_avelocity(&robot->dev.right_motor);

  if (dbg_tick++ % 15 == 0)
    printf(
        "lmv %.2f; rmv %.2f; x %i; y %i; rx %.2f; ry %.2f; hdn %d; hdr %.2f; g0 %.0f; g1 %.0f; g2 %.0f\n", v->lmv,
        v->rmv, v->coord.x, v->coord.y, v->rcoord.x, v->rcoord.y, v->heading, v->heading_rad, v->gs0_v, v->gs1_v,
        v->gs2_v);

  get_image(&robot->dev, &v->img);
}

void robot_explore(Robot *robot)
{
  // define checks for RobotExplore, basically continue if not DONE
  RobotExploration *e = &robot->exp;

  switch (e->state)
  {
  case EXP_SETUP: // await spin, then start exploration
    if (robot->nav.state != NAV_SETUP) e->state = EXP_EXPLORE;
    break;

  case EXP_EXPLORE: // select next target coordinate or finish mapping
    if (robot->nav.state != NAV_ROUTE) break;

    Coord target;
    if (next_target(robot, &target))
    {
      robot->nav.goal = target;
      printf("[expl] goal=(%d,%d) top=%d\n", target.x, target.y, robot->exp.top);
    }
    else
    {
      e->state = EXP_DONE;
      map_save("robot.map");
      printf("exploration done!!!\n");
    }
    break;

  case EXP_DONE: // cleanup, maybe?
    break;
  }
}

void robot_navigate(Robot *robot)
{
  // define checks for RobotNavigation, try altering states by what happened over senses
  RobotNavigation *n = &robot->nav;
  RobotValues *v = &robot->now;
  RobotScan *s = &n->scanFlags;

  switch (n->state)
  {
  case NAV_SETUP: // 360 spin to map origin, then start exploring
    if (!s->done) break;
    if (!s->spin)
    {
      commit_cell(robot);
      s->spin = true;
      n->target_heading = round(v->heading_rad / (M_PI / 2)) * (M_PI / 2);
      printf("[setup] snap target=%.2f from hdr=%.2f\n", n->target_heading, v->heading_rad);
    }

    if (fabs(angle_diff(n->target_heading, v->heading_rad)) < CP_HEADING_TOL)
    {
      scan_reset(s);
      n->state = NAV_ROUTE;
      printf("[nav] SETUP->ROUTE t=%.2f\n", wb_robot_get_time());
    }
    break;

  case NAV_MAP: // senses cell while driving
    // 1. LATCH, gs are primary factual and camera is residual temporal redundancy
    Direction headn = v->heading;
    bool cl, cr;
    camera_sides(&v->img, &cl, &cr);

    if ((v->gs1_v < GS_WHITE_THRESHOLD) || camera_ahead(&v->img)) s->edges[headn] = LINE;
    if ((v->gs0_v < GS_WHITE_THRESHOLD) || cl) s->edges[translate_heading(headn, TURN_LEFT)] = LINE;
    if ((v->gs2_v < GS_WHITE_THRESHOLD) || cr) s->edges[translate_heading(headn, TURN_RIGHT)] = LINE;

    // 2. PREVIEW, camera picks next heading preferring straight-ahead when possible
    if (!cell_center(v->rcoord, n->target_cell)) break;

    // 3. CONSOLIDATE at cell center, translate to north
    s->edges[translate_heading(headn, TURN_BACK)] = LINE;
    commit_cell(robot);

    // 4. CONTINUE straigh ahead if possible, else turn
    Coord fwd = cell_step(v->coord, headn);
    bool go_ahead = (s->edges[headn] == LINE) && !cell_visited(fwd);
    scan_reset(s);

    if (go_ahead)
    { // Overrides DFS stack by picking next cell as straight ahead
      n->target_cell = fwd;
      n->path_len = 1;
      n->path_iter = 0;
      n->path[0] = fwd;
      n->state = NAV_NAVIGATE;
      printf("[nav] MAP->NAVIGATE fwd=(%d,%d)\n", fwd.x, fwd.y);
    }
    else
    {
      printf("[nav] MAP->ROUTE\n");
      n->state = NAV_ROUTE;
    }
    break;

  case NAV_ROUTE: // select next cell to map and calculate route there
    break;

  case NAV_NAVIGATE: // move robot controller to get to next cell
    break;
  }
}

void robot_actuate(Robot *robot)
{
  switch (robot->nav.state)
  {
  case NAV_SETUP:
    if (!robot->nav.scanFlags.done) robot->nav.scanFlags.done = scan_tick(robot);
    else spin_tick(robot);
    break;

  case NAV_NAVIGATE:
    // todo: drive_tick / turn dispatch
    break;

  default:
    set_avelocity(&robot->dev, 0, 0);
    break;
  }
}

#pragma endregion

// PRIVATE METHODS
// ==========================================

#pragma region PRIVATE METHODS

/// @brief Stops the robot, only returns when linear (GPS) velocity is 0.
/// @param robot
/// @return
// bool stop(RobotDevices *robot) {}

// int turn(Robot *robot, TurnDirection dir) {}

// int spin(RobotDevices *robot, SpinDirection spinDir)
// {
//   while (stop(robot));
//
//   double start_headn = get_heading_rad(robot);
//   double curr_headn = start_headn;
//
//   do
//   {
//     double spinL = spinDir == CLOCKWISE ? MT_CRUISE_VELOCITY_RAD : -MT_CRUISE_VELOCITY_RAD;
//     double spinR = spinDir == CLOCKWISE ? -CRUISE_VELOCITY_RAD : MT_CRUISE_VELOCITY_RAD;
//     wb_motor_set_velocity(robot->left_motor, spinL);
//     wb_motor_set_velocity(robot->right_motor, spinR);
//
//     curr_headn = get_heading_rad(robot);
//
//   } while (curr_headn != start_headn); // needs another stopping condition
//
//   return 1;
// }

bool scan_tick(Robot *robot)
{
  RobotScan *s = &robot->nav.scanFlags;
  RobotValues *v = &robot->now;

  printf(
      "[scan] latch dir=%d hdr=%.2f g= (%.0f, %.0f, %.0f)\n", v->heading, v->heading_rad, v->gs0_v, v->gs1_v, v->gs2_v);

  if (!s->primed)
  {
    scan_reset(s);
    s->prev_rad = v->heading_rad;
    s->primed = true;
  }

  s->accum += fabs(angle_diff(v->heading_rad, s->prev_rad));
  s->prev_rad = v->heading_rad;

  if (v->gs0_v < GS_WHITE_THRESHOLD || v->gs1_v < GS_WHITE_THRESHOLD || v->gs2_v < GS_WHITE_THRESHOLD)
  {
    if (s->edges[v->heading] != LINE)
      printf(
          "[scan] latch dir=%d hdr=%.2f g=(%.0f,%.0f,%.0f)\n", v->heading, v->heading_rad, v->gs0_v, v->gs1_v,
          v->gs2_v);
    s->edges[v->heading] = LINE;
  }

  if (s->accum >= 2.0 * M_PI)
  {
    set_avelocity(&robot->dev, 0, 0);
    printf(
        "[scan] done accum=%.2f edges NESW=%d%d%d%d\n", s->accum, s->edges[0], s->edges[1], s->edges[2], s->edges[3]);
    return true;
  }

  set_avelocity(&robot->dev, -MT_SCAN_VELOCITY_RAD, MT_SCAN_VELOCITY_RAD);
  return false;
}

void scan_reset(RobotScan *scan)
{
  for (int dir = 0; dir < MAP_DIR_COUNT; dir++) scan->edges[dir] = UNKNOWN;
  scan->accum = 0;
  scan->primed = false;
  scan->done = false;
  scan->spin = false;
}

void commit_cell(Robot *robot)
{
  RobotScan *s = &robot->nav.scanFlags;
  Coord here = robot->now.coord;

  for (int dir = 0; dir < MAP_DIR_COUNT; dir++)
    if (s->edges[dir] == UNKNOWN) s->edges[dir] = NO_LINE;

  cell_record(here, s->edges);

  for (int dir = 0; dir < MAP_DIR_COUNT; dir++)
  {
    if (s->edges[dir] != LINE) continue;

    Coord neighb = cell_step(here, dir);
    if (!cell_visited(neighb)) robot->exp.explStack[robot->exp.top++] = neighb;

    printf(
        "[commit] (%d,%d) NESW=%d%d%d%d top=%d\n", here.x, here.y, s->edges[0], s->edges[1], s->edges[2], s->edges[3],
        robot->exp.top);
  }
}

bool next_target(Robot *robot, Coord *out)
{
  RobotExploration *e = &robot->exp;

  while (e->top > 0)
  {
    Coord c = e->explStack[--e->top];
    if (!cell_visited(c))
    {
      *out = c;
      return true;
    }
  }
  return false;
}

#pragma endregion

// SENSING METHODS
// ==========================================

#pragma region SENSING METHODS

RCoord get_raw_coord(RobotDevices *dev)
{
  RCoord coord;

  const double *coords = wb_gps_get_values(dev->gps);
  coord.x = coords[0];
  coord.y = coords[1];

  return coord;
}

double get_raw_x(RobotDevices *dev)
{
  RCoord coord = get_raw_coord(dev);
  return coord.x;
}

double get_raw_y(RobotDevices *dev)
{
  RCoord coord = get_raw_coord(dev);
  return coord.y;
}

Coord get_coord(RobotDevices *dev)
{
  RCoord rcoord = get_raw_coord(dev);
  return cell_normalize(rcoord);
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

void camera_free(CameraImage *img)
{
  free(img->px);
  img->px = NULL;
}

bool get_image(RobotDevices *dev, CameraImage *img)
{
  const unsigned char *raw = wb_camera_get_image(dev->camera);
  if (raw == NULL) return false;

  memcpy(img->px, raw, (size_t)img->w * img->h * sizeof *img->px);
  return true;
}

bool camera_sides(CameraImage *img, bool *cl, bool *cr)
{
  *cl = false;
  *cr = false;

  int camx = img->w / 2;

  for (int y = img->h - CM_NEAR_BAND; y < img->h; y++)
  {
    if (!px_dark(img, camx, y)) continue;

    int run = 0;
    for (int x = camx - 1; x >= 0 && px_dark(img, x, y); x--) run++;
    if (run >= CM_SIDE_MIN_PX) *cl = true;

    run = 0;
    for (int x = camx + 1; x < img->w && px_dark(img, x, y); x++) run++;
    if (run >= CM_SIDE_MIN_PX) *cr = true;
  }

  return *cl || *cr;
}

bool camera_ahead(CameraImage *img)
{
  int camx = img->w / 2;
  int rows = 0;

  for (int y = img->h / 4; y < img->h / 2; y++)
  {
    bool dark = false;
    for (int x = camx - 2; x <= camx + 2; x++)
      if (px_dark(img, x, y))
      {
        dark = true;
        break;
      }
    if (dark) rows++;
  }

  return rows >= CM_AHEAD_ROWS;
}

/// @brief Gets robot heading in compass values (N,S,E,W)
Direction get_heading(RobotDevices *dev)
{
  double hr = get_heading_rad(dev);     // (-pi/pi]
  int q = (int)lround(hr / (M_PI / 2)); // -2..2

  switch (q)
  {
  case 0:
    return DIR_EAST;
  case 1:
    return DIR_NORTH;
  case -1:
    return DIR_SOUTH;
  default:
    return DIR_WEST;
  }
}

/// @brief Gets robot heading in radians (-pi/pi]
double get_heading_rad(RobotDevices *dev)
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
  static RCoord prevc;
  static double prevt;
  static bool primed = false;

  RCoord coord = get_raw_coord(dev);
  double inst = wb_robot_get_time();

  if (!primed)
  {
    prevc = coord;
    prevt = inst;
    primed = true;
    return 0.0;
  }

  double dt = inst - prevt;
  double v = (dt > 0) ? hypot(coord.x - prevc.x, coord.y - prevc.y) / dt : 0.0;

  prevc = coord;
  prevt = inst;
  return v;
}

#pragma endregion

// ACTUATION METHODS
// ==========================================

#pragma region ACTUATION METHODS

void set_avelocity(RobotDevices *dev, double lv, double rv)
{
  set_motor_avelocity(dev->left_motor, lv);
  set_motor_avelocity(dev->right_motor, rv);
}

void set_motor_avelocity(WbDeviceTag mt, double mv)
{
  if (mv > MT_MAX_VELOCITY_RAD) mv = MT_MAX_VELOCITY_RAD;
  if (mv < -MT_MAX_VELOCITY_RAD) mv = -MT_MAX_VELOCITY_RAD;

  wb_motor_set_velocity(mt, mv);
}

#pragma endregion

// HELPER METHODS
// ==========================================

#pragma region HELPER METHODS

bool px_dark(const CameraImage *img, int x, int y)
{
  CameraPixel p = img->px[y * img->w + x];
  return (p.r + p.g + p.b) / 3 < CM_DARK;
}

Direction translate_heading(Direction headn, TurnDirection dir) { return (headn + dir) % MAP_DIR_COUNT; }

double angle_diff(double a, double b) { return atan2(sin(a - b), cos(a - b)); }
bool spin_tick(Robot *robot) { return align_tick(robot, 0.0); }
bool turn_tick(Robot *robot) { return align_tick(robot, MT_TURN_VELOCITY_RAD); }
bool drive_tick(Robot *robot) { return align_tick(robot, MT_CRUISE_VELOCITY_RAD); }

/// @brief
/// @param robot
/// @param wfv Wheel Forward Velocity (rad/s), how much the robot's wheel advances in relation to its Angular Velocity
/// @return
bool align_tick(Robot *robot, double wfv)
{
  double err = angle_diff(robot->nav.target_heading, robot->now.heading_rad);

  if (fabs(err) < CP_HEADING_TOL)
  {
    set_avelocity(&robot->dev, wfv, wfv);
    return true;
  }

  // get and clamp the robot's Wheel Steering Velocity (rad/s)
  double wsv = MT_KP_TURN * err;
  if (wsv > MT_CRUISE_VELOCITY_RAD) wsv = MT_CRUISE_VELOCITY_RAD;
  if (wsv < -MT_CRUISE_VELOCITY_RAD) wsv = -MT_CRUISE_VELOCITY_RAD;

  if (wfv == 0.0)
  {
    if (wsv > 0 && wsv < MT_MIN_TURN) wsv = MT_MIN_TURN;
    if (wsv < 0 && wsv > -MT_MIN_TURN) wsv = -MT_MIN_TURN;
  }

  set_avelocity(&robot->dev, wfv + wsv, wfv - wsv);
  return false;
}

#pragma endregion
