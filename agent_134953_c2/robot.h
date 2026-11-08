#ifndef ROBOT_H
#define ROBOT_H

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "map.h"

// ROBOT DEFINITIONS
// ==========================================

#pragma region ROBOT DEFINITIONS

// Ground Sensors
#define GROUND_SENSORS_COUNT 3
#define WHITE_THRESHOLD 500

// Motors
#define WHEEL_RADIUS_CM 3.0
#define CRUISE_VELOCITY_RAD 3.0
#define MAX_VELOCITY_RAD 6.28

typedef struct
{
  unsigned char b, g, r, a;
} CameraPixel;

typedef struct
{
  int w, h;
  CameraPixel *px;
} CameraImage;

typedef enum
{
  EXP_SETUP = 0, // Startup spin
  EXP_EXPLORE,   // Mapping maze
  EXP_DONE       // No more nodes to map
} ExplorationState;

typedef enum
{
  NAV_SETUP = 0, // Startup spin
  NAV_MAP,       // At target cell
  NAV_PATHFIND,  // Select next cell to map
  NAV_NAVIGATE,  // Move to next cell
} NavigationState;

typedef enum
{
  RBT_IDLE = 0,     // gps velocity at zero
  RBT_ACCELERATING, // wb_motor_set_velocity up
  RBT_CRUISING,     // wb_motor_set_velocity = CRUISE_VELOCITY
  RBT_DECELERATING, // wb_motor_set_velocity down
  RBT_TURNING
} ControlState;

typedef enum
{
  NEXT_NODE,
  STRAIGHT,
  RIGHT,
  LEFT,
  BACK
} TurnDirection;

typedef enum
{
  CLOCKWISE = 0,
  COUNTERCLOCKWISE,
} SpinDirection;

typedef struct
{
  int timestep;
  WbDeviceTag gps;
  WbDeviceTag camera;
  WbDeviceTag compass;
  WbDeviceTag gs0;
  WbDeviceTag gs1;
  WbDeviceTag gs2;
  WbDeviceTag left_motor;
  WbDeviceTag right_motor;
} RobotDevices;

typedef struct
{
  RCoord rcoord;   // x, y (raw)
  Coord coord;     // x, y
  CameraImage img; // todo: camera data structure
  double heading;
  double gs0_v;
  double gs1_v;
  double gs2_v;
  double speed; // linear velocity m/s
  double lmv;   // radians/s
  double rmv;   // radians/s
} RobotValues;

typedef struct
{
  NavigationState state;
  Coord goal;
  Coord target_cell;
  double target_heading;
  Coord path[MAX_CELLS];
  int path_len, path_iter;
} RobotNavigation;

typedef struct
{
  ExplorationState state;
  Coord explStack[MAX_CELLS];
  int top;
} RobotExploration;

typedef struct
{
  RobotDevices dev;
  RobotValues now;
  RobotNavigation nav;
  RobotExploration exp;
} Robot;

#pragma endregion

// ROBOT METHODS
// ==========================================

#pragma region ROBOT METHODS

/// @brief Creates Robot instance to be used
void robot_init(Robot *robot);
int robot_kill(Robot *robot);

/// @brief Reads all sensors of robot
void robot_sense(Robot *robot);

/// @brief Process all sensor readings of the robot
// void robot_process(Robot *robot);

/// @brief DFS of the current map
void robot_explore(Robot *robot);

/// @brief Controls navigation of current route
void robot_navigate(Robot *robot);

/// @brief Controls actuation of motors
void robot_actuate(Robot *robot);

double get_avelocity(RobotDevices *dev);
double get_motor_avelocity(WbDeviceTag *mt);
double get_lvelocity(RobotDevices *dev);

RCoord get_raw_coord(RobotDevices *dev);
double get_raw_x(RobotDevices *dev);
double get_raw_y(RobotDevices *dev);
Coord get_coord(RobotDevices *dev);
double get_x(RobotDevices *dev);
double get_y(RobotDevices *dev);

void camera_init(RobotDevices *dev, CameraImage *img);
bool get_image(RobotDevices *dev, CameraImage *img);
void camera_free(CameraImage *img);

double get_heading(RobotDevices *dev);

double get_gsvalue(WbDeviceTag *gs);
const double *get_gsvalues(RobotDevices *dev);

void set_avelocity(RobotDevices *dev, double v);
void set_motor_avelocity(WbDeviceTag mt, double mv);

double calculate_heading_rad(RobotDevices *dev, TurnDirection dir);

#pragma endregion

#endif // ROBOT_H
