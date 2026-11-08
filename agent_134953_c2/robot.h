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

// compass
#define CP_HEADING_TOL 0.05 // radians

// ground Sensors
#define GS_COUNT 3
#define GS_WHITE_THRESHOLD 500

// camera
#define CM_DARK 80       // gray threshold
#define CM_AHEAD_ROWS 4  // mid rows around center line
#define CM_NEAR_BAND 8   // bottom rows near robot
#define CM_SIDE_MIN_PX 6 // minimum LINE length from center to considered as PATH

// motors
#define MT_WHEEL_RADIUS_CM 3.0 // unknown
#define MT_SCAN_VELOCITY_RAD 2.0
#define MT_CRUISE_VELOCITY_RAD 5.0
#define MT_TURN_VELOCITY_RAD 2.0
#define MT_MAX_VELOCITY_RAD 6.0
#define MT_KP_TURN 4.0  // P gain: err rad -> whell rad/s
#define MT_MIN_TURN 0.4 // window so P don't slow when close to target

// camera
typedef struct
{
  unsigned char b, g, r, a;
} CameraPixel;

typedef struct
{
  int w, h;
  CameraPixel *px;
} CameraImage;

// exploration
typedef enum
{
  EXP_SETUP = 0, // Startup spin
  EXP_EXPLORE,   // Mapping maze
  EXP_DONE       // No more nodes to map
} ExplorationState;

// navigation
typedef enum
{
  NAV_SETUP = 0, // Startup spin
  NAV_MAP,       // At target cell
  NAV_ROUTE,     // Select next cell to map and get a route there
  NAV_NAVIGATE,  // Move to next cell
} NavigationState;

typedef struct
{
  Path edges[MAP_DIR_COUNT];
  double accum;
  double prev_rad;
  bool primed;
  bool done;
  bool spin;
} RobotScan;

// actuation
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
  TURN_AHEAD = 0,
  TURN_RIGHT,
  TURN_BACK,
  TURN_LEFT,
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
  RCoord rcoord;      // x, y (raw)
  Coord coord;        // x, y (normalized)
  CameraImage img;    // camera image
  Direction heading;  // compass values (N,S,E,W)
  double heading_rad; // radians (-pi/pi]
  double gs0_v;       // 0..999
  double gs1_v;       // 0..999
  double gs2_v;       // 0..999
  double speed;       // linear velocity m/s
  double lmv;         // radians/s
  double rmv;         // radians/s
} RobotValues;

typedef struct
{
  NavigationState state;
  Coord goal;
  Coord target_cell;
  double target_heading;
  Coord path[MAP_MAX_CELLS];
  int path_len, path_iter;
  RobotScan scanFlags;
} RobotNavigation;

typedef struct
{
  ExplorationState state;
  Coord explStack[MAP_MAX_CELLS * 4];
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

#pragma region ROBOT PUBLIC METHODS

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

#pragma endregion

// ROBOT PRIVATE METHODS
// ==========================================

#pragma region ROBOT PRIVATE METHODS

bool next_target(Robot *robot, Coord *out);
bool scan_tick(Robot *robot);
void scan_reset(RobotScan *scan);
void commit_cell(Robot *robot);

#pragma endregion

// ROBOT SENSING METHODS
// ==========================================

#pragma region ROBOT SENSING METHODS

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
void camera_free(CameraImage *img);
bool get_image(RobotDevices *dev, CameraImage *img);
bool camera_sides(CameraImage *img, bool *cl, bool *cr);
bool camera_ahead(CameraImage *img);

Direction get_heading(RobotDevices *dev);
double get_heading_rad(RobotDevices *dev);

double get_gsvalue(WbDeviceTag *gs);
const double *get_gsvalues(RobotDevices *dev);

#pragma endregion

// ROBOT ACTUATION METHODS
// ==========================================

#pragma region ROBOT SENSING METHODS

void set_avelocity(RobotDevices *dev, double lv, double rv);
void set_motor_avelocity(WbDeviceTag mt, double mv);

#pragma endregion

// HELPER METHODS
// ==========================================

#pragma region HELPER METHODS

bool px_dark(const CameraImage *img, int x, int y);

Direction translate_heading(Direction headn, TurnDirection dir);

double angle_diff(double a, double b);
bool align_tick(Robot *robot, double base);
bool spin_tick(Robot *robot);
bool turn_tick(Robot *robot);
bool drive_tick(Robot *robot);

#pragma endregion

#endif // ROBOT_H
