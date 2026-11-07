#include <stdio.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

// WORLD DEFINITIONS
// ==========================================

#define TIME_STEP_MS 16
#define DURATION_MS 350000

// ROBOT DEFINITIONS
// ==========================================

#define GROUND_SENSORS_COUNT 3
#define WHITE_THRESHOLD 500
#define WHEEL_RADIUS_CM 3.0
#define CRUISE_VELOCITY_RAD 3.0
#define MAX_VELOCITY_RAD 6.28

typedef struct
{
  int timestep;
  WbDeviceTag left_motor;
  WbDeviceTag right_motor;
  WbDeviceTag gps;
  WbDeviceTag camera;
  WbDeviceTag compass;
  WbDeviceTag gs0;
  WbDeviceTag gs1;
  WbDeviceTag gs2;
} RobotDevices;

typedef struct
{
  double lmv;   // radians/s
  double rmv;   // radians/s
  Coord coord;  // x, y
  Coord rcoord; // x, y (raw)
  char *img;    // todo: camera data structure
  double heading;
  double gs0_v;
  double gs1_v;
  double gs2_v;
} RobotValues;

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
  RobotDevices dev;
  RobotValues now;
  NavigationState state;
} Robot;

// ROBOT FUNCTIONS
// ==========================================

/// @brief Creates Robot instance to be used
void robot_init(Robot *robot);

/// @brief Reads all sensors of robot
RobotValues robot_sense(Robot *robot);

/// @brief DFS of the current map
void robot_explore(Robot *robot);

/// @brief Controls navigation of current route
void robot_navigate(Robot *robot);

/// @brief Controls actuation of motors
void robot_actuate(Robot *robot);

double get_avelocity(RobotDevices *r);
double get_motor_avelocity(WbDeviceTag *m);
double get_pvelocity(RobotDevices *r);
Coord get_raw_coord(RobotDevices *r);
double get_raw_x(RobotDevices *r);
double get_raw_y(RobotDevices *r);
Coord get_coord(RobotDevices *r);
double get_x(RobotDevices *r);
double get_y(RobotDevices *r);
unsigned char *get_image(RobotDevices *r);
double get_heading(RobotDevices *r);
double get_gsvalue(WbDeviceTag *g);
const double *get_gsvalues(RobotDevices *r);

void set_avelocity(RobotDevices *r, double v);
void set_motor_velocity(WbDeviceTag m, double v);
