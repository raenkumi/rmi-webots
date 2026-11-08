#ifndef MAP_H
#define MAP_H

#include <stdbool.h>
#include <string.h>

// WORLD DEFINITIONS
// ==========================================
#pragma region WORLD DEFINITIONS

#define WL_DURATION_MS 350000

#pragma endregion

// MAP DEFINITIONS
// ==========================================

#pragma region MAP DEFINITIONS

#define MAP_MAX_CELLS (14 * 7)
#define MAP_DIR_COUNT 4
#define CELL_DISTANCE_M 0.15
#define CELL_GPS_TOLERANCE_M 0.01
#define CELL_MAP_WINDOW_M 0.05

// storage grid: coords are origin-relative and can go negative, so the
// grid spans +-(14-1) x and +-(7-1) y around the start cell
#define GRID_W (2 * 14 - 1) // 27
#define GRID_H (2 * 7 - 1)  // 13
#define GRID_OX (14 - 1)    // grid offset of coord (0,0)
#define GRID_OY (7 - 1)

typedef enum
{
  GOAL_NONE = 0,
  GOAL_RED = 1,
  GOAL_GREEN = 2,
  GOAL_YELLOW = 3,
  GOAL_MAGENTA = 4,
  GOAL_CYAN = 5,
  GOAL_ORANGE = 6,
  GOAL_WHITE = 7
} Goals;

typedef enum
{
  DIR_NORTH = 0,
  DIR_EAST,
  DIR_SOUTH,
  DIR_WEST
} Direction;

typedef enum
{
  UNKNOWN = 0,
  NO_LINE,
  LINE
} Path;

typedef struct
{
  double x, y;
} RCoord;

typedef struct
{
  int x, y;
} Coord;

typedef struct
{
  bool visited;
  Coord normalized;
  RCoord raw;
  Path n, s, e, w;
} Cell;

typedef struct
{
  Cell cells[GRID_W * GRID_H];
} Map;

#pragma endregion

// MAP METHODS
// ==========================================

#pragma region MAP METHODS

void world_init(RCoord orgn);
// MAP FUNCTIONS

/// @brief Adds new cell to the map
void map_init(Map *map);
void map_mark(Map *map, Coord c, Direction d, Path p);
void map_save(const char *filename);
void map_print();
// CELL FUNCTIONS

Coord cell_normalize(RCoord rcoords);
bool cell_equal(Coord a, Coord b);
bool cell_center(RCoord a, Coord b);
bool cell_visited(Coord cell);
void cell_record(Coord cell, const Path edges[MAP_DIR_COUNT]);
Coord cell_step(Coord cell, Direction dir);
double cell_dist(RCoord a, Coord b);

/// @brief Creates a route between start and goal cell using Dijkstra's.
/// @param start Starting cell, can be current one.
/// @param goal Goal cell.
int route_create(Coord start, Coord goal, Coord *out);
void route_print();

#pragma endregion

#endif // MAP_H