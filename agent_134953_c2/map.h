#ifndef MAP_H
#define MAP_H

#include <stdbool.h>

// WORLD DEFINITIONS
// ==========================================

#define DURATION_MS 350000

// MAP DEFINITIONS
// ==========================================

#define MAX_CELLS (14 * 7)
#define CELL_DISTANCE_M 0.15

typedef enum
{
  GOAL_RED = 0,
  GOAL_GREEN = 1,
  GOAL_YELLOW = 2,
  GOAL_MAGENTA = 3,
  GOAL_CYAN = 4,
  GOAL_ORANGE = 5,
  GOAL_WHITE = 6
} Goals;

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
  Cell cells[255];
} Map;

void world_init(RCoord orgn);

// MAP FUNCTIONS

/// @brief Adds new cell to the map
void add_cell(void);

/// @brief Creates a route between start and goal cell using Dijkstra's.
/// @param start Starting cell, can be current one.
/// @param goal Goal cell.
void create_route(Cell start, Cell goal);

Coord normalize(RCoord rcoords);

#endif // MAP_H
