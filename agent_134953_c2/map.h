#ifndef MAP_H
#define MAP_H

#include <stdbool.h>

// WORLD DEFINITIONS
// ==========================================

#define TIME_STEP_MS 16
#define DURATION_MS 350000

// MAP DEFINITIONS
// ==========================================

#define MAX_CELLS (14 * 7)
#define CELL_ORIGIN_M 0.075
#define CELL_DISTANCE_M 0.15

typedef enum
{
  RED = 0,
  GREEN,
  YELLOW,
  MAGENTA,
  CYAN,
  ORANGE,
  WHITE
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
} Coord;

typedef struct
{
  bool visited;
  Coord normalized;
  Coord raw;
  Path n, s, e, w;
} Cell;

typedef struct
{
  Cell cells[255];
} Map;

// MAP FUNCTIONS

/// @brief Adds new cell to the map
void add_cell();

/// @brief Creates a route between start and goal cell using Dijkstra's
/// @param start Starting cell, can be current one
/// @param goal Goal cell
void create_route(Cell start, Cell goal);

Coord normalize(const double *coords);

#endif // MAP_H
