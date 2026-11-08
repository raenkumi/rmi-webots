#include <math.h>

#include "map.h"

// static Map map;
static RCoord origin; // x, y (raw)

void world_init(RCoord orgn)
{
  // note for future self: if capturing garbage at init,
  // wait 3 ticks to get origin, else supervisor may have not
  // teleported bot at starting cell.
  origin = orgn;
}

// MAP FUNCTIONS

void map_init(Map *map) {}
void map_mark(Map *map, Coord c, Direction d, Path p) {}
void map_save(const char *filename) {}
void map_print() {}

// CELL FUNCTIONS
Coord cell_normalize(RCoord rcoords)
{
  return (Coord){
      (int)lround((rcoords.x - origin.x) / CELL_DISTANCE_M),
      (int)lround((rcoords.y - origin.y) / CELL_DISTANCE_M),
  };
}

bool cell_equal(Coord a, Coord b) { return false; }
bool cell_center(RCoord a, Coord b) { return false; }
bool cell_visited(Coord cell) { return false; }
void cell_record(Coord cell, const Path edges[MAP_DIR_COUNT]) {}

Coord cell_step(Coord cell, Direction dir)
{
  static const int DX[MAP_DIR_COUNT] = {0, 1, 0, -1};
  static const int DY[MAP_DIR_COUNT] = {1, 0, -1, 0};

  return (Coord){cell.x + DX[dir], cell.y + DY[dir]};
}