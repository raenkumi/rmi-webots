#include <math.h>

#include "map.h"

static RCoord origin; // x, y (raw)

void world_init(RCoord orgn) { origin = orgn; }

// MAP FUNCTIONS

void add_cell();

void create_route(Cell start, Cell goal) {}

Coord normalize(RCoord rcoords)
{
  return (Coord){
      (int)lround((rcoords.x - origin.x) / CELL_DISTANCE_M),
      (int)lround((rcoords.y - origin.y) / CELL_DISTANCE_M),
  };
}