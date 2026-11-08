#include <math.h>

#include "map.h"

static Coord origin;

void world_init(Coord orgn) { origin = orgn; }

// MAP FUNCTIONS

void add_cell();

void create_route(Cell start, Cell goal) {}

Coord normalize(const double *coords)
{
  return (Coord){
      (int)lround((coords[0] - origin.x) / CELL_DISTANCE_M),
      (int)lround((coords[1] - origin.y) / CELL_DISTANCE_M),
  };
}