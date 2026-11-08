#include <math.h>

#include "map.h"

// MAP FUNCTIONS

void add_cell();

void create_route(Cell start, Cell goal) {}

Coord normalize(const double *coords)
{
  return (Coord){
      (int)lround((coords[0] - CELL_ORIGIN_M) / CELL_DISTANCE_M),
      (int)lround((coords[1] - CELL_ORIGIN_M) / CELL_DISTANCE_M),
  };
}