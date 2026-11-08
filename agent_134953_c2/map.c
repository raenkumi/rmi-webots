#include <math.h>

#include "map.h"

static Map map;
static RCoord origin; // x, y (raw)

void world_init(RCoord orgn)
{
  // note for future self: if capturing garbage at init,
  // wait 3 ticks to get origin, else supervisor may have not
  // teleported bot at starting cell.
  origin = orgn;
}

// MAP FUNCTIONS

Coord cell_normalize(RCoord rcoords)
{
  return (Coord){
      (int)lround((rcoords.x - origin.x) / CELL_DISTANCE_M),
      (int)lround((rcoords.y - origin.y) / CELL_DISTANCE_M),
  };
}