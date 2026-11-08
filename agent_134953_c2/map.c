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

void map_init(Map *map) {}
void map_mark(Map *map, Coord c, Direction d, Path p) {}
void map_save(const char *filename) {}
void map_print() {}

// CELL FUNCTIONS

/// @brief Maps an origin-relative Coord to its storage Cell.
/// Returns NULL when the coord falls outside the possible arena span.
static Cell *cell_at(Coord c)
{
  int gx = c.x + GRID_OX;
  int gy = c.y + GRID_OY;

  if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) return NULL;
  return &map.cells[gy * GRID_W + gx];
}

Coord cell_normalize(RCoord rcoords)
{
  return (Coord){
      (int)lround((rcoords.x - origin.x) / CELL_DISTANCE_M),
      (int)lround((rcoords.y - origin.y) / CELL_DISTANCE_M),
  };
}

bool cell_equal(Coord a, Coord b) { return a.x == b.x && a.y == b.y; }

/// @brief True when raw position is within tolerance of cell b's center.
bool cell_center(RCoord a, Coord b)
{
  double cx = origin.x + b.x * CELL_DISTANCE_M;
  double cy = origin.y + b.y * CELL_DISTANCE_M;

  return hypot(a.x - cx, a.y - cy) < CELL_GPS_TOLERANCE_M;
}

bool cell_visited(Coord cell)
{
  Cell *c = cell_at(cell);
  return c != NULL && c->visited;
}

/// @brief Records edge evidence for a cell: merges non-UNKNOWN entries,
/// marks it visited, and writes the reciprocal edge on known neighbors.
void cell_record(Coord cell, const Path edges[MAP_DIR_COUNT])
{
  Cell *c = cell_at(cell);
  if (c == NULL) return;

  c->visited = true;
  c->normalized = cell;

  // enum order N,E,S,W -> struct members
  Path *e[MAP_DIR_COUNT] = {&c->n, &c->e, &c->s, &c->w};

  for (int dir = 0; dir < MAP_DIR_COUNT; dir++)
  {
    if (edges[dir] == UNKNOWN) continue;
    *e[dir] = edges[dir];

    Cell *nb = cell_at(cell_step(cell, dir));
    if (nb == NULL) continue;

    Path *ne[MAP_DIR_COUNT] = {&nb->n, &nb->e, &nb->s, &nb->w};
    *ne[(dir + 2) % MAP_DIR_COUNT] = edges[dir]; // reciprocal edge
  }
}

Coord cell_step(Coord cell, Direction dir)
{
  static const int DX[MAP_DIR_COUNT] = {0, 1, 0, -1};
  static const int DY[MAP_DIR_COUNT] = {1, 0, -1, 0};

  return (Coord){cell.x + DX[dir], cell.y + DY[dir]};
}

int route_create(Coord start, Coord goal, Coord *out)
{
  static int parent[GRID_W * GRID_H]; /* -1 unvisited, else index   */
  static int queue[GRID_W * GRID_H];
  int head = 0, tail = 0;

  if (cell_equal(start, goal)) return 0;

  for (int i = 0; i < GRID_W * GRID_H; i++) parent[i] = -1;

  int si = (start.y + GRID_OY) * GRID_W + (start.x + GRID_OX);
  int gi = (goal.y + GRID_OY) * GRID_W + (goal.x + GRID_OX);

  parent[si] = si; /* mark visited; self-parent flags the root */
  queue[tail++] = si;

  while (head < tail)
  {
    int ci = queue[head++];
    if (ci == gi) break;

    Coord c = {ci % GRID_W - GRID_OX, ci / GRID_W - GRID_OY};
    Cell *cell = cell_at(c);
    if (cell == NULL) continue;

    const Path e[MAP_DIR_COUNT] = {cell->n, cell->e, cell->s, cell->w};

    for (int dir = 0; dir < MAP_DIR_COUNT; dir++)
    {
      if (e[dir] != LINE) continue;

      Coord nc = cell_step(c, dir);
      int ni = (nc.y + GRID_OY) * GRID_W + (nc.x + GRID_OX);
      if (ni < 0 || ni >= GRID_W * GRID_H || parent[ni] != -1) continue;

      parent[ni] = ci;
      queue[tail++] = ni;
    }
  }

  if (parent[gi] == -1) return 0; /* unreachable */

  // walk back goal -> start counting hops, then fill out forward
  int len = 0;
  for (int i = gi; i != si; i = parent[i]) len++;

  int i = gi;
  for (int k = len - 1; k >= 0; k--)
  {
    out[k] = (Coord){i % GRID_W - GRID_OX, i / GRID_W - GRID_OY};
    i = parent[i];
  }

  return len;
}

double cell_dist(RCoord a, Coord b)
{
  double cx = origin.x + b.x * CELL_DISTANCE_M;
  double cy = origin.y + b.y * CELL_DISTANCE_M;
  return hypot(a.x - cx, a.y - cy);
}