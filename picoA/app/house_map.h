#pragma once
// The whole-house map (doc/ARCHITECTURE.md, the map): each 10 cm column's class
// (column_t, 4 bits) of everything the map window (cell_map) has seen, in 2 x 2 m
// tiles, only the tiles seen. The window writes into it what scrolls out of it and
// starts from it what scrolls in. A known class overwrites what a column had; an
// unknown one keeps it (the robot didn't look there this time).
//
// Columns are numbered by world cell: column (cx, cy) covers x from cx * 10 cm to
// (cx + 1) * 10 cm in the map frame (pose.h). Tile (tx, ty) holds columns
// cx = 20 tx - 10 … 20 tx + 9, so it covers 2 tx ± 1 m: the tile of the power-up
// point is centred on it.
//
// Each tile changed since it was last handed to a snapshot is dirty
// (house_map_next_dirty()); a new server marks them all dirty again.
#include <stdbool.h>
#include <stdint.h>
#include "cell_map.h"

#define HOUSE_TILE_COLS  20                                     // columns across a tile: 2 m
#define HOUSE_TILE_BYTES (HOUSE_TILE_COLS * HOUSE_TILE_COLS / 2) // 4 bits per column
#define HOUSE_TILES      100                                    // a 20 x 20 m flat

void house_map_clear(void);
column_t house_map_get(int cx, int cy);
// A known class overwrites, COLUMN_UNKNOWN keeps the column. False if the tile
// would be new and none is left (said once on the console).
bool house_map_put(int cx, int cy, column_t c);
int house_map_tile_of(int cx); // the tile a column is in (the same for x and y)

// Tiles in use: 0 … house_map_tiles() - 1. A tile's coordinates and its columns:
// byte (ix * 20 + iy) / 2, the low 4 bits for an even ix * 20 + iy; ix, iy 0-19
// from the tile's low corner.
int house_map_tiles(void);
void house_map_tile(int i, int16_t *tx, int16_t *ty, const uint8_t **columns);
// The first dirty tile from i on, or -1; handing it out clears its dirt.
int house_map_next_dirty(int i);
void house_map_sent(int i);
void house_map_all_dirty(void);
uint32_t house_map_clears(void); // counts house_map_clear() calls: the PC starts again when it changes
