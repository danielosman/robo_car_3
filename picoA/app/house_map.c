#include <stdio.h>
#include <string.h>
#include "house_map.h"

typedef struct {
    int16_t tx, ty;
    bool dirty;
    uint8_t columns[HOUSE_TILE_BYTES];
} tile_t;

static tile_t tiles[HOUSE_TILES];
static int n_tiles, last_found; // last_found: the tile found last (columns come in runs)
static bool full_said;
static uint32_t clears;

void house_map_clear(void) {
    n_tiles = last_found = 0;
    full_said = false;
    clears++;
}

int house_map_tile_of(int cx) {
    int shifted = cx + HOUSE_TILE_COLS / 2;
    return shifted >= 0 ? shifted / HOUSE_TILE_COLS : -((-shifted + HOUSE_TILE_COLS - 1) / HOUSE_TILE_COLS);
}

static tile_t *find(int tx, int ty) {
    if (last_found < n_tiles && tiles[last_found].tx == tx && tiles[last_found].ty == ty) return &tiles[last_found];
    for (int i = 0; i < n_tiles; i++)
        if (tiles[i].tx == tx && tiles[i].ty == ty) { last_found = i; return &tiles[i]; }
    return NULL;
}

// Where column (cx, cy) is in its tile: the byte and whether in its high 4 bits.
static int nibble_at(int cx, int cy, int tx, int ty, bool *high) {
    int ix = cx - (tx * HOUSE_TILE_COLS - HOUSE_TILE_COLS / 2), iy = cy - (ty * HOUSE_TILE_COLS - HOUSE_TILE_COLS / 2);
    int k = ix * HOUSE_TILE_COLS + iy;
    *high = k & 1;
    return k / 2;
}

column_t house_map_get(int cx, int cy) {
    int tx = house_map_tile_of(cx), ty = house_map_tile_of(cy);
    const tile_t *t = find(tx, ty);
    if (!t) return COLUMN_UNKNOWN;
    bool high;
    uint8_t b = t->columns[nibble_at(cx, cy, tx, ty, &high)];
    return (column_t)(high ? b >> 4 : b & 15);
}

bool house_map_put(int cx, int cy, column_t c) {
    if (c == COLUMN_UNKNOWN) return true;
    int tx = house_map_tile_of(cx), ty = house_map_tile_of(cy);
    tile_t *t = find(tx, ty);
    if (!t) {
        if (n_tiles == HOUSE_TILES) {
            if (!full_said) printf("Map: the whole-house map is full (%d tiles of 2 x 2 m): not kept beyond\n", HOUSE_TILES);
            full_said = true;
            return false;
        }
        t = &tiles[n_tiles];
        last_found = n_tiles++;
        *t = (tile_t){.tx = (int16_t)tx, .ty = (int16_t)ty}; // all COLUMN_UNKNOWN (0)
    }
    bool high;
    uint8_t *b = &t->columns[nibble_at(cx, cy, tx, ty, &high)];
    uint8_t v = high ? (uint8_t)((*b & 0x0f) | c << 4) : (uint8_t)((*b & 0xf0) | c);
    if (v != *b) { *b = v; t->dirty = true; }
    return true;
}

int house_map_tiles(void) { return n_tiles; }

void house_map_tile(int i, int16_t *tx, int16_t *ty, const uint8_t **columns) {
    *tx = tiles[i].tx;
    *ty = tiles[i].ty;
    *columns = tiles[i].columns;
}

int house_map_next_dirty(int i) {
    for (; i < n_tiles; i++)
        if (tiles[i].dirty) return i;
    return -1;
}

void house_map_sent(int i) { tiles[i].dirty = false; }

void house_map_all_dirty(void) {
    for (int i = 0; i < n_tiles; i++) tiles[i].dirty = true;
}

uint32_t house_map_clears(void) { return clears; }
