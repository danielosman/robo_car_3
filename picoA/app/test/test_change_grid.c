// Host test for picoA/app/change_grid.c: scores adding up over the window, the
// lower threshold with a scoring neighbour, old scores forgotten, blobs. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -o build/test_change_grid picoA/app/test/test_change_grid.c && build/test_change_grid
#include <assert.h>
#include <stdio.h>
#include "../change_grid.c"

#define COLS 8
#define ROWS 8

static change_grid_t g;
static float s[COLS * ROWS];

static void frame(void) {
    change_grid_add(&g, s);
}

static void quiet(int frames) {
    for (int i = 0; i < COLS * ROWS; i++) s[i] = 0;
    for (int f = 0; f < frames; f++) frame();
}

int main(void) {
    change_grid_init(&g, COLS, ROWS, 6.0f, 4.0f);

    // One strong frame is noise; two make movement.
    s[10] = 4;
    frame();
    assert(!change_grid_moved(&g, 10));
    frame();
    assert(change_grid_moved(&g, 10));
    quiet(CHANGE_GRID_WINDOW);
    assert(!change_grid_moved(&g, 10) && change_grid_sum(&g, 10) == 0);
    printf("ok: two strong frames move a cell, old scores are forgotten\n");

    // Weak evidence: alone it isn't enough, with a neighbour it is.
    s[20] = 2.2f;
    frame(); frame();
    assert(!change_grid_moved(&g, 20));
    quiet(CHANGE_GRID_WINDOW);
    s[20] = s[21] = 2.2f;
    frame(); frame();
    assert(change_grid_moved(&g, 20) && change_grid_moved(&g, 21));
    // Not across the row's end: cell 7 and cell 8 are not neighbours.
    quiet(CHANGE_GRID_WINDOW);
    s[7] = s[8] = 2.2f;
    frame(); frame();
    assert(!change_grid_moved(&g, 7) && !change_grid_moved(&g, 8));
    quiet(CHANGE_GRID_WINDOW);
    printf("ok: weak scores need a scoring neighbour (not wrapping around rows)\n");

    // Blobs: two touching cells, one apart; diagonal cells don't touch.
    s[0] = s[1] = 4;  // blob
    s[27] = 4;        // blob
    s[36] = 4;        // diagonal to 27: its own blob
    frame(); frame();
    uint8_t label[COLS * ROWS];
    assert(change_grid_blobs(&g, label) == 3);
    assert(label[0] && label[0] == label[1] && label[27] && label[36] && label[27] != label[36]);
    assert(label[2] == 0 && label[10] == 0);
    printf("ok: blobs\n");
    printf("change_grid: all tests passed\n");
    return 0;
}
