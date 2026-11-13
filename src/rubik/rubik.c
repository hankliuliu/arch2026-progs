#include "rubik.h"

#include "rubik_tables.h"

/* Optimal 2x2x2 solver: IDA* over (permutation, orientation) coordinates,
 * the C stepping stone for rubik.S. Ported from Homework 1
 * (hankliuliu/minirubik, stage4/ida_v4.c).
 *
 * A state is reduced to two coordinates: p, the Lehmer rank of the cubie
 * permutation (0..5039), and o, the base-3 rank of the first six orientations
 * (0..728; the seventh follows from the sum). A move is then two table
 * lookups, rk_pt and rk_ot, with no cube arithmetic and no division.
 *
 * The search is iterative deepening A*: a depth-first search that cuts a
 * branch as soon as moves made + a lower bound of the moves left exceeds the
 * current bound, repeated with bound + 1 until a solution appears. The lower
 * bound is max(PD, OPD), two pattern databases computed by gen_tables.py:
 * PD solves the permutation alone, OPD the orientation together with the
 * places of cubies 0 and 3. Each is a distance in a simplified cube, so it
 * never overestimates and the first solution found is a shortest one.
 * Two quarter-turn sequences of the same face in a row are never optimal, so
 * a node does not turn the face of the move that produced it: 6 children
 * instead of 9.
 *
 * Memory: about 98 KiB of tables (rubik_tables.h), a path of 11 bytes and a
 * recursion at most 11 calls deep; no heap.
 */

enum { CUBIES = 7, MAX_DEPTH = 11, NO_FACE = 3 };

/* lehmer_weight[i][d] = d * (6 - i)!: digit d of the Lehmer code at
 * position i, so the rank is a sum of lookups instead of products.
 */
static const uint16_t lehmer_weight[CUBIES][CUBIES] = {
    {0, 720, 1440, 2160, 2880, 3600, 4320},
    {0, 120, 240, 360, 480, 600, 720},
    {0, 24, 48, 72, 96, 120, 144},
    {0, 6, 12, 18, 24, 30, 36},
    {0, 2, 4, 6, 8, 10, 12},
    {0, 1, 2, 3, 4, 5, 6},
    {0, 0, 0, 0, 0, 0, 0},
};

static const char move_text[9][3] = {"R",  "R2", "R'", "B", "B2",
                                     "B'", "D",  "D2", "D'"};

/* Checks the 14 characters and converts them to coordinates. Returns 0, or
 * -1 for any malformed state. A NUL inside the first 14 characters fails a
 * digit test before anything past it is read.
 */
static int parse_state(const char *s, uint32_t *p_out, uint32_t *o_out)
{
    uint8_t perm[CUBIES];
    uint32_t seen = 0, twist = 0, o = 0, p = 0;

    for (int i = 0; i < CUBIES; i++) {
        uint32_t d = (uint32_t) (unsigned char) s[i] - '1';
        if (d >= CUBIES || ((seen >> d) & 1))
            return -1;
        seen |= 1u << d;
        perm[i] = (uint8_t) d;
    }
    for (int i = 0; i < CUBIES; i++) {
        uint32_t t = (uint32_t) (unsigned char) s[CUBIES + i] - '1';
        if (t > 2)
            return -1;
        twist += t;
        if (i < CUBIES - 1)
            o = o * 3 + t;
    }
    if (s[2 * CUBIES] != '\0')
        return -1;
    /* twist <= 14; bits 0, 3, 6, 9 and 12 mark the multiples of 3 */
    if (!((0x1249u >> twist) & 1))
        return -1;

    for (int i = 0; i < CUBIES; i++) {
        uint32_t smaller = 0;
        for (int j = i + 1; j < CUBIES; j++)
            smaller += perm[j] < perm[i];
        p += lehmer_weight[i][smaller];
    }
    *p_out = p;
    *o_out = o;
    return 0;
}

/* Depth-first search below the node (p, o), reached by a turn of face last.
 * rem is the number of moves allowed after the next one. Writes the moves
 * from path onward and returns 1 when a solution fits the bound.
 */
static int dfs(uint32_t p,
               uint32_t o,
               uint32_t rem,
               uint32_t last,
               uint8_t *path)
{
    for (uint32_t f = 0; f < 3; f++) {
        if (f == last)
            continue;
        uint32_t np = p;
        for (uint32_t t = 0; t < 3; t++) {
            np = rk_pt[f][np];
            if (rk_pd[np] > rem)
                continue;
            uint32_t no = rk_ot[3 * f + t][o];
            if (rk_opd[(no << 6) | rk_pos[np]] > rem)
                continue;
            *path = (uint8_t) (3 * f + t);
            /* Both bounds 0 means solved: PD is 0 only for the solved
             * permutation, OPD only for zero twists.
             */
            if (rem == 0 || dfs(np, no, rem - 1, f, path + 1))
                return 1;
        }
    }
    return 0;
}

int rubik_solve(const char *state, char *out, size_t cap)
{
    uint32_t p, o;
    if (parse_state(state, &p, &o))
        return -1;

    uint8_t path[MAX_DEPTH];
    uint32_t hp = rk_pd[p], ho = rk_opd[(o << 6) | rk_pos[p]];
    int n = 0;
    if (hp || ho) {
        uint32_t bound = hp > ho ? hp : ho;
        while (!dfs(p, o, bound - 1, NO_FACE, path))
            bound++; /* every state is solved within 11 moves */
        n = (int) bound;
    }

    /* Text length: the names, n - 1 spaces and the NUL. */
    size_t need = 1;
    for (int i = 0; i < n; i++)
        need += (size_t) (move_text[path[i]][1] ? 3 : 2);
    if (n)
        need--;
    if (cap < need) {
        if (cap)
            out[0] = '\0';
        return -2;
    }

    char *w = out;
    for (int i = 0; i < n; i++) {
        const char *name = move_text[path[i]];
        if (i)
            *w++ = ' ';
        *w++ = name[0];
        if (name[1])
            *w++ = name[1];
    }
    *w = '\0';
    return n;
}
