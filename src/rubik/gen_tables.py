#!/usr/bin/env python3
"""Generate the search tables of the rubik solver from the cube model.

Writes two files next to this script (or into the directory given as the
first argument):

  rubik_tables.h    static const arrays for the C version (rubik.c)
  rubik_tables.inc  the same data for the assembly version (rubik.S), in the
                    layout the RV32I code reads directly

The cube model is the one of sysprog21/minirubik solver.c, also written out
in tests/trubik.c: seven movable cubies (the eighth corner never moves under
R, B and D), a permutation p and orientations o, and three faces whose quarter
turn moves the cubie and twist of source[f][i] to position i.

Coordinates:
  p  Lehmer rank of the permutation, 0..5039 (7!)
  o  base-3 rank of o[0..5], 0..728 (3^6); o[6] follows from the sum

Tables:
  PT[f][p]       permutation after a quarter turn of face f
  OT[m][o]       orientation after move m = 3f + t - 1 (t quarter turns)
  PD[p]          moves needed to solve the permutation alone (BFS)
  POS[p]         positions of cubies 0 and 3, coded a * 6 + (b > a ? b - 1 : b)
  OPD[o][pos]    moves needed to solve the orientation together with the
                 positions of cubies 0 and 3 (BFS over 729 x 42 states),
                 stored with a row stride of 64, padding 0xFF

Both PD and OPD are distances in a projection of the cube, so neither
overestimates the true distance and max(PD, OPD) is an admissible heuristic
for IDA*. The assembly tables store p and o as byte offsets (2p, 2o) and
interleave PD and POS (PDPOS[2p] = PD[p], PDPOS[2p + 1] = POS[p]).

This is the generator of the Homework 1 tables (hankliuliu/minirubik,
stage2/gen_tables.c and stage4/gen_tables4.c) rewritten as one script.
"""

import os
import sys
from collections import deque
from itertools import permutations

CUBIES = 7
NPERM = 5040
NORI = 729
NPOS = 42
OSTRIDE = 64

SOURCE = (
    (1, 4, 2, 0, 3, 5, 6),
    (0, 1, 2, 4, 5, 6, 3),
    (0, 2, 5, 3, 1, 4, 6),
)
TWIST = (
    (1, 2, 0, 2, 1, 0, 0),
    (0, 0, 0, 1, 2, 1, 2),
    (0, 0, 0, 0, 0, 0, 0),
)
FACT = (720, 120, 24, 6, 2, 1, 1)


def rank_p(p):
    r = 0
    for i in range(CUBIES):
        r += sum(1 for j in range(i + 1, CUBIES) if p[j] < p[i]) * FACT[i]
    return r


def rank_o(o):
    r = 0
    for i in range(6):
        r = r * 3 + o[i]
    return r


def unrank_o(r):
    o = [0] * CUBIES
    for i in range(5, -1, -1):
        o[i] = r % 3
        r //= 3
    o[6] = (3 - sum(o[:6]) % 3) % 3
    return o


def pos_code(p):
    a = p.index(0)
    b = p.index(3)
    return a * 6 + (b - 1 if b > a else b)


def build():
    perms = [None] * NPERM
    for p in permutations(range(CUBIES)):
        perms[rank_p(p)] = p
    assert all(perms)

    pt = [[0] * NPERM for _ in range(3)]
    pos = [0] * NPERM
    for r, p in enumerate(perms):
        pos[r] = pos_code(p)
        for f in range(3):
            pt[f][r] = rank_p([p[SOURCE[f][i]] for i in range(CUBIES)])

    ot1 = [[0] * NORI for _ in range(3)]
    for r in range(NORI):
        o = unrank_o(r)
        for f in range(3):
            ot1[f][r] = rank_o(
                [(o[SOURCE[f][i]] + TWIST[f][i]) % 3 for i in range(CUBIES)])
    ot = []
    for f in range(3):
        for t in range(1, 4):
            row = []
            for r in range(NORI):
                x = r
                for _ in range(t):
                    x = ot1[f][x]
                row.append(x)
            ot.append(row)

    pd = [0xFF] * NPERM
    pd[0] = 0
    q = deque([0])
    while q:
        x = q.popleft()
        for f in range(3):
            y = pt[f][x]
            while y != x:
                if pd[y] == 0xFF:
                    pd[y] = pd[x] + 1
                    q.append(y)
                y = pt[f][y]
    assert 0xFF not in pd

    opd = [0xFF] * (NORI * OSTRIDE)
    opd[pos[0]] = 0
    q = deque([(0, 0)])
    reached = 1
    while q:
        p, o = q.popleft()
        d = opd[o * OSTRIDE + pos[p]]
        for f in range(3):
            np_, no = p, o
            for _ in range(3):
                np_ = pt[f][np_]
                no = ot1[f][no]
                i = no * OSTRIDE + pos[np_]
                if opd[i] == 0xFF:
                    opd[i] = d + 1
                    q.append((np_, no))
                    reached += 1
    assert reached == NORI * NPOS
    return pt, ot, pd, pos, opd


def rows(values, per_line, fmt):
    for i in range(0, len(values), per_line):
        yield fmt(values[i:i + per_line])


def emit_c(path, pt, ot, pd, pos, opd):
    def arr(lines, decl, values):
        lines.append(decl + " = {")
        lines.extend(
            rows(values, 16, lambda v: "    " + ", ".join(map(str, v)) + ","))
        lines.append("};")
        lines.append("")

    def arr2(lines, decl, table):
        lines.append(decl + " = {")
        for row in table:
            lines.append("    {")
            lines.extend(
                rows(row, 16,
                     lambda v: "        " + ", ".join(map(str, v)) + ","))
            lines.append("    },")
        lines.append("};")
        lines.append("")

    lines = [
        "/* Generated by gen_tables.py from the cube model; do not edit. */",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        "/* clang-format off */",
    ]
    arr2(lines, "static const uint16_t rk_pt[3][%d]" % NPERM, pt)
    arr2(lines, "static const uint16_t rk_ot[9][%d]" % NORI, ot)
    arr(lines, "static const uint8_t rk_pd[%d]" % NPERM, pd)
    arr(lines, "static const uint8_t rk_pos[%d]" % NPERM, pos)
    arr(lines, "static const uint8_t rk_opd[%d * %d]" % (NORI, OSTRIDE), opd)
    lines.append("/* clang-format on */")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def emit_s(path, pt, ot, pd, pos, opd):
    lines = [
        "# Generated by gen_tables.py from the cube model; do not edit.",
        "#",
        "#   rk_pt[f][p]     2 * PT[f][p], halfwords, 3 rows of 5040",
        "#   rk_ot[m][o]     2 * OT[m][o], halfwords, 9 rows of 729",
        "#   rk_pdpos[2p]    PD[p], rk_pdpos[2p + 1] = POS[p]",
        "#   rk_opd[o * 64 + pos]  OPD, padding 0xFF",
        "",
        "    .section .rodata",
        "    .balign 4",
    ]

    def block(label, values, directive):
        lines.append(label + ":")
        lines.extend(
            rows(values, 16,
                 lambda v: "    %s %s" % (directive, ",".join(map(str, v)))))

    block("rk_pt", [2 * v for row in pt for v in row], ".half")
    block("rk_ot", [2 * v for row in ot for v in row], ".half")
    pdpos = []
    for p in range(NPERM):
        pdpos += [pd[p], pos[p]]
    block("rk_pdpos", pdpos, ".byte")
    block("rk_opd", opd, ".byte")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(
        os.path.abspath(__file__))
    tables = build()
    emit_c(os.path.join(out, "rubik_tables.h"), *tables)
    emit_s(os.path.join(out, "rubik_tables.inc"), *tables)


if __name__ == "__main__":
    main()
