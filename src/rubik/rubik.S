# rubik_solve_asm: optimal 2x2x2 cube solver, hand-written RV32I.
#
# int rubik_solve_asm(const char *state, char *out, size_t cap)
#                                   a0 = state, a1 = out, a2 = cap; result in a0
#
# The same search as src/rubik/rubik.c (IDA* with the PD and OPD pattern
# databases, see the comments there), on the same tables in a layout made for
# the load instructions (rubik_tables.inc, written by gen_tables.py):
#
#   rk_pt     2 * next permutation, so p travels as the byte offset 2p
#   rk_ot     2 * next orientation, one row per move, o travels as 2o
#   rk_pdpos  PD and POS of a permutation side by side at offset 2p
#   rk_opd    OPD at (2o << 5) | POS = o * 64 + POS
#
# The search core is the Homework 1 assembly (hankliuliu/minirubik,
# stage4/solver_a4.S), now a callable function: the callee-saved registers
# and the search frames live on the stack, there is no .data or .bss, and the
# harness of Homework 1 (environment calls, replay, LED output) is replaced by
# the validation and the text output of the contract.
#
# Search core, compared with the C version:
#   * No recursion and no generic child loop. There is one copy of the code
#     per face of the move that led to a node: after a move of R the node
#     tries B then D, after B it tries R then D, after D it tries R then B,
#     and every face keeps its table rows in registers. The tests of one face
#     against its predecessor disappear.
#   * A child that passes PD is tested against OPD with "jal ra, opd_<m>", so
#     ra is the address where the parent resumes. A cut returns there; a pass
#     enters the child, which stores ra in the parent's frame, and leaving the
#     child is a "jr" to the stored address. The frame below the root is a
#     sentinel whose resume address starts the next bound.
#   * A child's orientation is looked up only after its permutation passed PD.
#
# Registers during the search:
#   s0 rk_pdpos   s1 rk_opd    s2 rem = bound - 1 - depth   s3 next frame
#   s4 2p         s5 2o        s6 s7 s8 rk_pt rows of R, B, D
#   s9 a2 a7      rk_ot rows of R2, B2, D2 (the three turns at -/0/+ NORI2)
#   s10 bound     s11 frame of the root    a3 move of the child
#   a4 a5         2p and 2o of the root
#   t0-t5         scratch
#
# Stack frame (ST_SIZE bytes, 16-byte aligned): ra and s0-s11, the output
# pointer and capacity, the path (MAX_DEPTH bytes) and MAX_DEPTH + 2 search
# frames of FRAME bytes: resume address, 2p, 2o, move.
#
# RV32I and Zicsr only: no mul, div, rem, no floating point, no compressed
# instructions, no heap, no recursion. Tests: tests/trubik.c.

    .equ    CUBIES, 7
    .equ    MAX_DEPTH, 11
    .equ    NPERM2, 2 * 5040        # bytes per rk_pt row
    .equ    NORI2, 2 * 729          # bytes per rk_ot row

    .equ    FRAME, 12
    .equ    F_RA, 0
    .equ    F_P, 4
    .equ    F_O, 6
    .equ    F_MOVE, 8

    .equ    ST_RA, 0
    .equ    ST_S0, 4                # s0-s11 at 4..48
    .equ    ST_OUT, 52
    .equ    ST_CAP, 56
    .equ    ST_PATH, 60
    .equ    ST_FRAMES, 72
    .equ    ST_SIZE, 240            # 72 + 13 * FRAME = 228, rounded up

    .text

# Child t1 = 2p' of move m: cut if PD > rem, else test OPD. A cut falls
# through to the next child.
    .macro  TRY m
    add     t0, s0, t1
    lbu     t2, 0(t0)
    bltu    s2, t2, .Lcut\@
    jal     ra, .Lopd_\m
.Lcut\@:
    .endm

# The three quarter turns of the face whose rk_pt row is in register pt,
# chained through t1.
    .macro  FACE pt, m1, m2, m3
    add     t0, \pt, s4
    lhu     t1, 0(t0)
    TRY     \m1
    add     t0, \pt, t1
    lhu     t1, 0(t0)
    TRY     \m2
    add     t0, \pt, t1
    lhu     t1, 0(t0)
    TRY     \m3
    .endm

# o' = rk_ot[m][o] in t3; return to ra if OPD > rem, else descend into the
# child through the entry of face f. t0 points at the child's rk_pdpos entry.
    .macro  OPD m, ot, off, f
.Lopd_\m:
    add     t3, \ot, s5
    lhu     t3, \off(t3)
    lbu     t2, 1(t0)               # POS of the child
    slli    t4, t3, 5
    or      t4, t4, t2
    add     t4, s1, t4
    lbu     t4, 0(t4)
    bltu    s2, t4, .Lresume
    li      a3, \m
    j       .Ldown_\f
    .endm

# Saves the parent with its resume address and enters the child (t1, t3).
# With no moves left the child passed both bounds of 0: it is solved.
    .macro  DOWN
    beqz    s2, .Lfound
    sw      ra, F_RA(s3)
    sh      s4, F_P(s3)
    sh      s5, F_O(s3)
    sb      a3, F_MOVE(s3)
    addi    s3, s3, FRAME
    addi    s2, s2, -1
    mv      s4, t1
    mv      s5, t3
    .endm

    .globl  rubik_solve_asm
    .type   rubik_solve_asm, @function
    .align  2
rubik_solve_asm:
    addi    sp, sp, -ST_SIZE
    sw      ra, ST_RA(sp)
    sw      s0, ST_S0 + 0(sp)
    sw      s1, ST_S0 + 4(sp)
    sw      s2, ST_S0 + 8(sp)
    sw      s3, ST_S0 + 12(sp)
    sw      s4, ST_S0 + 16(sp)
    sw      s5, ST_S0 + 20(sp)
    sw      s6, ST_S0 + 24(sp)
    sw      s7, ST_S0 + 28(sp)
    sw      s8, ST_S0 + 32(sp)
    sw      s9, ST_S0 + 36(sp)
    sw      s10, ST_S0 + 40(sp)
    sw      s11, ST_S0 + 44(sp)
    sw      a1, ST_OUT(sp)
    sw      a2, ST_CAP(sp)

    # Permutation digits: '1'..'7', each once. The Lehmer digit at position
    # i is the number of smaller cubies still to come, that is the cubie
    # minus the smaller ones already placed (popcount of the seen mask below
    # it); rk_rankw turns it into 2 * digit * (6 - i)!.
    la      a3, rk_rankw
    la      a4, rk_pop
    li      a5, CUBIES
    li      a7, 1
    li      t0, 0                   # bit c set once cubie c is placed
    li      s2, 0                   # 2p
    mv      t1, a0
    addi    t6, a0, CUBIES
.Lparse_perm:
    lbu     t3, 0(t1)
    addi    t3, t3, -'1'
    bgeu    t3, a5, .Linvalid       # also catches an early NUL
    sll     t4, a7, t3
    and     t5, t0, t4
    bnez    t5, .Linvalid           # repeated cubie
    addi    t5, t4, -1
    and     t5, t5, t0
    add     t5, a4, t5
    lbu     t5, 0(t5)               # smaller cubies already placed
    or      t0, t0, t4
    sub     t5, t3, t5              # Lehmer digit
    slli    t5, t5, 1
    add     t5, a3, t5
    lhu     t5, 0(t5)
    add     s2, s2, t5
    addi    a3, a3, 2 * CUBIES      # next row of rk_rankw
    addi    t1, t1, 1
    bne     t1, t6, .Lparse_perm

    # Orientation digits: '1'..'3'. The first six form o in base 3 by
    # Horner's rule (3x = 2x + x), all seven must sum to a multiple of 3, and
    # the string must end right after them.
    li      a6, 3
    li      s3, 0                   # o
    li      s4, 0                   # twist sum
    addi    t6, a0, 2 * CUBIES - 1
.Lparse_ori:
    lbu     t3, 0(t1)
    addi    t3, t3, -'1'
    bgeu    t3, a6, .Linvalid
    add     s4, s4, t3
    slli    t4, s3, 1
    add     s3, s3, t4
    add     s3, s3, t3
    addi    t1, t1, 1
    bne     t1, t6, .Lparse_ori
    lbu     t3, 0(t1)               # seventh twist: checked, not ranked
    addi    t3, t3, -'1'
    bgeu    t3, a6, .Linvalid
    add     s4, s4, t3
    lbu     t3, 1(t1)
    bnez    t3, .Linvalid           # longer than 14 characters
    li      t3, 0x1249              # bits 0, 3, 6, 9, 12: multiples of 3
    srl     t3, t3, s4
    andi    t3, t3, 1
    beqz    t3, .Linvalid
    slli    s3, s3, 1               # 2o

    # Search setup: table rows in registers, the sentinel frame, the bound.
    la      s0, rk_pdpos
    la      s1, rk_opd
    la      s6, rk_pt
    li      t0, NPERM2
    add     s7, s6, t0
    add     s8, s7, t0
    la      s9, rk_ot + 1 * NORI2
    la      a2, rk_ot + 4 * NORI2
    la      a7, rk_ot + 7 * NORI2
    addi    s11, sp, ST_FRAMES + FRAME
    la      t0, .Lnext_bound
    sw      t0, F_RA - FRAME(s11)
    mv      a4, s2
    mv      a5, s3
    add     t0, s0, a4
    lbu     t1, 0(t0)               # PD of the root
    lbu     t2, 1(t0)
    slli    t3, a5, 5
    or      t3, t3, t2
    add     t3, s1, t3
    lbu     t3, 0(t3)               # OPD of the root
    or      t4, t1, t3
    li      a0, 0
    beqz    t4, .Loutput            # solved: no moves
    mv      s10, t1                 # bound = max(PD, OPD)
    bgeu    t1, t3, .Lbound_loop
    mv      s10, t3
.Lbound_loop:
    addi    s2, s10, -1
    mv      s3, s11
    mv      s4, a4
    mv      s5, a5
    FACE    s6, 0, 1, 2
    j       .Lafter_0

.Ldown_0:                           # entered by a move of R
    DOWN
.Lafter_0:
    FACE    s7, 3, 4, 5
.Lafter_0_d:
    FACE    s8, 6, 7, 8
.Lup:
    mv      t1, s4                  # the child just left: the parent's last turn
    addi    s3, s3, -FRAME
    lw      t5, F_RA(s3)
    lhu     s4, F_P(s3)
    lhu     s5, F_O(s3)
    addi    s2, s2, 1
    jr      t5

.Ldown_1:                           # entered by a move of B
    DOWN
    FACE    s6, 0, 1, 2
    j       .Lafter_0_d

.Ldown_2:                           # entered by a move of D
    DOWN
    FACE    s6, 0, 1, 2
    FACE    s7, 3, 4, 5
    j       .Lup

    OPD     0, s9, -NORI2, 0
    OPD     1, s9, 0, 0
    OPD     2, s9, NORI2, 0
    OPD     3, a2, -NORI2, 1
    OPD     4, a2, 0, 1
    OPD     5, a2, NORI2, 1
    OPD     6, a7, -NORI2, 2
    OPD     7, a7, 0, 2
    OPD     8, a7, NORI2, 2
.Lresume:
    jr      ra

.Lnext_bound:                       # the root is exhausted
    addi    s10, s10, 1
    li      t0, MAX_DEPTH
    bgeu    t0, s10, .Lbound_loop
    j       .Linvalid               # unreachable: the diameter is 11

.Lfound:                            # copy the moves of the frames, then a3
    addi    t0, sp, ST_PATH
    mv      t1, s11
1:  beq     t1, s3, 2f
    lbu     t2, F_MOVE(t1)
    sb      t2, 0(t0)
    addi    t0, t0, 1
    addi    t1, t1, FRAME
    j       1b
2:  sb      a3, 0(t0)
    mv      a0, s10

    # Text output. a0 = n moves at sp + ST_PATH. The text takes one byte per
    # move, one more for a two-character name, n - 1 spaces and the NUL:
    # 2n + (two-character names), or 1 for n = 0.
.Loutput:
    lw      a1, ST_OUT(sp)
    lw      a2, ST_CAP(sp)
    la      t2, move_name           # two bytes per move, NUL-padded
    addi    t0, sp, ST_PATH
    add     t1, t0, a0              # end of the path
    li      t3, 1
    beqz    a0, 2f
    slli    t3, a0, 1
    mv      t4, t0
1:  lbu     t5, 0(t4)
    slli    t5, t5, 1
    add     t5, t2, t5
    lbu     t5, 1(t5)
    snez    t5, t5
    add     t3, t3, t5
    addi    t4, t4, 1
    bne     t4, t1, 1b
2:  bgeu    a2, t3, .Lwrite
    li      a0, -2                  # too small: empty string if cap >= 1
    beqz    a2, .Lreturn
    sb      zero, 0(a1)
    j       .Lreturn
.Lwrite:
    mv      t4, a1
    beq     t0, t1, 5f
    li      t6, ' '
3:  lbu     t5, 0(t0)
    slli    t5, t5, 1
    add     t5, t2, t5
    lbu     t3, 0(t5)
    sb      t3, 0(t4)
    lbu     t3, 1(t5)
    beqz    t3, 4f
    sb      t3, 1(t4)
    addi    t4, t4, 1
4:  sb      t6, 1(t4)
    addi    t4, t4, 2
    addi    t0, t0, 1
    bne     t0, t1, 3b
    addi    t4, t4, -1              # the last space becomes the NUL
5:  sb      zero, 0(t4)

.Lreturn:
    lw      ra, ST_RA(sp)
    lw      s0, ST_S0 + 0(sp)
    lw      s1, ST_S0 + 4(sp)
    lw      s2, ST_S0 + 8(sp)
    lw      s3, ST_S0 + 12(sp)
    lw      s4, ST_S0 + 16(sp)
    lw      s5, ST_S0 + 20(sp)
    lw      s6, ST_S0 + 24(sp)
    lw      s7, ST_S0 + 28(sp)
    lw      s8, ST_S0 + 32(sp)
    lw      s9, ST_S0 + 36(sp)
    lw      s10, ST_S0 + 40(sp)
    lw      s11, ST_S0 + 44(sp)
    addi    sp, sp, ST_SIZE
    ret

.Linvalid:
    li      a0, -1
    j       .Lreturn
    .size   rubik_solve_asm, .-rubik_solve_asm

    .section .rodata
    .balign 4
# rk_rankw[i][d] = 2 * d * (6 - i)!: Lehmer digit d at position i.
rk_rankw:
    .half   0, 1440, 2880, 4320, 5760, 7200, 8640
    .half   0, 240, 480, 720, 960, 1200, 1440
    .half   0, 48, 96, 144, 192, 240, 288
    .half   0, 12, 24, 36, 48, 60, 72
    .half   0, 4, 8, 12, 16, 20, 24
    .half   0, 2, 4, 6, 8, 10, 12
    .half   0, 0, 0, 0, 0, 0, 0
# rk_pop[x] = number of bits set in x, for the 7-bit seen masks.
rk_pop:
    .byte   0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4
    .byte   1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5
    .byte   1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5
    .byte   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6
    .byte   1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5
    .byte   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6
    .byte   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6
    .byte   3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7
move_name:
    .ascii  "R\0R2R'B\0B2B'D\0D2D'"

    .include "src/rubik/rubik_tables.inc"
