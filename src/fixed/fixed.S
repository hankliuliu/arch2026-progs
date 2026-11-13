# op1, op2, op3 and q_mul: hand-written RV32I.
#
# uint32_t op1_asm(uint32_t a, uint32_t b, int *n)   a0, a1, a2 -> a0
# uint32_t op2_asm(uint32_t a, uint32_t b)           a0, a1 -> a0
# uint32_t op3_asm(uint32_t a, uint32_t b)           a0, a1 -> a0
# int32_t  q_mul_asm(int32_t a, int32_t b)           a0, a1 -> a0
#
# The semantics are those of src/fixed/fixed.h. All four are leaf functions
# that use only argument and temporary registers, so none needs a stack frame.
#
# RV32I and Zicsr only: no mul, div, rem, no floating point, no compressed
# instructions. q_mul forms a 64-bit product in a register pair. Tests:
# tests/tfixed.c.

    .equ    Q, 20

    .text

# ---------------------------------------------------------------------------
# op1_asm: a + b by carry ripple, with the pass count stored to *n.
#
#   a0  a, the partial sum
#   a1  b, the carry word
#   a2  n
#   t0  a & b
#   t1  passes counted so far, in steps of four
#
# Each pass is c = (a & b) << 1, a ^= b, b = c, until b is zero. The loop is
# unrolled four times: a pass inside the group ends with a branch to an exit
# that adds the passes of the partial group, so the counter moves once per
# four passes. Every pass costs four instructions, the last of a group five.
# ---------------------------------------------------------------------------
    .macro  OP1PASS exit
    and     t0, a0, a1
    xor     a0, a0, a1
    slli    a1, t0, 1
    beqz    a1, \exit
    .endm

    .globl  op1_asm
    .type   op1_asm, @function
    .align  2
op1_asm:
    li      t1, 0
    beqz    a1, .Lop1_store         # op1(x, 0) = x in 0 passes
.Lop1_loop:
    OP1PASS .Lop1_exit1
    OP1PASS .Lop1_exit2
    OP1PASS .Lop1_exit3
    and     t0, a0, a1
    xor     a0, a0, a1
    slli    a1, t0, 1
    addi    t1, t1, 4
    bnez    a1, .Lop1_loop
.Lop1_store:
    sw      t1, 0(a2)
    ret
.Lop1_exit3:
    addi    t1, t1, 1
.Lop1_exit2:
    addi    t1, t1, 1
.Lop1_exit1:
    addi    t1, t1, 1
    sw      t1, 0(a2)
    ret
    .size   op1_asm, .-op1_asm

# ---------------------------------------------------------------------------
# op2_asm: a - b modulo 2^32, the plain subtraction.
# ---------------------------------------------------------------------------
    .globl  op2_asm
    .type   op2_asm, @function
    .align  2
op2_asm:
    sub     a0, a0, a1
    ret
    .size   op2_asm, .-op2_asm

# ---------------------------------------------------------------------------
# op3_asm: floor((a + b) / 2) without overflow, as (a & b) + ((a ^ b) >> 1).
#
#   t0  a ^ b
# ---------------------------------------------------------------------------
    .globl  op3_asm
    .type   op3_asm, @function
    .align  2
op3_asm:
    xor     t0, a0, a1
    and     a0, a0, a1
    srli    t0, t0, 1
    add     a0, a0, t0
    ret
    .size   op3_asm, .-op3_asm

# ---------------------------------------------------------------------------
# q_mul_asm: (a * b + 2^19) >> 20 on the 64-bit product, low 32 bits.
#
#   a0, a1  operands; then the multiplicand x and the multiplier m (the
#           larger and the smaller magnitude); then the product, lo and hi
#   a2, a3  xl, xh: the 16-bit halves of x
#   a4, a5  partial products of the current 16-bit chunk of m with xl, xh
#   a6      mh, the upper chunk of m
#   a7      sign of the product, bit 31 of a ^ b
#   t0      ml, the lower chunk of m, consumed by the first loop; scratch
#   t1      carry
#   t3      bit under test
#   t4, t5  xl and xh shifted along in the first loop
#
# The product of the magnitudes is built from 16 x 16 partial products, which
# fit 32-bit registers: one loop over the bits of a 16-bit chunk of m forms
# xl * chunk and xh * chunk together, so a bit test is shared by two
# products. The loop runs least significant bit first and stops when the
# chunk has no bits left, and m is the smaller magnitude, so the loop length
# is the bit length of the smaller operand; a multiplier below 2^16, common
# in Q11.20 where 1.0 is 2^20, skips the second chunk altogether.
#
#   x * m = xl*ml + (xh*ml + xl*mh) << 16 + xh*mh << 32
#
# The sign is applied by a 64-bit negation, then 2^19 is added and bits
# 20..51 are extracted: the low 32 bits of the arithmetic shift.
# ---------------------------------------------------------------------------
    .globl  q_mul_asm
    .type   q_mul_asm, @function
    .align  2
q_mul_asm:
    xor     a7, a0, a1              # sign of the product
    srai    t0, a0, 31
    xor     a0, a0, t0
    sub     a0, a0, t0              # |a|, 0x80000000 stays as unsigned
    srai    t0, a1, 31
    xor     a1, a1, t0
    sub     a1, a1, t0              # |b|
    bgeu    a0, a1, 1f              # x = larger, m = smaller magnitude
    mv      t0, a0
    mv      a0, a1
    mv      a1, t0
1:
    srli    a3, a0, 16              # xh
    slli    a2, a0, 16
    srli    a2, a2, 16              # xl
    srli    a6, a1, 16              # mh
    slli    t0, a1, 16
    srli    t0, t0, 16              # ml
    li      a4, 0
    li      a5, 0
    beqz    t0, .Lq_low_done
    mv      t4, a2
    mv      t5, a3
.Lq_low_loop:                       # a4 += xl << i, a5 += xh << i for bit i
    andi    t3, t0, 1
    beqz    t3, 2f
    add     a4, a4, t4
    add     a5, a5, t5
2:  slli    t4, t4, 1
    slli    t5, t5, 1
    srli    t0, t0, 1
    bnez    t0, .Lq_low_loop
.Lq_low_done:
    slli    t0, a5, 16              # lo = xl*ml + (xh*ml << 16)
    add     a0, a4, t0
    sltu    t1, a0, t0
    srli    a1, a5, 16              # hi = (xh*ml >> 16) + carry
    add     a1, a1, t1
    beqz    a6, .Lq_sign            # m < 2^16: the product is complete

    li      a4, 0
    li      a5, 0
.Lq_high_loop:                      # a4 = xl * mh, a5 = xh * mh
    andi    t3, a6, 1
    beqz    t3, 3f
    add     a4, a4, a2
    add     a5, a5, a3
3:  slli    a2, a2, 1
    slli    a3, a3, 1
    srli    a6, a6, 1
    bnez    a6, .Lq_high_loop
    slli    t0, a4, 16              # (hi:lo) += xl*mh << 16
    add     a0, a0, t0
    sltu    t1, a0, t0
    add     a1, a1, t1
    srli    t0, a4, 16
    add     a1, a1, t0
    add     a1, a1, a5              # hi += xh*mh

.Lq_sign:
    bgez    a7, .Lq_round
    seqz    t0, a0                  # -(hi:lo) = (~hi + (lo == 0)) : -lo
    not     a1, a1
    add     a1, a1, t0
    neg     a0, a0
.Lq_round:
    lui     t0, 1 << (Q - 1 - 12)   # 2^19, half a unit
    add     a0, a0, t0
    sltu    t1, a0, t0
    add     a1, a1, t1
    srli    a0, a0, Q               # bits 20..31 of lo
    slli    a1, a1, 32 - Q          # bits 32..51 from hi
    or      a0, a0, a1
    ret
    .size   q_mul_asm, .-q_mul_asm
