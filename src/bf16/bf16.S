# BFloat16 routines: hand-written RV32I.
#
# uint16_t bf16_from_f32_asm(uint32_t f32bits)   a0 -> a0
# uint16_t bf16_mul_asm(uint16_t a, uint16_t b)  a0, a1 -> a0
# uint16_t bf16_sqrt_asm(uint16_t a)             a0 -> a0
#
# The semantics are those of src/bf16/bf16.h and src/bf16/bf16.c; the
# comments below say where the assembly takes a different route to the same
# result. All three are leaf functions that touch only a0-a3 and t0-t6, so
# none of them needs a stack frame.
#
# RV32I and Zicsr only: no mul, div, rem, no floating point, no compressed
# instructions. Tests: tests/tbf16.c.

    .equ    BF16_POS_INF, 0x7F80
    .equ    BF16_QNAN, 0x7FC0
    .equ    BF16_EXP_BIAS, 127

    .text

# ---------------------------------------------------------------------------
# bf16_from_f32_asm: round to nearest, ties to even.
#
#   a0  input pattern, then the rounded pattern, then the result
#   a1  kept (upper) half of the input
#   a2  0x7FFF + bit 16 of the input
#   a3  0x7FFF; later the exponent mask 0x7F80
#
# The exponent test of the C version is needed only when the rounding carries
# into the kept half: without a carry the result is the kept half, which is
# also the answer for Inf and NaN. So the common path rounds first and looks
# at the exponent only after a carry.
# ---------------------------------------------------------------------------
    .globl  bf16_from_f32_asm
    .type   bf16_from_f32_asm, @function
    .align  2
bf16_from_f32_asm:
    srli    a1, a0, 16              # kept half
    andi    a2, a1, 1               # tie breaker: bit 16
    li      a3, 0x7FFF
    add     a2, a2, a3
    add     a0, a0, a2
    srli    a0, a0, 16              # rounded half
    bne     a0, a1, .Lf32_carry
    ret
.Lf32_carry:
    li      a3, BF16_POS_INF        # exponent field mask, 0xFF << 7
    and     a2, a1, a3
    bne     a2, a3, .Lf32_ret       # finite: keep the rounded value
    mv      a0, a1                  # Inf or NaN: the kept half unchanged
.Lf32_ret:
    ret
    .size   bf16_from_f32_asm, .-bf16_from_f32_asm

# ---------------------------------------------------------------------------
# bf16_mul_asm
#
#   a0, a1  operands a and b (zero-extended 16-bit patterns); result in a0
#   t0, t1  exponents of a and b; after a denormal is normalized they hold
#           1 minus the shift, so no separate adjustment is carried
#   t2, t3  significands of a and b, leading one at bit 7
#   t4      0xFF
#   t5      scratch
#   t6      sign of the product at bit 15
#   a2      product of the significands, then the fraction
#   a3      fraction bits of b, aligned to bit 31 for the multiply
#
# The 8 x 8 multiply is shift-and-add, most significant bit first, with no
# call to __mulsi3. Bit 7 of b is the hidden one, so the product starts as a;
# each of the seven fraction bits of b doubles the partial product and adds a
# when the bit, moved up to bit 31, makes the register negative. A b with an
# all-zero fraction (a power of two) skips the loop: the product is a << 7.
# ---------------------------------------------------------------------------
    .macro  MULSTEP
    slli    a2, a2, 1
    bgez    a3, 1f
    add     a2, a2, t2
1:
    slli    a3, a3, 1
    .endm

    .globl  bf16_mul_asm
    .type   bf16_mul_asm, @function
    .align  2
bf16_mul_asm:
    xor     t6, a0, a1
    srli    t6, t6, 15
    slli    t6, t6, 15              # product sign
    srli    t0, a0, 7
    andi    t0, t0, 0xFF            # exponent of a
    srli    t1, a1, 7
    andi    t1, t1, 0xFF            # exponent of b
    andi    t2, a0, 0x7F
    andi    t3, a1, 0x7F
    li      t4, 0xFF
    beq     t0, t4, .Lmul_a_special # a is Inf or NaN
    beq     t1, t4, .Lmul_b_special # b is Inf or NaN
    beqz    t0, .Lmul_a_low         # a is zero or denormal
    ori     t2, t2, 0x80            # hidden one
.Lmul_a_ok:
    beqz    t1, .Lmul_b_low         # b is zero or denormal
    ori     t3, t3, 0x80
.Lmul_b_ok:
    slli    a3, t3, 25              # fraction of b at bits 31..25
    beqz    a3, .Lmul_pow2
    mv      a2, t2                  # hidden one of b: product = a
    MULSTEP
    MULSTEP
    MULSTEP
    MULSTEP
    MULSTEP
    MULSTEP
    slli    a2, a2, 1               # last bit: no shift of a3 needed
    bgez    a3, .Lmul_norm
    add     a2, a2, t2
.Lmul_norm:
    # The product is in [2^14, 2^16). When it reaches bit 15 the exponent
    # goes up by one and the fraction starts one bit higher: both follow
    # from bit 15 without a branch. The fraction is truncated.
    add     t0, t0, t1
    addi    t0, t0, -BF16_EXP_BIAS
    srli    t5, a2, 15
    add     t0, t0, t5
    addi    t5, t5, 7
    srl     a2, a2, t5
    andi    a2, a2, 0x7F
    bge     t0, t4, .Lmul_inf
    blez    t0, .Lmul_under
    slli    t0, t0, 7
    or      a0, t6, t0
    or      a0, a0, a2
    ret

.Lmul_pow2:
    slli    a2, t2, 7               # b's significand is exactly 0x80
    j       .Lmul_norm

.Lmul_under:
    # Exponent <= 0: shift the fraction right by 1 - exponent; below -6
    # nothing is left and the result is a signed zero.
    li      t5, -6
    blt     t0, t5, .Lmul_zero
    li      t5, 1
    sub     t5, t5, t0
    srl     a2, a2, t5
    or      a0, t6, a2
    ret

.Lmul_a_low:
    beqz    t2, .Lmul_zero          # a = +-0, b finite: signed zero
    li      t0, 1
1:  slli    t2, t2, 1               # normalize: leading one up to bit 7
    addi    t0, t0, -1
    andi    t5, t2, 0x80
    beqz    t5, 1b
    j       .Lmul_a_ok

.Lmul_b_low:
    beqz    t3, .Lmul_zero
    li      t1, 1
1:  slli    t3, t3, 1
    addi    t1, t1, -1
    andi    t5, t3, 0x80
    beqz    t5, 1b
    j       .Lmul_b_ok

.Lmul_a_special:
    bnez    t2, .Lmul_ret           # a is NaN: a unchanged
    slli    t5, a1, 17              # b without its sign
    beqz    t5, .Lmul_nan           # Inf * 0
    j       .Lmul_inf
.Lmul_b_special:
    bnez    t3, .Lmul_ret_b         # b is NaN: b unchanged
    slli    t5, a0, 17
    beqz    t5, .Lmul_nan           # 0 * Inf
.Lmul_inf:
    li      a0, BF16_POS_INF
    or      a0, a0, t6
    ret
.Lmul_nan:
    li      a0, BF16_QNAN
    ret
.Lmul_zero:
    mv      a0, t6
    ret
.Lmul_ret_b:
    mv      a0, a1
.Lmul_ret:
    ret
    .size   bf16_mul_asm, .-bf16_mul_asm

# ---------------------------------------------------------------------------
# bf16_sqrt_asm
#
#   a0  operand; result
#   t0  sign and exponent (a >> 7, nine bits); later the result exponent
#   t1  N = 128 * m + 127, consumed by the square root
#   t2  r, the square root being built
#   t4  0xFF
#   t5, t6  scratch
#
# The C version searches for the largest mid with (mid * mid) / 128 <= m,
# which is the largest mid with mid * mid <= 128 * m + 127, that is
# isqrt(128 * m + 127). The assembly computes that integer square root digit
# by digit (two bits of N per step, eight steps, no multiply) instead of a
# binary search over products.
#
# For a positive normal input m is in [128, 512), so N is in [16511, 65535]
# and r in [128, 255]: the result needs no renormalization, the top digit of
# r is always 1 (the first step is folded into the setup), and the result
# exponent (e + 127) / 2 stays in [64, 190], so the overflow and underflow
# tests of the C version can never fire. Since r = 128 + fraction, the
# result is ((new exponent - 1) << 7) + r.
# ---------------------------------------------------------------------------
    .macro  SQSTEP bit
    addi    t5, t2, \bit            # trial r + bit
    srli    t2, t2, 1
    bltu    t1, t5, 1f
    sub     t1, t1, t5              # digit is 1
    addi    t2, t2, \bit
1:
    .endm

    .globl  bf16_sqrt_asm
    .type   bf16_sqrt_asm, @function
    .align  2
bf16_sqrt_asm:
    beqz    a0, .Lsq_ret            # +0
    srli    t0, a0, 7               # sign:exponent
    li      t4, 0xFF
    bgeu    t0, t4, .Lsq_high       # +Inf, NaN or negative
    beqz    t0, .Lsq_zero           # positive denormal

    andi    t1, a0, 0x7F
    ori     t1, t1, 0x80            # m in [128, 256)
    andi    t5, t0, 1
    xori    t5, t5, 1               # even exponent field = odd e
    sll     t1, t1, t5              # m doubled for an odd e
    slli    t1, t1, 7
    ori     t1, t1, 127             # N = 128 * m + 127

    lui     t2, 4                   # r = 1 << 14, the first digit
    sub     t1, t1, t2
    lui     t6, 1                   # bit 1 << 12 does not fit addi
    add     t5, t2, t6
    srli    t2, t2, 1
    bltu    t1, t5, 1f
    sub     t1, t1, t5
    add     t2, t2, t6
1:
    SQSTEP  1024
    SQSTEP  256
    SQSTEP  64
    SQSTEP  16
    SQSTEP  4
    addi    t5, t2, 1               # last digit: N is no longer needed
    srli    t2, t2, 1
    bltu    t1, t5, 1f
    addi    t2, t2, 1
1:
    addi    t0, t0, BF16_EXP_BIAS - 2
    srli    t0, t0, 1               # new exponent - 1 = (exp + 125) / 2
    slli    t0, t0, 7
    add     a0, t0, t2
    ret

.Lsq_high:
    beq     t0, t4, .Lsq_ret        # +Inf or a NaN with clear sign
    li      t5, 0x8000
    beq     a0, t5, .Lsq_zero       # -0 gives +0
    andi    t1, a0, 0x7F
    beqz    t1, .Lsq_nan            # -Inf or a negative value
    li      t5, 0x1FF
    beq     t0, t5, .Lsq_ret        # a NaN with the sign set: unchanged
.Lsq_nan:
    li      a0, BF16_QNAN
    ret
.Lsq_zero:
    li      a0, 0
.Lsq_ret:
    ret
    .size   bf16_sqrt_asm, .-bf16_sqrt_asm
