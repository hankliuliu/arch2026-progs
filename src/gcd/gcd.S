# gcd_u32_asm: binary GCD, hand-written RV32I.
#
# uint32_t gcd_u32_asm(uint32_t a, uint32_t b)   a in a0, b in a1, result in a0
#
# Registers:
#   a0  a, made odd, then the running odd operand; holds the result
#   a1  b, made odd before each comparison
#   a2  k, the number of factors of two shared by a and b
#   t0  a | b while k is counted
#   t1  scratch: the low bit under test
#
# Algorithm (Stein): strip the common factors of two and remember their count
# k; make both operands odd; then repeatedly subtract the smaller from the
# larger and strip the factors of two of the (even) difference. When the
# difference is zero the other operand is the odd part of the gcd, and the
# result is that value shifted left by k.
#
# Differences from the C version (src/gcd/gcd.c):
#   * No swap. After the comparison the code takes one of two mirrored paths,
#     b -= a then strip b, or a -= b then strip a, so the operands never move
#     between registers.
#   * The strip loops after a subtraction shift first and test afterwards: the
#     difference of two odd values is even and nonzero, so at least one shift
#     is always due. Each factor of two costs three instructions.
#   * When a or b is odd (the common case, k = 0) the k loop is skipped with a
#     single branch.
#
# RV32I and Zicsr only: no mul, div, rem, no floating point, no compressed
# instructions. Tests: tests/tgcd.c.

    .text

    .globl  gcd_u32_asm
    .type   gcd_u32_asm, @function
    .align  2
gcd_u32_asm:
    beqz    a0, .Lret_b             # gcd(0, b) = b
    beqz    a1, .Lret               # gcd(a, 0) = a
    or      t0, a0, a1
    andi    t1, t0, 1
    li      a2, 0
    bnez    t1, .Lodd               # a or b odd: no common factor of two

    # k = ctz(a | b); then shift both operands right by k
.Lcount_k:
    srli    t0, t0, 1
    addi    a2, a2, 1
    andi    t1, t0, 1
    beqz    t1, .Lcount_k
    srl     a0, a0, a2
    srl     a1, a1, a2

.Lodd:
    # Make a odd (it may already be).
    andi    t1, a0, 1
    bnez    t1, .Lstrip_b0
.Lstrip_a0:
    srli    a0, a0, 1
    andi    t1, a0, 1
    beqz    t1, .Lstrip_a0

.Lstrip_b0:
    # Make b odd (it may already be).
    andi    t1, a1, 1
    bnez    t1, .Lcompare
.Lstrip_b0_loop:
    srli    a1, a1, 1
    andi    t1, a1, 1
    beqz    t1, .Lstrip_b0_loop

    # Both odd from here on.
.Lcompare:
    bltu    a1, a0, .La_minus_b

.Lb_minus_a:                        # b >= a
    sub     a1, a1, a0
    beqz    a1, .Ldone              # b == a: a is the odd gcd
.Lstrip_b:                          # b is even and nonzero
    srli    a1, a1, 1
    andi    t1, a1, 1
    beqz    t1, .Lstrip_b
    bgeu    a1, a0, .Lb_minus_a

.La_minus_b:                        # a > b, so a - b is even and nonzero
    sub     a0, a0, a1
.Lstrip_a:
    srli    a0, a0, 1
    andi    t1, a0, 1
    beqz    t1, .Lstrip_a
    bltu    a1, a0, .La_minus_b
    j       .Lb_minus_a

.Ldone:
    sll     a0, a0, a2              # restore the common factors of two
.Lret:
    ret

.Lret_b:
    mv      a0, a1
    ret
    .size   gcd_u32_asm, .-gcd_u32_asm
