#include "fixed.h"

/* Arithmetic from logic and Q11.20 fixed point (Quiz 3 Problem C), the C
 * stepping stone for fixed.S.
 */

/* Ripple of carries: a ^ b is the sum without carries and (a & b) << 1 the
 * carries; feeding the carries back in until none is left gives a + b modulo
 * 2^32. A carry moves up one bit per pass and leaves through bit 31, so there
 * are at most 32 passes.
 */
uint32_t op1(uint32_t a, uint32_t b, int *n)
{
    int passes = 0;
    while (b) {
        uint32_t c = (a & b) << 1;
        a ^= b;
        b = c;
        passes++;
    }
    *n = passes;
    return a;
}

/* a - b = a + ~b + 1 in two's complement; the plain subtraction is the same
 * function and meets the instruction budget.
 */
uint32_t op2(uint32_t a, uint32_t b)
{
    return a - b;
}

/* a + b = 2 * (a & b) + (a ^ b): the shared bits counted twice plus the
 * differing bits once. Halving term by term never forms the 33-bit sum, and
 * the dropped low bit of a ^ b is exactly the one the floor discards.
 */
uint32_t op3(uint32_t a, uint32_t b)
{
    return (a & b) + ((a ^ b) >> 1);
}

/* Q11.20 product: (a * b + 2^19) >> 20 on the exact 64-bit product, low 32
 * bits kept. RV32I has no 64-bit multiply and libgcc's __muldi3 is not
 * linked, so the product of the magnitudes is assembled from four 16 x 16
 * partial products (each a 32-bit multiply, lowered to __mulsi3) and the sign
 * is applied to the 64-bit sum afterwards.
 */
int32_t q_mul(int32_t a, int32_t b)
{
    uint32_t ma = a < 0 ? 0u - (uint32_t) a : (uint32_t) a;
    uint32_t mb = b < 0 ? 0u - (uint32_t) b : (uint32_t) b;
    uint32_t al = ma & 0xFFFF, ah = ma >> 16;
    uint32_t bl = mb & 0xFFFF, bh = mb >> 16;

    uint64_t mag = (uint64_t) (al * bl);
    mag += (uint64_t) (al * bh) << 16;
    mag += (uint64_t) (ah * bl) << 16;
    mag += (uint64_t) (ah * bh) << 32;

    uint64_t p = (a < 0) != (b < 0) ? 0u - mag : mag;
    p += (uint64_t) 1 << (Q - 1);
    /* Bits 20..51 of the rounded product: the arithmetic shift and the
     * logical one agree on them.
     */
    return (int32_t) (uint32_t) (p >> Q);
}
