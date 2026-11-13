#include "gcd.h"

/* Binary GCD (Stein's algorithm), the C stepping stone for gcd.S.
 *
 * gcd(2^k * u, 2^k * v) = 2^k * gcd(u, v), and for odd u, gcd(u, v) does not
 * change when factors of two are removed from v or when the smaller operand
 * is subtracted from the larger one. The difference of two odd values is
 * even, so every subtraction is followed by at least one shift: the loop runs
 * at most about 64 times, without any division.
 *
 * __builtin_ctz is not used: on RV32I without Zbb it becomes a call to
 * libgcc's __ctzsi2, which the bare-metal link does not provide.
 */
uint32_t gcd_u32(uint32_t a, uint32_t b)
{
    if (!a)
        return b;
    if (!b)
        return a;

    /* k = number of factors of two shared by a and b */
    int k = 0;
    while (!((a | b) & 1)) {
        a >>= 1;
        b >>= 1;
        k++;
    }

    /* From here on a is odd. */
    while (!(a & 1))
        a >>= 1;

    do {
        while (!(b & 1))
            b >>= 1;
        /* Both odd: keep a <= b, then b - a is even and nonnegative. */
        if (a > b) {
            uint32_t t = a;
            a = b;
            b = t;
        }
        b -= a;
    } while (b);

    return a << k;
}
