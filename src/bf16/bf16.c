#include "bf16.h"

/* BFloat16 routines of Quiz 1 Problem B, the C stepping stone for bf16.S.
 *
 * A bf16 value is the upper half of a binary32: sign (bit 15), exponent
 * (bits 14..7, bias 127) and a 7-bit fraction. Only integer operations are
 * used; the significand of a normal value is the fraction with the hidden
 * one at bit 7, an integer in [128, 256) that stands for [1.0, 2.0).
 */

#define BF16_EXP_BIAS 127
#define BF16_POS_INF 0x7F80u
#define BF16_QNAN 0x7FC0u

/* Round to nearest, ties to even: adding 0x7FFF carries into bit 16 exactly
 * when the discarded half is above 0x8000, and bit 16 itself breaks the tie
 * at 0x8000 toward an even kept half. Inf and NaN pass through unchanged.
 */
uint16_t bf16_from_f32(uint32_t f32bits)
{
    if (((f32bits >> 23) & 0xFF) == 0xFF)
        return (uint16_t) (f32bits >> 16);
    f32bits += 0x7FFFu + ((f32bits >> 16) & 1u);
    return (uint16_t) (f32bits >> 16);
}

uint16_t bf16_mul(uint16_t a, uint16_t b)
{
    uint32_t sign = (uint32_t) ((a ^ b) & 0x8000u);
    int32_t exp_a = (a >> 7) & 0xFF, exp_b = (b >> 7) & 0xFF;
    uint32_t mant_a = a & 0x7Fu, mant_b = b & 0x7Fu;

    /* Specials, a first and then b: NaN propagates, Inf * 0 is NaN. */
    if (exp_a == 0xFF) {
        if (mant_a)
            return a;
        if (!exp_b && !mant_b)
            return BF16_QNAN;
        return (uint16_t) (sign | BF16_POS_INF);
    }
    if (exp_b == 0xFF) {
        if (mant_b)
            return b;
        if (!exp_a && !mant_a)
            return BF16_QNAN;
        return (uint16_t) (sign | BF16_POS_INF);
    }
    if ((!exp_a && !mant_a) || (!exp_b && !mant_b))
        return (uint16_t) sign;

    /* Normalize denormals: move the leading one up to bit 7. */
    int32_t exp_adjust = 0;
    if (!exp_a) {
        while (!(mant_a & 0x80)) {
            mant_a <<= 1;
            exp_adjust--;
        }
        exp_a = 1;
    } else {
        mant_a |= 0x80;
    }
    if (!exp_b) {
        while (!(mant_b & 0x80)) {
            mant_b <<= 1;
            exp_adjust--;
        }
        exp_b = 1;
    } else {
        mant_b |= 0x80;
    }

    /* 8 x 8 -> 16-bit product in [2^14, 2^16): renormalize by one bit when it
     * reaches bit 15, then truncate to 7 fraction bits.
     */
    uint32_t prod = mant_a * mant_b;
    int32_t exp = exp_a + exp_b - BF16_EXP_BIAS + exp_adjust;
    uint32_t frac;
    if (prod & 0x8000u) {
        frac = (prod >> 8) & 0x7F;
        exp++;
    } else {
        frac = (prod >> 7) & 0x7F;
    }

    if (exp >= 0xFF)
        return (uint16_t) (sign | BF16_POS_INF);
    if (exp <= 0) {
        if (exp < -6)
            return (uint16_t) sign;
        return (uint16_t) (sign | (frac >> (1 - exp)));
    }
    return (uint16_t) (sign | ((uint32_t) exp << 7) | frac);
}

uint16_t bf16_sqrt(uint16_t a)
{
    uint32_t sign = a >> 15;
    int32_t exp = (a >> 7) & 0xFF;
    uint32_t mant = a & 0x7Fu;

    if (exp == 0xFF) {
        if (mant)
            return a;                /* NaN */
        return sign ? BF16_QNAN : a; /* -Inf is NaN, +Inf stays */
    }
    if (!exp && !mant)
        return 0; /* +0 and -0 give +0 */
    if (sign)
        return BF16_QNAN;
    if (!exp)
        return 0; /* denormals flush to +0 */

    /* sqrt(2^e * m) = 2^(e/2) * sqrt(m), with e made even by doubling m. */
    int32_t e = exp - BF16_EXP_BIAS;
    uint32_t m = 0x80u | mant;
    int32_t new_exp;
    if (e & 1) {
        m <<= 1;
        new_exp = ((e - 1) >> 1) + BF16_EXP_BIAS;
    } else {
        new_exp = (e >> 1) + BF16_EXP_BIAS;
    }

    /* Largest mid in [90, 256] with (mid * mid) / 128 <= m. */
    uint32_t low = 90, high = 256, result = 128;
    while (low <= high) {
        uint32_t mid = (low + high) >> 1;
        if ((mid * mid) / 128 <= m) {
            result = mid;
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    /* Renormalize into [128, 256). */
    if (result >= 256) {
        result >>= 1;
        new_exp++;
    } else {
        while (result < 128 && new_exp > 1) {
            result <<= 1;
            new_exp--;
        }
    }

    if (new_exp >= 0xFF)
        return BF16_POS_INF;
    if (new_exp <= 0)
        return 0;
    return (uint16_t) (((uint32_t) new_exp << 7) | (result & 0x7F));
}
