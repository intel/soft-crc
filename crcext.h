/*******************************************************************************
 Copyright (c) 2009-2017, Intel Corporation

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/**
 * Header file with CRC external definitions
 *
 */

#ifndef __CRCEXT_H__
#define __CRCEXT_H__

#include <x86intrin.h>
#include "types.h"

/**
 * Flag indicating availability of PCLMULQDQ instruction
 * Only valid after running CRCInit() function.
 */
extern int pclmulqdq_available;

/**
 * Flag indicating availability of PCLMULQDQ instruction
 * Only valid after running CRCInit() function.
 */
extern __m128i crc_xmm_be_le_swap128;
extern const uint8_t crc_xmm_shift_tab[48];

/**
 * @brief Shifts right 128 bit register by specified number of bytes
 *
 * @param reg 128 bit value
 * @param num number of bytes to shift right \a reg by (0-16)
 *
 * @return \a reg >> (\a num * 8)
 */
__forceinline
__m128i xmm_shift_right(__m128i reg, const unsigned int num)
{
        const __m128i *p = (const __m128i *)(crc_xmm_shift_tab + 16 + num);

        return _mm_shuffle_epi8(reg, _mm_loadu_si128(p));
}

/**
 * @brief Shifts left 128 bit register by specified number of bytes
 *
 * @param reg 128 bit value
 * @param num number of bytes to shift left \a reg by (0-16)
 *
 * @return \a reg << (\a num * 8)
 */
__forceinline
__m128i xmm_shift_left(__m128i reg, const unsigned int num)
{
        const __m128i *p = (const __m128i *)(crc_xmm_shift_tab + 16 - num);

        return _mm_shuffle_epi8(reg, _mm_loadu_si128(p));
}

/**
 * @brief Computes constant for CLMUL algorithm
 *
 * Result is: X^exp mod poly
 *
 * @param poly polynomial
 * @param exp exponent
 *
 * @return constant value
 */
static inline uint32_t
get_poly_constant(const uint32_t poly, const uint32_t exp)
{
        uint32_t i, res = poly;

        for (i = 32; i < exp; i++)
                if (res & 0x80000000)
                        res = (res << 1) ^ poly;
                else
                        res = (res << 1);

        return res;
}

/**
 * @brief Calculates quotient and reminder of X^64 / P(X)
 *
 * @param poly P(X)
 * @param q_ptr place to store quotient
 * @param r_ptr place to store reminder
 */
static inline void
div_poly(const uint64_t poly,
         uint64_t *q_ptr,
         uint64_t *r_ptr)
{
        uint64_t p = 0, q = 0, r = 0;
        int i;

        p = poly | 0x100000000ULL;

        r = p;
        r = r << 32;

        i = 32;
        do {
                uint64_t one_shl_n = 0;

                q = q << 1;

                if ((i + 32) < 64)
                        one_shl_n = 1ULL << (32 + i);

                if (r & one_shl_n) {
                        r ^= (p << i);
                        q |= 1;
                }
                i--;
        } while (i >= 0);

        if (q_ptr != NULL)
                *q_ptr = q;

        if (r_ptr != NULL)
                *r_ptr = r;
}

/**
 * @brief Initializes CRC module.
 * @note It is mandatory to run it before using any of CRC API's.
 */
extern void CRCInit(void);

#endif /* __CRCEXT_H__ */
