/*
 * Copyright (c) 2017, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Intel Corporation nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Implementation of reflected CRCs
 *
 */
#include "crcr.h"

/**
 * Global data
 *
 */

/**
 * Common macros
 *
 */

/**
 * Common use local data
 *
 */

/**
 * Common use function prototypes
 *
 */
static uint32_t
reflect_32bits( const uint32_t x );

/**
 * ========================
 *
 * 32-bit LUT METHOD
 *
 * ========================
 */

/**
 * @brief Reflect the bits about the middle
 *
 * @param x value to be reflected
 *
 * @return reflected value
 */
static uint32_t
reflect_32bits( const uint32_t val )
{
    uint32_t i, res = 0;

    for (i = 0; i < 32; i++)
        if ((val & (1 << i)) != 0)
            res |= (uint32_t)(1 << (31 - i));

    return res;
}

/**
 * @brief Initializes reflected look-up-table (LUT) for given 32 bit polynomial
 *
 * @param poly CRC polynomial
 * @param rlut pointer to reflected 256 x 32bits look-up-table to be initialized
 */
void
crcr32_init_lut( const uint32_t poly,
                 uint32_t *rlut )
{
        uint_fast32_t i, j;

        if(rlut == NULL)
                return;

        for (i = 0; i < 256; i++){
                uint_fast32_t crc = reflect_32bits(i);

                for (j = 0; j < 8; j++){
                        if (crc & 0x80000000L)
                                crc = (crc << 1) ^ poly ;
                        else
                                crc <<= 1;
                }

                rlut[i] = reflect_32bits(crc);
        }
}
