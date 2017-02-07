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
 * Header file for module with implementation of reflected CRCs
 *
 */

#ifndef __CRCR_H__
#define __CRCR_H__

#include "crcext.h"
#include "types.h"

/**
 * Functions and prototypes
 *
 */

/**
 * @brief Initializes reflected look-up-table (LUT) for given 32 bit polynomial
 *
 * @param poly CRC polynomial
 * @param rlut pointer to reflected 256x32bits look-up-table to be initialized
 */
void crcr32_init_lut(const uint32_t poly, uint32_t *rlut);

/**
 * @brief Calculates 32 bit reflected CRC using LUT method.
 *
 * @param crc CRC initial value
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 * @param reflected lut 256x32bits look-up-table pointer
 *
 * @return New CRC value
 */
__forceinline
uint32_t crcr32_calc_lut(const uint8_t *data,
                         uint32_t data_len,
                         uint32_t crc,
                         const uint32_t *rlut)
{
        if (unlikely(data == NULL || rlut == NULL))
                return crc;

        while (data_len--)
                crc = rlut[(crc ^ *data++) & 0xffL] ^ (crc >> 8);

        return crc;
}

#endif /* __CRCR_H__ */
