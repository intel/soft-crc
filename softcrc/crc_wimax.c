/*
 * Copyright (c) 2009-2017, Intel Corporation
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
 * Implementation of WiMAX CRCs
 *
 */
#include "crc.h"
#include "crc_wimax.h"

/**
 * Local data
 *
 */
static uint32_t wimax_crc32_lut[256];
static struct crc_pclmulqdq_ctx wimax_crc32_pclmulqdq;

static uint8_t wimax_hcs_lut[256];

/**
 * Implementation
 *
 */

/**
 * @brief Initializes data structures for WiMAX OFDMA crc32 calculations.
 *
 */
void WiMAXCrcInit(void)
{
        crc32_init_lut(WIMAX_OFDMA_CRC32_POLYNOMIAL, wimax_crc32_lut);

        crc32_init_pclmulqdq(&wimax_crc32_pclmulqdq,
                WIMAX_OFDMA_CRC32_POLYNOMIAL);

        crc8_init_lut(WIMAX_OFDMA_HCS_POLYNOMIAL, wimax_hcs_lut);
}

/**
 * @brief Calculates WiMAX OFDMA CRC32 using LUT method
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
WiMAXCrc32CalculateLUT(const uint8_t *data, uint32_t data_len)
{
        return ~crc32_calc_lut(data, data_len, 0xffffffff, wimax_crc32_lut);
}

/**
 * @brief Calculates WiMAX OFDMA CRC32
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
WiMAXCrc32CalculateCLMUL(const uint8_t *data, uint32_t data_len)
{
        return ~crc32_calc_pclmulqdq(data, data_len, 0xffffffffUL,
                &wimax_crc32_pclmulqdq);
}

/**
 * @brief Calculates WiMAX HCS
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint8_t
WiMAXHCSCalculateLUT(const uint8_t *data, uint32_t data_len)
{
        return crc8_calc_lut(data, data_len, 0, wimax_hcs_lut);
}
