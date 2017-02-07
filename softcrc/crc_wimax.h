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
 * Header file for module with implementation of WiMAX CRCs
 *
 */

#ifndef __CRC_WIMAX_H__
#define __CRC_WIMAX_H__

/**
 * CRC polynomials
 */
#define WIMAX_OFDMA_CRC32_POLYNOMIAL 0x04c11db7UL
#define WIMAX_OFDMA_HCS_POLYNOMIAL   0x07

/**
 * @brief Initializes data structures for WiMAX OFDMA crc32 calculations.
 *
 */
extern void WiMAXCrcInit(void);

/**
 * @brief Calculates WiMAX OFDMA CRC32 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t WiMAXCrc32CalculateLUT(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates WiMAX OFDMA CRC32 using PCLMULQDQ method
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t WiMAXCrc32CalculateCLMUL(const uint8_t *data,
        uint32_t data_len);

/**
 * @brief Calculates WiMAX MAC HCS (header checksum) using LUT method
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New HCS value
 */
extern uint8_t WiMAXHCSCalculateLUT(const uint8_t *data, uint32_t data_len);
#endif /* __CRC_WIMAX_H__ */
