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
 * Header file for module with implementation SCTP CRCs
 *
 */

#ifndef __CRC_SCTP_H__
#define __CRC_SCTP_H__

/**
 * CRC polynomials
 */

/* x^32+x^28+x^27+x^26+x^25+x^23+x^22+x^20+x^19+
 * x^18+x^14+x^13+x^11+x^10+x^9+x^8+x^6+x^0 */
#define SCTP_CRC32C_POLYNOMIAL 0x1EDC6F41

/**
 * @brief Initializes data structures for SCTP crc32c calculations.
 *
 */
extern void SCTPCrc32cInit(void);

/**
 * @brief Calculates SCTP CRC32c using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t SCTPCrc32cCalculateLUT(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates SCTP CRC32c using Slice-By-4 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t SCTPCrc32cCalculateS4(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates SCTP CRC32c using CLMUL method and reduction.
 *
 * Additional bits added to the front of a buffer to align data to multiples
 * of 16bytes. Implemented with Intrinsic
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t SCTPCrc32cCalculateCLMUL(const uint8_t *data,
        uint32_t data_len);

/**
 * CRC function pointers to most efficient CRC implementation of SCTP CRC32x.
 * By default point to slice-by-3 implementation after running CRCInit()
 * and positive detection of PCLMULQDQ availability this will point to
 * CLMUL implementation.
 */
extern uint32_t (*SCTPCrc32cCalculate)(const uint8_t *, uint32_t);

#endif /* __CRC_SCTP_H__ */
