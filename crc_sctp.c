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
 * Implementation of SCTP CRCs
 *
 */

#include "crc.h"
#include "crc_sctp.h"


/**
 * Global data
 *
 */

/**
 * CRC function pointers to most efficient CRC implementation.
 * By default point to slice-by-X implementations after running CRCInit()
 * and positive detection of PCLMULQDQ availability they will point to
 * CLMUL implementations.
 */

uint32_t (*SCTPDataCrc32Calculate)(const uint8_t *, uint32_t) =
        SCTPCrc32cCalculateS4;

/**
 * Local data
 *
 */

static uint32_t sctp_crc32c_lut[256];
static uint32_t sctp_crc32c_slice1[256];
static uint32_t sctp_crc32c_slice2[256];
static uint32_t sctp_crc32c_slice3[256];
static uint32_t sctp_crc32c_slice4[256];

static struct crc_pclmulqdq_ctx sctp_crc32c_pclmulqdq;

/**
 * @brief Initializes data structures for SCTP crc32c calculations.
 *
 */
void SCTPCrc32cInit(void)
{
        crc32_init_lut(SCTP_CRC32C_POLYNOMIAL, sctp_crc32c_lut);

        crc32_init_slice4(SCTP_CRC32C_POLYNOMIAL, sctp_crc32c_slice1,
                sctp_crc32c_slice2, sctp_crc32c_slice3, sctp_crc32c_slice4);

        crc32_init_pclmulqdq(&sctp_crc32c_pclmulqdq, SCTP_CRC32C_POLYNOMIAL);

        if (pclmulqdq_available)
                SCTPDataCrc32Calculate = SCTPCrc32cCalculateCLMUL;
}

/**
 * @brief Calculates SCTP CRC32c using LUT method
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
SCTPCrc32cCalculateLUT(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_lut(data, data_len, 0, sctp_crc32c_lut);
}

/**
 * @brief Calculates FP CRC16 using Slice-By-4 method
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
SCTPCrc32cCalculateS4(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_slice4(data, data_len, 0, sctp_crc32c_slice1,
                sctp_crc32c_slice2, sctp_crc32c_slice3, sctp_crc32c_slice4);
}

/**
 * @brief Calculates SCTP CRC32c
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
SCTPCrc32cCalculateCLMUL(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_pclmulqdq(data, data_len, 0, &sctp_crc32c_pclmulqdq);
}
