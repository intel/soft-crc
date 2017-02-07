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
 * Implementation of RNC/LTE CRCs
 *
 */

#include "crc.h"
#include "crc_rnc.h"


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

uint16_t (*IUUPDataCrc10Calculate)(const uint8_t *, uint32_t) =
        IUUPDataCrc10CalculateS2;

uint16_t (*FPDataCrc16Calculate)(const uint8_t *, uint32_t) =
        FPDataCrc16CalculateS2;

/**
 * Local data
 *
 */


/**
 * Implementation
 *
 */
static uint8_t fp_hdr_crc7_lut[256];                    /**< LUT */

static uint16_t fp_hdr_crc11_lut[256];                  /**< LUT */

static uint16_t fp_data_crc16_lut[256];                 /**< LUT */
static uint16_t fp_data_crc16_slice1[256];              /**< slice-by-2 */
static uint16_t fp_data_crc16_slice2[256];              /**< slice-by-2 */
static struct crc_pclmulqdq_ctx fp_crc16_pclmulqdq;     /**< CLMUL */

static uint8_t iuup_hdr_crc6_lut[256];                  /**< LUT */

static uint16_t iuup_data_crc10_lut[256];               /**< LUT */
static uint16_t iuup_data_crc10_slice1[256];            /**< slice-by-2 */
static uint16_t iuup_data_crc10_slice2[256];            /**< slice-by-2 */
static struct crc_pclmulqdq_ctx iuup_crc10_pclmulqdq;   /**< CLMUL */

static uint32_t lte_crc24a_lut[256];                    /**< LUT */
static uint32_t lte_crc24a_slice1[256];                 /**< slice-by-4 */
static uint32_t lte_crc24a_slice2[256];                 /**< slice-by-4 */
static uint32_t lte_crc24a_slice3[256];                 /**< slice-by-4 */
static uint32_t lte_crc24a_slice4[256];                 /**< slice-by-4 */
static struct crc_pclmulqdq_ctx lte_crc24a_pclmulqdq;   /**< CLMUL */

static uint32_t lte_crc24b_lut[256];                    /**< LUT */
static uint32_t lte_crc24b_slice1[256];                 /**< slice-by-4 */
static uint32_t lte_crc24b_slice2[256];                 /**< slice-by-4 */
static uint32_t lte_crc24b_slice3[256];                 /**< slice-by-4 */
static uint32_t lte_crc24b_slice4[256];                 /**< slice-by-4 */
static struct crc_pclmulqdq_ctx lte_crc24b_pclmulqdq;   /**< CLMUL */

/**
 * ===================================================================
 *
 * FP CRC7 HEADER
 *
 * ===================================================================
 */

/**
 * @brief Initializes data structures for FP header CRC7 calculations.
 *
 */
void FPHdrCrc7Init(void)
{
        crc8_init_lut(FP_HEADER_CRC7_POLYNOMIAL<<1, fp_hdr_crc7_lut);
}

/**
 * @brief Calculates FP header CRC7 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint8_t
FPHdrCrc7Calculate(const uint8_t *data, uint32_t data_len)
{
        return crc8_calc_lut(data, data_len, 0, fp_hdr_crc7_lut) >> 1;
}

/**
 * ===================================================================
 *
 * FP CRC11 HEADER
 *
 * ===================================================================
 */

/**
 * @brief Initializes data structures for FP header CRC11 calculations.
 *
 */
void FPHdrCrc11Init(void)
{
        crc16_init_lut(FP_HEADER_CRC11_POLYNOMIAL << 5, fp_hdr_crc11_lut);
}

/**
 * @brief Calculates FP header CRC11 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint16_t
FPHdrCrc11Calculate(const uint8_t *data, uint32_t data_len)
{
        return crc16_calc_lut(data, data_len, 0, fp_hdr_crc11_lut) >> 5;
}

/**
 * ===================================================================
 *
 * FP CRC16 PAYLOAD
 *
 * ===================================================================
 */

/**
 * @brief Initializes data struct for FP CRC16 calculations.
 *
 */
void FPDataCrc16Init(void)
{
        crc16_init_lut(FP_DATA_CRC16_POLYNOMIAL, fp_data_crc16_lut);

        crc16_init_slice2(FP_DATA_CRC16_POLYNOMIAL, fp_data_crc16_slice1,
                fp_data_crc16_slice2);

        crc32_init_pclmulqdq(&fp_crc16_pclmulqdq,
                FP_DATA_CRC16_POLYNOMIAL << 16);

        if (pclmulqdq_available)
                FPDataCrc16Calculate = FPDataCrc16CalculateCLMUL;
}

/**
 * @brief Calculates FP CRC16 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint16_t
FPDataCrc16CalculateLUT(const uint8_t *data, uint32_t data_len)
{
        return crc16_calc_lut(data, data_len, 0, fp_data_crc16_lut);
}

/**
 * @brief Calculates FP CRC16 using Slice-By-2 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint16_t
FPDataCrc16CalculateS2(const uint8_t *data, uint32_t data_len)
{
        return crc16_calc_slice2(data, data_len, 0, fp_data_crc16_slice1,
                fp_data_crc16_slice2);
}

/**
 * @brief Calculates FP CRC16 using CLMUL method with intrinsics.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint16_t
FPDataCrc16CalculateCLMUL(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_pclmulqdq(data, data_len, 0,
                &fp_crc16_pclmulqdq) >> 16;
}

/**
 * ===================================================================
 *
 * IuUP CRC6 HEADER
 *
 * ===================================================================
 */

/**
 * @brief Initializes data structures for IuUP header CRC6 calculations.
 *
 */
void IUUPHdrCrc6Init(void)
{
        crc8_init_lut(IUUP_HDR_CRC6_POLYNOMIAL << 2, iuup_hdr_crc6_lut);
}

/**
 * @brief Calculates IuUP header CRC6 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint8_t
IUUPHdrCrc6Calculate(const uint8_t *data, uint32_t data_len)
{
        return crc8_calc_lut(data, data_len, 0, iuup_hdr_crc6_lut) >> 2;
}

/**
 * ===================================================================
 *
 * IuUP CRC10 PAYLOAD
 *
 * ===================================================================
 */

/**
 * @brief Initializes data structures for IuUP CRC10 calculations.
 *
 */
void IUUPDataCrc10Init(void)
{
        crc16_init_lut(IUUP_DATA_CRC10_POLYNOMIAL << 6, iuup_data_crc10_lut);

        crc16_init_slice2(IUUP_DATA_CRC10_POLYNOMIAL << 6,
                iuup_data_crc10_slice1, iuup_data_crc10_slice2);

        crc32_init_pclmulqdq(&iuup_crc10_pclmulqdq,
                IUUP_DATA_CRC10_POLYNOMIAL << 22);

        if (pclmulqdq_available) {
                /**
                 * PCLMULQDQ is available we can init some of the constants
                 * for the algorithm using this instruction.
                 */
                IUUPDataCrc10Calculate = IUUPDataCrc10CalculateCLMUL;
        }
}

/**
 * @brief Calculates IuUP CRC10 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint16_t
IUUPDataCrc10CalculateLUT(const uint8_t *data, uint32_t data_len)
{
        return crc16_calc_lut(data, data_len, 0, iuup_data_crc10_lut) >> 6;
}

/**
 * @brief Calculates IuUP CRC10 using Slice-By-2 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint16_t
IUUPDataCrc10CalculateS2(const uint8_t *data, uint32_t data_len)
{
        return crc16_calc_slice2(data, data_len, 0, iuup_data_crc10_slice1,
                iuup_data_crc10_slice2) >> 6;
}

/**
 * @brief Calculates IuUP CRC10 using CLMUL method with intrinsics.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint16_t
IUUPDataCrc10CalculateCLMUL(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_pclmulqdq(data, data_len, 0,
                &iuup_crc10_pclmulqdq) >> (16 + 6);
}

/**
 * ===================================================================
 *
 * LTE CRCs
 * 36.212-880-Multiplexing and channel coding
 * ===================================================================
 */


/**
 * @brief Initializes data structures for LTE CRC calculations.
 *
 */
void LTECrcInit(void)
{
        crc32_init_lut(LTE_CRC24A_POLYNOMIAL << 8, lte_crc24a_lut);

        crc32_init_slice4(LTE_CRC24A_POLYNOMIAL << 8, lte_crc24a_slice1,
                lte_crc24a_slice2, lte_crc24a_slice3, lte_crc24a_slice4);

        crc32_init_pclmulqdq(&lte_crc24a_pclmulqdq, LTE_CRC24A_POLYNOMIAL << 8);

        crc32_init_lut(LTE_CRC24B_POLYNOMIAL << 8, lte_crc24b_lut);

        crc32_init_slice4(LTE_CRC24B_POLYNOMIAL << 8, lte_crc24b_slice1,
                lte_crc24b_slice2, lte_crc24b_slice3, lte_crc24b_slice4);

        crc32_init_pclmulqdq(&lte_crc24b_pclmulqdq, LTE_CRC24B_POLYNOMIAL << 8);
}

/**
 * @brief Calculates LTE CRC24A using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
LTECrc24ACalculateLUT(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_lut(data, data_len, 0, lte_crc24a_lut) >> 8;
}

/**
 * @brief Calculates LTE CRC24A using Slice-By-4 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
LTECrc24ACalculateS4(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_slice4(data, data_len, 0, lte_crc24a_slice1,
                lte_crc24a_slice2, lte_crc24a_slice3, lte_crc24a_slice4) >> 8;
}

/**
 * @brief Calculates LTE CRC24A using CLMUL method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
LTECrc24ACalculateCLMUL(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_pclmulqdq(data, data_len, 0,
                &lte_crc24a_pclmulqdq) >> 8;
}

/**
 * @brief Calculates LTE CRC24B using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
LTECrc24BCalculateLUT(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_lut(data, data_len, 0, lte_crc24b_lut) >> 8;
}

/**
 * @brief Calculates LTE CRC24B using Slice-By-4 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
LTECrc24BCalculateS4(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_slice4(data, data_len, 0, lte_crc24b_slice1,
                lte_crc24b_slice2, lte_crc24b_slice3, lte_crc24b_slice4) >> 8;
}

/**
 * @brief Calculates LTE CRC24B using CLMUL method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
uint32_t
LTECrc24BCalculateCLMUL(const uint8_t *data, uint32_t data_len)
{
        return crc32_calc_pclmulqdq(data, data_len, 0,
                &lte_crc24b_pclmulqdq) >> 8;
}
