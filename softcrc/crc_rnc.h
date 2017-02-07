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
 * Header file for module with implementation of RNC/LTE CRCs
 *
 */

#ifndef __CRC_RNC_H__
#define __CRC_RNC_H__

/**
 * Definitions of FP and IuUP polynomials
 */

/**
 * 3GPP TS 25.435, 3GPP TS 25.427
 * Framing Protocol CRC polynomials
 */
#define FP_HEADER_CRC7_POLYNOMIAL  0x45     /**< x^7 + x^6 + x^2 + 1 */
#define FP_HEADER_CRC11_POLYNOMIAL 0x307/**< x^11+x^9+x^8+x^2+x+1 EDCH header */
#define FP_DATA_CRC16_POLYNOMIAL   0x8005   /**< x^16 + x^15 + x^2 + 1 */

/**
 * 3GPP TS 25.415
 * IuUP CRC polynomials
 */
#define IUUP_HDR_CRC6_POLYNOMIAL   0x2F /**< x^6 + x^5 + x^3 + x^2 + x + 1 */
#define IUUP_DATA_CRC10_POLYNOMIAL 0x233/**< x^11 + x^10 + + x^5 + x^4 + x + 1*/

/**
 * 3GPP TS 36.212-880-Multiplexing and channel coding
 * LTE CRC polynomials
 */
#define LTE_CRC24A_POLYNOMIAL   0x864CFB
#define LTE_CRC24B_POLYNOMIAL   0x800063

/**
 * @brief Initializes data structures for FP header CRC7 calculations.
 *
 */
extern void FPHdrCrc7Init(void);

/**
 * @brief Calculates FP header CRC7 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint8_t FPHdrCrc7Calculate(const uint8_t *data, uint32_t data_len);

/**
 * @brief Initializes data structures for FP header CRC11 calculations.
 *
 */
extern void FPHdrCrc11Init(void);

/**
 * @brief Calculates FP header CRC11 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint16_t FPHdrCrc11Calculate(const uint8_t *data, uint32_t  data_len);

/**
 * @brief Initializes data struct for FP CRC16 calculations.
 *
 */
extern void FPDataCrc16Init(void);

/**
 * @brief Calculates FP CRC16 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint16_t FPDataCrc16CalculateLUT(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates FP CRC16 using Slice-By-2 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint16_t FPDataCrc16CalculateS2(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates FP CRC16 using CLMUL method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint16_t FPDataCrc16CalculateCLMUL(const uint8_t *data,
        uint32_t data_len);

/**
 * CRC function pointers to most efficient CRC implementation of FP CRC16.
 * By default point to slice-by-2 implementation after running CRCInit()
 * and positive detection of PCLMULQDQ availability this will point to
 * CLMUL implementation.
 */
extern uint16_t (*FPDataCrc16Calculate)(const uint8_t *, uint32_t);

/**
 * @brief Initializes data structures for IuUP header CRC6 calculations.
 *
 */
extern void IUUPHdrCrc6Init(void);

/**
 * @brief Calculates IuUP header CRC6 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint8_t IUUPHdrCrc6Calculate(const uint8_t *data, uint32_t data_len);

/**
 * @brief Initializes data structures for IuUP CRC10 calculations.
 *
 */
extern void IUUPDataCrc10Init(void);

/**
 * @brief Calculates IuUP CRC10 using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint16_t IUUPDataCrc10CalculateLUT(const uint8_t *data,
        uint32_t data_len);

/**
 * @brief Calculates IuUP CRC10 using Slice-By-2 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint16_t IUUPDataCrc10CalculateS2(const uint8_t *data,
        uint32_t data_len);

/**
 * @brief Calculates IuUP CRC10 using CLMUL method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint16_t IUUPDataCrc10CalculateCLMUL(const uint8_t *data,
        uint32_t data_len);

/**
 * CRC function pointers to most efficient CRC implementation of IuUP CRC10.
 * By default point to slice-by-2 implementation after running CRCInit()
 * and positive detection of PCLMULQDQ availability this will point to
 * CLMUL implementation.
 */
extern uint16_t (*IUUPDataCrc10Calculate)(const uint8_t *, uint32_t);

/**
 * @brief Initializes data structures for LTE TrCH calculations.
 *
 */
extern void LTECrcInit(void);

/**
 * @brief Calculates LTE CRC24A using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t LTECrc24ACalculateLUT(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates LTE CRC24A using Slice-By-4 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t LTECrc24ACalculateS4(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates LTE CRC24A using CLMUL method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t LTECrc24ACalculateCLMUL(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates LTE CRC24B using LUT method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t LTECrc24BCalculateLUT(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates LTE CRC24B using Slice-By-4 method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t LTECrc24BCalculateS4(const uint8_t *data, uint32_t data_len);

/**
 * @brief Calculates LTE CRC24A using CLMUL method.
 *
 * @param data pointer to data block to calculate CRC for
 * @param data_len size of data block
 *
 * @return New CRC value
 */
extern uint32_t LTECrc24BCalculateCLMUL(const uint8_t *data, uint32_t data_len);
#endif /* __CRC_RNC_H__ */
