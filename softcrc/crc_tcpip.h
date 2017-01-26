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
 * Header file for module with implementation of TCPIP CRCs and checksums
 * 
 */

#ifndef __CRC_TCPIP_H__
#define __CRC_TCPIP_H__

/**
 * @brief Initializes TCP/IP checksum & CRC calculation module
 *
 */
extern void IPChecksumInit( void );

/**
 * @brief TCP/IP 16 bit checksum (standard implementation)
 *
 * @param data pointer to data block to calculate checksum for
 * @param data_len size of data block
 *
 * @return Checksum value
 */
extern uint16_t
IPChecksum( const uint8_t *data,
            uint32_t data_len);

/**
 * @brief TCP/IP 16 bit checksum (SSE implementation)
 *
 * @param data pointer to data block to calculate checksum for
 * @param data_len size of data block
 *
 * @note This function can read up to 15 bytes placed after
 *       specified data block. 
 *
 * @return Checksum value
 */
extern uint16_t
IPChecksumSSE( const uint8_t *data,
               uint32_t data_len);

/**
 * @brief UDP/IPv4 16 bit checksum (standard implementation)
 *
 * @param data pointer to packet IP header
 * @param data_len IPv4 header length (20 bytes) + UDP header length
 *        (8 bytes) + UDP payload length
 *
 * @note   Assumed:
 *         - standard IP header of 20 bytes
 *         - UDP header follows IP header.
 *
 * @return Checksum value
 */
extern uint16_t
IPv4UDPChecksum( const uint8_t *data,
                 uint32_t data_len);

/**
 * @brief UDP/IPv4 16 bit checksum (SSE implementation)
 *
 * @param data pointer to a packet (points at IP header)
 * @param data_len IPv4 header length (20 bytes) + UDP header length
 *        (8 bytes) + UDP payload length
 *
 * @note   Assumed:
 *         - standard IP header of 20 bytes
 *         - UDP header follows IP header.
 *
 * @note This function can read up to 15 bytes placed after
 *       specified data block. 
 *
 * @return Checksum value
 */
extern uint16_t
IPv4UDPChecksumSSE( const uint8_t *data,
                    uint32_t data_len);


#endif /* __CRC_TCPIP_H__ */
