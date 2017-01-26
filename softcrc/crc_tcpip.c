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
 * Implementation of TCPIP CRCs and checksums
 *
 * Please refer to RFC 1071, 1141 and 1624 for details on
 * checksum calculation algorithm for Internet protocols.
 */

#include "crc.h"
#include "crc_tcpip.h"


/**
 * Data types
 * 
 */

/**
 * IPv4 header
 */
struct ipv4_header {
        uint8_t  version;            /**< version */
        uint8_t  tos;                /**< type of service */
        uint16_t length;             /**< total length */
        uint16_t id;                 /**< identification */
        uint16_t offset;             /**< fragment offset field */
        uint8_t  ttl;                /**< time to live */
        uint8_t  protocol;           /**< protocol */
        uint16_t checksum;           /**< checksum */
        uint32_t src_addr;           /**< source address */
        uint32_t dst_addr;           /**< destination address */
} __attribute__((packed));

/**
 * UDP header
 */
struct udp_header {
        uint16_t src_port;           /**< source port */
        uint16_t dst_port;           /**< destination port */
        uint16_t length;             /**< length of UDP header and data */
        uint16_t checksum;           /**< checksum value */
} __attribute__((packed));

/**
 * IPv4 pseudo header for UDP
 */
struct ipv4_pseudo_header {
        uint32_t src_addr;           /**< ipv4 source address */
        uint32_t dst_addr;           /**< ipv4 destination address */
        uint8_t  reserved;           /**< zero */
        uint8_t  protocol;           /**< ipv4 protocol (UDP 0x11) */
        uint16_t udp_length;         /**< UDP length */
} __attribute__((packed));

/**
 * Global data
 * 
 */

/**
 * Macros
 * 
 */

/**
 * Local data
 * 
 */
static __m128i xmm_be_le_swap16a;
static __m128i xmm_be_le_swap16b;
static __m128i xmm_udp_mask;

/**
 * Common use functions
 * 
 */

/** 
 * @brief Calculates 32-bit checksum for \a data
 *        that is reduced for OC16 calculation. 
 *
 * OC16 stands for One's Complement 16-bit
 *
 * @param data pointer to packet data
 * @param data_len size of the \a data packet
 * 
 * @return Calculated 32-bit checksum value
 */
static INLINE uint32_t
csum_oc16( const uint8_t *data, uint32_t nwords ) INLINE_ATTR;

static INLINE uint32_t
csum_oc16( const uint8_t *data, uint32_t data_len )
{
        const uint16_t *data16 = (const uint16_t *)data;
        uint32_t n, sum = 0;

        for( n=0; n<(data_len/sizeof(uint16_t)); n++ )
                sum += (uint32_t) data16[n];

        if(data_len&1) {
                sum += (uint32_t)data[data_len-1];
        }

        return sum;
}

/** 
 * @brief Reduces 32-bit checksum to RFC defined OC16
 * 
 * @param sum 32-bit checksum calculated by csum_oc16()
 * 
 * @return OC16 checksum as defined by RFC1071,RFC1141,RFC1624 
 */
static INLINE uint16_t csum_oc16_reduce( uint32_t sum ) INLINE_ATTR;

static INLINE uint16_t
csum_oc16_reduce( uint32_t sum )
{
        /**
         * Final part looks the same as in original algorithm
         */
        while(sum>>16)
                sum = (sum & 0xFFFF) + (sum >> 16);
        
        return (uint16_t) (~sum);
}

/** 
 * @brief Calculates 32-bit checksum for \a data
 *        that is reduced for OC16 calculation. 
 *
 * This function uses Intel SSE3 instruction set
 * and can be modified to take advantage of AVX2
 * instruction set in the future.
 *
 * OC16 stands for One's Complement 16-bit
 *
 * @param data pointer to packet data
 * @param data_len size of the \a data packet
 * @param sum32a initial value for checksum vector register A
 * @param sum32b initial value for checksum vector register B
 * 
 * @return Calculated 32-bit checksum value
 */
static INLINE uint32_t
csum_oc16_sse( const uint8_t *data, uint32_t data_len,
               __m128i sum32a, __m128i sum32b ) INLINE_ATTR;

static INLINE uint32_t
csum_oc16_sse( const uint8_t *data, uint32_t data_len,
               __m128i sum32a, __m128i sum32b )
{
        uint32_t n = 0;

#define swap16a xmm_be_le_swap16a
#define swap16b xmm_be_le_swap16b

        /**
         * If payload is big enough try to process 64 bytes
         * in one iteration.
         */
        for( n = 0 ; (n+64) <= data_len ; n += 64 ) {
                /**
                 * Load first 32 bytes
                 * - make use of SNB enhanced load
                 */
                __m128i dblock1, dblock2;

                dblock1 = _mm_loadu_si128((__m128i*)(&data[n]));
                dblock2 = _mm_loadu_si128((__m128i*)(&data[n+16]));

                sum32a = _mm_add_epi32( sum32a,
                                        _mm_shuffle_epi8(dblock1,swap16a) );
                sum32b = _mm_add_epi32( sum32b,
                                        _mm_shuffle_epi8(dblock1,swap16b) );
                sum32a = _mm_add_epi32( sum32a,
                                        _mm_shuffle_epi8(dblock2,swap16a) );
                sum32b = _mm_add_epi32( sum32b,
                                        _mm_shuffle_epi8(dblock2,swap16b) );

                /**
                 * Load second 32 bytes
                 * - make use of SNB enhanced load
                 */
                dblock1 = _mm_loadu_si128((__m128i*)(&data[n+32]));
                dblock2 = _mm_loadu_si128((__m128i*)(&data[n+48]));

                sum32a = _mm_add_epi32( sum32a,
                                        _mm_shuffle_epi8(dblock1,swap16a) );
                sum32b = _mm_add_epi32( sum32b,
                                        _mm_shuffle_epi8(dblock1,swap16b) );
                sum32a = _mm_add_epi32( sum32a,
                                        _mm_shuffle_epi8(dblock2,swap16a) );
                sum32b = _mm_add_epi32( sum32b,
                                        _mm_shuffle_epi8(dblock2,swap16b) );
        }

        /**
         * If rest of payload smaller than 64 bytes process 16 bytes
         * in one iteration.
         */
        while( (n+16) <= data_len ) {
                __m128i dblock;
                dblock = _mm_loadu_si128((__m128i*)(&data[n]));
                sum32a = _mm_add_epi32( sum32a,
                                        _mm_shuffle_epi8(dblock,swap16a) );
                sum32b = _mm_add_epi32( sum32b,
                                        _mm_shuffle_epi8(dblock,swap16b) );
                n += 16;
        }

        if(likely(n!=data_len)) {
                /**
                 * Process very likely case of having less than 15 bytes 
                 * left at the end of the payload.
                 */
                __m128i dblock;
                dblock = _mm_loadu_si128((__m128i*)&data[n]);
                dblock = xmm_shift_left( dblock, 16 - (data_len&15) );
                dblock = xmm_shift_right( dblock, 16 - (data_len&15) );
                sum32a = _mm_add_epi32( sum32a,
                                        _mm_shuffle_epi8(dblock,swap16a) );
                sum32b = _mm_add_epi32( sum32b,
                                        _mm_shuffle_epi8(dblock,swap16b) );
        }

        /**
         * Aggregate two 4x32 sum registers into one
         */
        sum32a = _mm_add_epi32( sum32a, sum32b );

        /**
         * Use horizontal dword add to go from 4x32-bits to 1x32-bits
         */
        sum32a = _mm_hadd_epi32( sum32a, _mm_setzero_si128() );
        sum32a = _mm_hadd_epi32( sum32a, _mm_setzero_si128() );
        return _mm_extract_epi32( sum32a, 0 );
}


/**
 * ===================================================================
 *
 * IP checksum
 *
 * ===================================================================
 */

/**
 * @brief Initializes TCP/IP checksum & CRC calculation module
 *
 */
void
IPChecksumInit( void )
{
        /**
         * swap16a mask converts least significant 8 bytes of
         * XMM register as follows:
         * - converts 16-bit words from big endian to little endian
         * - converts 16-bit words to 32-bit words
         */
        xmm_be_le_swap16a =
                _mm_setr_epi16( 0x0001, 0xffff, 0x0203, 0xffff,
                                0x0405, 0xffff, 0x0607, 0xffff );
        /**
         * swap16b mask converts most significant 8 bytes of
         * XMM register as follows:
         * - converts 16-bit words from big endian to little endian
         * - converts 16-bit words to 32-bit words
         */
        xmm_be_le_swap16b =
                _mm_setr_epi16( 0x0809, 0xffff, 0x0a0b, 0xffff,
                                0x0c0d, 0xffff, 0x0e0f, 0xffff );

        /**
         * UDP mask converts 16-bit words from big endian to little endian.
         * Duplicates length field
         * Extends 16-bit words to 32-bit ones
         */
        xmm_udp_mask =
                _mm_setr_epi8( 0x01, 0x00, 0xff, 0xff,
                               0x03, 0x02, 0xff, 0xff,
                               0x05, 0x04, 0xff, 0xff,  /**< do length twice */
                               0x05, 0x04, 0xff, 0xff );
}

/** 
 * @brief Calculates 16 bit TCP/IP checksum on a data block
 *
 * This is standard implementation which gets vectroized by C compiler
 * 
 * @param data pointer to data block
 * @param data_len length of data block in bytes
 * 
 * @return TCP/IP 16 bit checksum
 */
uint16_t
IPChecksum( const uint8_t *data,
            uint32_t data_len)
{
        uint32_t sum;

        sum = csum_oc16( data, data_len );
        sum = bswap4(sum);

        return csum_oc16_reduce(sum);
}

/** 
 * @brief Calculates 16 bit TCP/IP checksum on a data block
 * 
 * Intel SSE implementation
 *
 * @param data pointer to data block
 * @param data_len length of data block in bytes
 * 
 * @return TCP/IP 16 bit checksum
 */
uint16_t
IPChecksumSSE( const uint8_t *data,
               uint32_t data_len)
{
        uint32_t sum;

        if(unlikely(data==NULL))
                return 0xffff;

        if(unlikely(data_len==0))
                return 0xffff;

#ifdef __KERNEL__
        /**
         * Preserve FPU context
         */
        kernel_fpu_begin();
#endif

        sum = csum_oc16_sse( data, data_len,
                             _mm_setzero_si128(),
                             _mm_setzero_si128() );

#ifdef __KERNEL__
        /**
         * - restore FPU context
         */
        kernel_fpu_end();
#endif

        return csum_oc16_reduce( sum );
}

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
uint16_t
IPv4UDPChecksum( const uint8_t *data, uint32_t data_len)
{
        const struct ipv4_header *iph =
                (const struct ipv4_header *)data;
        const struct udp_header *udph =
                (const struct udp_header *)(data+sizeof(struct ipv4_header));
        uint32_t sum = 0;

        if(unlikely(data_len<(sizeof(*iph)+sizeof(*udph))))
                return 0xffff;

        /**
         * Do IPv4 pseudo header
         */
        sum = ((uint32_t) iph->protocol)<<8;
        sum += csum_oc16((const uint8_t *)&iph->src_addr,
                         sizeof(iph->src_addr));
        sum += csum_oc16((const uint8_t *)&iph->dst_addr,
                         sizeof(iph->dst_addr));
        sum += csum_oc16((const uint8_t *)&udph->length,
                         sizeof(udph->length));
        
        /**
         * Do UDP header without checksum.
         * Zero value should be assumed for checksum field.
         */
        sum += csum_oc16((const uint8_t *)udph,
                         sizeof(*udph)-sizeof(uint16_t));

        /**
         * Do UDP payload
         */
        sum += csum_oc16(data+sizeof(*iph)+sizeof(*udph),
                         data_len - sizeof(*iph) - sizeof(*udph));

        /**
         * Swap byte order before reduction
         */
        sum = bswap4(sum);

        return csum_oc16_reduce(sum);
}

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
 * @return Checksum value
 */
uint16_t
IPv4UDPChecksumSSE( const uint8_t *data, uint32_t data_len)
{
        const struct ipv4_header *iph =
                (const struct ipv4_header *)data;
        __m128i sum32a, sum32b;
        uint32_t sum;

        if(unlikely(data==NULL))
                return 0xffff;

        if(unlikely(data_len<
                    (sizeof(struct ipv4_header)+sizeof(struct udp_header))))
                return 0xffff;

#ifdef __KERNEL__
        /**
         * Preserve FPU context
         */
        kernel_fpu_begin();
#endif
        /**
         * Do pseudo IPv4 header
         * Load source and destination addresses from IP header
         * Swap 16-bit words from big endian to little endian
         * Extend 16 bit words to 32 bit words for further with SSE
         */
        sum32a = _mm_loadu_si128((__m128i*)
                                 (data+offsetof(struct ipv4_header,src_addr)));
        sum32a = _mm_shuffle_epi8(sum32a,xmm_be_le_swap16a);

        /**
         * Read UDP header
         * Duplicate length field as it wasn't included in IPv4 pseudo header
         * Swap 16-bit words from big endian to little endian
         * Extend 16 bit words to 32 bit words for further with SSE
         */
        sum32b = _mm_loadu_si128((__m128i*)(data+sizeof(struct ipv4_header)));
        sum32b = _mm_shuffle_epi8(sum32b,xmm_udp_mask);

        sum = csum_oc16_sse( data + sizeof(struct ipv4_header) +
                             sizeof(struct udp_header),
                             data_len - sizeof(struct ipv4_header) -
                             sizeof(struct udp_header),
                             sum32a, sum32b ) +
                ((uint32_t) iph->protocol);

#ifdef __KERNEL__
        /**
         * - restore FPU context
         */
        kernel_fpu_end();
#endif

        return csum_oc16_reduce(sum);
}
