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
 * Include files
 *
 */

#define _GNU_SOURCE

#ifdef DEBUG
#include <assert.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sched.h>
#include <unistd.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <time.h>
#include <sys/timeb.h>
#include <limits.h> /* LINE_MAX */

#include "crcext.h"
#include "crc_rnc.h"
#include "crc_wimax.h"
#include "crc_sctp.h"
#include "crc_tcpip.h"
#include "crc_ether.h"
#include "crc_cable.h"

/**
 * Macros
 *
 */

#ifdef DEBUG
#define ASSERT(x) assert(x)
#else
#define ASSERT(x)
#endif

#define DIM(x) (sizeof(x)/sizeof(x[0]))

#define DEFAULT_ITERATIONS   1000000
#define NUM_DIFF_VECT_SIZES  1000
#define DEFAULT_VECT_SIZE    32

/**
 * Data types
 *
 */
typedef uint32_t (*crc32fn_t)(const uint8_t *, uint32_t);
typedef uint16_t (*crc16fn_t)(const uint8_t *, uint32_t);
typedef uint8_t (*crc8fn_t)(const uint8_t *, uint32_t);

typedef enum {
        CRC_FUNC_TYPE_CRC8,
        CRC_FUNC_TYPE_CRC16,
        CRC_FUNC_TYPE_CRC32,
        CRC_FUNC_TYPE_NUM_OF
} func_type_t;

struct enum_map {
        uint64_t tag;
        char *string_name;
};

/**
 * TAG bit field definition
 *
 */

/**
 * bits 0:6 application type
 */
#define TAG_APP_FP    (1ULL<<0)
#define TAG_APP_IUUP  (1ULL<<1)
#define TAG_APP_LTE   (1ULL<<2)
#define TAG_APP_SCTP  (1ULL<<3)
#define TAG_APP_WIMAX (1ULL<<4)
#define TAG_APP_TCPIP (1ULL<<5)
#define TAG_APP_CABLE (1ULL<<6)

/**
 * bits 7:9 algorithm type
 */
#define TAG_ALG_LUT   (1ULL<<7)
#define TAG_ALG_SLICE (1ULL<<8)
#define TAG_ALG_CLMUL (1ULL<<9)

/**
 * bits 10:11 special type
 */
#define TAG_EXECUTE   (1ULL<<10)        /**< Run test only when set to 1 */
#define TAG_REQ_CLMUL (1ULL<<11)        /**< requires PCLMULQDQ? */

/**
 * bits 12:63 function type
 */
#define TAG_ID_START_BIT (12)
#define TAG_ID_START     (1ULL<<(TAG_ID_START_BIT))
#define TAG_ID_FPCRC7    (1ULL<<(TAG_ID_START_BIT+0))
#define TAG_ID_FPCRC11   (1ULL<<(TAG_ID_START_BIT+1))
#define TAG_ID_FPCRC16   (1ULL<<(TAG_ID_START_BIT+2))
#define TAG_ID_IUUPCRC6  (1ULL<<(TAG_ID_START_BIT+3))
#define TAG_ID_IUUPCRC10 (1ULL<<(TAG_ID_START_BIT+4))
#define TAG_ID_LTECRC24A (1ULL<<(TAG_ID_START_BIT+5))
#define TAG_ID_LTECRC24B (1ULL<<(TAG_ID_START_BIT+6))
#define TAG_ID_SCTPCRC32C (1ULL<<(TAG_ID_START_BIT+7))
#define TAG_ID_WIMAXCRC32 (1ULL<<(TAG_ID_START_BIT+8))
#define TAG_ID_WIMAXHCS   (1ULL<<(TAG_ID_START_BIT+9))
#define TAG_ID_TCPIPSUM16 (1ULL<<(TAG_ID_START_BIT+10))
#define TAG_ID_UDPIPV4SUM (1ULL<<(TAG_ID_START_BIT+11))
#define TAG_ID_ETHERCRC32 (1ULL<<(TAG_ID_START_BIT+12))
#define TAG_ID_CABLECRC16 (1ULL<<(TAG_ID_START_BIT+13))
#define TAG_ID_END        (1ULL<<63)

/**
 * Data declaration
 *
 */
static double cpu_clock = 0;

static struct crc_function_ctx {
        func_type_t fntype;
        void *fn;
        const char *ctx;        /**< string describing function */
        uint32_t ref_vector_crc;/**< CRC value for reference vector */
        uint64_t tag;
        uint32_t temp_crc;      /**< Used when validating CRC algorithms */
} fntable[] = {
        { CRC_FUNC_TYPE_CRC8, FPHdrCrc7Calculate, "FP CRC7 LUT", 0x11,
          TAG_EXECUTE | TAG_APP_FP | TAG_ALG_LUT | TAG_ID_FPCRC7 },
        { CRC_FUNC_TYPE_CRC16, FPHdrCrc11Calculate, "FP CRC11 LUT", 0x624,
          TAG_EXECUTE | TAG_APP_FP | TAG_ALG_LUT | TAG_ID_FPCRC11 },
        { CRC_FUNC_TYPE_CRC16, FPDataCrc16CalculateLUT, "FP CRC16 LUT", 0x5309,
          TAG_EXECUTE | TAG_APP_FP | TAG_ALG_LUT | TAG_ID_FPCRC16 },
        { CRC_FUNC_TYPE_CRC16, FPDataCrc16CalculateS2, "FP CRC16 Slice By2",
          0x5309, TAG_EXECUTE | TAG_APP_FP | TAG_ALG_SLICE |
          TAG_ID_FPCRC16 },
        { CRC_FUNC_TYPE_CRC16, FPDataCrc16CalculateCLMUL, "FP CRC16 PCLMULQDQ",
          0x5309, TAG_EXECUTE | TAG_APP_FP | TAG_ALG_CLMUL |
          TAG_ID_FPCRC16 | TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC16, IUUPHdrCrc6Calculate, "IuUP CRC6 LUT", 0x1e,
          TAG_EXECUTE | TAG_APP_IUUP | TAG_ALG_LUT | TAG_ID_IUUPCRC6 },
        { CRC_FUNC_TYPE_CRC16, IUUPDataCrc10CalculateLUT, "IuUP CRC10 LUT",
          0x27a, TAG_EXECUTE | TAG_APP_IUUP | TAG_ALG_LUT |
          TAG_ID_IUUPCRC10 },
        { CRC_FUNC_TYPE_CRC16, IUUPDataCrc10CalculateS2, "IUUP CRC10 Slice By2",
          0x27a, TAG_EXECUTE | TAG_APP_IUUP | TAG_ALG_SLICE |
          TAG_ID_IUUPCRC10 },
        { CRC_FUNC_TYPE_CRC16, IUUPDataCrc10CalculateCLMUL,
          "IUUP CRC10 PCLMULQDQ", 0x27a, TAG_EXECUTE | TAG_APP_IUUP |
          TAG_ALG_CLMUL | TAG_ID_IUUPCRC10 | TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC32, LTECrc24ACalculateLUT, "LTE CRC24A LUT",
          0x6a1a5b, TAG_EXECUTE | TAG_APP_LTE | TAG_ALG_LUT |
          TAG_ID_LTECRC24A },
        { CRC_FUNC_TYPE_CRC32, LTECrc24ACalculateS4, "LTE CRC24A Slice By4",
          0x6a1a5b, TAG_EXECUTE | TAG_APP_LTE | TAG_ALG_SLICE |
          TAG_ID_LTECRC24A },
        { CRC_FUNC_TYPE_CRC32, LTECrc24ACalculateCLMUL, "LTE CRC24A PCLMULQDQ",
          0x6a1a5b, TAG_EXECUTE | TAG_APP_LTE | TAG_ALG_CLMUL |
          TAG_ID_LTECRC24A | TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC32, LTECrc24BCalculateLUT, "LTE CRC24B LUT",
          0xe8c129, TAG_EXECUTE | TAG_APP_LTE | TAG_ALG_LUT |
          TAG_ID_LTECRC24B },
        { CRC_FUNC_TYPE_CRC32, LTECrc24BCalculateS4, "LTE CRC24B Slice By4",
          0xe8c129, TAG_EXECUTE | TAG_APP_LTE | TAG_ALG_SLICE |
          TAG_ID_LTECRC24B },
        { CRC_FUNC_TYPE_CRC32, LTECrc24BCalculateCLMUL, "LTE CRC24B PCLMULQDQ",
          0xe8c129, TAG_EXECUTE | TAG_APP_LTE | TAG_ALG_CLMUL |
          TAG_ID_LTECRC24B | TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC32, SCTPCrc32cCalculateLUT, "SCTP CRC32c LUT",
          0x9d405ff6, TAG_EXECUTE | TAG_APP_SCTP | TAG_ALG_LUT |
          TAG_ID_SCTPCRC32C },
        { CRC_FUNC_TYPE_CRC32, SCTPCrc32cCalculateS4, "SCTP CRC32c SliceBy4",
          0x9d405ff6, TAG_EXECUTE | TAG_APP_SCTP | TAG_ALG_SLICE |
          TAG_ID_SCTPCRC32C },
        { CRC_FUNC_TYPE_CRC32, SCTPCrc32cCalculateCLMUL,
          "SCTP CRC32c PCLMULQDQ", 0x9d405ff6, TAG_EXECUTE |
          TAG_APP_SCTP | TAG_ALG_CLMUL | TAG_ID_SCTPCRC32C |
          TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC32, WiMAXCrc32CalculateLUT, "WiMAX OFDMA CRC32 LUT",
          0x5788ff55, TAG_EXECUTE | TAG_APP_WIMAX | TAG_ALG_LUT |
          TAG_ID_WIMAXCRC32 },
        { CRC_FUNC_TYPE_CRC32, WiMAXCrc32CalculateCLMUL,
          "WiMAX OFDMA CRC32 PCLMULQDQ", 0x5788ff55, TAG_EXECUTE |
          TAG_APP_WIMAX | TAG_ALG_CLMUL | TAG_ID_WIMAXCRC32 |
          TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC32, WiMAXHCSCalculateLUT, "WiMAX HCS LUT", 0x72,
          TAG_EXECUTE | TAG_APP_WIMAX | TAG_ALG_LUT | TAG_ID_WIMAXHCS },
        { CRC_FUNC_TYPE_CRC16, IPChecksum, "TCP/IP Checksum", 0x5a4a,
          TAG_EXECUTE | TAG_APP_TCPIP | TAG_ALG_LUT | TAG_ID_TCPIPSUM16 },
        { CRC_FUNC_TYPE_CRC16, IPChecksumSSE, "TCP/IP Checksum SSE", 0x5a4a,
          TAG_EXECUTE | TAG_APP_TCPIP | TAG_ALG_CLMUL |
          TAG_ID_TCPIPSUM16 | TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC16, IPv4UDPChecksum, "IPv4 UDP Checksum", 0xc27f,
          TAG_EXECUTE | TAG_APP_TCPIP | TAG_ALG_LUT | TAG_ID_UDPIPV4SUM },
        { CRC_FUNC_TYPE_CRC16, IPv4UDPChecksumSSE, "IPv4 UDP Checksum SSE",
          0xc27f, TAG_EXECUTE | TAG_APP_TCPIP | TAG_ALG_CLMUL |
          TAG_ID_UDPIPV4SUM | TAG_REQ_CLMUL },
        { CRC_FUNC_TYPE_CRC32, EtherCrc32CalculateCLMUL,
          "Ethernet CRC32 PCLMULQDQ", 0xb491aab4, TAG_EXECUTE |
          TAG_APP_CABLE | TAG_ALG_CLMUL | TAG_ID_ETHERCRC32 },
        { CRC_FUNC_TYPE_CRC32, EtherCrc32CalculateLUT, "Ethernet CRC32 LUT",
          0xb491aab4, TAG_EXECUTE | TAG_APP_CABLE | TAG_ALG_LUT |
          TAG_ID_ETHERCRC32 },
        { CRC_FUNC_TYPE_CRC16, CableCrc16CalculateLUT, "Cable CRC16 X.25 LUT",
          0x6bec, TAG_EXECUTE | TAG_APP_CABLE | TAG_ALG_LUT |
          TAG_ID_CABLECRC16 },
        { CRC_FUNC_TYPE_CRC16, CableCrc16CalculateCLMUL,
          "Cable CRC16 X.25 CLMUL", 0x6bec,
          TAG_EXECUTE | TAG_APP_CABLE | TAG_ALG_CLMUL | TAG_ID_CABLECRC16 }
};

static const struct enum_map enum_func_map[] = {
        {TAG_ID_FPCRC7,     "FPCRC7"},
        {TAG_ID_FPCRC11,    "FPCRC11"},
        {TAG_ID_FPCRC16,    "FPCRC16"},
        {TAG_ID_IUUPCRC6,   "IUUPCRC6"},
        {TAG_ID_IUUPCRC10,  "IUUPCRC10"},
        {TAG_ID_LTECRC24A,  "LTECRC24A"},
        {TAG_ID_LTECRC24B,  "LTECRC24B"},
        {TAG_ID_SCTPCRC32C, "SCTPCRC32C"},
        {TAG_ID_WIMAXCRC32, "WIMAXCRC32"},
        {TAG_ID_WIMAXHCS,   "WIMAXHCS"},
        {TAG_ID_TCPIPSUM16, "TCPIPSUM16"},
        {TAG_ID_UDPIPV4SUM, "UDPIPV4SUM"},
        {TAG_ID_ETHERCRC32, "ETHERCRC32"},
        {TAG_ID_CABLECRC16, "CABLECRC16"}
};

static const struct enum_map enum_alg_map[] = {
        {TAG_ALG_LUT,      "LUT"},
        {TAG_ALG_SLICE,    "SLICE"},
        {TAG_ALG_CLMUL,    "CLMUL"}
};

static const struct enum_map enum_app_map[] = {
        {TAG_APP_FP,       "FP"},
        {TAG_APP_IUUP,     "IUUP"},
        {TAG_APP_LTE,      "LTE"},
        {TAG_APP_SCTP,     "SCTP"},
        {TAG_APP_WIMAX,    "WIMAX"},
        {TAG_APP_TCPIP,    "TCPIP"},
        {TAG_APP_CABLE,    "CABLE"}
};

/**
 * Functions prototypes
 *
 */

static void get_cpu_clock(const unsigned int cpuid);

static uint8_t *
generate_vector(const unsigned size);

static double
time_diff(struct timeb *t1, struct timeb *t2);

static void
print_perf_results(const double tdiff, const char *crctype, const char *context,
                   const uint32_t *vector_size, const uint32_t vector_num,
                   const uint32_t iterations);

static void
crc_perf_test8(crc8fn_t fn8, uint8_t **vector_data, uint32_t *vector_size,
               uint32_t vector_num, const uint32_t iterations,
               const char *context);

static void
crc_perf_test16(crc16fn_t fn16, uint8_t **vector_data, uint32_t *vector_size,
                uint32_t vector_num, const uint32_t iterations,
                const char *context);

static void
crc_perf_test32(crc32fn_t fn32, uint8_t **vector_data, uint32_t *vector_size,
                uint32_t vector_num, const uint32_t iterations,
                const char *context);

static void
crc_perf_test(const func_type_t fntype, void *fn, uint8_t **vector_data,
              uint32_t *vector_size, uint32_t vector_num,
              const uint32_t iterations, const char *context);

static void
print_help(void);

static int
conf_test(void);

static void
select_test_groups(const struct enum_map *pmap, const unsigned int map_len,
                   const char *user_tags);

/**
 * Implementation
 *
 */

/**
 * @brief Get BOGOMIPS value and divide by 2 to get CPU clock
 *
 * Function sets cpu_clock variable.
 *
 * @param cpuid CPU id that we are looking clock speed for
 *
 */
static void get_cpu_clock(const unsigned int cpuid)
{
        char cb[256];
        FILE *fp = NULL;
        float bogomips = 0;

        snprintf(cb, DIM(cb), "cat /proc/cpuinfo | grep bogo | cut -d : -f 2 |"
                 " head -n %u | tail -n 1", (cpuid + 1));

        fp = popen(cb, "r");
        if (fp != NULL) {
                if (fscanf(fp, "%f", &bogomips) > 0)
                        printf("BOGOMIPS %.2f\n", bogomips);
                else
                        bogomips = 0;

                pclose(fp);
        }

        cpu_clock = ((double)bogomips) / 2;
        printf("CPU clock %.2f [MHz]\n", cpu_clock);
}


/**
 * @brief Initializes data vector of given size.
 *
 * @param size size of the vector in bytes
 *
 * @return Pointer to initialized vector or NULL on error
 */
static uint8_t *
generate_vector(const unsigned size)
{
        uint8_t *data_vector = NULL;
        unsigned i;

        data_vector = (uint8_t *)malloc(size);

        if (data_vector == NULL) {
                printf("Error allocating data vector size %u!\n", size);
                exit(EXIT_FAILURE);
        }

        for (i = 0; i < size; i++)
                data_vector[i] = (uint8_t) (i & 255);

        return data_vector;
}

/**
 * @brief Calculates difference between two time stamps in [ms].
 *
 * @param time1 first time stamp
 * @param time2 second time stamp
 *
 * @return time2 - time1 [ms]
 */
static double
time_diff(struct timeb *t1, struct timeb *t2)
{
        double r = 0;

        r = (double)(t2->time - t1->time) * 1000.0;
        r += (double)t2->millitm - (double)t1->millitm;
        return r;
}


/**
 * @brief Calculates test statistics and prints test info onto console.
 *
 * @param tdiff time span in milliseconds
 * @param crctype text describing CRC type
 * @param context text describing wider CRC context
 * @param vector_size table with test vector sizes
 * @param vector_num number of test vectors
 * @param iterations number of iterations
 */
static void
print_perf_results(const double tdiff, const char *crctype, const char *context,
                   const uint32_t *vector_size, const uint32_t vector_num,
                   const uint32_t iterations)
{
        uint_fast32_t j = 0;
        uint64_t total_data = 0;
        double cycles_per_byte = 0;

        for (total_data = 0, j = 0; j < vector_num; j++)
                total_data += vector_size[j];

        total_data = (total_data * iterations);

        /**
         * CPU clock is in 1 [MHz] = 10^6 [Hz]
         * tdiff is in 1 [ms] = 10^-3 [s]
         * In order to get normalized number of CPU clocks in given time span
         * we have to multiply: tdiff x CPU_Clock x 1000
         */
        cycles_per_byte = (1000.0 * tdiff * cpu_clock) / (double)total_data;

        printf("%-28s %-4s  %-8.3lf  %-11.2f\n", context, crctype,
               tdiff / 1000.0, cycles_per_byte);
}

/**
 * @brief Exercises CRC8 function against test vectors and measures its
 * performance.
 *
 * @param fn8 CRC8 function pointer (mutually exclusive to \a fn16)
 * @param vector_data Array of pointers to vector data to be tested
 * @param vector_size Array of data vector sizes
 * @param vector_num Number of data vectors to be exercised
 * @param iterations Number of iterations
 * @param context Context string to be promoted together with result
 */
static void
crc_perf_test8(crc8fn_t fn8, uint8_t **vector_data, uint32_t *vector_size,
               uint32_t vector_num, const uint32_t iterations,
               const char *context)
{
        uint_fast32_t i = 0, j = 0;
        struct timeb tb1, tb2;
        double tdiff = 0;

        ASSERT(fn8 != NULL);

        /**
         * Benchmarking CRC8 type of function
         */
        ftime(&tb1);
        for (j = 0; j < vector_num; j++)
                for (i = 0; i < iterations; i++)
                        (void) fn8(vector_data[j], vector_size[j]);

        ftime(&tb2);
        tdiff = time_diff(&tb1, &tb2);

        print_perf_results(tdiff, "8", context, vector_size, vector_num,
                           iterations);
}

/**
 * @brief Exercises CRC16 function against test vectors and measures its
 * performance.
 *
 * @param fn16 CRC16 function pointer
 * @param vector_data Array of pointers to vector data to be tested
 * @param vector_size Array of data vector sizes
 * @param vector_num Number of data vectors to be exercised
 * @param iterations Number of iterations
 * @param context Context string to be promoted together with result
 */
static void
crc_perf_test16(crc16fn_t fn16, uint8_t **vector_data, uint32_t *vector_size,
                uint32_t vector_num, const uint32_t iterations,
                const char *context)
{
        uint_fast32_t i = 0, j = 0;
        double tdiff = 0;
        struct timeb tb1, tb2;

        ASSERT(fn16 != NULL);

        /**
         * Benchmarking CRC16 type of function
         */
        ftime(&tb1);
        for (j = 0; j < vector_num; j++)
                for (i = 0; i < iterations; i++)
                        (void) fn16(vector_data[j], vector_size[j]);

        ftime(&tb2);
        tdiff = time_diff(&tb1, &tb2);

        print_perf_results(tdiff, "16", context, vector_size, vector_num,
                           iterations);
}

/**
 * @brief Exercises CRC32 function against test vectors and measures its
 * performance.
 *
 * @param fn32 CRC32 function pointer
 * @param vector_data Array of pointers to vector data to be tested
 * @param vector_size Array of data vector sizes
 * @param vector_num Number of data vectors to be exercised
 * @param iterations Number of iterations
 * @param context Context string to be promoted together with result
 */
static void
crc_perf_test32(crc32fn_t fn32, uint8_t **vector_data, uint32_t *vector_size,
                uint32_t vector_num, const uint32_t iterations,
                const char *context)
{
        uint_fast32_t i = 0, j = 0;
        struct timeb tb1, tb2;
        double tdiff = 0;

        ASSERT(fn32 != NULL);

        /**
         * Benchmarking CRC32 type of function
         */
        ftime(&tb1);
        for (j = 0; j < vector_num; j++)
                for (i = 0; i < iterations; i++)
                        (void) fn32(vector_data[j], vector_size[j]);

        ftime(&tb2);
        tdiff = time_diff(&tb1, &tb2);

        print_perf_results(tdiff, "32", context, vector_size, vector_num,
                           iterations);
}

/**
 * @brief Exercises CRC function against test vectors and measures its
 * performance.
 *
 * @param fntype CRC function type
 * @param fn CRC function pointer
 * @param vector_data Array of pointers to vector data to be tested
 * @param vector_size Array of data vector sizes
 * @param vector_num Number of data vectors to be exercised
 * @param iterations Number of iterations
 * @param context Context string to be promoted together with result
 */
static void
crc_perf_test(const func_type_t fntype, void *fn, uint8_t **vector_data,
              uint32_t *vector_size, uint32_t vector_num,
              const uint32_t iterations, const char *context)
{
        ASSERT(fn != NULL);

        switch (fntype) {
        case CRC_FUNC_TYPE_CRC8:
                crc_perf_test8((crc8fn_t) fn, vector_data, vector_size,
                               vector_num, iterations, context);
                break;
        case CRC_FUNC_TYPE_CRC16:
                crc_perf_test16((crc16fn_t) fn, vector_data, vector_size,
                                vector_num, iterations, context);
                break;
        case CRC_FUNC_TYPE_CRC32:
                crc_perf_test32((crc32fn_t) fn, vector_data, vector_size,
                                vector_num, iterations, context);
                break;
        case CRC_FUNC_TYPE_NUM_OF:
        default:
                ASSERT(0);
                break;
        }
}


/**
 * ===========================
 * UTILS
 *
 */

/**
 * @brief CRC test procedure
 *
 * - executes tests against reference vectors
 * - executes tests against a range of payloads
 *
 * @return Test result code
 * @retval 0 success
 * @retval -1 failure
 */
static int
conf_test(void)
{
        static const uint8_t ref_vector[32 + 16] = {
                '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j',
                'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
                0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
        };
        static const uint32_t ref_vector_size = 32;
        int i, errcnt = 0;
        uint8_t *vec = NULL;

        /**
         * Run sanity check on all CRC functions.
         * This will execute each of the CRC functions put in fntable
         * against reference vector and function result will be verified against
         * reference value.
         */
        for (i = 0; i < DIM(fntable); i++) {
                uint32_t lcrc = 0;

                if ((fntable[i].tag & TAG_REQ_CLMUL) && (!pclmulqdq_available))
                        continue;

                ASSERT(fntable[i].fn != NULL);

                switch (fntable[i].fntype) {
                case CRC_FUNC_TYPE_CRC8:
                        {
                                crc8fn_t fn = (crc8fn_t)(fntable[i].fn);

                                lcrc = (uint32_t) fn(ref_vector,
                                                     ref_vector_size);
                        }
                        break;
                case CRC_FUNC_TYPE_CRC16:
                        {
                                crc16fn_t fn = (crc16fn_t)(fntable[i].fn);

                                lcrc = (uint32_t) fn(ref_vector,
                                                     ref_vector_size);
                        }
                        break;
                case CRC_FUNC_TYPE_CRC32:
                        {
                                crc32fn_t fn = (crc32fn_t)(fntable[i].fn);

                                lcrc = (uint32_t) fn(ref_vector,
                                                     ref_vector_size);
                        }
                        break;
                case CRC_FUNC_TYPE_NUM_OF:
                default:
                        ASSERT(0);
                        break;
                }

                if (lcrc == fntable[i].ref_vector_crc)
                        continue;

                printf("! CRC%s %s for reference vector is 0x%x, expected "
                       "value 0x%x !\n",
                       (fntable[i].fntype == CRC_FUNC_TYPE_CRC32) ? "32" :
                       ((fntable[i].fntype == CRC_FUNC_TYPE_CRC16) ? "16":"8"),
                       fntable[i].ctx, lcrc, fntable[i].ref_vector_crc);
                errcnt++;
        }

        if (errcnt) {
                printf("!!! REFERENCE VECTOR TEST FAILED !!!\n");
                return -1;
        } else {
                printf("REFERENCE VECTOR TEST: PASSED\n");
        }

        /**
         * Test varying test vectors
         */
        vec = generate_vector(NUM_DIFF_VECT_SIZES + 16);

        for (i = 0; i <= NUM_DIFF_VECT_SIZES; i++) {
                int j;
                uint64_t k;

                for (j = 0; j < DIM(fntable); j++) {

                        if ((fntable[j].tag & TAG_REQ_CLMUL) &&
                            (!pclmulqdq_available)) {
                                printf("%s requires PCLMULQDQ instruction. "
                                       "This is unsupported on current "
                                       "platform. Skipping ...\n",
                                       fntable[j].ctx);
                                continue;
                        }

                        if (fntable[j].fntype == CRC_FUNC_TYPE_CRC8) {
                                crc8fn_t fn = (crc8fn_t)(fntable[j].fn);

                                fntable[j].temp_crc = (uint32_t) fn(vec, i);
                        } else if (fntable[j].fntype == CRC_FUNC_TYPE_CRC16) {
                                crc16fn_t fn = (crc16fn_t)(fntable[j].fn);

                                fntable[j].temp_crc = (uint32_t) fn(vec, i);
                        } else if (fntable[j].fntype == CRC_FUNC_TYPE_CRC32) {
                                crc32fn_t fn = (crc32fn_t)(fntable[j].fn);

                                fntable[j].temp_crc = (uint32_t) fn(vec, i);
                        } else {
                                fntable[j].temp_crc = 0;
                        }

                }

                for (k = TAG_ID_START; k != TAG_ID_END; k <<= 1) {
                        int index = -1;

                        for (j = 0; j < DIM(fntable); j++) {

                                if ((fntable[j].tag & TAG_REQ_CLMUL) &&
                                    (!pclmulqdq_available)) {
                                        continue;
                                }

                                if (index == -1) {
                                        if (fntable[j].tag & k)
                                                index = j;

                                        continue;
                                }

                                if ((fntable[j].temp_crc !=
                                     fntable[index].temp_crc) &&
                                    (fntable[j].tag & k)) {
                                        printf("Error! CRC's do not match!\n"
                                               "    %s() = 0x%x\n"
                                               "    %s() = 0x%x\n"
                                               "    payload size %d\n",
                                               fntable[j].ctx,
                                               fntable[j].temp_crc,
                                               fntable[index].ctx,
                                               fntable[index].temp_crc, i);
                                        errcnt++;
                                        /* exit(1); */
                                }
                        }
                }
        }

        if (vec != NULL)
                free(vec);

        if (errcnt) {
                printf("!!! VECTOR RANGE TEST FAILED !!!\n");
                return -1;
        } else {
                printf("VECTOR RANGE TEST: PASSED\n");
        }

        return 0;
}


/**
 * @brief Prints help page
 *
 */
static void
print_help(void)
{
        printf("crctest [-h]\n"
               "        [-c CPUNUMBER]\n"
               "        [-r VECTOR_SIZE_START]\n"
               "        [-i NUM_ITERATIONS]\n"
               "        [-s [VECTOR_SIZE|VECTOR_SIZE_END] ]\n"
               "        [-f [FN1,FN2,...,FNN][, ] ]\n"
               "        [-l [ALG1,ALG2,...,ALGN][, ] ]\n"
               "        [-a [APP1,APP2,...,APPN][, ] ]\n\n"
               "        -h                   :print help\n"
               "        -c CPUNUMBER         :set CPU number to run thread\n"
               "        -r VECTOR_SIZE_START :runs test across range of "
               "vectors\n"
               "        -i NUM_ITERATIONS    :set number of iterations x "
               "1000\n"
               "        -s [VECTOR_SIZE|VECTOR_SIZE_END]\n"
               "                             :set test vector size in bytes "
               "or end of vector size for range check\n"
               "        -f [<function type 1>],[<function type 2>],...\n"
               "                             :narrow down test base by given "
               "function type(s)\n"
               "               Available function types are:\n"
               "               - FPCRC16, FPCRC11, FPCRC7 for Framing "
               "Protocol CRCs\n"
               "               - IUUPCRC6, IUUPCRC10 for IuUP Protocol CRCs\n"
               "               - LTECRC24A, LTECRC24B for LTE PHY CRCs\n"
               "               - SCTPCRC32C for SCTP Protocol CRC32c\n"
               "               - WIMAXCRC32, WIMAXHCS for IEEE802.16 Protocol "
               "CRCs\n"
               "               - TCPIPSUM16, UDPIPV4SUM for TCP/IP checksum\n"
               "               - ETHERCRC32, CABLECRC16 for Cable CRCs\n"
               "        -l [LUT|SLICE|CLMUL],...\n"
               "                             :narrow down test base by given "
               "algorithm type(s)\n"
               "        -a [FP|IUUP|LTE|SCTP|WIMAX],...\n"
               "                             :narrow down test base by given "
               "application type(s)\n"
               "               Available application types are:\n"
               "               - FP for Framing Protocol\n"
               "               - IUUP for IuUP Protocol\n"
               "               - LTE for LTE PHY\n"
               "               - SCTP for SCTP Protocol\n"
               "               - WIMAX for IEEE802.16 Protocol\n"
               "               - TCPIP for TCP/IP\n"
               "               - CABLE for cable CRC16 X.25\n"
               " Examples:\n"
               "   crctest -c 7 -s 100 -i 1000\n"
               "   Runs tests pinned to CPU ID 7 for vector size 100 bytes,\n"
               "   1000 000 iterations\n\n"
               "   crctest -r 40 -s 100 -i 10\n"
               "   Runs tests on vector range from 40 to 100 bytes,\n"
               "   10 000 iterations for each vector from the range.\n"
               "\n\n"
               );
}

/**
 * @brief Selects CRC tests for execution based on user selection.
 *
 * Function responsible for parsing user input arguments
 * in first execution function will enable tests that match those tags
 * in second and following execution test will be restricted by user's tags
 *
 * @param pmap pointer to an array of structs containing a string to enum map
 * @param map_len length of pmap array
 * @param user_tags string with user tag selection
 *
 */
static void
select_test_groups(const struct enum_map *pmap, const unsigned int map_len,
                   const char *user_tags)
{
        static int reset_test_exec = 1;
        static int restrictive_tags = 0;

        unsigned int i, j;
        char *pch = NULL;
        uint64_t tag_sum = 0;
        char *user_tags_cpy = NULL;
        size_t user_tags_len = 0;

        if (pmap == NULL || user_tags == NULL || map_len == 0)
                return;

        user_tags_len = strlen(user_tags);
        if (user_tags_len == 0)
                return;

        if (user_tags_len > LINE_MAX)
                user_tags_len = LINE_MAX;

        user_tags_cpy = (char *)malloc(user_tags_len + 1);
        if (user_tags_cpy == NULL) {
                printf("Memory allocation error in %s at %d. "
                       "Exiting program ...\n", __FILE__, __LINE__);
                exit(EXIT_FAILURE);
        }

        memset(user_tags_cpy, 0, user_tags_len + 1);
        strncpy(user_tags_cpy, user_tags, user_tags_len);

        /**
         * Reset all exec bits in the all available test
         * later on some test will be enabled according to user's selection
         */
        if (reset_test_exec) {
                for (i = 0; i < DIM(fntable); i++)
                        fntable[i].tag &= (~TAG_EXECUTE);

                reset_test_exec = 0; /**<set to 0 to not execute reset again */
        }

        if (!restrictive_tags)
                printf("Test execution filter: ");
        else
                printf(" && ");

        for (i = 0, pch = strtok(user_tags_cpy, " ,");
             pch != NULL;
             i++, pch = strtok(NULL, " ,")) {
                /**
                 * Match user selection with available enums
                 * On success value of tag will be added up to
                 * previous selections (if any)
                 */

                printf("%s%s", (i == 0) ? "( " : " || ", pch);

                for (j = 0; j < map_len; j++) {
                        if (strcasecmp(pch, pmap[j].string_name) == 0) {
                                tag_sum |= pmap[j].tag;
                                break;
                        }
                }

                if (j >= map_len) {
                        printf("\n\"%s\" is not a valid selection\n\n", pch);
                        print_help();
                        exit(EXIT_FAILURE);
                }
        }

        if (i > 0)
                printf(" )");

        for (i = 0; i < DIM(fntable); i++) {
                if (!restrictive_tags) {
                        /**
                         * Expand test base by following tests (enum_sum)
                         */
                        if (fntable[i].tag & tag_sum)
                                fntable[i].tag |= TAG_EXECUTE;
                } else {
                        /**
                         * Remove tests from test base by following tests
                         * (enum_sum)
                         */
                        if ((fntable[i].tag & TAG_EXECUTE))
                                if (!(fntable[i].tag & tag_sum))
                                        fntable[i].tag &= (~TAG_EXECUTE);

                }
        }

        if (user_tags_cpy != NULL)
                free(user_tags_cpy);

        restrictive_tags++;
}

/**
 * @brief Main function of CRC test program
 *
 * @param argc number of cmd line arguments
 * @param argv array of cmd line arguments
 *
 * @return Program status
 */
int
main(int argc, char **argv)
{
        cpu_set_t cpuset;
        int cpuid = 0, res, command, average = 0;
        uint32_t vector_size_start  = DEFAULT_VECT_SIZE;
        uint32_t vector_size_end  = DEFAULT_VECT_SIZE;
        unsigned num_of_iterations = DEFAULT_ITERATIONS;
        uint32_t *vector_size = &vector_size_end;
        uint8_t *vector_data_local = NULL;
        uint8_t **vector_data = &vector_data_local;
        uint_fast32_t i;
        uint32_t vector_data_dim = 1;

        /**
         * Parse command line options
         *
         */
        while ((command = getopt(argc, argv, "hr:c:s:i:t:a:l:f:")) != -1) {

                switch (command) {

                case 'h':
                        print_help();
                        return EXIT_SUCCESS;

                case 'r':
                        vector_size_start = strtoul(optarg, NULL, 10);
                        average = 1;
                        if (vector_size_start > 16000) {
                                printf("Vector size is limited to 16000!\n");
                                exit(EXIT_FAILURE);
                        }
                        break;

                case 'c':
                        cpuid = atoi(optarg);
                        ASSERT(cpuid >= 0);
                        break;

                case 's':
                        vector_size_end = strtoul(optarg, NULL, 10);
                        if (vector_size_end > 16000) {
                                printf("Vector size is limited to 16000!\n");
                                exit(EXIT_FAILURE);
                        }
                        break;

                case 'i':
                        num_of_iterations = strtoul(optarg, NULL, 10) * 1000;
                        if (num_of_iterations == 0 ||
                            num_of_iterations > (20*1000*1000)) {
                                printf("Invalid number of iterations %u error! "
                                       "Valid range is 1 to 20 000 000\n",
                                       num_of_iterations);
                                exit(EXIT_FAILURE);
                        }
                        break;

                case 'a':
                        select_test_groups(enum_app_map, DIM(enum_app_map),
                                           optarg);
                        break;

                case 'l':
                        select_test_groups(enum_alg_map, DIM(enum_alg_map),
                                           optarg);
                        break;

                case 'f':
                        select_test_groups(enum_func_map, DIM(enum_func_map),
                                           optarg);
                        break;

                default:
                        printf("No such option: %c\n", optopt);
                case '?':
                        print_help();
                        return EXIT_SUCCESS;
                }
        }

        /**
         * Print selected options and verify settings
         *
         */
        printf("\nCPU ID is %d\n", cpuid);

        if (average) {
                if (vector_size_start > vector_size_end) {
                        printf("Vector size range start %u is greater than end "
                               "%u!\n", vector_size_start, vector_size_end);
                        exit(EXIT_FAILURE);
                }

                if (vector_size_start == vector_size_end) {
                        printf("Vector size range start and end are the same "
                               "%u!\n", vector_size_start);
                        exit(EXIT_FAILURE);
                }

                printf("Test range of vector sizes %u..%u\n", vector_size_start,
                       vector_size_end);

        } else {
                printf("Vector size is %u\n", vector_size_end);
        }

        printf("Number of iterations per vector is %u (%u x 1000)\n",
               num_of_iterations, num_of_iterations / 1000);

        /**
         * Get CPU clock
         *
         */
        get_cpu_clock(cpuid);

        /**
         * Set PID affinity
         *
         */
        CPU_ZERO(&cpuset);
        CPU_SET(cpuid, &cpuset);
        res = sched_setaffinity(0, sizeof(cpuset), &cpuset);
        ASSERT(res == 0);
        if (res != 0) {
                perror("Error when setting core affinity ");
                exit(EXIT_FAILURE);
        }

        /**
         * Init CRC tables and structures
         */
        CRCInit();

        /**
         * Check if PCLMULQDQ instruction is supported
         * by testing variable initialized by CRCInit().
         */
        if (pclmulqdq_available)
                printf("PCLMULQDQ supported!\n");
        else
                printf("! PCLMULQDQ NOT supported !\n");

        /**
         * Run conformance test
         */
        if (conf_test() != 0)
                exit(EXIT_FAILURE);

        /**
         * Now we can prepare vectors for performance tests
         */
        if (average) {
                uint8_t *cb = NULL;

                vector_data_dim = vector_size_end - vector_size_start;
                cb = generate_vector(vector_size_end);
                vector_size = malloc(sizeof(uint32_t) * vector_data_dim);
                vector_data = malloc(sizeof(void *) * vector_data_dim);
                if (vector_size == NULL || vector_data == NULL) {
                        perror("Memory allocation error ");
                        exit(EXIT_FAILURE);
                }
                ASSERT(cb != NULL);
                ASSERT(vector_data != NULL);
                ASSERT(vector_size != NULL);
                for (i = 0; i < vector_data_dim; i++) {
                        vector_size[i] = vector_size_start + i + 1;
                        vector_data[i] = cb;
                }
        } else {
                vector_data[0] = generate_vector(vector_size_start);
        }

        /**
         * Run performance tests
         */
        printf("--------------------------------------------------------\n"
               "Function                     CRC   Time      Cycles\n"
               "Name                         Size  [s]       per byte\n"
               "--------------------------------------------------------\n");

        for (i = 0; i < DIM(fntable); i++) {

                if (!(fntable[i].tag & TAG_EXECUTE))
                        continue;

                if ((fntable[i].tag & TAG_REQ_CLMUL) &&
                    (!pclmulqdq_available)) {
                        printf("%s requires PCLMULQDQ instruction unsupported "
                               "on current platform. Skipping ...\n",
                               fntable[i].ctx);
                        continue;
                }

                crc_perf_test(fntable[i].fntype,
                              fntable[i].fn,
                              vector_data,
                              vector_size,
                              vector_data_dim,
                              num_of_iterations,
                              fntable[i].ctx);
        }

        printf("--------------------------------------------------------\n");

        if (average) {
                free(vector_size);
                free(vector_data[0]);
                free(vector_data);
        }

        return EXIT_SUCCESS;
}
