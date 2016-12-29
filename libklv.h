#ifndef LIBKLV_H_INCLUDED
#define LIBKLV_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "list.h"

static const uint8_t klv_key[]              = { 0x06,0x0e,0x2b,0x34 };
static const uint8_t klv_universal_key[]    = { 0x06,0x0e,0x2b,0x34,0x02,0x0b,0x01,0x01,0x0e,0x01,0x03,0x01,0x01,0x00,0x00,0x00 };


typedef struct klv_item_s
{
    uint8_t     id;
    uint8_t     len;

    /* TODO: figure out a better way to differentiate between signed and unsigned key values */
    uint64_t    value;    /* stores unsigned values */
    int64_t     signed_val; /* stores signed values */

    double      mapped_val;

    void        *data;      /* used for variable length values. almost exclusively strings */

    struct list_head    list;
} klv_item_t;

typedef struct klv_ctx_s
{
    uint8_t     *buffer;        /* start of the buffer */
    uint16_t    buffer_size;    /* maximum buffer size */
    uint8_t     *buf_ptr;       /* current position in the buffer */
    uint8_t     *buf_end;       /* end of the data */

    klv_item_t  klv_items;     /* list of klv items parsed from a packet */

    uint64_t    payload_len;    /* length of payload according to BER value in packet */
    uint16_t    checksum;   /* store checksum retrieved from packet (not calculated) */
} klv_ctx_t;


/*
 * Local prototypes
 */
static uint64_t klv_get_ber_length(klv_ctx_t *p);
static int decode_klv_values(klv_item_t *item, klv_ctx_t *klv_ctx);
static double libklv_map_val(double value, double a, double b, double targetA, double targetB);
static int sync_to_klv_key(uint8_t *p, uint16_t len, klv_ctx_t *klv_ctx);
static inline uint64_t libklv_readUINT64(klv_ctx_t *p);
static inline uint32_t libklv_readUINT32(klv_ctx_t *p);
static inline uint16_t libklv_readUINT16(klv_ctx_t *p);
static inline uint8_t libklv_readUINT8(klv_ctx_t *p);
static char *libklv_strdup(klv_ctx_t *src, uint8_t len);
static bool has_valid_checksum(klv_ctx_t *ctx);
/*
 * Global prototypes
 */
klv_ctx_t* libklv_init();
int libklv_parse_data(uint8_t *p_data, uint16_t pkt_len, klv_ctx_t *klv_ctx);
void libklv_cleanup(klv_ctx_t *ctx);

#endif // LIBKLV_H_INCLUDED
