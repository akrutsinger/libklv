#include "libklv.h"

/*****************************************************************************
 * libklv_read
 *****************************************************************************/
static inline size_t libklv_read(uint8_t *dst, klv_ctx_t *src, size_t len)
{
    size_t i = 0;
    for (i = 0; i < len; i++)
        dst[i] = *src->buf_ptr++;
    return i;
}

/*****************************************************************************
 * libklv_readUINT64
 *****************************************************************************/
static inline uint64_t libklv_readUINT64(klv_ctx_t *p)
{
    uint64_t ret = 0;
    ret = *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    return ret;
}

/*****************************************************************************
 * libklv_readUINT32
 *****************************************************************************/
static inline uint32_t libklv_readUINT32(klv_ctx_t *p)
{
    uint32_t ret = 0;
    ret = *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    return ret;
}

/*****************************************************************************
 * libklv_readINT32
 *****************************************************************************/
static inline int32_t libklv_readINT32(klv_ctx_t *p)
{
    int32_t ret = 0;
    ret = *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    return ret;
}

/*****************************************************************************
 * libklv_readUINT16
 *****************************************************************************/
static inline uint16_t libklv_readUINT16(klv_ctx_t *p)
{
    uint16_t ret = 0;
    ret = *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    return ret;
}

/*****************************************************************************
 * libklv_readINT16
 *****************************************************************************/
static inline int16_t libklv_readINT16(klv_ctx_t *p)
{
    int16_t ret = 0;
    ret = *p->buf_ptr++;
    ret = (ret << 8) | *p->buf_ptr++;
    return ret;
}

/*****************************************************************************
 * libklv_readUINT8
 *****************************************************************************/
static inline uint8_t libklv_readUINT8(klv_ctx_t *p)
{
    return *p->buf_ptr++;
}

/*****************************************************************************
 * libklv_readINT8
 *****************************************************************************/
static inline int8_t libklv_readINT8(klv_ctx_t *p)
{
    return *p->buf_ptr++;
}

/*****************************************************************************
 * map_val
 *****************************************************************************/
static double libklv_map_val(double value, double a, double b, double targetA, double targetB)
{
    double t = (value - a)/(b-a);
    return (targetA + (t*(targetB-targetA)));
}

/*****************************************************************************
 * libklv_strdup
 *****************************************************************************/
static char *libklv_strdup(klv_ctx_t *src, uint8_t len)
{
    char *tmp = (char *)malloc(len+1);  /* allocate extra byte for null-terminator */
    for (int i = 0; i < len; i++)
        tmp[i] = *src->buf_ptr++;
    tmp[len] = '\0';
    return tmp;
}

/*****************************************************************************
 * decode_klv_values
 *****************************************************************************/
static int decode_klv_values(klv_item_t *item, klv_ctx_t *klv_ctx)
{
    uint8_t i;
    uint8_t stn_num, substn_num, wpn_type, wpn_var;

    switch (item->id)
    {
        case 0x01:  /* misb std 0601 checksum */
            item->value = libklv_readUINT16(klv_ctx);
            klv_ctx->checksum = item->value;
            printf("  0x%02X (checksum):\t%d\n", item->id, item->value);
            break;
        case 0x02:  /* unix time stamp */
            /* microseconds since 00:00:00:00, January 1st 1970 */
            item->value = libklv_readUINT64(klv_ctx);
            printf("  0x%02X (unix epoch):\t%llu\n", item->id, item->value);
            break;
        case 0x03:  /* mission id */
            item->data = libklv_strdup(klv_ctx, item->len);
            printf("  0x%02X (mission id):\t%s", item->id, item->data);
            break;
        case 0x04:  /* platform tail number */
            item->data = libklv_strdup(klv_ctx, item->len);
            printf("  0x%02X (tail number):\t%s", item->id, item->data);
            break;
        case 0x05:  /* platform heading angle */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..360 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 360.0);
            printf("  0x%02X (platform heading ang):\t%.15f\n", item->id, item->mapped_val);
            break;
        case 0x06:  /* platform pitch angle */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-20 */
             item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -20.0, 20.0);
            printf("  0x%02X (platform pitch angle):\t%.12f\n", item->id,  item->mapped_val);
            break;
        case 0x07:  /* platform roll angle */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-50 */
             item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -50.0, 50.0);
            printf("  0x%02X (platform roll angle):\t%.12f\n", item->id,  item->mapped_val);
            break;
        case 0x08:  /* platform true airspeed */
            /* 0..255 */
            item->value = libklv_readUINT8(klv_ctx);
            printf("  0x%02X (platform roll angle):\t%d", item->id, item->value);
            break;
        case 0x09:  /* platform indicated airspeed */
            /* 0..255 */
            item->value = libklv_readUINT8(klv_ctx);
            printf("  0x%02X (platform indicated speed):\t%d\n", item->id, item->value);
            break;
        case 0x0A:  /* platform designation */
            item->data = libklv_strdup(klv_ctx, item->len);
            printf("  0x%02X (platform designation):\t%s\n", item->id, item->data);
            break;
        case 0x0B:  /* image source sensor */
            item->data = libklv_strdup(klv_ctx, item->len);
            printf("  0x%02X (image source sensor):\t%s\n", item->id, item->data);
            break;
        case 0x0C:  /* image coordinate system */
            item->data = libklv_strdup(klv_ctx, item->len);
            printf("  0x%02X (coordinate system):\t%s\n", item->id, item->data);
            break;
        case 0x0D:  /* sensor latitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            printf("  0x%02X (sensor latitude):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x0E:  /* sensor longitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            printf("  0x%02X (sensor longitude):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x0F:  /* sensor true altitude */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19,000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (sensor true altitude):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x10:  /* sensor horizontal field of view */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..180 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 180.0);
            printf("  0x%02X (sensor horizontal FOV):\t%.15f\n", item->id, item->mapped_val);
            break;
        case 0x11:  /* sensor vertical field of view */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..180 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 180.0);
            printf("  0x%02X (sensor vertical FOV):\t%.15f\n", item->id, item->mapped_val);
            break;
        case 0x12:  /* sensor relative azimuth angle */
            item->value = libklv_readUINT32(klv_ctx);
            /* Map 0..(2^32-1) to 0..360 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 4294967295.0, 0.0, 360.0);
            printf("  0x%02X (sensor rel az ang):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x13:  /* sensor relative elevation angle */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            printf("  0x%02X (sensor rel elev ang):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x14:  /* sensor relative roll angle */
            item->value = libklv_readUINT32(klv_ctx);
            /* Map 0..(2^32-1) to 0..360 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 4294967295.0, -180.0, 180.0);
            printf("  0x%02X (sensor rel roll ang):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x15:  /* slant range */
            item->value = libklv_readUINT32(klv_ctx);
            /* Map 0..(2^32-1) to 0..5000,000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 4294967295.0, 0.0, 5000000.0);
            printf("  0x%02X (slant range):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x16:  /* target width */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..10000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 10000.0);
            printf("  0x%02X (target width):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x17:  /* frame center latitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            printf("  0x%02X (frame center lat):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x18:  /* frame center longitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            printf("  0x%02X (frame center lon):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x19:  /* frame center elevation */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19,000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (frame center elev):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x1A:  /* offset corner lat pt.1 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lat 1):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x1B:  /* offset corner lon pt.1 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lon 1):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x1C:  /* offset corner lat pt.2 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lat 2):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x1D:  /* offset corner lon pt.2 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lon 2):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x1E:  /* offset corner lat pt.3 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lat 3):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x1F:  /* offset corner lon pt.3 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lon 3):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x20:  /* offset corner lat pt.4 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lat 4):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x21:  /* offset corner lon pt.4 */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-0.075 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -0.075, 0.075);
            printf("  0x%02X (offset crnr lon 4):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x22:  /* icing detected */
            printf("  0x%02X (icing detected):\t", item->id);
            item->value = libklv_readUINT8(klv_ctx);
            switch (item->value)
            {
                case 0:
                    item->data = strdup("detector off", item->len);
                    break;
                case 1:
                    item->data = strdup("no icing detected", item->len);
                    break;
                case 2:
                    item->data = strdup("icing detected", item->len);
                    break;
                default:
                    item->data = strdup("unsupported value", item->len);
            }
            break;
        case 0x23:  /* wind direction */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..360*/
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 360.0);
            printf("  0x%02X (wind direction):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x24:  /* wind speed */
            item->value = libklv_readUINT8(klv_ctx);
            /* Map 0..255 to 0..100*/
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 360.0);
            printf("  0x%02X (wind speed):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x25:  /* static pressure */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..5,000 mbar*/
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 5000.0);
            printf("  0x%02X (static pressure (mbar)):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x26:  /* density altitude */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19000*/
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (density altitude):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x27:  /* outside air temperature */
            item->signed_val = libklv_readINT8(klv_ctx);
            printf("  0x%02X (outside air temp):\t%d", item->id, item->signed_val);
            break;
        case 0x28:  /* target location latitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            printf("  0x%02X (target location lat):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x29:  /* target location longitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            printf("  0x%02X (target location lon):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x2A:  /* target location elevation */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (target location elev):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x2B:  /* target track gate width */
            item->value = libklv_readUINT8(klv_ctx);
            printf("  0x%02X (target track gate wdth):\t%.14f\n", item->id, item->value);
            break;
        case 0x2C:  /* target track gate height */
            item->value = libklv_readUINT8(klv_ctx);
            printf("  0x%02X (target track gate ht.):\t%.14f\n", item->id, item->value);
            break;
        case 0x2D:  /* target error estimate CE90 */
            item->value = libklv_readUINT16(klv_ctx);
            printf("  0x%02X (target err est. CE90):\t%.14f\n", item->id, item->value);
            break;
        case 0x2E:  /* target error estimate LE90 */
            item->value = libklv_readUINT16(klv_ctx);
            printf("  0x%02X (target err est. LE90):\t%.14f\n", item->id, item->value);
            break;
        case 0x2F:  /* generic flag data 01 */
            item->value = libklv_readUINT8(klv_ctx);
            printf("  0x%02X (generic flag data 01):\tLaser Range: %d\n", item->id, (item->value & 0x01));
            printf("                                     Auto-Track: %d\n", (item->value & 0x02));
            printf("                                     IR Polarity: %d\n", (item->value & 0x04));
            printf("                                     Icing detected: %d\n", (item->value & 0x08));
            printf("                                     Slant Range: %d\n", (item->value & 0x10));
            printf("                                     Image Invalid: %d\n", (item->value & 0x20));
            break;
        case 0x30:  /* security local metadata set */
            item->data = malloc(item->len);
            libklv_read(item->data, klv_ctx, item->len);
            printf("  0x%02X (sec local metadata set):\tset size=%d\n", item->id, item->len);
            break;
        case 0x31:  /* differential pressure */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..-(2^16-1) to 0..5000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 5000.0);
            printf("  0x%02X (differential pressure):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x32:  /* platform angle of attack */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-20 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -20.0, 20.0);
            printf("  0x%02X (platform angle of atk):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x33:  /* platform vertical speed */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -180.0, 180.0);
            printf("  0x%02X (platform vert speed):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x34:  /* platform slideslip angle */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to +/-20 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -20.0, 20.0);
            printf("  0x%02X (platform slideslip ang):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x35:  /* airfieled barometric pressure */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..5,000 mbar*/
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 5000.0);
            printf("  0x%02X (airfield pressure):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x36:  /* airfieled elevation */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19,000 mbar*/
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (airfield elev):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x37:  /* relative humidity */
            item->value = libklv_readUINT8(klv_ctx);
            /* Map 0..(2^8-1) to 0..100 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 255.0, 0.0, 100.0);
            printf("  0x%02X (relative humidity):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x38:  /* platform ground speed */
            /* 0..255 */
            item->value = libklv_readUINT8(klv_ctx);
            printf("  0x%02X (platform gnd speed):\t%d\n", item->id, item->value);
            break;
        case 0x39:  /* ground range */
            item->value = libklv_readUINT32(klv_ctx);
            /* Map 0..(2^32-1) to 0..5000000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 4294967295.0, 0.0, 5000000.0);
            printf("  0x%02X (ground range):\t%d\n", item->id, item->mapped_val);
            break;
        case 0x3A:  /* platform fuel remaining */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..10000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 10000.0);
            printf("  0x%02X (platform fuel remain):\t%d\n", item->id, item->mapped_val);
            break;
        case 0x3B:  /* platform call sign */
            item->data = libklv_strdup(klv_ctx, item->len);
            printf("  0x%02X (platform call sign):\t%s\n", item->id, item->data);
            break;
        case 0x3C:  /* weapon load */
            item->value = libklv_readUINT16(klv_ctx);
            /* TODO: figure out weapon variants and type names */
            /*       byte 1           byte 2
             * +--------+--------+--------+--------+
             * |  nib1  |  nib2  |  nib1  |  nib2  |
             * +--------+--------+--------+--------+
             * 32                16                1
             *
             * byte1 - nib1 = Station number
             * byte1 - nib2 = Substation Number
             * byte2 - nib1 = Weapon type
             * byte2 - nib2 = Weapon variant
             */
            stn_num = (item->value >> 8) - (item->value >> 12);
            substn_num = (item->value >> 8) - ((item->value >> 8) & 0x0F);
            wpn_type = (item->value & 0x00FF) - ((item->value >> 4) & 0x0F);
            wpn_var = (item->value & 0x00FF) - (item->value & 0x000F);
            printf("  0x%02X (weapon load):\t%d|%d|%d|%d\n", item->id, stn_num, substn_num, wpn_type, wpn_var);
            break;
        case 0x3D:  /* weapon fired */
            item->value = libklv_readUINT8(klv_ctx);
            /* +--------+--------+
             * |  STN # |SUBSTN #|
             * +--------+--------+
             * 8                 1
             */
            stn_num = (item->value) - ((item->value >> 4) & 0x0F);
            substn_num = (item->value) - (item->value & 0x0F);
            printf("  0x%02X (weapon fired):\t%d|%d\n", item->id, stn_num, substn_num);
            break;
        case 0x3E:  /* laser prf code */
            item->value = libklv_readUINT16(klv_ctx);
            printf("  0x%02X (laser prf code):\t%d\n", item->id, item->value);
            break;
        case 0x3F:  /* sensor field of view name */
            item->value = libklv_readUINT8(klv_ctx);
            switch (item->value)
            {
                case 0:
                    item->data = strdup("Ultranarrow", item->len);
                    break;
                case 1:
                    item->data = strdup("Narrow", item->len);
                    break;
                case 2:
                    item->data = strdup("Medium", item->len);
                    break;
                case 3:
                    item->data = strdup("Wide", item->len);
                    break;
                case 4:
                    item->data = strdup("Ultrawide", item->len);
                    break;
                case 5:
                    item->data = strdup("Narrow Medium", item->len);
                    break;
                case 6:
                    item->data = strdup("2x Ultranarrow", item->len);
                    break;
                case 7:
                    item->data = strdup("4x Ultranarrow", item->len);
                    break;
            }
            printf("  0x%02X (sensor fov name):\t%s", item->id, item->data);
            break;
        case 0x40:  /* platform magnetic heading angle */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..360 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 360.0);
            printf("  0x%02X (platform mag head ang):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x41:  /* uas ls version number */
            item->value = libklv_readUINT8(klv_ctx);
            printf("  0x%02X (uas ls version num):\tST0601.%d\n", item->id, item->value);
            break;
        case 0x42:  /* target location covariance matrix */
            /* TODO: implement in the future. According to ST0601.8 this field is TBD */
            break;
        case 0x43:  /* alternate platform latitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* LAT: Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            printf("  0x%02X (alt platfm latitude):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x44:  /* alternate platform longitude */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* LAT: Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            printf("  0x%02X (alt platfm longitude):\t%.14f\n", item->id, item->mapped_val);
            break;
        case 0x45:  /* alternate platform altitude */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19,000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (alt altitude):\t%.12f\n", item->id, item->mapped_val);
            break;
        case 0x46:  /* alternate platform name */
            item->data = libklv_strdup(klv_ctx, item->len);
            printf("  0x%02X (alt pltfrm name):\t%s", item->id, item->data);
            break;
        case 0x47:  /* alternate platform heading */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to 0..360 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, 0.0, 360.0);
            printf("  0x%02X (platform heading ang):\t%.15f\n", item->id, item->mapped_val);
            break;
        case 0x48:  /* event start time */
            item->value = libklv_readUINT64(klv_ctx);
            printf("  0x%02X (event start time):\t%llu\n", item->id, item->value);
            break;
        case 0x49:  /* rvt local set */
            item->data = malloc(item->len);
            libklv_read(item->data, klv_ctx, item->len);
            break;
        case 0x4A:  /* vmti data set */
            item->data = malloc(item->len);
            libklv_read(item->data, klv_ctx, item->len);
            break;
        case 0x4B:  /* sensor ellipsoid height */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (sensor ellip ht.):\t%.15f\n", item->id, item->mapped_val);
            break;
        case 0x4C:  /* alternate platform ellipsoid height */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            printf("  0x%02X (alt pltfm elpsd ht.):\t%.15f\n", item->id, item->mapped_val);
            break;
        case 0x4D:  /* operational mode */
            item->value = libklv_readUINT8(klv_ctx);
            switch (item->value)
            {
                case 0x00:
                    item->data = strdup("Other", item->len);
                    break;
                case 0x01:
                    item->data = strdup("Operational", item->len);
                    break;
                case 0x02:
                    item->data = strdup("Training", item->len);
                    break;
                case 0x03:
                    item->data = strdup("Exercise", item->len);
                    break;
                case 0x04:
                    item->data = strdup("Maintenance", item->len);
                    break;
                case 0x05:
                    item->data = strdup("Test", item->len);
                    break;
            }
            printf("  0x%02X (operational mode):\t%s", item->id, item->data);
            break;
        case 0x4E:  /* frame center height above ellipsoid */
            item->value = libklv_readUINT16(klv_ctx);
            /* Map 0..(2^16-1) to -900..19000 */
            item->mapped_val = libklv_map_val((double)item->value, 0.0, 65535.0, -900.0, 19000.0);
            break;
        case 0x4F:  /* sensor north velocity */
            item->signed_val = libklv_readUINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to -327..327 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -327.0, 327.0);
            break;
        case 0x50:  /* sensor east velocity */
            item->signed_val = libklv_readINT16(klv_ctx);
            /* Map -(2^15-1)..(2^15-1) to -327..327 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -32767.0, 32767.0, -327.0, 327.0);
            break;
        case 0x51:  /* image horizon pixel pack */
            /* TODO: implement decoding this */
            break;
        case 0x52:  /* corner latitude point 1 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x53:  /* corner longitude point 1 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            break;
        case 0x54:  /* corner latitude point 2 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x55:  /* corner longtitude point 2 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            break;
        case 0x56:  /* corner latitude point 3 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x57:  /* corner longitude point 3 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            break;
        case 0x58:  /* corner latitude point 4 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x59:  /* corner longitude point 4 (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-180 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -180.0, 180.0);
            break;
        case 0x5A:  /* platform pitch angle (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x5B:  /* platform roll angle (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x5C:  /* platform angle of attack (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x5D:  /* platform sideslip angle (full) */
            item->signed_val = libklv_readINT32(klv_ctx);
            /* Map -(2^31-1)..(2^31-1) to +/-90 */
            item->mapped_val = libklv_map_val((double)item->signed_val, -2147483647.0, 2147483647.0, -90.0, 90.0);
            break;
        case 0x5E:  /* miis core item->identifier */
            /* TODO: implement based off of ST 1204 standards document (http://www.gwg.nga.mil/) */
            break;
        case 0x5F:  /* sar motion imagery metadata */
            /* TODO: implement based off of ST 1206 standards document (http://www.gwg.nga.mil/) */
            break;
        case 0x60:  /* target width extended */
            /* According to ST1601.9 the conversion formula is in MISB ST 1201. */
            /* TODO: implement appropriate conversion */
            break;
        default:
            printf("  KEY NOT HANDLED: %02X", item->id);
            printf("\n");
            break;
    }

    /* TODO: what makes the most sense for the return value? length or id
    /* return the start of the next tag */
    return item->len;
}

/*****************************************************************************
 * libklv_get_ber_length
 *****************************************************************************/
static uint64_t klv_get_ber_length(klv_ctx_t *p)
{
    uint64_t size = *p->buf_ptr++;
    if (size & 0x80) { /* long form */
        int bytes_num = size & 0x7f;
        /* SMPTE 379M 5.3.4 guarantee that bytes_num must not exceed 8 bytes */
        if (bytes_num > 8)
            return 0xFF;
        size = 0;
        while (bytes_num--)
            size = size << 8 | *p->buf_ptr++;
    }
    return size;
}

/*****************************************************************************
 * sync_to_klv_key
 *****************************************************************************/
static int sync_to_klv_key(uint8_t *p, uint16_t len, klv_ctx_t *klv_ctx)
{
    for (int i = 0; i < klv_ctx->buffer_size; i++) {
        if (klv_ctx->buffer[i] == klv_universal_key[0]) {
            /* check for 16-byte KLV unique identifier */
            if (memcmp(klv_ctx->buffer + i, &klv_universal_key, 16) == 0) {
                klv_ctx->buf_ptr += (i + 16);   /* start of key uid + 16-byte key uid length */
                return i;   /* return offset to start of klv universal local set key */
            }
        }
    }

    return -1;
}

/*****************************************************************************
 * delete_klv_item_list
 *****************************************************************************/
static void delete_klv_item_list(klv_item_t *items)
{
    klv_item_t *p_tmp_item = NULL;  /* temporary pointer for iterating through the klv items list */
    struct list_head *pos, *q;	/* used to keep position while iterating through list in list_for_each() */

   	list_for_each_safe(pos, q, &items->list) {
		p_tmp_item = list_entry(pos, klv_item_t, list);
		if (p_tmp_item->data != NULL)
            free(p_tmp_item->data); /* free any allocated data; typically strings */
		list_del(pos);      /* remove from list */
		free(p_tmp_item);   /* free memory allocation */
	}
}

/*****************************************************************************
 * libklv_cleanup
 *****************************************************************************/
static bool has_valid_checksum(klv_ctx_t *ctx)
{
    uint8_t i;
    uint16_t bcc = 0;
    /* sum each 16-bit chunk within the buffer into a checksum */
    for (i = 0; i < ctx->payload_len - 2; i++) {
        bcc += ctx->buffer[i] << (8 * ((i + 1) % 2));
    }
    return (bcc == ctx->checksum) ? true : false;
}

/*****************************************************************************
 * libklv_update_ctx_buffer
 *****************************************************************************/
int libklv_update_ctx_buffer(klv_ctx_t *ctx, void *src, size_t len)
{
    if (ctx->buffer != NULL)
        /* TODO: reallocate buffer instead of just freeing it */
        free(ctx->buffer);

    ctx->buffer = malloc(len);

    memcpy(ctx->buffer, src, len);

    ctx->buffer_size = len;
    ctx->buf_end = ctx->buffer + ctx->buffer_size;
    ctx->buf_ptr = ctx->buffer;

    return 0;
}

/*****************************************************************************
 * libklv_init
 *****************************************************************************/
klv_ctx_t* libklv_init()
{
    klv_ctx_t *ctx = (klv_ctx_t *)malloc(sizeof(klv_ctx_t));    /* create context on heap */

    INIT_LIST_HEAD(&ctx->klv_items.list);      /* initialize the items list */

    return ctx;
}

/*****************************************************************************
 * libklv_cleanup
 *****************************************************************************/
void libklv_cleanup(klv_ctx_t *ctx)
{
    if (ctx != NULL) {
        delete_klv_item_list(&ctx->klv_items);
        free(ctx);
    }
}

/*****************************************************************************
 * libklv_parse_data
 *****************************************************************************/
int libklv_parse_data(uint8_t *p_data, uint16_t pkt_len, klv_ctx_t *klv_ctx)
{
    klv_item_t *p_tmp_item = NULL;  /* temporary pointer for iterating through the klv items list */
    uint8_t bytes_read;


    /* we don't want multiple lists allocated, but we want the user to have access after a packet is parsed */
	delete_klv_item_list(&klv_ctx->klv_items);

    uint16_t   ts_pid = ((uint16_t)(klv_ctx->buffer[1] & 0x1f) << 8) + klv_ctx->buffer[2];
    int        ts_cc = (klv_ctx->buffer[3] & 0x0f);

    /* sync klv context with the start of klv key within the metadata packet */
    if (sync_to_klv_key(NULL, 0, klv_ctx) < 0) {
//        printf("[!] KLV key not found within stream %d\n", ts_pid);
        return -1;
    }

    printf("\n[+] pid %u (0x%04x), cc %d\n", ts_pid, ts_pid, ts_cc);

    /* packet length follows 16-byte uid */
    /* TODO: confirm this correctly reads for a short BER packet length */
    klv_ctx->payload_len = klv_get_ber_length(klv_ctx);

    /* print header */
    printf("  KEY                               VALUE\n");
    printf("  ---------------------------------------\n");

    /* iterate through the payload and decode the fields */
    while (klv_ctx->buf_ptr < klv_ctx->buf_end) {
        p_tmp_item = (klv_item_t *)malloc(sizeof(klv_item_t));
        memset(p_tmp_item, 0, sizeof(klv_item_t));

        p_tmp_item->id = *klv_ctx->buf_ptr++;
        p_tmp_item->len = klv_get_ber_length(klv_ctx);

        bytes_read = decode_klv_values(p_tmp_item, klv_ctx);

        list_add_tail(&p_tmp_item->list, &klv_ctx->klv_items.list);
    }

    /* calculate checksum. According to ST0601.1, packet should be discarded if calculated checksum doesn't match embedded value */
    /* TODO: calculate checksum */
    if (has_valid_checksum(klv_ctx) == false)
        printf("Invalid checksum\n");
}
