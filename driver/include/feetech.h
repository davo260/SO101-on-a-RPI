/*
 * feetech.h - Userspace driver for Feetech STS-series bus servos (STS3215)
 * over a half-duplex TTL bus exposed as a Linux tty (termios).
 *
 * Protocol (Feetech "protocol 0", little endian on STS series):
 *   Instruction: FF FF ID LEN INST P0..Pn CHK     LEN = n_params + 2
 *   Status:      FF FF ID LEN ERR  D0..Dn CHK     CHK = ~(ID+LEN+..+last) & 0xFF
 */
#ifndef FEETECH_H
#define FEETECH_H

#include <stddef.h>
#include <stdint.h>

#define FTS_BROADCAST_ID 0xFE
#define FTS_MAX_PKT      250
#define FTS_RXBUF_SIZE   1024

/* Instructions */
enum {
    FTS_INST_PING       = 0x01,
    FTS_INST_READ       = 0x02,
    FTS_INST_WRITE      = 0x03,
    FTS_INST_REG_WRITE  = 0x04,
    FTS_INST_ACTION     = 0x05,
    FTS_INST_SYNC_READ  = 0x82,
    FTS_INST_SYNC_WRITE = 0x83,
};

/* STS3215 control table (subset). Addresses match LeRobot's tables.py */
enum {
    STS_ID                  = 5,
    STS_BAUD_RATE           = 6,
    STS_RETURN_DELAY        = 7,
    STS_MIN_POS_LIMIT       = 9,   /* 2 bytes */
    STS_MAX_POS_LIMIT       = 11,  /* 2 bytes */
    STS_HOMING_OFFSET       = 31,  /* 2 bytes, sign bit 11 */
    STS_OPERATING_MODE      = 33,
    STS_TORQUE_ENABLE       = 40,
    STS_ACCELERATION        = 41,
    STS_GOAL_POSITION       = 42,  /* 2 bytes, sign bit 15 */
    STS_GOAL_TIME           = 44,
    STS_GOAL_VELOCITY       = 46,  /* 2 bytes, sign bit 15 */
    STS_TORQUE_LIMIT        = 48,
    STS_LOCK                = 55,
    STS_PRESENT_POSITION    = 56,  /* 2 bytes, sign bit 15 */
    STS_PRESENT_VELOCITY    = 58,  /* 2 bytes, sign bit 15 */
    STS_PRESENT_LOAD        = 60,  /* 2 bytes, sign bit 10 */
    STS_PRESENT_VOLTAGE     = 62,  /* 1 byte, units of 0.1 V */
    STS_PRESENT_TEMPERATURE = 63,  /* 1 byte, deg C */
    STS_STATUS              = 65,
    STS_MOVING              = 66,
    STS_PRESENT_CURRENT     = 69,  /* 2 bytes */
};

/* Return codes (negative = error) */
enum {
    FTS_OK           = 0,
    FTS_ERR_IO       = -1,
    FTS_ERR_TIMEOUT  = -2,
    FTS_ERR_CHECKSUM = -3,
    FTS_ERR_BAD_PKT  = -4,
    FTS_ERR_ARG      = -5,
    FTS_ERR_ID       = -6,  /* response from unexpected servo */
};

typedef struct {
    uint8_t        id;
    uint8_t        error;  /* servo error/status byte */
    const uint8_t *data;   /* points into caller's buffer */
    size_t         ndata;
} fts_packet;

typedef struct {
    uint32_t tx_packets;
    uint32_t rx_ok;
    uint32_t timeouts;
    uint32_t checksum_errors;
    uint32_t bytes_discarded;
} fts_stats;

typedef struct {
    int       fd;
    int       timeout_ms;   /* timeout per expected status packet */
    int       echo;         /* 1 if the adapter echoes TX bytes back on RX */
    uint8_t   last_servo_error;
    fts_stats stats;
    uint8_t   rxbuf[FTS_RXBUF_SIZE];
    size_t    rxlen;
} fts_bus;

/* ---- Pure protocol functions (no I/O, unit-testable) ---- */
uint8_t fts_checksum(const uint8_t *bytes, size_t n);
size_t  fts_build_packet(uint8_t *out, uint8_t id, uint8_t inst,
                         const uint8_t *params, size_t nparams);
/* Scans buf for a status packet. Always sets *consumed (bytes the caller may
 * drop). Returns 1 = packet found (pkt filled), 0 = need more bytes,
 * FTS_ERR_CHECKSUM / FTS_ERR_BAD_PKT = corrupt data (drop *consumed, retry). */
int     fts_parse_status(const uint8_t *buf, size_t len, fts_packet *pkt,
                         size_t *consumed);
int      fts_decode_sm(uint16_t raw, int sign_bit);   /* sign-magnitude -> int */
uint16_t fts_encode_sm(int value, int sign_bit);      /* int -> sign-magnitude */

/* ---- Bus I/O ---- */
int  fts_open(fts_bus *bus, const char *path, int baud);
void fts_close(fts_bus *bus);
int  fts_ping(fts_bus *bus, uint8_t id);
int  fts_read(fts_bus *bus, uint8_t id, uint8_t addr, uint8_t *out, uint8_t len);
int  fts_write(fts_bus *bus, uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len);
/* out must hold n*len bytes, filled in the order of ids[] */
int  fts_sync_read(fts_bus *bus, const uint8_t *ids, size_t n, uint8_t addr,
                   uint8_t len, uint8_t *out);
/* data holds n*len bytes, in the order of ids[]. No response expected. */
int  fts_sync_write(fts_bus *bus, const uint8_t *ids, size_t n, uint8_t addr,
                    uint8_t len, const uint8_t *data);
/* Sends a ping and dumps every raw byte received into raw (up to cap).
 * Used to detect whether the adapter echoes TX bytes. Returns bytes received. */
int  fts_raw_ping(fts_bus *bus, uint8_t id, uint8_t *tx, size_t *txlen,
                  uint8_t *raw, size_t cap, int wait_ms);

const char *fts_strerror(int code);

#endif /* FEETECH_H */
