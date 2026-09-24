/*
 * feetech.c - Userspace driver for Feetech STS bus servos over termios.
 */
#define _GNU_SOURCE
#include "feetech.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/* Pure protocol                                                       */
/* ------------------------------------------------------------------ */

uint8_t fts_checksum(const uint8_t *bytes, size_t n)
{
    unsigned sum = 0;
    for (size_t i = 0; i < n; i++)
        sum += bytes[i];
    return (uint8_t)(~sum & 0xFF);
}

size_t fts_build_packet(uint8_t *out, uint8_t id, uint8_t inst,
                        const uint8_t *params, size_t nparams)
{
    if (nparams + 6 > FTS_MAX_PKT)
        return 0;
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = id;
    out[3] = (uint8_t)(nparams + 2);
    out[4] = inst;
    if (nparams)
        memcpy(&out[5], params, nparams);
    out[5 + nparams] = fts_checksum(&out[2], nparams + 3);
    return nparams + 6;
}

int fts_parse_status(const uint8_t *buf, size_t len, fts_packet *pkt,
                     size_t *consumed)
{
    size_t i = 0;

    /* 1. Find header FF FF */
    while (i + 1 < len && !(buf[i] == 0xFF && buf[i + 1] == 0xFF))
        i++;
    if (i + 1 >= len) {          /* no header yet; keep a trailing 0xFF */
        *consumed = (len && buf[len - 1] == 0xFF) ? len - 1 : len;
        return 0;
    }
    /* 2. Need ID and LEN */
    if (len - i < 4) {
        *consumed = i;
        return 0;
    }
    uint8_t id = buf[i + 2];
    uint8_t plen = buf[i + 3];
    if (id == 0xFF || plen < 2 || plen > FTS_MAX_PKT - 4) {
        *consumed = i + 1;       /* bad header, resync one byte further */
        return FTS_ERR_BAD_PKT;
    }
    size_t total = (size_t)plen + 4;
    if (len - i < total) {
        *consumed = i;
        return 0;
    }
    /* 3. Verify checksum over ID..last data byte */
    if (fts_checksum(&buf[i + 2], total - 3) != buf[i + total - 1]) {
        *consumed = i + 2;
        return FTS_ERR_CHECKSUM;
    }
    pkt->id = id;
    pkt->error = buf[i + 4];
    pkt->data = &buf[i + 5];
    pkt->ndata = plen - 2;
    *consumed = i + total;
    return 1;
}

int fts_decode_sm(uint16_t raw, int sign_bit)
{
    uint16_t mask = (uint16_t)(1u << sign_bit);
    int mag = raw & (mask - 1);
    return (raw & mask) ? -mag : mag;
}

uint16_t fts_encode_sm(int value, int sign_bit)
{
    uint16_t mask = (uint16_t)(1u << sign_bit);
    if (value < 0)
        return (uint16_t)(((unsigned)(-value) & (mask - 1)) | mask);
    return (uint16_t)((unsigned)value & (mask - 1));
}

/* ------------------------------------------------------------------ */
/* Low-level I/O                                                       */
/* ------------------------------------------------------------------ */

static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 1000000: return B1000000;
    case 500000:  return B500000;
    case 115200:  return B115200;
    case 57600:   return B57600;
    case 38400:   return B38400;
    default:      return 0;
    }
}

int fts_open(fts_bus *bus, const char *path, int baud)
{
    memset(bus, 0, sizeof(*bus));
    bus->fd = -1;
    bus->timeout_ms = 10;

    speed_t spd = baud_to_speed(baud);
    if (!spd)
        return FTS_ERR_ARG;

    int fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0)
        return FTS_ERR_IO;

    /* Exclusive access: a second process (e.g. LeRobot) cannot open the bus */
    ioctl(fd, TIOCEXCL);

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) {
        close(fd);
        return FTS_ERR_IO;
    }
    cfmakeraw(&tio);                 /* 8N1, no echo, no translation */
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;              /* reads never block; we use poll() */
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        close(fd);
        return FTS_ERR_IO;
    }
    tcflush(fd, TCIOFLUSH);
    bus->fd = fd;
    return FTS_OK;
}

void fts_close(fts_bus *bus)
{
    if (bus->fd >= 0)
        close(bus->fd);
    bus->fd = -1;
}

static int write_all(int fd, const uint8_t *buf, size_t n)
{
    while (n) {
        ssize_t w = write(fd, buf, n);
        if (w < 0) {
            if (errno == EINTR)
                continue;
            return FTS_ERR_IO;
        }
        buf += w;
        n -= (size_t)w;
    }
    return FTS_OK;
}

/* Read whatever is available into rxbuf, waiting at most until deadline. */
static int fill_rx(fts_bus *bus, int64_t deadline)
{
    int64_t remaining = deadline - now_ms();
    if (remaining < 0)
        remaining = 0;
    struct pollfd pfd = { .fd = bus->fd, .events = POLLIN };
    int r = poll(&pfd, 1, (int)remaining);
    if (r < 0)
        return errno == EINTR ? 0 : FTS_ERR_IO;
    if (r == 0)
        return FTS_ERR_TIMEOUT;
    /* Adapter unplugged / pty closed: poll() reports HUP/ERR forever and
     * read() returns 0, which would otherwise spin until the deadline... or
     * forever. Treat it as an I/O error. */
    if (!(pfd.revents & POLLIN) && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)))
        return FTS_ERR_IO;
    if (bus->rxlen >= FTS_RXBUF_SIZE) {      /* should never happen */
        bus->stats.bytes_discarded += (uint32_t)bus->rxlen;
        bus->rxlen = 0;
    }
    ssize_t n = read(bus->fd, bus->rxbuf + bus->rxlen,
                     FTS_RXBUF_SIZE - bus->rxlen);
    if (n < 0)
        return errno == EINTR ? 0 : FTS_ERR_IO;
    if (n == 0)
        return FTS_ERR_IO;               /* EOF: device gone */
    bus->rxlen += (size_t)n;
    return FTS_OK;
}

static void drop_rx(fts_bus *bus, size_t n)
{
    if (n >= bus->rxlen) {
        bus->rxlen = 0;
        return;
    }
    memmove(bus->rxbuf, bus->rxbuf + n, bus->rxlen - n);
    bus->rxlen -= n;
}

/* Transmit a packet; flush stale input first and swallow echo if present. */
static int tx_packet(fts_bus *bus, const uint8_t *pkt, size_t len)
{
    tcflush(bus->fd, TCIFLUSH);
    bus->rxlen = 0;
    int rc = write_all(bus->fd, pkt, len);
    if (rc)
        return rc;
    bus->stats.tx_packets++;

    if (bus->echo) {                 /* discard exactly len echoed bytes */
        int64_t deadline = now_ms() + bus->timeout_ms;
        while (bus->rxlen < len) {
            rc = fill_rx(bus, deadline);
            if (rc == FTS_ERR_TIMEOUT)
                break;
            if (rc < 0)
                return rc;
        }
        drop_rx(bus, len);
    }
    return FTS_OK;
}

/* Receive one status packet from expect_id; copies up to cap data bytes. */
static int rx_status(fts_bus *bus, uint8_t expect_id, uint8_t *out,
                     size_t cap, size_t *nout)
{
    int64_t deadline = now_ms() + bus->timeout_ms;

    for (;;) {
        fts_packet pkt;
        size_t consumed = 0;
        int r = fts_parse_status(bus->rxbuf, bus->rxlen, &pkt, &consumed);

        if (r == 1) {
            /* consumed includes junk before the header */
            size_t junk = consumed - (pkt.ndata + 6);
            bus->stats.bytes_discarded += (uint32_t)junk;
            if (pkt.id != expect_id) {
                drop_rx(bus, consumed);
                return FTS_ERR_ID;
            }
            size_t n = pkt.ndata < cap ? pkt.ndata : cap;
            if (n && out)
                memcpy(out, pkt.data, n);
            if (nout)
                *nout = n;
            bus->last_servo_error = pkt.error;
            drop_rx(bus, consumed);
            bus->stats.rx_ok++;
            return FTS_OK;
        }
        if (r < 0) {
            if (r == FTS_ERR_CHECKSUM)
                bus->stats.checksum_errors++;
            bus->stats.bytes_discarded += (uint32_t)consumed;
            drop_rx(bus, consumed);
            continue;
        }
        /* r == 0: need more bytes (the deadline bounds garbage floods too) */
        if (now_ms() > deadline) {
            bus->stats.timeouts++;
            return FTS_ERR_TIMEOUT;
        }
        int rc = fill_rx(bus, deadline);
        if (rc == FTS_ERR_TIMEOUT) {
            bus->stats.timeouts++;
            return FTS_ERR_TIMEOUT;
        }
        if (rc < 0)
            return rc;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int fts_ping(fts_bus *bus, uint8_t id)
{
    uint8_t pkt[FTS_MAX_PKT];
    size_t n = fts_build_packet(pkt, id, FTS_INST_PING, NULL, 0);
    int rc = tx_packet(bus, pkt, n);
    if (rc)
        return rc;
    return rx_status(bus, id, NULL, 0, NULL);
}

int fts_read(fts_bus *bus, uint8_t id, uint8_t addr, uint8_t *out, uint8_t len)
{
    if (!len || !out)
        return FTS_ERR_ARG;
    uint8_t params[2] = { addr, len };
    uint8_t pkt[FTS_MAX_PKT];
    size_t n = fts_build_packet(pkt, id, FTS_INST_READ, params, 2);
    int rc = tx_packet(bus, pkt, n);
    if (rc)
        return rc;
    size_t got = 0;
    rc = rx_status(bus, id, out, len, &got);
    if (rc)
        return rc;
    return got == len ? FTS_OK : FTS_ERR_BAD_PKT;
}

int fts_write(fts_bus *bus, uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len)
{
    if (!len || !data || (size_t)len + 7 > FTS_MAX_PKT)
        return FTS_ERR_ARG;
    uint8_t params[FTS_MAX_PKT];
    params[0] = addr;
    memcpy(&params[1], data, len);
    uint8_t pkt[FTS_MAX_PKT];
    size_t n = fts_build_packet(pkt, id, FTS_INST_WRITE, params, (size_t)len + 1);
    int rc = tx_packet(bus, pkt, n);
    if (rc)
        return rc;
    if (id == FTS_BROADCAST_ID)
        return FTS_OK;
    return rx_status(bus, id, NULL, 0, NULL);
}

int fts_sync_read(fts_bus *bus, const uint8_t *ids, size_t n, uint8_t addr,
                  uint8_t len, uint8_t *out)
{
    if (!n || !len || !out || n + 8 > FTS_MAX_PKT)
        return FTS_ERR_ARG;
    uint8_t params[FTS_MAX_PKT];
    params[0] = addr;
    params[1] = len;
    memcpy(&params[2], ids, n);
    uint8_t pkt[FTS_MAX_PKT];
    size_t plen = fts_build_packet(pkt, FTS_BROADCAST_ID, FTS_INST_SYNC_READ,
                                   params, n + 2);
    int rc = tx_packet(bus, pkt, plen);
    if (rc)
        return rc;

    int first_err = FTS_OK;
    /* Servos answer one after another, in the order of ids[] */
    for (size_t k = 0; k < n; k++) {
        size_t got = 0;
        rc = rx_status(bus, ids[k], out + k * len, len, &got);
        if (rc == FTS_OK && got != len)
            rc = FTS_ERR_BAD_PKT;
        if (rc && !first_err)
            first_err = rc;
        if (rc == FTS_ERR_TIMEOUT)
            break;                    /* bus is silent, don't wait n timeouts */
    }
    return first_err;
}

int fts_sync_write(fts_bus *bus, const uint8_t *ids, size_t n, uint8_t addr,
                   uint8_t len, const uint8_t *data)
{
    size_t nparams = 2 + n * ((size_t)len + 1);
    if (!n || !len || !data || nparams + 6 > FTS_MAX_PKT)
        return FTS_ERR_ARG;
    uint8_t params[FTS_MAX_PKT];
    params[0] = addr;
    params[1] = len;
    size_t p = 2;
    for (size_t k = 0; k < n; k++) {
        params[p++] = ids[k];
        memcpy(&params[p], data + k * len, len);
        p += len;
    }
    uint8_t pkt[FTS_MAX_PKT];
    size_t plen = fts_build_packet(pkt, FTS_BROADCAST_ID, FTS_INST_SYNC_WRITE,
                                   params, nparams);
    return tx_packet(bus, pkt, plen);
}

int fts_raw_ping(fts_bus *bus, uint8_t id, uint8_t *tx, size_t *txlen,
                 uint8_t *raw, size_t cap, int wait_ms)
{
    *txlen = fts_build_packet(tx, id, FTS_INST_PING, NULL, 0);
    tcflush(bus->fd, TCIFLUSH);
    int rc = write_all(bus->fd, tx, *txlen);
    if (rc)
        return rc;
    size_t got = 0;
    int64_t deadline = now_ms() + wait_ms;
    while (got < cap) {
        int64_t rem = deadline - now_ms();
        if (rem <= 0)
            break;
        struct pollfd pfd = { .fd = bus->fd, .events = POLLIN };
        if (poll(&pfd, 1, (int)rem) <= 0)
            break;
        ssize_t r = read(bus->fd, raw + got, cap - got);
        if (r <= 0)
            break;
        got += (size_t)r;
    }
    return (int)got;
}

const char *fts_strerror(int code)
{
    switch (code) {
    case FTS_OK:           return "ok";
    case FTS_ERR_IO:       return "I/O error";
    case FTS_ERR_TIMEOUT:  return "timeout (no response)";
    case FTS_ERR_CHECKSUM: return "checksum error";
    case FTS_ERR_BAD_PKT:  return "malformed packet";
    case FTS_ERR_ARG:      return "invalid argument";
    case FTS_ERR_ID:       return "response from unexpected ID";
    default:               return "unknown error";
    }
}
