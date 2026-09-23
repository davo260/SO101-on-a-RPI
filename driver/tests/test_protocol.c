/* Unit tests for the pure protocol layer (no hardware needed). */
#include "feetech.h"

#include <stdio.h>
#include <string.h>

static int failures = 0, checks = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        checks++;                                                     \
        if (!(cond)) {                                                \
            failures++;                                               \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, msg);      \
        }                                                             \
    } while (0)

static void test_build_ping(void)
{
    uint8_t out[16];
    size_t n = fts_build_packet(out, 1, FTS_INST_PING, NULL, 0);
    const uint8_t exp[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
    CHECK(n == sizeof exp && !memcmp(out, exp, n), "ping packet ID1");
}

static void test_build_read(void)
{
    uint8_t out[16], p[2] = {STS_PRESENT_POSITION, 2};
    size_t n = fts_build_packet(out, 1, FTS_INST_READ, p, 2);
    /* ~(0x01+0x04+0x02+0x38+0x02) = ~0x41 = 0xBE */
    const uint8_t exp[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
    CHECK(n == sizeof exp && !memcmp(out, exp, n), "read present position ID1");
}

static void test_build_sync_read(void)
{
    uint8_t out[32], p[] = {STS_PRESENT_POSITION, 2, 1, 2, 3};
    size_t n = fts_build_packet(out, FTS_BROADCAST_ID, FTS_INST_SYNC_READ, p, sizeof p);
    CHECK(n == 11, "sync read length");
    CHECK(out[2] == 0xFE && out[3] == 7 && out[4] == 0x82, "sync read header");
    CHECK(out[10] == fts_checksum(&out[2], 8), "sync read checksum");
}

static void test_parse_valid(void)
{
    /* Status from ID 3, no error, position = 0x0800 (2048) */
    uint8_t pkt[] = {0xFF, 0xFF, 0x03, 0x04, 0x00, 0x00, 0x08, 0x00};
    pkt[7] = fts_checksum(&pkt[2], 5);
    fts_packet s;
    size_t used;
    int r = fts_parse_status(pkt, sizeof pkt, &s, &used);
    CHECK(r == 1, "valid packet parsed");
    CHECK(used == sizeof pkt, "consumed whole packet");
    CHECK(s.id == 3 && s.error == 0 && s.ndata == 2, "fields");
    CHECK((s.data[0] | s.data[1] << 8) == 2048, "payload");
}

static void test_parse_junk_prefix(void)
{
    uint8_t buf[] = {0x12, 0x00, 0xFF, 0xFF, 0xFF, 0x01, 0x02, 0x00, 0x00};
    buf[8] = fts_checksum(&buf[5], 3);
    fts_packet s;
    size_t used;
    int r;
    size_t off = 0, total = sizeof buf;
    /* Driver loop: drop 'used' on errors, retry */
    do {
        r = fts_parse_status(buf + off, total - off, &s, &used);
        off += used;
    } while (r < 0);
    CHECK(r == 1 && s.id == 1, "packet found after junk and extra 0xFF");
    CHECK(off == total, "all bytes consumed");
}

static void test_parse_partial(void)
{
    uint8_t pkt[] = {0xFF, 0xFF, 0x02, 0x02, 0x00, 0x00};
    pkt[5] = fts_checksum(&pkt[2], 3);
    fts_packet s;
    size_t used;
    for (size_t cut = 0; cut < sizeof pkt; cut++) {
        int r = fts_parse_status(pkt, cut, &s, &used);
        CHECK(r == 0, "partial packet -> need more");
        CHECK(used == 0 || (cut == 1 && used == 0), "partial keeps header bytes");
    }
    CHECK(fts_parse_status(pkt, sizeof pkt, &s, &used) == 1, "complete packet");
}

static void test_parse_bad_checksum(void)
{
    uint8_t pkt[] = {0xFF, 0xFF, 0x02, 0x02, 0x00, 0x42};
    fts_packet s;
    size_t used;
    int r = fts_parse_status(pkt, sizeof pkt, &s, &used);
    CHECK(r == FTS_ERR_CHECKSUM, "checksum error detected");
    CHECK(used == 2, "drops header to resync");
}

static void test_sign_magnitude(void)
{
    CHECK(fts_decode_sm(0x0005, 15) == 5, "positive");
    CHECK(fts_decode_sm(0x8005, 15) == -5, "negative bit 15");
    CHECK(fts_decode_sm(0x0405, 10) == -5, "negative bit 10 (load)");
    CHECK(fts_decode_sm(0x0805, 11) == -5, "negative bit 11 (homing offset)");
    for (int v = -2047; v <= 2047; v += 97)
        CHECK(fts_decode_sm(fts_encode_sm(v, 11), 11) == v, "round trip bit 11");
}

int main(void)
{
    test_build_ping();
    test_build_read();
    test_build_sync_read();
    test_parse_valid();
    test_parse_junk_prefix();
    test_parse_partial();
    test_parse_bad_checksum();
    test_sign_magnitude();
    printf("%d/%d checks passed\n", checks - failures, checks);
    return failures ? 1 : 0;
}
