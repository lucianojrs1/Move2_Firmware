#include "sd_batch.h"
#include "sd_record.h"
#include "gps_format.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
void test_metrics(void);

typedef struct {
    char data[SD_BATCH_CAPACITY];
    size_t used, chunk;
    int calls, fail_on_call, sync_calls;
    bool fail_sync, zero_write;
} fake_card_t;

static ptrdiff_t fake_write(void *ctx, const char *data, size_t size)
{
    fake_card_t *card = ctx;
    ++card->calls;
    if (card->calls == card->fail_on_call) return -1;
    if (card->zero_write) return 0;
    if (size > card->chunk) size = card->chunk;
    assert(card->used + size <= sizeof(card->data));
    memcpy(card->data + card->used, data, size);
    card->used += size;
    return (ptrdiff_t)size;
}
static bool fake_sync(void *ctx)
{
    fake_card_t *card = ctx;
    card->sync_calls++;
    return !card->fail_sync;
}

static void test_durability(void)
{
    sd_batch_t batch = {0};
    const char payload[] = "{\"seq\":1}\n{\"seq\":2}\n";
    assert(sd_batch_append(&batch, payload, sizeof(payload) - 1));
    fake_card_t card = {.chunk = 3, .fail_on_call = 3};
    assert(!sd_batch_commit(&batch, &card, fake_write, fake_sync));
    assert(card.used == 6 && card.sync_calls == 0);
    assert(batch.used == sizeof(payload) - 1 && batch.count == 1);
    assert(!memcmp(batch.data, payload, batch.used));

    // New file after removal: replay all bytes, not just the unwritten tail.
    card = (fake_card_t){.chunk = 4};
    assert(sd_batch_commit(&batch, &card, fake_write, fake_sync));
    assert(card.used == sizeof(payload) - 1 && card.sync_calls == 1);
    assert(!memcmp(card.data, payload, card.used));
    assert(!batch.used && !batch.count);

    assert(sd_batch_append(&batch, payload, sizeof(payload) - 1));
    card = (fake_card_t){.chunk = sizeof(card.data), .fail_sync = true};
    assert(!sd_batch_commit(&batch, &card, fake_write, fake_sync));
    assert(batch.used == sizeof(payload) - 1 && batch.count == 1);
    card = (fake_card_t){.chunk = 1};
    assert(sd_batch_commit(&batch, &card, fake_write, fake_sync));
    assert(!memcmp(card.data, payload, card.used));

    assert(sd_batch_append(&batch, payload, sizeof(payload) - 1));
    card = (fake_card_t){.chunk = 1, .zero_write = true};
    assert(!sd_batch_commit(&batch, &card, fake_write, fake_sync));
    assert(card.calls == 1 && card.sync_calls == 0 && batch.used);
    size_t saved_size = batch.used;
    assert(!sd_batch_append(&batch, payload, SD_BATCH_CAPACITY));
    assert(batch.used == saved_size);
    batch.used = batch.count = 0;
    assert(sd_batch_commit(&batch, &card, fake_write, fake_sync));
    assert(card.calls == 1); // Empty batch must not issue IO.

    // Full-size batches, repeated rotation/retry boundary, no buffer overrun.
    char full[SD_BATCH_CAPACITY]; memset(full, 'X', sizeof(full));
    assert(sd_batch_append(&batch, full, sizeof(full)));
    assert(!sd_batch_append(&batch, "x", 1));
    card = (fake_card_t){.chunk = 511};
    assert(sd_batch_commit(&batch, &card, fake_write, fake_sync));
    assert(card.used == sizeof(full) && !memcmp(card.data, full, sizeof(full)));
}

static void emit(sd_record_t *record, uint64_t epoch)
{
    char line[SD_RECORD_LINE_MAX];
    size_t n = sd_record_format(line, sizeof(line), record, UINT64_C(0x123456789abcdef0), epoch);
    assert(n && n == strlen(line) && line[n - 1] == '\n');
    fputs(line, stdout);
}

static void emit_gps(const gps_status_t *gps, uint32_t now_ms)
{
    char payload[GPS_PAYLOAD_MAX];
    size_t n = gps_format_payload(payload, sizeof(payload), gps, now_ms, UINT64_C(1757800000123));
    assert(n && n == strlen(payload));
    puts(payload);
}

static void test_gps_visibility(void)
{
    gps_status_t g = {0};
    assert(gps_position_state(&g, 1000) == GPS_NO_DATA);
    emit_gps(&g, 1000);
    g.uart_seen = true;
    assert(gps_position_state(&g, 1000) == GPS_NO_FIX);
    emit_gps(&g, 1000);
    g.fix = g.location_valid = true;
    g.lat = -8.055338; g.lon = -34.951803;
    g.sats_used = 8; g.fix_quality = 1; g.last_fix_ms = 500;
    assert(gps_position_state(&g, 1000) == GPS_VALID);
    emit_gps(&g, 1000);
    assert(gps_position_state(&g, 3500) == GPS_STALE);
    emit_gps(&g, 3500);
    g.last_fix_ms = UINT32_MAX - 100;
    assert(gps_position_state(&g, 50) == GPS_VALID); // 32-bit monotonic wrap.
    g.fix = false;
    assert(gps_position_state(&g, 50) == GPS_NO_FIX); // Invalid GGA overrides old coordinates.
    g.fix = true; g.lat = NAN;
    assert(gps_position_state(&g, 50) == GPS_NO_FIX);
    g.lat = 91;
    assert(gps_position_state(&g, 50) == GPS_NO_FIX);
    g.lat = g.lon = 0; g.last_fix_ms = 500;
    assert(gps_position_state(&g, 1000) == GPS_VALID); // A real fix at zero is valid.
    emit_gps(&g, 1000);
    char tiny[4] = {'a','b','c','d'};
    assert(!gps_format_payload(tiny, 3, &g, 1000, 0)); assert(tiny[3] == 'd');
    assert(!gps_format_payload(NULL, 0, &g, 1000, 0));
}

int main(void)
{
    test_durability();
    sd_record_t r = {.type = SD_RECORD_CAN, .seq = UINT64_C(4294967297), .mono_us = UINT64_C(987654321098)};
    r.value.can.id = 0x1803F3F4;
    r.value.can.extended = true;
    r.value.can.dlc = 8;
    for (unsigned i = 0; i < 8; ++i) r.value.can.data[i] = (uint8_t)(i * 17);
    emit(&r, UINT64_C(1757800000123));
    r.value.can.rtr = true; emit(&r, 0);
    r.value.can.rtr = false; r.value.can.dlc = 15; emit(&r, 0);
    r.value.can.dlc = 16;
    char tiny[4] = {'a','b','c','d'}, out[SD_RECORD_LINE_MAX];
    assert(!sd_record_format(out, sizeof(out), &r, 0, 0));
    r.value.can.dlc = 0;
    assert(!sd_record_format(tiny, 3, &r, 0, 0)); assert(tiny[3] == 'd');
    assert(!sd_record_format(NULL, 0, &r, 0, 0));
    r = (sd_record_t){.type = SD_RECORD_IMU};
    r.value.imu.axes[0] = -32768; r.value.imu.axes[1] = 32767;
    r.value.imu.temperature = -123; r.value.imu.sensor_time = UINT32_MAX;
    r.value.imu.acc_conf = r.value.imu.gyr_conf = 0x4028;
    emit(&r, 0);
    r = (sd_record_t){.type = SD_RECORD_NMEA};
    strcpy(r.value.nmea, "$GNRMC,\"quoted\",\\,\t*00"); emit(&r, 0);
    memset(r.value.nmea, 1, sizeof(r.value.nmea) - 1);
    r.value.nmea[sizeof(r.value.nmea) - 1] = '\0'; emit(&r, 0);
    memset(r.value.nmea, 'x', sizeof(r.value.nmea));
    assert(!sd_record_format(out, sizeof(out), &r, 0, 0));
    r = (sd_record_t){.type = SD_RECORD_HEALTH};
    r.value.health.dropped[0] = 12; r.value.health.io_errors = 2;
    r.value.health.last_data_us = 12345; r.value.health.last_sync_us = 45678;
    r.value.health.committed_bytes = UINT64_C(4294967297); emit(&r, 0);
    r.type = (sd_record_type_t)99;
    assert(!sd_record_format(out, sizeof(out), &r, 0, 0));
    test_gps_visibility();
    test_metrics();
    fprintf(stderr, "SD C tests passed: partial/zero writes, full card, sync failure, retry, bounds, encoding.\n");
    return 0;
}
