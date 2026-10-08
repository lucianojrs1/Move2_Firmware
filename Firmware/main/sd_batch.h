#ifndef SD_BATCH_H
#define SD_BATCH_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SD_BATCH_CAPACITY 8192
typedef struct {
    char data[SD_BATCH_CAPACITY];
    size_t used;
    uint32_t count;
} sd_batch_t;
typedef ptrdiff_t (*sd_batch_write_fn)(void *ctx, const char *data, size_t size);
typedef bool (*sd_batch_sync_fn)(void *ctx);
bool sd_batch_append(sd_batch_t *batch, const char *line, size_t size);
// Retains the ENTIRE batch on any error; clears it only after successful sync.
// After a failure the caller must use a new segment before retrying.
bool sd_batch_commit(sd_batch_t *batch, void *ctx, sd_batch_write_fn write_fn, sd_batch_sync_fn sync_fn);
#endif
