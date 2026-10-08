#include "sd_batch.h"
#include <string.h>

bool sd_batch_append(sd_batch_t *b, const char *line, size_t size)
{
    if (!b || !line || !size || b->used > sizeof(b->data) || size > sizeof(b->data) - b->used) return false;
    memcpy(b->data + b->used, line, size);
    b->used += size;
    b->count++;
    return true;
}

bool sd_batch_commit(sd_batch_t *b, void *ctx, sd_batch_write_fn write_fn, sd_batch_sync_fn sync_fn)
{
    if (!b || !write_fn || !sync_fn || b->used > sizeof(b->data)) return false;
    if (!b->used) return true;
    size_t sent = 0;
    while (sent < b->used) {
        ptrdiff_t n = write_fn(ctx, b->data + sent, b->used - sent);
        if (n <= 0 || (size_t)n > b->used - sent) return false;
        sent += (size_t)n;
    }
    if (!sync_fn(ctx)) return false;
    b->used = 0;
    b->count = 0;
    return true;
}
