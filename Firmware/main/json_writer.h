#ifndef JSON_WRITER_H
#define JSON_WRITER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
typedef struct { char *out; size_t cap, used; bool failed; } json_writer_t;
static inline void json_add(json_writer_t *w, const char *fmt, ...) {
    if (w->failed || !w->out || w->used >= w->cap) { w->failed = true; return; }
    va_list args; va_start(args, fmt);
    int n = vsnprintf(w->out + w->used, w->cap - w->used, fmt, args);
    va_end(args);
    if (n < 0 || (size_t)n >= w->cap - w->used) w->failed = true;
    else w->used += (size_t)n;
}
static inline void json_number(json_writer_t *w, double value, bool valid) {
    if (valid && isfinite(value)) json_add(w, "%.3f", value);
    else json_add(w, "null");
}
#endif
