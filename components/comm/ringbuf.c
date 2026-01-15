#include "comm/ringbuf.h"

#include <string.h>

static uint16_t rb_mask(const comm_ringbuf_t *rb, uint16_t idx)
{
    if (rb->size == 0) {
        return 0;
    }
    return (uint16_t)(idx % rb->size);
}

void comm_ringbuf_init(comm_ringbuf_t *rb, uint8_t *storage, uint16_t size)
{
    if (rb == NULL) {
        return;
    }
    rb->buf = storage;
    rb->size = size;
    rb->w = 0;
    rb->r = 0;
}

uint16_t comm_ringbuf_available(const comm_ringbuf_t *rb)
{
    if (rb == NULL) {
        return 0;
    }
    return (uint16_t)(rb->w - rb->r);
}

uint16_t comm_ringbuf_free(const comm_ringbuf_t *rb)
{
    if (rb == NULL || rb->size == 0) {
        return 0;
    }
    // 保留 1 字节用于区分满/空
    uint16_t used = comm_ringbuf_available(rb);
    if (used >= rb->size) {
        return 0;
    }
    return (uint16_t)(rb->size - used - 1u);
}

uint16_t comm_ringbuf_write(comm_ringbuf_t *rb, const uint8_t *data, uint16_t len)
{
    if (rb == NULL || rb->buf == NULL || data == NULL || rb->size == 0) {
        return 0;
    }

    uint16_t free_bytes = comm_ringbuf_free(rb);
    if (len > free_bytes) {
        len = free_bytes;
    }
    if (len == 0) {
        return 0;
    }

    uint16_t w = rb_mask(rb, rb->w);
    uint16_t first = len;
    uint16_t until_end = (uint16_t)(rb->size - w);
    if (first > until_end) {
        first = until_end;
    }

    memcpy(&rb->buf[w], data, first);
    if (first < len) {
        memcpy(&rb->buf[0], &data[first], (size_t)(len - first));
    }

    rb->w = (uint16_t)(rb->w + len);
    return len;
}

uint16_t comm_ringbuf_read(comm_ringbuf_t *rb, uint8_t *out, uint16_t len)
{
    if (rb == NULL || rb->buf == NULL || out == NULL || rb->size == 0) {
        return 0;
    }

    uint16_t avail = comm_ringbuf_available(rb);
    if (len > avail) {
        len = avail;
    }
    if (len == 0) {
        return 0;
    }

    uint16_t r = rb_mask(rb, rb->r);
    uint16_t first = len;
    uint16_t until_end = (uint16_t)(rb->size - r);
    if (first > until_end) {
        first = until_end;
    }

    memcpy(out, &rb->buf[r], first);
    if (first < len) {
        memcpy(&out[first], &rb->buf[0], (size_t)(len - first));
    }

    rb->r = (uint16_t)(rb->r + len);
    return len;
}

