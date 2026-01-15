#ifndef COMPONENTS_COMM_RINGBUF_H
#define COMPONENTS_COMM_RINGBUF_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单生产者(ISR) / 单消费者(task) 环形缓冲
 *
 * 约束：
 * - 写端：仅在 ISR 中写
 * - 读端：仅在任务中读
 * - 不做动态内存分配
 */
typedef struct {
    uint8_t *buf;
    uint16_t size; // 必须为 2 的幂更高效（但非强制）
    volatile uint16_t w;
    volatile uint16_t r;
} comm_ringbuf_t;

void comm_ringbuf_init(comm_ringbuf_t *rb, uint8_t *storage, uint16_t size);

// 返回当前可读字节数
uint16_t comm_ringbuf_available(const comm_ringbuf_t *rb);

// 返回剩余可写空间（不含 1 字节保留位）
uint16_t comm_ringbuf_free(const comm_ringbuf_t *rb);

// 写入数据，返回实际写入字节数（满则丢弃多余部分）
uint16_t comm_ringbuf_write(comm_ringbuf_t *rb, const uint8_t *data, uint16_t len);

// 读取数据，返回实际读取字节数
uint16_t comm_ringbuf_read(comm_ringbuf_t *rb, uint8_t *out, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_COMM_RINGBUF_H */

