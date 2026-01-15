#ifndef COMPONENTS_COMM_VOFA_JUSTFLOAT_H
#define COMPONENTS_COMM_VOFA_JUSTFLOAT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VOFA+ JustFloat 帧尾（固定 4B）：00 00 80 7F
 *
 * 常见用法：
 * - 发送 N 个 float32（小端）作为通道数据
 * - 紧跟 4 字节帧尾，VOFA+ 以此分帧
 */
#define VOFA_JUSTFLOAT_TAIL0 0x00u
#define VOFA_JUSTFLOAT_TAIL1 0x00u
#define VOFA_JUSTFLOAT_TAIL2 0x80u
#define VOFA_JUSTFLOAT_TAIL3 0x7Fu

/**
 * @brief 打包 N 通道 JustFloat 帧
 * @param[out] out 输出缓冲区
 * @param[in] out_len 输出缓冲区长度
 * @param[in] ch 通道数组（float32，小端）
 * @param[in] ch_count 通道数
 * @return 实际写入字节数（= ch_count*4 + 4），失败返回 0
 */
size_t vofa_justfloat_pack(uint8_t *out, size_t out_len, const float *ch, size_t ch_count);

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_COMM_VOFA_JUSTFLOAT_H */
