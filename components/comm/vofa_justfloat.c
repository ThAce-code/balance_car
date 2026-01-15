#include "comm/vofa_justfloat.h"

#include <string.h>

size_t vofa_justfloat_pack(uint8_t *out, size_t out_len, const float *ch, size_t ch_count)
{
    if (out == NULL || ch == NULL) {
        return 0;
    }

    size_t payload_len = ch_count * sizeof(float);
    size_t total = payload_len + 4;
    if (out_len < total) {
        return 0;
    }

    // float32 小端序（STM32F4 为小端），直接按内存布局拷贝即可。
    memcpy(&out[0], ch, payload_len);
    out[payload_len + 0] = VOFA_JUSTFLOAT_TAIL0;
    out[payload_len + 1] = VOFA_JUSTFLOAT_TAIL1;
    out[payload_len + 2] = VOFA_JUSTFLOAT_TAIL2;
    out[payload_len + 3] = VOFA_JUSTFLOAT_TAIL3;
    return total;
}
