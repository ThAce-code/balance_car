/**
 * @file comm.c
 * @brief P2 阶段一：统一通信模块（协议解析 + 命令分发）实现
 *
 * 注意：
 * - 本文件不会定义任何 HAL_UART_* 回调，避免与现有模块冲突；
 *   后续阶段可在 `APP/uart3_hal_callbacks.c` 中路由到 Comm_On*Isr()。
 * - 本阶段不改动现有 `uart_echo.c`，comm 仅作为可接入的模块存在。
 */

#include "comm.h"

#include "comm/ringbuf.h"
#include "mydefine.h"   // HostCommand_t / StatusData_t
#include "status_store.h"

#include <string.h>

/* =========================
 * Internal config
 * ========================= */

#define COMM_RX_RING_SIZE        (512u)  // ISR write, task read
#define COMM_PARSE_BUF_SIZE      (256u)  // task-local
#define COMM_MAX_PAYLOAD_LEN     (32u)   // stage-1: keep frames small
#define COMM_TX_BUF_SIZE         (96u)

// Frame layout:
// SOF(2) | VER(1) | LEN(2) | MSG_ID(1) | FLAGS(1) | SEQ(2) | PAYLOAD(LEN) | CRC16(2)
#define COMM_HDR_WITH_SOF_LEN    (9u)
#define COMM_MIN_FRAME_LEN       (11u)  // payload=0 => 2+7+0+2

typedef struct {
  /* bindings */
  UART_HandleTypeDef *huart;
  osMessageQueueId_t cmd_queue;

  /* RX DMA */
  uint8_t *rx_dma_buf;
  uint16_t rx_dma_buf_size;

  /* RX ringbuf (ISR->task) */
  comm_ringbuf_t rb;
  uint8_t rb_storage[COMM_RX_RING_SIZE];

  /* parser buffer (task) */
  uint8_t parse_buf[COMM_PARSE_BUF_SIZE];
  uint16_t parse_len;

  /* telemetry config storage (for later stages) */
  comm_telem_state_t telem;
  uint32_t next_telem_ms;

  /* stats */
  comm_stats_t stats;

  /* TX */
  uint8_t tx_buf[COMM_TX_BUF_SIZE];
  volatile uint8_t tx_busy;
  uint16_t local_seq;
} comm_ctx_t;

static comm_ctx_t g_comm = {0};

/* =========================
 * Little-endian helpers (avoid unaligned struct casts)
 * ========================= */

static uint16_t comm_read_le_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t comm_read_le_u32(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static float comm_read_le_f32(const uint8_t *p)
{
  uint32_t u = comm_read_le_u32(p);
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

static void comm_write_le_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void comm_write_le_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void comm_write_le_i16(uint8_t *p, int16_t v)
{
  comm_write_le_u16(p, (uint16_t)v);
}

static void comm_write_le_f32(uint8_t *p, float f)
{
  uint32_t u;
  memcpy(&u, &f, sizeof(u));
  comm_write_le_u32(p, u);
}

/* =========================
 * CRC16/CCITT-FALSE
 * poly=0x1021 init=0xFFFF xorout=0x0000, no reflection
 * range: VER..PAYLOAD (i.e. excluding SOF, excluding CRC)
 * ========================= */

static uint16_t comm_crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void comm_send_frame(uint8_t msg_id, uint8_t flags, uint16_t seq,
                            const uint8_t *payload, uint16_t payload_len)
{
  if (g_comm.huart == NULL) {
    return;
  }
  if (g_comm.tx_busy) {
    return;
  }

  uint16_t frame_len = (uint16_t)(9u + payload_len + 2u);
  if (frame_len > COMM_TX_BUF_SIZE) {
    return;
  }

  uint8_t *buf = g_comm.tx_buf;
  buf[0] = COMM_SOF0;
  buf[1] = COMM_SOF1;
  buf[2] = COMM_VER;
  buf[3] = (uint8_t)(payload_len & 0xFFu);
  buf[4] = (uint8_t)((payload_len >> 8) & 0xFFu);
  buf[5] = msg_id;
  buf[6] = flags;
  buf[7] = (uint8_t)(seq & 0xFFu);
  buf[8] = (uint8_t)((seq >> 8) & 0xFFu);
  if (payload_len > 0u && payload != NULL) {
    memcpy(&buf[9], payload, payload_len);
  }

  uint16_t crc = comm_crc16_ccitt_false(&buf[2], (uint16_t)(7u + payload_len));
  buf[9u + payload_len] = (uint8_t)(crc & 0xFFu);
  buf[10u + payload_len] = (uint8_t)((crc >> 8) & 0xFFu);

  if (HAL_UART_Transmit_DMA(g_comm.huart, buf, frame_len) == HAL_OK) {
    g_comm.tx_busy = 1u;
  }
}

static void comm_restart_rx_dma_isr(void)
{
  if (g_comm.huart == NULL || g_comm.rx_dma_buf == NULL || g_comm.rx_dma_buf_size == 0) {
    return;
  }
  (void)HAL_UARTEx_ReceiveToIdle_DMA(g_comm.huart, g_comm.rx_dma_buf, g_comm.rx_dma_buf_size);
  if (g_comm.huart->hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(g_comm.huart->hdmarx, DMA_IT_HT);
  }
}

/* =========================
 * Command dispatch (task context)
 * ========================= */

static void comm_queue_put_latest(const HostCommand_t *cmd)
{
  if (cmd == NULL) {
    return;
  }
  if (g_comm.cmd_queue == NULL) {
    return;
  }

  osStatus_t st = osMessageQueuePut(g_comm.cmd_queue, cmd, 0, 0);
  if (st != osOK) {
    HostCommand_t drop;
    (void)osMessageQueueGet(g_comm.cmd_queue, &drop, NULL, 0);
    (void)osMessageQueuePut(g_comm.cmd_queue, cmd, 0, 0);
  }
}

static void comm_dispatch_cmd(const comm_header_t *hdr, const uint8_t *payload)
{
  if (hdr == NULL || payload == NULL) {
    return;
  }

  if (g_comm.cmd_queue == NULL) {
    return;
  }

  switch ((comm_msg_id_t)hdr->msg_id) {
    case COMM_MSG_SET_TARGET: {
      // payload: f32 speed | f32 yaw_rate | u8 mode  => 9 bytes
      if (hdr->len < 9u) {
        break;
      }
      HostCommand_t cmd = {
        .target_speed_mps = comm_read_le_f32(&payload[0]),
        .target_yaw_rate_dps = comm_read_le_f32(&payload[4]),
        .mode = payload[8],
      };
      comm_queue_put_latest(&cmd);
      break;
    }

    case COMM_MSG_ESTOP: {
      HostCommand_t cmd = {0};
      cmd.mode = 2;
      cmd.target_speed_mps = 0.0f;
      cmd.target_yaw_rate_dps = 0.0f;
      comm_queue_put_latest(&cmd);
      break;
    }

    case COMM_MSG_TELEM_CONFIG: {
      // payload: u16 period_ms | u16 mask => 4 bytes
      if (hdr->len < 4u) {
        break;
      }
      g_comm.telem.period_ms = comm_read_le_u16(&payload[0]);
      g_comm.telem.mask = comm_read_le_u16(&payload[2]);
      g_comm.next_telem_ms = 0;
      break;
    }

    default:
      break;
  }
}

/* =========================
 * Parser (task context)
 * ========================= */

static void comm_drop_prefix(uint16_t n)
{
  if (n == 0 || n > g_comm.parse_len) {
    return;
  }
  uint16_t remain = (uint16_t)(g_comm.parse_len - n);
  if (remain > 0) {
    memmove(g_comm.parse_buf, &g_comm.parse_buf[n], remain);
  }
  g_comm.parse_len = remain;
}

static void comm_pump_rb_to_parsebuf(void)
{
  uint16_t avail = comm_ringbuf_available(&g_comm.rb);
  if (avail == 0) {
    return;
  }

  uint16_t free_space = (uint16_t)(COMM_PARSE_BUF_SIZE - g_comm.parse_len);
  if (free_space == 0) {
    // Parser stuck or host flooding; drop buffered bytes and resync.
    g_comm.parse_len = 0;
    g_comm.stats.rx_sof_resync++;
    free_space = COMM_PARSE_BUF_SIZE;
  }

  uint16_t to_read = (avail < free_space) ? avail : free_space;
  (void)comm_ringbuf_read(&g_comm.rb, &g_comm.parse_buf[g_comm.parse_len], to_read);
  g_comm.parse_len = (uint16_t)(g_comm.parse_len + to_read);
}

static void comm_parse_available_frames(void)
{
  while (g_comm.parse_len >= COMM_MIN_FRAME_LEN) {
    // 1) Search SOF at current buffer head; if not aligned, scan forward.
    uint16_t sof_pos = 0xFFFFu;
    for (uint16_t i = 0; i + 1u < g_comm.parse_len; i++) {
      if (g_comm.parse_buf[i] == COMM_SOF0 && g_comm.parse_buf[i + 1u] == COMM_SOF1) {
        sof_pos = i;
        break;
      }
    }

    if (sof_pos == 0xFFFFu) {
      // Keep last byte in case it's 0xAA (potential start of SOF).
      if (g_comm.parse_len > 1u) {
        comm_drop_prefix((uint16_t)(g_comm.parse_len - 1u));
        g_comm.stats.rx_sof_resync++;
      }
      return;
    }

    if (sof_pos != 0u) {
      comm_drop_prefix(sof_pos);
      g_comm.stats.rx_sof_resync++;
      if (g_comm.parse_len < COMM_MIN_FRAME_LEN) {
        return;
      }
    }

    // Now parse_buf[0..] begins with SOF.
    uint8_t ver = g_comm.parse_buf[2];
    uint16_t len = (uint16_t)g_comm.parse_buf[3] | ((uint16_t)g_comm.parse_buf[4] << 8);

    if (ver != COMM_VER) {
      // Unknown version: drop one byte and resync.
      comm_drop_prefix(1);
      g_comm.stats.rx_sof_resync++;
      continue;
    }

    if (len > COMM_MAX_PAYLOAD_LEN) {
      // Payload too large for stage-1 parser.
      comm_drop_prefix(1);
      g_comm.stats.rx_len_reject++;
      continue;
    }

    uint16_t frame_len = (uint16_t)(COMM_HDR_WITH_SOF_LEN + len + 2u);
    if (frame_len < COMM_MIN_FRAME_LEN) {
      comm_drop_prefix(1);
      continue;
    }
    if (g_comm.parse_len < frame_len) {
      return; // wait for more bytes
    }

    // Verify CRC16 over VER..PAYLOAD.
    uint16_t crc_rx = (uint16_t)g_comm.parse_buf[COMM_HDR_WITH_SOF_LEN + len] |
                      ((uint16_t)g_comm.parse_buf[COMM_HDR_WITH_SOF_LEN + len + 1u] << 8);
    uint16_t crc_calc = comm_crc16_ccitt_false(&g_comm.parse_buf[2], (uint16_t)(7u + len));
    if (crc_rx != crc_calc) {
      // Bad frame: drop one byte and resync.
      comm_drop_prefix(1);
      g_comm.stats.rx_crc_fail++;
      continue;
    }

    // Extract header + payload and dispatch.
    comm_header_t hdr = {
      .ver = ver,
      .len = len,
      .msg_id = g_comm.parse_buf[5],
      .flags = g_comm.parse_buf[6],
      .seq = (uint16_t)g_comm.parse_buf[7] | ((uint16_t)g_comm.parse_buf[8] << 8),
    };

    uint8_t payload[COMM_MAX_PAYLOAD_LEN];
    if (len > 0u) {
      memcpy(payload, &g_comm.parse_buf[9], len);
    }
    comm_dispatch_cmd(&hdr, payload);
    g_comm.stats.rx_frames_ok++;

    // Consume this frame.
    comm_drop_prefix(frame_len);
  }
}

/* =========================
 * Public API
 * ========================= */

void Comm_Init(const comm_config_t *cfg)
{
  memset(&g_comm, 0, sizeof(g_comm));
  comm_ringbuf_init(&g_comm.rb, g_comm.rb_storage, (uint16_t)sizeof(g_comm.rb_storage));

  if (cfg == NULL) {
    return;
  }

  g_comm.huart = cfg->huart;
  g_comm.cmd_queue = cfg->cmd_queue;
  g_comm.rx_dma_buf = cfg->rx_dma_buf;
  g_comm.rx_dma_buf_size = cfg->rx_dma_buf_size;

  // default telemetry config (for later stages)
  g_comm.telem.period_ms = 20;
  g_comm.telem.mask = 0x0001;
  g_comm.next_telem_ms = 0;
  g_comm.tx_busy = 0;
  g_comm.local_seq = 0;
}

void Comm_StartRx(void)
{
  if (g_comm.huart == NULL || g_comm.rx_dma_buf == NULL || g_comm.rx_dma_buf_size == 0) {
    return;
  }

  (void)HAL_UARTEx_ReceiveToIdle_DMA(g_comm.huart, g_comm.rx_dma_buf, g_comm.rx_dma_buf_size);
  if (g_comm.huart->hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(g_comm.huart->hdmarx, DMA_IT_HT);
  }
}

void Comm_Poll(void)
{
  comm_pump_rb_to_parsebuf();
  comm_parse_available_frames();
}

static bool comm_send_status_payload(const StatusData_t *s)
{
  if (s == NULL) {
    return false;
  }

  // Serialize explicitly to avoid any packed-struct alignment traps.
  // Layout (little-endian): u32 + 4*f32 + 2*i16 + u8 + u16 = 27 bytes
  uint8_t payload[27];
  comm_write_le_u32(&payload[0], s->timestamp_ms);
  comm_write_le_f32(&payload[4], s->pitch_deg);
  comm_write_le_f32(&payload[8], s->pitch_acc_deg);
  comm_write_le_f32(&payload[12], s->wheel_l_mps);
  comm_write_le_f32(&payload[16], s->wheel_r_mps);
  comm_write_le_i16(&payload[20], s->pwm_l);
  comm_write_le_i16(&payload[22], s->pwm_r);
  payload[24] = s->mode;
  comm_write_le_u16(&payload[25], s->fault_bits);

  comm_send_frame((uint8_t)COMM_MSG_STATUS, 0u, g_comm.local_seq++, payload, (uint16_t)sizeof(payload));
  return true;
}

void Comm_TelemetryTick(void)
{
  if (g_comm.huart == NULL) {
    return;
  }

  if ((g_comm.telem.mask & 0x0001u) == 0u) { // bit0=STATUS
    return;
  }

  uint16_t period = g_comm.telem.period_ms;
  if (period == 0u) {
    return;
  }

  uint32_t now_ms = HAL_GetTick();
  if (g_comm.next_telem_ms == 0u) {
    g_comm.next_telem_ms = now_ms + period;
    return;
  }

  if ((int32_t)(now_ms - g_comm.next_telem_ms) < 0) {
    return;
  }
  g_comm.next_telem_ms = now_ms + period;

  StatusData_t s;
  if (StatusStore_Read(&s)) {
    (void)comm_send_status_payload(&s);
  }
}

comm_telem_state_t Comm_GetTelemState(void)
{
  return g_comm.telem;
}

comm_stats_t Comm_GetStats(void)
{
  return g_comm.stats;
}

/* =========================
 * HAL callback routing (optional)
 * ========================= */

void Comm_OnRxEventIsr(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart == NULL || huart != g_comm.huart) {
    return;
  }
  if (g_comm.rx_dma_buf == NULL || g_comm.rx_dma_buf_size == 0) {
    return;
  }

  if (size > g_comm.rx_dma_buf_size) {
    size = g_comm.rx_dma_buf_size;
  }

  uint16_t written = comm_ringbuf_write(&g_comm.rb, g_comm.rx_dma_buf, size);
  if (written != size) {
    g_comm.stats.rx_overflow++;
  }

  comm_restart_rx_dma_isr();
}

void Comm_OnTxCpltIsr(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart != g_comm.huart) {
    return;
  }
  g_comm.tx_busy = 0u;
}

void Comm_OnErrorIsr(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart != g_comm.huart) {
    return;
  }
  g_comm.tx_busy = 0u;
  comm_restart_rx_dma_isr();
}
