#include "radar.h"

#include "config.h" /* 板级配置：RADAR_*_UART_NUM / _RX_PIN / _TX_PIN / _BAUD 在这里（B 区） */
#include "ld2450.h"
#include "r60abd1.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 雷达采集：两路 UART 非阻塞轮询 → 纯 C 解析器 → 限频日志 + 快照。
 *
 * 【为什么是「轮询」而不是中断/DMA】两种雷达都是「上电即喷数据」的只读外设，
 * 不下发任何配置命令；数据率极低（LD2450 满帧 30B×10Hz = 300B/s，R60 更低），
 * 所以 10ms 轮询 + 1024B ring 余量极大，中断和 DMA 都是无谓的复杂度。
 *
 * 【移植说明】本文件从 brufik / 小歪 项目的 firmware/radar/radar.cpp 改写：
 *   Arduino → ESP-IDF：
 *     HardwareSerial        → uart_driver_install / uart_read_bytes
 *     millis()              → esp_timer_get_time() / 1000
 *     constrain()           → clampi()
 *     log_warn/log_error    → ESP_LOGW/ESP_LOGE
 *     utils_task_create_pinned → xTaskCreatePinnedToCore
 *   已删除（见 radar.h 的说明）：
 *     舵机转向/回中（head.h）、状态与事件上行（ws_transport）、
 *     三个动作 FSM（蹲下/挥手/入座）、断线恢复 resume_from_pause()
 */

#define TAG "radar"

namespace {

constexpr size_t kRadarTaskStack = 8 * 1024;
/** 优先级 3：与其它非实时任务同档。雷达只做解析 + 限频日志，不需要更高。 */
constexpr UBaseType_t kRadarTaskPrio = 3;
/** 钉在 APP core（core 1）—— core 0 常驻 WiFi/协议栈，别去抢。
 *  ⚠️ 原项目用的是 Arduino 的 APP_CPU_NUM，IDF 里没有这个宏，直接写 1。 */
constexpr BaseType_t kRadarTaskCore = 1;

/** 每轮每路最多消化的字节数：防止某一路突发把任务按在这里、饿死另一路。 */
constexpr int kDrainMaxBytes = 128;
/** 轮询周期（ms）。两路合计约 300B/s，10ms 轮询余量极大。 */
constexpr uint32_t kPollIntervalMs = 10;
/** UART RX ring（字节）。日志出口在大主机忙时可能阻塞，靠它兜住。 */
constexpr int kRxRingBytes = 1024;

r60_parser_t    s_r60;
ld2450_parser_t s_ld2450;

TaskHandle_t s_task = nullptr;
bool         s_ready = false;

uint32_t s_last_log_ms = 0;
/** 各路最近一次「有效帧数增加」的时刻，用于无数据排障。 */
uint32_t s_r60_ok_ms = 0;
uint32_t s_ld_ok_ms = 0;
uint32_t s_r60_prev_ok = 0;
uint32_t s_ld_prev_ok = 0;
bool     s_r60_warned = false;
bool     s_ld_warned = false;
/** 上次记录的「有人」/目标数，-1 = 尚未初始化（首帧不报跳变）。 */
int s_prev_presence = -1;
int s_prev_targets = -1;

/* 心率/呼吸各自的新鲜度锚点（见 RADAR_VITAL_TTL_MS）。
 * ⚠️ 不能用 r60_status()->last_update_ms 代替：解析器对**每一帧**都盖那个时间戳，
 *    心率 3 秒才一帧，但体动帧 1Hz 会把它一直刷新 → 永远判「新鲜」。 */
uint32_t s_heart_ms = 0;
uint32_t s_breath_ms = 0;

/* ──────────────────────────────────────────────────────────────────────
 * 小工具
 * ──────────────────────────────────────────────────────────────────── */

int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/** 毫秒时基。int64 微秒 → uint32 毫秒，49.7 天回绕一次
 *  （与原项目 millis() 的行为一致，所有差值比较都用无符号回绕安全的写法）。 */
uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

/* ──────────────────────────────────────────────────────────────────────
 * UART
 * ──────────────────────────────────────────────────────────────────── */

/** 配置并启动一路「只收不发」的 UART。 */
bool uart_begin(uart_port_t port, int rx_pin, int tx_pin, int baud, const char* label) {
  const uart_config_t cfg = {
      .baud_rate = baud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  /* tx_buffer_size = 0：本模块只收不发，不需要 TX ring。 */
  esp_err_t err = uart_driver_install(port, kRxRingBytes, 0, 0, nullptr, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s uart%d driver_install 失败: %s", label, (int)port, esp_err_to_name(err));
    return false;
  }
  err = uart_param_config(port, &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s uart%d param_config 失败: %s", label, (int)port, esp_err_to_name(err));
    return false;
  }
  /* 显式指定引脚 —— 会自动走 GPIO 矩阵重映射。
   * ⚠️ 必须显式设：ESP32-S3 上 UART1 与 UART2 的默认引脚是同一对(GPIO17/18)，
   *    不设就会互相抢。 */
  err = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s uart%d set_pin 失败: %s", label, (int)port, esp_err_to_name(err));
    return false;
  }
  return true;
}

/** 每轮只消化「当前已可读」的字节，读完立刻返回，绝不等待。 */
void drain_r60(uint32_t now) {
  uint8_t buf[kDrainMaxBytes];
  const int n = uart_read_bytes((uart_port_t)RADAR_R60_UART_NUM, buf, sizeof(buf), 0);
  if (n <= 0) {
    return;
  }
  for (int i = 0; i < n; ++i) {
    r60_feed(&s_r60, buf[i], now);
  }
}

void drain_ld2450(uint32_t now) {
  uint8_t buf[kDrainMaxBytes];
  const int n = uart_read_bytes((uart_port_t)RADAR_LD2450_UART_NUM, buf, sizeof(buf), 0);
  if (n <= 0) {
    return;
  }
  for (int i = 0; i < n; ++i) {
    ld2450_feed(&s_ld2450, buf[i], now);
  }
}

/* ──────────────────────────────────────────────────────────────────────
 * 解析辅助
 * ──────────────────────────────────────────────────────────────────── */

int ld2450_valid_count(const ld2450_status_t* l) {
  int count = 0;
  for (int i = 0; i < LD2450_TARGET_COUNT; ++i) {
    if (l->targets[i].valid) {
      ++count;
    }
  }
  return count;
}

/** 从当前 LD2450 快照里挑第一个「有效、且在距离窗口内」的目标。
 *
 *  窗口的超参由调用方给：方位（radar_snapshot）用 DESKBOT_RADAR_*_DIST_MM。
 *  注意是「第一个满足的」而不是「最近的」—— 与 LD2450 上报顺序一致。 */
const ld2450_target_t* pick_ld2450_target(int16_t min_dist_mm, int16_t max_dist_mm) {
  const ld2450_status_t* l = ld2450_status(&s_ld2450);
  for (int i = 0; i < LD2450_TARGET_COUNT; ++i) {
    const ld2450_target_t& t = l->targets[i];
    if (t.valid && t.y_mm > min_dist_mm && t.y_mm < max_dist_mm) {
      return &t;
    }
  }
  return nullptr;
}

/* ──────────────────────────────────────────────────────────────────────
 * 帧回调
 * ──────────────────────────────────────────────────────────────────── */

/** R60 帧回调：本版只用来打心率/呼吸的**新鲜度锚点**。
 *
 *  心率/呼吸的值本身由解析器写进 r60_status_t，但「它们各自什么时候更新过」
 *  只有帧回调知道 —— 而这正是 radar_snapshot() 判 TTL 的依据。
 *  （原项目这里还把体动幅度分发给蹲下 FSM，本版没移植 FSM。） */
void on_r60_frame(uint8_t ctl, uint8_t cmd, const uint8_t* /*data*/, uint16_t /*len*/,
                  void* /*user*/) {
  if (ctl == R60_CTL_BREATH && cmd == R60_CMD_BREATH_VAL) {
    s_breath_ms = now_ms();
  } else if (ctl == R60_CTL_HEART && cmd == R60_CMD_HEART_VAL) {
    s_heart_ms = now_ms();
  }
}

/* ──────────────────────────────────────────────────────────────────────
 * 日志（全部用 ESP_LOGW/ESP_LOGE —— 运行期看得到）
 * ──────────────────────────────────────────────────────────────────── */

/** 状态跳变即时输出（有人/无人、目标数），便于直接观察实时活动。 */
void log_events() {
  const r60_status_t* r = r60_status(&s_r60);
  const int presence = r->presence ? 1 : 0;
  if (presence != s_prev_presence) {
    if (s_prev_presence >= 0) {
      ESP_LOGW(TAG, "[RADAR/R60] presence: %d -> %d", s_prev_presence, presence);
    }
    s_prev_presence = presence;
  }

  const int targets = ld2450_valid_count(ld2450_status(&s_ld2450));
  if (targets != s_prev_targets) {
    if (s_prev_targets >= 0) {
      ESP_LOGW(TAG, "[RADAR/LD2450] targets: %d -> %d", s_prev_targets, targets);
    }
    s_prev_targets = targets;
  }
}

/** 记录「有效帧数增加」的时刻；恢复后允许再次告警。 */
void update_liveness(uint32_t now) {
  if (s_r60.frames_ok != s_r60_prev_ok) {
    s_r60_prev_ok = s_r60.frames_ok;
    s_r60_ok_ms = now;
    s_r60_warned = false;
  }
  if (s_ld2450.frames_ok != s_ld_prev_ok) {
    s_ld_prev_ok = s_ld2450.frames_ok;
    s_ld_ok_ms = now;
    s_ld_warned = false;
  }
}

/** 无数据排障：某路连续 RADAR_SILENT_WARN_MS 无有效帧就打一次。
 *  文案直接引导排查方向 —— 这是验证接线的主要手段：
 *    frames_bad == 0 → 一个字节都没进来 → 接线/供电
 *    frames_bad 在涨 → 有字节但帧校验不过 → 多半是波特率不匹配或未共地 */
void log_silent_diag(uint32_t now) {
  if (!s_r60_warned && s_r60_ok_ms != 0 &&
      (uint32_t)(now - s_r60_ok_ms) >= RADAR_SILENT_WARN_MS) {
    s_r60_warned = true;
    if (s_r60.frames_bad > 0) {
      ESP_LOGW(TAG, "[RADAR/R60] %us 无有效帧(bad=%u 在增长) — 多是波特率不匹配/未共地，应为 %d 8N1",
               (unsigned)(RADAR_SILENT_WARN_MS / 1000), (unsigned)s_r60.frames_bad, RADAR_R60_BAUD);
    } else {
      ESP_LOGW(TAG, "[RADAR/R60] %us 无任何数据 — 查接线: 模块TX->GPIO%d, 模块RX->GPIO%d, %d 8N1, 3V3/GND",
               (unsigned)(RADAR_SILENT_WARN_MS / 1000), RADAR_R60_RX_PIN, RADAR_R60_TX_PIN,
               RADAR_R60_BAUD);
    }
  }

  if (!s_ld_warned && s_ld_ok_ms != 0 &&
      (uint32_t)(now - s_ld_ok_ms) >= RADAR_SILENT_WARN_MS) {
    s_ld_warned = true;
    if (s_ld2450.frames_bad > 0) {
      ESP_LOGW(TAG, "[RADAR/LD2450] %us 无有效帧(bad=%u 在增长) — 多是波特率不匹配/未共地，应为 %d 8N1",
               (unsigned)(RADAR_SILENT_WARN_MS / 1000), (unsigned)s_ld2450.frames_bad,
               RADAR_LD2450_BAUD);
    } else {
      ESP_LOGW(TAG, "[RADAR/LD2450] %us 无任何数据 — 查接线: 模块TX->GPIO%d, 模块RX->GPIO%d, %d 8N1, 3V3/GND",
               (unsigned)(RADAR_SILENT_WARN_MS / 1000), RADAR_LD2450_RX_PIN, RADAR_LD2450_TX_PIN,
               RADAR_LD2450_BAUD);
    }
  }
}

/** 周期摘要：计数器 + 关键字段。LD2450 的 mm 换算成 cm 更易读。 */
void log_summary(uint32_t now) {
#if RADAR_LOG_INTERVAL_MS > 0
  if (s_last_log_ms != 0 && (uint32_t)(now - s_last_log_ms) < RADAR_LOG_INTERVAL_MS) {
    return;
  }
  s_last_log_ms = now;

  const r60_status_t* r = r60_status(&s_r60);
  /* 不打印 pos= —— R60 的 DP5 坐标解析在 r60abd1.c 里已 #if 0，恒为 0 */
  ESP_LOGW(TAG, "[RADAR/R60] ok=%u bad=%u | presence=%d motion=%u move=%u breath=%u heart=%u",
           (unsigned)s_r60.frames_ok, (unsigned)s_r60.frames_bad, r->presence ? 1 : 0,
           (unsigned)r->motion, (unsigned)r->body_move, (unsigned)r->breath_rate,
           (unsigned)r->heart_rate);

  const ld2450_status_t* l = ld2450_status(&s_ld2450);
  char tgt[128];
  int off = 0;
  const int n = ld2450_valid_count(l);
  if (n == 0) {
    snprintf(tgt, sizeof(tgt), "none");
  } else {
    for (int i = 0; i < LD2450_TARGET_COUNT && off < (int)sizeof(tgt) - 24; ++i) {
      if (!l->targets[i].valid) {
        continue;
      }
      off += snprintf(tgt + off, sizeof(tgt) - off, "%sT%d(x=%d,y=%d,v=%d)", off ? " " : "", i,
                      (int)(l->targets[i].x_mm / 10), (int)(l->targets[i].y_mm / 10),
                      (int)(l->targets[i].speed_cms / 10));
    }
  }
  ESP_LOGW(TAG, "[RADAR/LD2450] ok=%u bad=%u | n=%d %s", (unsigned)s_ld2450.frames_ok,
           (unsigned)s_ld2450.frames_bad, n, tgt);
#else
  (void)now;
#endif
}

/* ──────────────────────────────────────────────────────────────────────
 * 任务
 * ──────────────────────────────────────────────────────────────────── */

void task_loop_radar(void* /*arg*/) {
  for (;;) {
    const uint32_t now = now_ms();

    drain_r60(now);
    drain_ld2450(now);
    update_liveness(now);

    log_events();
    log_summary(now);
    log_silent_diag(now);

    vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
  }
}

}  // namespace

/* ──────────────────────────────────────────────────────────────────────
 * 公开接口
 * ──────────────────────────────────────────────────────────────────── */

bool setup_radar() {
  if (s_ready) {
    return true;
  }

  r60_init(&s_r60, on_r60_frame, nullptr);
  /* LD2450 本版没有消费方（不做 FSM），帧回调传 nullptr */
  ld2450_init(&s_ld2450, nullptr, nullptr);

  /* R60ABD1：UART2，模块 TX→GPIO10(ESP RX)，模块 RX←GPIO9(ESP TX) */
  if (!uart_begin((uart_port_t)RADAR_R60_UART_NUM, RADAR_R60_RX_PIN, RADAR_R60_TX_PIN,
                  RADAR_R60_BAUD, "R60")) {
    return false;
  }
  /* LD2450：UART1，模块 TX→GPIO18(ESP RX)，模块 RX←GPIO17(ESP TX) */
  if (!uart_begin((uart_port_t)RADAR_LD2450_UART_NUM, RADAR_LD2450_RX_PIN, RADAR_LD2450_TX_PIN,
                  RADAR_LD2450_BAUD, "LD2450")) {
    return false;
  }

  const uint32_t now = now_ms();
  s_r60_ok_ms = now;
  s_ld_ok_ms = now;
  s_last_log_ms = now;
  s_ready = true;

  /* 就绪行带实际引脚/波特率，便于和实物接线逐一核对。 */
  ESP_LOGW(TAG, "[RADAR] R60ABD1 ready uart%d rx=G%d tx=G%d @%d", RADAR_R60_UART_NUM,
           RADAR_R60_RX_PIN, RADAR_R60_TX_PIN, RADAR_R60_BAUD);
  ESP_LOGW(TAG, "[RADAR] LD2450  ready uart%d rx=G%d tx=G%d @%d", RADAR_LD2450_UART_NUM,
           RADAR_LD2450_RX_PIN, RADAR_LD2450_TX_PIN, RADAR_LD2450_BAUD);
  return true;
}

void task_setup_radar() {
  if (s_task || !s_ready) {
    return;
  }
  /* ⚠️ 栈用 xTaskCreatePinnedToCore 的默认分配（内部 RAM），不是原项目 Arduino 封装
   * 的 PSRAM 栈。8KB 对「解析 + 限频日志」应该富余，但这是本板第一次这么用，
   * 若出现栈溢出告警就调大。 */
  const BaseType_t rc = xTaskCreatePinnedToCore(task_loop_radar, "radar", kRadarTaskStack,
                                                nullptr, kRadarTaskPrio, &s_task, kRadarTaskCore);
  if (rc != pdPASS) {
    ESP_LOGE(TAG, "[RADAR] task create rc=%d", (int)rc);
    s_task = nullptr;
    return;
  }
  ESP_LOGW(TAG, "[RADAR] task OK stack=%u prio=%u log_interval=%dms", (unsigned)kRadarTaskStack,
           (unsigned)kRadarTaskPrio, RADAR_LOG_INTERVAL_MS);
}

radar_snapshot_t radar_snapshot() {
  radar_snapshot_t out{};
  if (!s_ready) {
    return out; /* 未 setup_radar()：全零（present=false、生命体征未知） */
  }
  const uint32_t now = now_ms();

  /* 「有人在」用不限距离的有效目标数判 —— 与「能不能用来算方位」是两件事：
   * 人站在 3m 外时仍然「有人」，只是方位窗口（DESKBOT_RADAR_*_DIST_MM）够不着。 */
  out.present = ld2450_valid_count(ld2450_status(&s_ld2450)) > 0;

  const ld2450_target_t* t =
      pick_ld2450_target(DESKBOT_RADAR_MIN_DIST_MM, DESKBOT_RADAR_MAX_DIST_MM);
  if (t) {
    /* 符号规范化：对外契约固定为「负 = 机器人左侧」。
     * 实测反了（人站左边却报正数）→ 把 radar_config.h 的 DESKBOT_RADAR_X_SIGN 改成 -1。 */
    out.x_mm = (int16_t)((int)t->x_mm * DESKBOT_RADAR_X_SIGN);
    out.y_mm = t->y_mm;
  }

  /* 生命体征：超过 RADAR_VITAL_TTL_MS 没更新 → 保持 0（= 未知），
   * 而不是把断线/刚开机前的陈旧值当成当前读数交出去。 */
  const r60_status_t* r = r60_status(&s_r60);
  if (s_heart_ms != 0 && (uint32_t)(now - s_heart_ms) < (uint32_t)RADAR_VITAL_TTL_MS) {
    out.heart_rate = r->heart_rate;
  }
  if (s_breath_ms != 0 && (uint32_t)(now - s_breath_ms) < (uint32_t)RADAR_VITAL_TTL_MS) {
    out.breath_rate = r->breath_rate;
  }
  return out;
}

bool radar_where_text(const radar_snapshot_t* snap, char* out, size_t out_len) {
  if (!out || out_len == 0) {
    return false;
  }
  out[0] = '\0';
  if (!snap || !snap->present || snap->y_mm <= 0) {
    return false; /* 无目标 / 目标在正侧或身后：此时 atan2 无意义，不猜 */
  }

  const float x = (float)snap->x_mm;
  const float y = (float)snap->y_mm;
  /* 角度用 |x| 算，方向由 x 的符号给 —— 分档阈值与原项目服务端一致
   * （那边是 llm_tool_runner._radar_where_text，15° / 45°）。 */
  const float deg = atan2f(fabsf(x), y) * 180.0f / (float)M_PI;
  const bool left = (x < 0.0f);
  const char* where;
  if (deg <= 15.0f) {
    where = "正前方";
  } else if (deg <= 45.0f) {
    where = left ? "左前方" : "右前方";
  } else {
    where = left ? "左侧" : "右侧";
  }
  const float dist_m = sqrtf(x * x + y * y) / 1000.0f;
  snprintf(out, out_len, "%s %.1f米", where, (double)dist_m);
  return true;
}
