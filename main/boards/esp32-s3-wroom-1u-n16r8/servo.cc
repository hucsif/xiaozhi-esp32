#include "servo.h"

#include "config.h" /* 引脚：SERVO_X_GPIO / SERVO_Y_GPIO 在板级 config.h 的 B 区 */
#include "servo_config.h"
#include "radar.h"
#include "application.h"
#include "device_state.h"

#include <driver/ledc.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 舵机：对话时持续跟踪说话人，退出对话自动回中。
 *
 * ── 控制模型 ──────────────────────────────────────────────────────────
 * 由**状态**驱动，不是由事件驱动（这一点是刻意的，见文末「为什么不用 VAD 边沿」）：
 *
 *   进入对话（Listening / Speaking）
 *       │  每 20ms 一拍：
 *       │    ① 读 radar_snapshot() → 方位角 → 目标角
 *       │    ② 目标角做一阶低通（SERVO_TRACK_SMOOTH）—— 否则雷达抖动会透传给舵机
 *       │    ③ 舵机朝目标角**限速逼近**（kMaxStepDegPerTick），差 < 死区就不动
 *       ▼
 *   退出对话（Idle / Starting / …）
 *         目标角设回 SERVO_CENTER_DEG → 同一条限速路径平滑转回去（不是"啪"一下）
 *
 * 三个保护缺一不可：**低通**挡雷达抖动、**限速**挡突变、**死区**让人静止时头完全停住。
 *
 * ── 移植说明 ──────────────────────────────────────────────────────────
 * 从 brufik / 小歪 的 firmware/head.cpp（驱动）+ radar.cpp（角度）改写。
 * Arduino → ESP-IDF：
 *      ESP32Servo 库 attach(pin,1000,2000) → ledc_timer_config / ledc_channel_config
 *      servo.write(deg)                    → ledc_set_duty / ledc_update_duty
 *      millis()                            → esp_timer_get_time() / 1000
 *      constrain()                         → clampi()
 *      log_warn()                          → ESP_LOGW()
 *
 * 【相对原项目改掉的两样】
 *   · **pb 协议的 LOOK 模式位**（原项目服务端只发 `xm=3`，角度由设备回调算）：
 *     小智没有 pb，转向是纯设备本地行为，那层间接没有存在意义。
 *   · **两个独立的符号旋钮**：原项目的 `LOOK_GAIN` 作用于 LD2450 **原始** x_mm，
 *     而 `RADAR_X_SIGN` 作用于规范化后的 x_mm —— 两者打架时会出现
 *     「LLM 说人在左边、头却往右转」。本移植统一取自 radar_snapshot()，
 *     与 radar_where_text() 同源，只留 SERVO_LOOK_GAIN 一个（标定装配方向）。
 *
 * 【为什么不用 VAD 边沿触发】（前一版就是那么写的，已废弃）
 *   VAD 只给「电平」，而我们真正要做的是「整段对话期间一直朝向你」这件**持续的事**。
 *   用上升沿去逼近它有两个毛病：
 *     1. 一轮里 VAD 会反复 true/false（说话本来就有停顿）→ 头被**反复触发**，
 *        看起来像在跟踪，其实完全取决于你的停顿节奏；你安静走动它就不理你。
 *     2. `voice_detected_` 电平还会跨轮残留（EnableVoiceProcessing(false) 不发回调），
 *        边沿在有些轮次会落空。
 *   改成状态驱动之后这两个问题都不存在：跟踪就是"在对话中"这个层条件的直接结果。
 */

#define TAG "servo"

namespace {

/* ── 任务 ────────────────────────────────────────────────────────────── */

/** 只用 3KB：本任务只做「读两个状态 + 一趟三角函数 + 写一次 LEDC」，
 *  没有解析、没有格式化（比雷达任务的 8KB 轻得多）。 */
constexpr size_t kTaskStack = 3 * 1024;
/** 优先级 3：与雷达任务同档。跟踪不是实时任务，一拍 20ms 绰绰有余。 */
constexpr UBaseType_t kTaskPrio = 3;
/** 钉在 APP core（core 1）—— core 0 常驻 WiFi/协议栈，别去抢。 */
constexpr BaseType_t kTaskCore = 1;
/** 控制节拍（ms）。50Hz，与舵机 PWM 频率一致，一拍走一步。 */
constexpr uint32_t kTickMs = 20;

/* ── 由配置推导出的量 ────────────────────────────────────────────────── */

/** 每拍最多转多少度 —— 由「转满 180° 需要 SERVO_LOOK_MS」换算。
 *  下限夹到 1，避免 SERVO_LOOK_MS 被填得过大时头几乎不动；
 *  SERVO_LOOK_MS 填 0 会让 constexpr 除法直接编译失败（报错信息很难看），所以先夹一层。 */
constexpr int calc_max_step() {
  const int v = 180 * (int)kTickMs / ((SERVO_LOOK_MS) > 0 ? (int)(SERVO_LOOK_MS) : 1);
  return v > 0 ? v : 1;
}
constexpr int kMaxStepDegPerTick = calc_max_step();

/** 低通系数，夹到 (0,1]。填 0 会让目标永不更新、填 >1 会过冲。 */
constexpr float kSmooth =
    ((SERVO_TRACK_SMOOTH) > 0.0f && (SERVO_TRACK_SMOOTH) <= 1.0f) ? (SERVO_TRACK_SMOOTH)
                                                                 : 0.25f;

/** 「跟踪中连续多久看不到人」才打一次提示（ms）。 */
constexpr uint32_t kLostWarnMs = 5000;

/* ── LEDC 资源 ─────────────────────────────────────────────────────────
 * 本板运行期 LEDC 无冲突（已核全仓库）：
 *   · boards/common/backlight.cc 用 LEDC_TIMER_0 + CHANNEL_0，但本板
 *     GetBacklight() 返回 nullptr → 不实例化；
 *   · led/gpio_led.cc 用 LEDC_TIMER_1 + CHANNEL_0，但本板 GetLed() 返回
 *     NoLed → 不实例化；
 *   · otto-robot / electron-bot / lceda 的 oscillator.cc 也用 LEDC_TIMER_1，
 *     但那些是**别的板级目录**，file(GLOB) 只收本板目录，不会被编进来。 */
constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_1;
constexpr ledc_channel_t kLedcChanX = LEDC_CHANNEL_0;
#if SERVO_Y_ENABLE
constexpr ledc_channel_t kLedcChanY = LEDC_CHANNEL_1;
#endif
/** PWM 周期（µs）：50Hz = 20ms，舵机的标准频率。 */
constexpr uint32_t kPeriodUs = 20000;
/** duty 满量程（13 位）。duty = pulse_us / 20ms * 8192。 */
constexpr uint32_t kDutyFull = 1u << 13;

/* ── 状态 ──────────────────────────────────────────────────────────────
 * ⚠️ 下面这些**全部只有 servo 任务自己读写**，所以不需要锁：
 *    setup_servo() 在板级构造函数（主任务）里、**先于** xTaskCreatePinnedToCore
 *    跑完；任务起来之后就没有第二个访问者了。
 *    （本目录的 radar.cc 同样是"单任务独占状态、不加锁"的写法。）
 *    ⚠️ 如果哪天改成从别的任务/回调（例如 Led::OnStateChanged）驱动舵机，
 *       就必须补锁 —— 那样状态就有两个所有者了。 */
TaskHandle_t s_task = nullptr;
bool s_ready = false;

/** 当前角度（度）。这是「PWM 目标角」，**没有物理反馈**（舵机不带编码器）。 */
int s_x_deg = SERVO_CENTER_DEG;
/** 上一次真正写进 LEDC 的角度 —— 避免静止时每 20ms 重复写寄存器。 */
int s_written_x_deg = -1;

/** **滤波后**的目标角（浮点）。跟踪时朝它逼近；退出对话时它被设回中位。
 *  用浮点是为了让低通有亚度的精度 —— 如果存成 int，滤波在小差值时会被抹成 0。 */
float s_target_deg = SERVO_CENTER_DEG;

/** 上一次真的在雷达里看到人的时刻。 */
uint32_t s_last_seen_ms = 0;
/** 上一次周期日志的时刻。 */
uint32_t s_last_log_ms = 0;
/** 「跟踪中看不到人」是否已经提示过（看到人后复位）。 */
bool s_lost_warned = false;
/** 上一拍是否在跟踪；-1 表示尚未初始化（首拍不打跳变日志）。 */
int s_prev_tracking = -1;

/* ── 小工具 ──────────────────────────────────────────────────────────── */

int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/** 毫秒时基。int64 微秒 → uint32 毫秒，49.7 天回绕一次
 *  （与原项目 millis() 的行为一致；所有差值比较都用无符号回绕安全的写法）。 */
uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

/** 角度 → 脉宽（µs）。
 *
 *  分母固定 180：脉宽标定本身是「0~180° ↔ SERVO_PULSE_MIN/MAX_US」这件事，
 *  与软件限位无关 —— 限位只负责挡角度，不该改变标定曲线。 */
int deg_to_pulse_us(int deg) {
  deg = clampi(deg, 0, 180);
  return SERVO_PULSE_MIN_US +
         (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * deg / 180;
}

void ledc_write(ledc_channel_t ch, int pulse_us) {
  const uint32_t duty = (uint32_t)pulse_us * kDutyFull / kPeriodUs;
  ledc_set_duty(kLedcMode, ch, duty);
  ledc_update_duty(kLedcMode, ch);
}

void write_x(int deg) {
  deg = clampi(deg, SERVO_X_MIN_DEG, SERVO_X_MAX_DEG);
  if (deg == s_written_x_deg) {
    return; /* 静止时不重复写寄存器 */
  }
  s_written_x_deg = deg;
  ledc_write(kLedcChanX, deg_to_pulse_us(deg));
}

/* ── 跟踪 ────────────────────────────────────────────────────────────── */

/** 当前状态算不算「对话中」。
 *
 *  **Speaking 也算**：机器人回答时头要一直朝你 —— 否则每轮它一开口头就甩回正中、
 *  你说下一句再转回来，逐轮重复，很晃眼。只有回到 Idle 这类状态才回中。 */
bool is_conversing(DeviceState st) {
  return st == kDeviceStateListening || st == kDeviceStateSpeaking;
}

/** 读一次雷达，把**滤波后**的目标角更新到 s_target_deg。
 *  @return true = 本次真的看到了可用目标。 */
bool update_target_from_radar() {
  const radar_snapshot_t s = radar_snapshot();

  /* present 表示「LD2450 有任何一个有效目标」（不限距离，人在 3m 外也算）；
   * y_mm <= 0 则表示**没有目标落在雷达的距离窗口** [200, 3000]mm 内
   * （此时 x_mm/y_mm 都是 0）。跟踪要的是后者。 */
  if (!s.present || s.y_mm <= 0) {
    return false;
  }

  /* bearing：机身正前方 = 0°，右侧为正、左侧为负
   * —— 直接来自 x_mm 的契约（**负 = 机器人左侧**，见 radar.h）。
   * 因此 LLM 用 radar_where_text() 说的方位与本函数算出的朝向**必然一致**。 */
  const float bearing =
      atan2f((float)s.x_mm, (float)s.y_mm) * 180.0f / (float)M_PI;
  if (!isfinite(bearing)) {
    return false;
  }

  const int raw = clampi(SERVO_CENTER_DEG + (int)lroundf(bearing * SERVO_LOOK_GAIN),
                         SERVO_X_MIN_DEG, SERVO_X_MAX_DEG);

  /* 一阶低通。⚠️ 必须做：LD2450 的坐标抖几十 mm，1m 处就是好几度，
   * 不滤的话目标角一直跳、头跟着抖。 */
  s_target_deg += ((float)raw - s_target_deg) * kSmooth;
  return true;
}

/** 朝目标角走一拍：限速 + 死区。
 *
 *  **限速**（每拍最多 kMaxStepDegPerTick 度）：目标角突变时（唤醒首次锁定 /
 *  重新锁定 / 回中）头不会瞬移，而是以固定最大角速度追过去。
 *  **死区**：差值小于 SERVO_TRACK_DEADBAND_DEG 就不动 —— 人静止时头彻底停住，
 *  不会因为滤波后的微小漂移一直做无意义的微调。 */
void step_toward_target() {
  const int target =
      clampi((int)lroundf(s_target_deg), SERVO_X_MIN_DEG, SERVO_X_MAX_DEG);
  const int err = target - s_x_deg;

  if (err < SERVO_TRACK_DEADBAND_DEG && err > -SERVO_TRACK_DEADBAND_DEG) {
    return; /* 死区内：不动 */
  }

  int next;
  if (err > 0) {
    next = (err <= kMaxStepDegPerTick) ? target : s_x_deg + kMaxStepDegPerTick;
  } else {
    next = (-err <= kMaxStepDegPerTick) ? target : s_x_deg - kMaxStepDegPerTick;
  }
  s_x_deg = clampi(next, SERVO_X_MIN_DEG, SERVO_X_MAX_DEG);
  write_x(s_x_deg);
}

/** 跟踪期间的周期日志（可用 SERVO_TRACK_LOG_MS=0 关掉）。
 *  调试滤波/死区时看「目标 vs 实际」最直观。 */
void log_tracking(uint32_t now) {
#if SERVO_TRACK_LOG_MS > 0
  if (s_last_log_ms != 0 && (uint32_t)(now - s_last_log_ms) < SERVO_TRACK_LOG_MS) {
    return;
  }
  s_last_log_ms = now;
  ESP_LOGW(TAG, "[SERVO] 跟踪中：目标 %.1f° 舵机 %d°", (double)s_target_deg, s_x_deg);
#else
  (void)now;
#endif
}

/* ── 开机自检 ────────────────────────────────────────────────────────── */

#if SERVO_SELF_TEST_ON_BOOT

/** 阻塞地把 X 转到 to 度，耗时 ms（线性插值）。
 *
 *  **为什么单独写一份阻塞版**：正常跟踪是「任务循环每拍推一次」，
 *  自检要按顺序走完三段并各停 5 秒 —— 写成顺序代码最直观。
 *  只在开机自检里用，不属于正常路径。 */
void move_x_blocking(int to, uint32_t ms) {
  to = clampi(to, SERVO_X_MIN_DEG, SERVO_X_MAX_DEG);
  const int from = s_x_deg;
  if (ms == 0 || from == to) {
    s_x_deg = to;
    write_x(s_x_deg);
    return;
  }
  const uint32_t t0 = now_ms();
  for (;;) {
    const uint32_t el = (uint32_t)(now_ms() - t0);
    if (el >= ms) {
      break;
    }
    s_x_deg = from + (int)((long)(to - from) * (long)el / (long)ms);
    write_x(s_x_deg);
    vTaskDelay(pdMS_TO_TICKS(kTickMs));
  }
  s_x_deg = to;
  write_x(s_x_deg);
}

/** 开机自检：MAX → MIN → 归中，每站停 SERVO_SELF_TEST_HOLD_MS。
 *
 *  两端刻意用 SERVO_X_MIN/MAX_DEG（而不是写死 0/180）—— 你收窄软件限位之后，
 *  自检走过的正好是程序实际会用的那段行程，眼睛看到的和代码里的一致。 */
void run_self_test() {
  /* 行程：归中（上电已就位）→ MAX 停 5s → MIN 停 5s → 归中（不停，直接收尾）。 */
  const int stations[3] = {SERVO_X_MAX_DEG, SERVO_X_MIN_DEG, SERVO_CENTER_DEG};
  const unsigned total_s =
      (unsigned)((2 * SERVO_SELF_TEST_HOLD_MS + 3 * SERVO_LOOK_MS) / 1000);
  ESP_LOGW(TAG, "[SERVO] 自检开始：%d° → %d°(停%us) → %d°(停%us) → 归中，共约 %us",
           SERVO_CENTER_DEG, stations[0], (unsigned)(SERVO_SELF_TEST_HOLD_MS / 1000),
           stations[1], (unsigned)(SERVO_SELF_TEST_HOLD_MS / 1000), total_s);
  ESP_LOGW(TAG, "[SERVO] 脉宽标定：0°=%dµs  90°=%dµs  180°=%dµs（两端不对就改 SERVO_PULSE_*_US）",
           deg_to_pulse_us(0), deg_to_pulse_us(90), deg_to_pulse_us(180));

  for (int i = 0; i < 3; ++i) {
    move_x_blocking(stations[i], SERVO_LOOK_MS);
    if (i < 2) {
      /* 只有前两站停 —— 最后一站（归中）不停，直接接上正常跟踪。
       * 打上**实际发出的脉宽**：标定时一边看舵盘走了多少度、一边对这条日志，
       * 就能算出这颗舵机每度要多少 µs（= 该把 SERVO_PULSE_*_US 设成多少）。 */
      ESP_LOGW(TAG, "[SERVO] 自检停在 %d°（脉宽 %dµs，停 %us）", stations[i],
               deg_to_pulse_us(stations[i]), (unsigned)(SERVO_SELF_TEST_HOLD_MS / 1000));
      vTaskDelay(pdMS_TO_TICKS(SERVO_SELF_TEST_HOLD_MS));
    }
  }

  /* 自检直接改了 s_x_deg，把目标角同步过去，免得下一拍又被拉回去。 */
  s_target_deg = (float)s_x_deg;
  ESP_LOGW(TAG, "[SERVO] 自检结束，停在归中 %d°，开始正常跟踪", SERVO_CENTER_DEG);
}

#endif /* SERVO_SELF_TEST_ON_BOOT */

/* ── 任务 ────────────────────────────────────────────────────────────── */

void task_loop_servo(void* /*arg*/) {
#if SERVO_SELF_TEST_ON_BOOT
  /* 开机自检放在**任务里**（而不是 setup_servo 里）—— 它要顺序花掉十几秒，
   * 若放在构造函数里会把整个启动流程卡住（屏幕/音频都跟着晚）。 */
  run_self_test();
#endif

  for (;;) {
    auto& app = Application::GetInstance();
    const DeviceState st = app.GetDeviceState();
    const uint32_t now = now_ms();
    const bool tracking = is_conversing(st);

    /* 状态跳变日志：进/出对话各一条，方便对着观察。 */
    if (s_prev_tracking != (tracking ? 1 : 0)) {
      s_prev_tracking = tracking ? 1 : 0;
      if (tracking) {
        s_last_seen_ms = now; /* 从"此刻"起算丢目标计时，免得一进来就报"已丢失很久" */
        s_lost_warned = false;
        ESP_LOGW(TAG, "[SERVO] 进入对话 → 开始跟踪（对话期间持续朝向你）");
      } else {
        ESP_LOGW(TAG, "[SERVO] 退出对话 → 回中 %d°", SERVO_CENTER_DEG);
      }
    }

    if (tracking) {
      if (update_target_from_radar()) {
        s_last_seen_ms = now;
        s_lost_warned = false;
      } else if (!s_lost_warned &&
                 (uint32_t)(now - s_last_seen_ms) >= kLostWarnMs) {
        /* ⚠️ 限频：本函数**每 20ms** 被调一次，不限频就是 50 条/秒的日志洪水。
         * （照原项目 mic.cpp 里短读告警的限频做法。） */
        s_lost_warned = true;
        ESP_LOGW(TAG, "[SERVO] 雷达已 %us 看不到人 —— 保持当前朝向不动"
                      "（人完全静止时运动雷达会丢目标，属正常）",
                 (unsigned)(kLostWarnMs / 1000));
      }
      /* 看不到人时**故意什么都不做**：目标角维持上一次的值，头就停在原地。
       * 若改成"丢了就回中"，雷达一抖头就会来回摆。 */
      log_tracking(now);
    } else {
      /* 退出对话：目标角回中位。位置由 step_toward_target() 限速走过去，
       * 所以是一次**平滑回正**，不是"啪"一下。 */
      s_target_deg = (float)SERVO_CENTER_DEG;
    }

    step_toward_target();
    vTaskDelay(pdMS_TO_TICKS(kTickMs));
  }
}

}  // namespace

/* ── 公开接口 ────────────────────────────────────────────────────────── */

bool setup_servo() {
#if !SERVO_LOOK_ENABLE
  ESP_LOGW(TAG, "[SERVO] SERVO_LOOK_ENABLE=0 —— 舵机未启用");
  return false;
#else
  if (s_ready) {
    return true;
  }

  ledc_timer_config_t timer = {.speed_mode = kLedcMode,
                               .duty_resolution = LEDC_TIMER_13_BIT,
                               .timer_num = kLedcTimer,
                               .freq_hz = 50,
                               .clk_cfg = LEDC_AUTO_CLK};
  esp_err_t err = ledc_timer_config(&timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[SERVO] LEDC 定时器配置失败: %s", esp_err_to_name(err));
    return false;
  }

  /* duty 先给 0（不输出脉冲），紧接着 write_x() 就会写上归中角。
   * ⚠️ 不要在这里直接填一个非归中的 duty —— 上电瞬间会先弹到那个角度。 */
  ledc_channel_config_t ch = {.gpio_num = SERVO_X_GPIO,
                              .speed_mode = kLedcMode,
                              .channel = kLedcChanX,
                              .intr_type = LEDC_INTR_DISABLE,
                              .timer_sel = kLedcTimer,
                              .duty = 0,
                              .hpoint = 0};
  err = ledc_channel_config(&ch);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[SERVO] LEDC 通道配置失败: %s", esp_err_to_name(err));
    return false;
  }

#if SERVO_Y_ENABLE
  ledc_channel_config_t chy = {.gpio_num = SERVO_Y_GPIO,
                               .speed_mode = kLedcMode,
                               .channel = kLedcChanY,
                               .intr_type = LEDC_INTR_DISABLE,
                               .timer_sel = kLedcTimer,
                               .duty = 0,
                               .hpoint = 0};
  if (ledc_channel_config(&chy) == ESP_OK) {
    ledc_write(kLedcChanY, deg_to_pulse_us(SERVO_Y_CENTER_DEG));
  } else {
    ESP_LOGE(TAG, "[SERVO] Y 通道配置失败（X 不受影响）");
  }
#endif

  /* 开机归中：给出一个确定的初始朝向。 */
  s_x_deg = SERVO_CENTER_DEG;
  s_target_deg = (float)SERVO_CENTER_DEG;
  s_written_x_deg = -1; /* 强制真的写一次 */
  write_x(s_x_deg);

  s_ready = true;
  ESP_LOGW(TAG, "[SERVO] X ready gpio=%d 50Hz %d..%dus center=%d 限速=%d°/拍(%d°/s) 低通=%.2f 死区=%d°",
           (int)SERVO_X_GPIO, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US, SERVO_CENTER_DEG,
           kMaxStepDegPerTick, kMaxStepDegPerTick * 1000 / (int)kTickMs, (double)kSmooth,
           SERVO_TRACK_DEADBAND_DEG);
  return true;
#endif
}

void task_setup_servo() {
  if (s_task || !s_ready) {
    return;
  }
  const BaseType_t rc = xTaskCreatePinnedToCore(task_loop_servo, "servo", kTaskStack,
                                                nullptr, kTaskPrio, &s_task, kTaskCore);
  if (rc != pdPASS) {
    ESP_LOGE(TAG, "[SERVO] task create rc=%d", (int)rc);
    s_task = nullptr;
    return;
  }
  ESP_LOGW(TAG, "[SERVO] task OK stack=%u prio=%u tick=%ums", (unsigned)kTaskStack,
           (unsigned)kTaskPrio, (unsigned)kTickMs);
}

void servo_set_y_deg(int deg) {
#if SERVO_Y_ENABLE
  ledc_write(kLedcChanY, deg_to_pulse_us(clampi(deg, SERVO_Y_MIN_DEG, SERVO_Y_MAX_DEG)));
#else
  (void)deg; /* 本板没装 Y 舵机：接口保留，调用即空操作 */
#endif
}
