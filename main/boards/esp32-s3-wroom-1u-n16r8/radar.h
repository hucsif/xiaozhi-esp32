#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_config.h"

/* 雷达采集模块对外接口（R60ABD1 60GHz 呼吸睡眠 + LD2450 24GHz 运动追踪）。
 *
 * 本模块只做「采集 + 解析 + 日志 + 快照」：初始化两路 UART，把原始字节喂给两个
 * 纯 C 解析器，按周期打串口日志，并用 radar_snapshot() 对外暴露一份实时快照。
 * 不含姿态/睡眠等业务状态机，也不做任何网络上报 —— 消费方（MCP 工具）主动来读。
 *
 * 移植自 brufik / 小歪 项目的 firmware/radar/（原项目是 PlatformIO + Arduino）。
 * 本版已按 ESP-IDF 改写：UART 走驱动 API、日志走 ESP_LOGW。
 *
 * 【引脚】不在本模块定义 —— 由板级 `config.h` 的 B 区提供，见 radar_config.h 的说明。
 *
 * 【与原项目的差异】原项目还有「说话转向」「看不见人回中」「状态/事件上行」
 * 和三个动作 FSM（蹲下/挥手/入座），本版**全部未移植**：那些都依赖舵机与自研
 * pb 上行协议，而 xiaozhi 没有舵机概念、工具也是 MCP 拉取式 —— 搬过来没有消费方。
 */

/** 初始化两路雷达 UART + 解析器（幂等）。
 *  @return 始终 true（UART 配置失败会打 ESP_LOGE；接线问题由日志的排障提示暴露）。 */
bool setup_radar();

/** 创建雷达采集任务（幂等）：10ms 轮询两路 UART，按 RADAR_LOG_INTERVAL_MS 输出摘要。 */
void task_setup_radar();

/* ================================================================
 * 实时快照（供 MCP 工具读取）
 * ================================================================ */

/** 雷达实时快照。
 *
 * ⚠️ 各字段是「各自最近一次解析结果」，**不是同一时刻的原子快照** —— 两路雷达
 *    上报频率差很多（心率/呼吸 3s 一帧、LD2450 10Hz），字段间有秒级时间差。
 *
 * 参考系：方位相对**机身正前方**（LD2450 装在身体/底座上，不随任何东西转动）。 */
typedef struct {
  bool    present;     /* LD2450 是否检到目标（运动追踪，人极静时可能丢） */
  int16_t x_mm;        /* 横向 mm；**负 = 机器人左侧**（已按 DESKBOT_RADAR_X_SIGN 规范化） */
  int16_t y_mm;        /* 前方距离 mm */
  uint8_t heart_rate;  /* R60 心率 次/分；0 = 未知（含超 RADAR_VITAL_TTL_MS 未更新） */
  uint8_t breath_rate; /* R60 呼吸率 次/分；0 = 未知 */
} radar_snapshot_t;

/** 取一份当前快照（值语义，不返回内部指针 —— 避免把解析器状态泄给其它任务）。
 *  雷达未启动（setup_radar 未调用）时返回全零快照。 */
radar_snapshot_t radar_snapshot(void);

/** 把快照里的 x/y 变成「左前方 / 正前方 / 右侧」+ 距离的短句。
 *
 *  **为什么在设备端做**：原项目这步在服务端 Python 里做的（`llm_tool_runner.
 *  _radar_where_text`），而 xiaozhi 的服务端是别人的、改不了 —— 所以必须在
 *  交给 MCP 工具之前就在这里格式化好。
 *
 *  @param snap     快照（只读 x_mm / y_mm）
 *  @param out      输出缓冲
 *  @param out_len  缓冲长度（建议 ≥ 64）
 *  @return true = 成功写入；false = 无目标 / 坐标不可用（out 会被置为空串） */
bool radar_where_text(const radar_snapshot_t* snap, char* out, size_t out_len);
