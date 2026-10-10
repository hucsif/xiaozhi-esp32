#pragma once

/* ========== 雷达模块配置（行为参数）==========
 * 两种雷达都是「只上报」的 UART 外设：上电即自动输出数据帧，无需下发配置命令。
 *
 * ⚠️ **引脚 / UART 号 / 波特率不在这里** —— 由板级 `config.h` 的 B 区提供
 *    （`RADAR_R60_UART_NUM` / `RADAR_R60_RX_PIN` / … / `RADAR_LD2450_BAUD`）。
 *    原因：本文件从原项目（brufik）整体搬来，原项目把这些引脚宏也定义在本文件里；
 *    而本板级 config.h 已经定义了同名宏，两处都定义会编译报重复定义。
 *    所以搬过来时把引脚段删掉了，**引脚改由板级 config.h 统一提供**。
 *
 * 本文件只保留「与硬件无关的行为参数」。
 */

/* 总开关：置 0 可完全关闭雷达模块（不初始化 UART、不建任务、不输出日志）。
 * 代码本身始终参与编译，此开关只控制运行期是否启动。 */
#ifndef DESKBOT_RADAR_ENABLE
#define DESKBOT_RADAR_ENABLE 1
#endif

/* 周期摘要日志间隔（ms）；置 0 关闭周期日志（仅保留排障日志）。 */
#ifndef RADAR_LOG_INTERVAL_MS
#define RADAR_LOG_INTERVAL_MS 1000
#endif

/* 某路连续多久无有效帧就打一次排障提示（ms）。
 * 这是验证接线的主要手段，日志文案会直接引导排查方向（零字节 vs 校验失败）。 */
#ifndef RADAR_SILENT_WARN_MS
#define RADAR_SILENT_WARN_MS 5000
#endif

/* 心率/呼吸的新鲜度上限（ms）。R60 是 3 秒一帧，容忍连续几次没更新；
 * 超时后 radar_snapshot() 把该字段置 0（= 未知），而不是端出陈旧读数。
 *
 * ⚠️ 不能改用 R60 的 `last_update_ms` 代替：解析器对**每一帧**都盖那个时间戳，
 *    心率 3 秒才一帧，但体动帧 1Hz 会把它一直刷新 → 永远判「新鲜」。
 *    所以 radar.cpp 用帧回调自己打点（s_heart_ms / s_breath_ms）。 */
#ifndef RADAR_VITAL_TTL_MS
#define RADAR_VITAL_TTL_MS 12000
#endif

/* 方位窗口（mm）：超出 MAX 视为无目标；小于 MIN 视为贴脸噪声
 * （近距离时 atan2 会剧烈跳变）。与摘要日志同口径。 */
#ifndef DESKBOT_RADAR_MIN_DIST_MM
#define DESKBOT_RADAR_MIN_DIST_MM 200
#endif
#ifndef DESKBOT_RADAR_MAX_DIST_MM
#define DESKBOT_RADAR_MAX_DIST_MM 3000
#endif

/* 横向坐标符号标定：把对外契约固定为「负 = 机器人左侧」。
 * ⚠️ 必须实测一次 —— LD2450 数据手册说 x 左负右正，但实际装配朝向可能相反。
 *    标定：人站到机器人【左侧】问「我在哪」；
 *          若答成「右前方」，把本宏改成 -1 重新烧录。 */
#ifndef DESKBOT_RADAR_X_SIGN
#define DESKBOT_RADAR_X_SIGN 1
#endif
