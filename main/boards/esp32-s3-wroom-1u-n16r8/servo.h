#pragma once

/* 舵机模块对外接口（水平持续跟踪说话人）。
 *
 * 原项目 brufik / 小歪 里最有"机器人感"的那个动作：你在跟它说话时，
 * 它一直把头转向你；说完不理它了，它自己转回正前方。
 *
 * 【分工】
 *   · 转向的**时机与目标**：本模块自己判定 —— 由**设备状态**驱动
 *       - 对话中（`kDeviceStateListening` / `kDeviceStateSpeaking`）
 *         → 每 20ms 读一次 radar_snapshot()，低通滤波后**限速逼近**
 *       - 退出对话（Idle 等）→ 目标角设回 SERVO_CENTER_DEG，平滑回正
 *   · 转向的**方向**：读 radar 模块的 radar_snapshot()（与 radar_where_text() 同源，
 *     所以"LLM 说的方位"和"头朝的方向"必然一致）
 *
 * ⚠️ 本模块**不是**事件驱动的（不是"检测到说话就转一次"）——
 *    那种写法下，一轮对话里的停顿会让它反复触发，看着像跟踪其实完全取决于
 *    你的说话节奏，安静走动时反而不跟。详见 servo.cc 文件头的说明。
 *
 * 【引脚】不在本模块定义 —— 由板级 `config.h` 的 B 区提供，见 servo_config.h 的说明。
 *
 * 【移植自】brufik / 小歪 的 firmware/head.cpp（ESP32Servo 驱动）
 * 与 radar.cpp 的 look 部分。本版按 ESP-IDF 改写：PWM 走 LEDC 驱动、
 * 日志走 ESP_LOGW，并去掉了 pb 那层间接（小智没有 pb，转向是纯设备本地行为）。
 */

/** 初始化舵机（幂等）：配置 LEDC 定时器/通道，并写一次开机归中角。
 *  @return true = 就绪（可以接着调 task_setup_servo）；
 *          false = 被 SERVO_LOOK_ENABLE=0 关掉，或 LEDC 配置失败。 */
bool setup_servo();

/** 创建舵机任务（幂等）：20ms 一拍，做「按状态跟踪 / 回中 + 限速逼近」。
 *  ⚠️ 必须先 setup_servo() 成功；未就绪时本函数直接返回。 */
void task_setup_servo();

/** 手动设置 Y 轴（俯仰）角度（度）。
 *
 *  本板**没装** Y 舵机，`SERVO_Y_ENABLE=0` 时本函数是空操作（接口保留）。
 *  ⚠️ 该路径**未上机验证**。 */
void servo_set_y_deg(int deg);
