#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

/* ══════════════════════════════════════════════════════════════════════
 * ESP32-S3-WROOM-1U-N16R8（16MB Flash + 8MB Octal PSRAM）
 * —— 小歪 / Deskbot v2 自研主控板
 *
 * 引脚来源：本项目（brufik / 小歪）的 firmware/deskbot_config.h、camera.cpp、
 * radar/radar_config.h，全部经实机验证。
 *
 * ⚠️⚠️ 本板【摄像头】与【雷达】共用 4 个 GPIO，只能二选一：
 *        摄像头：SIOD=9   SIOC=10   D1(Y3)=18   D3(Y5)=17
 *        雷达  ：R60 rx=10 tx=9   |   LD2450 rx=18 tx=17
 *      本板选择【雷达】，**不接摄像头** —— 所以本文件不含任何 CAMERA_* 宏。
 *      （原项目也是靠 DESKBOT_HAS_CAMERA=0 把摄像头整个关掉的。）
 *
 * ── 本文件的组织方式 ─────────────────────────────────────────────────
 *   A 区 = 小智原本就有的标准宏：名字/格式照抄 xiaozhi 约定，值来自本板
 *   B 区 = 本项目新增宏：小智没有的概念（PDM 麦 / 功放使能 / 雷达 / 舵机）
 * 这样一眼能看出「哪些是照小智规矩填的、哪些是这套硬件特有的」。
 * ══════════════════════════════════════════════════════════════════════ */


/* ═══════════════ A. 小智标准宏（名字照 xiaozhi 约定，值来自本板）═══════════════ */

/* ---- 采样率 ---- */
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

/* ---- 音频输出：I2S 标准模式 → NS4168 功放 ----
 * 命名沿用 xiaozhi 的 simplex 风格（照抄 bread-compact-wifi-s3cam）。
 *
 * ⚠️ 小智标准的 AUDIO_I2S_MIC_GPIO_*（麦克风）本板**不适用** ——
 *    本板麦克风是 PDM 数字麦，引脚与驱动方式见 B 区的 AUDIO_PDM_*。 */
#define AUDIO_I2S_METHOD_SIMPLEX
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_40 /* NS4168 DIN  */
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_41 /* NS4168 BCLK */
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_42 /* NS4168 LRC  */

/* ---- 显示屏：ST7789P 240×284（模块 T183B7-C12-04），本板横向使用 = 284×240 ---- */
#define DISPLAY_MOSI_PIN      GPIO_NUM_5
#define DISPLAY_SCK_PIN       GPIO_NUM_4
#define DISPLAY_CS_PIN        GPIO_NUM_6
#define DISPLAY_DC_PIN        GPIO_NUM_7
#define DISPLAY_RST_PIN       GPIO_NUM_NC /* 屏模块 RST 硬接 3.3V，不经 GPIO */
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_NC /* BL 也硬接 3.3V，无 GPIO 可控 */
#define DISPLAY_SPI_SCLK_HZ   (40 * 1000 * 1000) /* 本板 v2 走线验证值（曾试 80MHz 会花屏） */
/* 屏幕挂哪个 SPI 控制器。用 SPI2（ESP32-S3 的 SPI0/1 被 Flash/PSRAM 占用）。 */
#define LCD_SPI_HOST          SPI2_HOST

#define DISPLAY_WIDTH        284 /* 横向宽 */
#define DISPLAY_HEIGHT       240
#define DISPLAY_INVERT_COLOR true /* 对应本项目的 invertDisplay(true) */
#define DISPLAY_RGB_ORDER    LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_SPI_MODE     0

/* ⚠️ 下面这几个「镜像 / 交换 / 偏移」**必须上机校准**，不能照抄：
 *    本项目用的是 Adafruit_ST7789 的 setRotation(3) + 自定义 xstart 偏移
 *    （见 firmware/display_panel.cpp 的 DESKBOT_DISPLAY_ROT3_XSTART_ADJ = -18），
 *    与小智的这几个宏不是一对一映射。
 *
 * 对齐依据：Adafruit 的 setRotation(3) = MADCTL 的 **MX | MV**
 *   → 对应 SWAP_XY=true（MV）+ MIRROR_X=true（MX）、MIRROR_Y=false。
 *
 * 上机实测：SWAP_XY=true 时 panel 的 X 轴就是屏幕【纵向】，
 * 所以缺 MX 表现为【上下颠倒】；现在已补上 MX=true。
 *
 * 若还要调，按这个表翻（SWAP_XY=true 的前提下）：
 *   上下颠倒 → MIRROR_X 翻   左右颠倒 → MIRROR_Y 翻
 *   180° 全反 → 两个都翻      色彩反了 → DISPLAY_INVERT_COLOR 翻 */
#define DISPLAY_MIRROR_X      true
#define DISPLAY_MIRROR_Y      false
#define DISPLAY_SWAP_XY       true
#define DISPLAY_OFFSET_X      0
#define DISPLAY_OFFSET_Y      0

/* ---- 按键 / 指示灯 ----
 * 本板**没有**独立的状态 LED —— firmware/led.cpp 里明写
 *「状态 LED 引脚与音频/相机复用，无硬件可初始化」；
 * 也没有触摸键 / 音量键。全部置 NC，避免小智去初始化不存在的硬件。 */
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define TOUCH_BUTTON_GPIO       GPIO_NUM_NC
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC


/* ═══════════════ B. 本项目新增宏（小智没有这些概念）═══════════════ */

/* ---- PDM 数字麦（聆麦 LMD2718T271，板载）----
 * 纯 PDM 麦：**物理上只有 CLK + DATA 两根线，不可能走 I2S 标准模式**
 * （没有 WS/BCLK 之分）。必须用 ESP32-S3 I2S 外设的 **PDM RX 模式**。
 *
 * 小智现有 codec 里**没有**「PDM 数字麦输入」的先例：
 * 它的 AdcPdmAudioCodec 是 PDM「输出」（PDM 喇叭）+ ADC 模拟麦输入，
 * 与本板正好相反。所以这几个宏是本项目定的，配套要写一个自定义
 * AudioCodec 子类（Read 走 PDM RX，Write 走上面的 I2S STD）。 */
#define AUDIO_PDM_MIC_GPIO_CLK  GPIO_NUM_1
#define AUDIO_PDM_MIC_GPIO_DATA GPIO_NUM_2

/* ---- 功放使能（NS4168 的 SD 脚）----
 * 高电平使能。⚠️ GPIO45 是 strapping 脚，只能 boot 之后拉高，
 * 不要复位前强拉低（原项目 speaker.cpp 里也是这么处理的）。 */
#define AUDIO_PA_CTRL_GPIO      GPIO_NUM_45

/* ---- 雷达（小智没有）----
 * 本板选了雷达、不接摄像头（见文件头的引脚冲突说明）。 */
/* R60ABD1 60GHz 呼吸/睡眠雷达：UART2，模块 TX→GPIO10，模块 RX→GPIO9 */
#define RADAR_R60_UART_NUM   2
#define RADAR_R60_RX_PIN     10
#define RADAR_R60_TX_PIN     9
#define RADAR_R60_BAUD       115200
/* LD2450 24GHz 运动追踪雷达：UART1，模块 TX→GPIO18，模块 RX→GPIO17 */
#define RADAR_LD2450_UART_NUM 1
#define RADAR_LD2450_RX_PIN   18
#define RADAR_LD2450_TX_PIN   17
#define RADAR_LD2450_BAUD     256000

/* ---- 舵机（小智没有）----
 * X = 水平（3.7g 小舵机，0~180°）；Y = 俯仰（9g 大舵机，70~110°）。
 * 原项目把「转向说话人」挂在 X 上：LD2450 的 atan2(x, y) 方位角 → 舵机角。
 * ⚠️ X 轴有装配零点偏差问题，原项目用机械方式校准（见 README）。 */
#define SERVO_X_GPIO GPIO_NUM_16
#define SERVO_Y_GPIO GPIO_NUM_15

#endif // _BOARD_CONFIG_H_
