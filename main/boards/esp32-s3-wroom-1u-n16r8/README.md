# ESP32-S3-WROOM-1U-N16R8（小歪 Deskbot v2 主控板）

自研桌面机器人主控板，迁移自 [brufik / 小歪](https://github.com/hucsif/xiaowai-robot) 项目。
本目录是它在 xiaozhi-esp32 上的板级适配。

---

## ⚠️ 先看这条：摄像头和雷达**共用 4 个 GPIO，只能二选一**

| | 引脚 |
|---|---|
| 摄像头 SCCB | `SIOD=9`、`SIOC=10` |
| R60 雷达 UART | `rx=10`、`tx=9` |
| 摄像头 Y3 / Y5 | `18` / `17` |
| LD2450 雷达 UART | `rx=18`、`tx=17` |

**本板选择【雷达】，不接摄像头** —— 所以 `config.h` 里**没有任何 `CAMERA_*` 宏**。

（原项目也是同样的取舍：`DESKBOT_HAS_CAMERA=0`，把摄像头整个关掉、引脚让给雷达。）

---

## 宏的两区划分

`config.h` 里的宏分两类，**看文件名前缀就能分清**：

| 区 | 含义 | 例子 |
|---|---|---|
| **A 区** | **小智原本就有的标准宏** —— 名字/格式照 xiaozhi 约定，值来自本板 | `DISPLAY_MOSI_PIN`、`AUDIO_I2S_SPK_GPIO_*`、`BOOT_BUTTON_GPIO` |
| **B 区** | **本项目新增宏** —— 小智没有这些概念 | `AUDIO_PDM_MIC_*`、`AUDIO_PA_CTRL_GPIO`、`RADAR_*`、`SERVO_*` |

**A 区里的例外**：小智标准的 `AUDIO_I2S_MIC_GPIO_*`（麦克风）**本板不适用** ——
本板麦克风是 **PDM 数字麦**，没有 WS/BCLK，走 B 区的 `AUDIO_PDM_*`。

---

## 引脚对照表

### 音频

| 用途 | 引脚 | 备注 |
|---|---|---|
| 麦克风（PDM） | `CLK=1`、`DATA=2` | 聆麦 LMD2718T271，**纯 PDM，不能走 I2S** |
| 功放 NS4168 | `DIN=40`、`BCLK=41`、`LRC=42` | I2S 标准模式 |
| 功放使能 | `SD=45` | 高电平使能；**GPIO45 是 strapping 脚，只能 boot 后拉高** |
| 采样率 | 入 16k / 出 24k | 与小智默认一致 |

### 显示（ST7789P 240×284，横向用 284×240）

| 用途 | 引脚 |
|---|---|
| SPI | `MOSI=5`、`SCK=4`、`CS=6`、`DC=7` |
| `RST` / `BL` | **硬接 3.3V，无 GPIO**（所以 `config.h` 里是 `GPIO_NUM_NC`） |
| SPI 时钟 | 40 MHz（本项目 v2 走线验证值；曾试 80MHz 会花屏） |
| 颜色 | `invert=true`、BGR |

### 雷达（本板选了雷达）

| 用途 | 引脚 |
|---|---|
| R60ABD1（60GHz 呼吸/睡眠） | UART2：模块 TX→`10`、模块 RX→`9`，115200 |
| LD2450（24GHz 运动追踪） | UART1：模块 TX→`18`、模块 RX→`17`，256000 |

### 舵机

| 用途 | 引脚 |
|---|---|
| X 水平（3.7g，0~180°） | `16` |
| Y 俯仰（9g，70~110°） | `15` |

### 没有的硬件

本板**没有**独立状态 LED（原项目 `led.cpp` 明写「状态 LED 引脚与音频/相机复用，
无硬件可初始化」）、没有触摸键、没有音量键 —— 相关宏全部置 `NC`。

---

## ✅ 注册（已完成）

三处都已改好，且符号名保持一致：

| # | 文件 | 改了什么 |
|---|---|---|
| ① | `main/Kconfig.projbuild` | 加 `config BOARD_TYPE_ESP32_S3_WROOM_1U_N16R8`（板型选项） |
| ② | `main/CMakeLists.txt` | 加 `elseif(CONFIG_BOARD_TYPE_ESP32_S3_WROOM_1U_N16R8)` → `set(BOARD_DIR "esp32-s3-wroom-1u-n16r8")` |
| ③ | 本目录 `esp32_s3_wroom_1u_n16r8.cc` | 板级类：功放使能 + SPI/ST7789 + 音频 codec + 按键 |

### 怎么构建

```bash
cd /home/zhu/esp32/xiaozhi-esp32
idf.py set-target esp32s3
idf.py menuconfig      # 选 Board Type → "ESP32-S3-WROOM-1U-N16R8 (小歪 Deskbot v2 主控板)"
idf.py build flash monitor
```

`menuconfig` 里对应的是：

```
CONFIG_BOARD_TYPE_ESP32_S3_WROOM_1U_N16R8=y
```

## 已知待办

| # | 事项 | 说明 |
|---|---|---|
| ~~1~~ | ~~PDM 数字麦 codec~~ | ✅ **不需要自己写** —— 用小智现成的 `NoAudioCodecSimplexPdm`（`main/audio/codecs/no_audio_codec.h`）：输出 I2S 标准、输入 PDM，正好是本板。已在 `esp32_s3_wroom_1u_n16r8.cc` 里用上 |
| 2 | **显示镜像/偏移校准** | `config.h` 里 `DISPLAY_MIRROR_*` / `SWAP_XY` / `OFFSET_*` 是从常见 ST7789 横向配置起的头，**必须上机看**（本项目原来用 Adafruit 的 `setRotation(3)` + 自定义 xstart，和小智这几个宏不是一对一） |
| 3 | **雷达代码** | 从原项目把整个 `firmware/radar/` 目录搬过来（纯 C，自包含） |
| 4 | **舵机代码** | 原项目 `head.cpp` 的插值逻辑 + 「转向说话人」的 `atan2(x,y)` 映射 |
| 5 | **AEC 参考信号**（待验证） | 原项目是显式喂播放参考（`audio_frontend_submit_playback_reference`）；小智的 `afe_audio_engine` 有自己的 `input_reference_` 概念，**不接对会有回声**（机器人自己说话被当成用户在说） |
| 6 | **分区表 / 16MB 空间** | `config.json` 里只开了 `FLASHSIZE_16MB` + `SPIRAM_MODE_OCT`；若构建报空间不足，再补 `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions/v2/16m.csv"` |

---

## 参考

- 原项目引脚权威来源：`hardware/firmware/deskbot_config.h`、`camera.cpp`、`radar/radar_config.h`
- 本目录 `config.h` 的 A 区格式照抄：`main/boards/bread-compact-wifi-s3cam/config.h`（simplex 音频）
  与 `main/boards/freenove-esp32s3-display-2.8-lcd/config.h`（ST7789 + BGR + invert）
- N16R8 的 `CONFIG_SPIRAM_MODE_OCT=y` 参照：`main/boards/dfrobot/df-k10/config.json`
