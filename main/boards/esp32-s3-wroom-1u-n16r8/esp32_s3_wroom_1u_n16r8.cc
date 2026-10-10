/* ESP32-S3-WROOM-1U-N16R8 —— 小歪 / Deskbot v2 自研主控板
 *
 * 这块板和常见 xiaozhi 板的三个不同点，都在下面注释里标了：
 *   1. 麦克风是 **PDM 数字麦**（不是 I2S 标准麦、也不是 ADC 模拟麦）
 *   2. 功放有 **独立的使能脚**（NS4168 的 SD），要自己拉高
 *   3. 屏幕的 RST / BL 都**硬接 3.3V**，没有 GPIO 可控
 *
 * 摄像头：本板**不接**（摄像头与雷达共用 4 个 GPIO，本板把引脚让给雷达），
 * 所以不覆盖 GetCamera()，基类默认返回 nullptr。
 */

#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>

#define TAG "Esp32S3Wroom1UN16R8"

class Esp32S3Wroom1UN16R8 : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_ = nullptr;

    /* 功放使能（NS4168 的 SD 脚，高电平开）。
     *
     * 为什么要在板级做：小智的 NoAudioCodec **不管使能脚**（它只初始化 I2S），
     * 而本板功放不拉高就是静音 —— 表现是「一切正常但没声音」，很容易查半天。
     *
     * ⚠️ GPIO45 是 ESP32-S3 的 strapping 脚，**只能 boot 之后拉高**，
     *    不要在复位前强拉低（原项目 speaker.cpp 也是这么处理的）。 */
    void InitializePaControl() {
        if (AUDIO_PA_CTRL_GPIO == GPIO_NUM_NC) {
            return;
        }
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << AUDIO_PA_CTRL_GPIO;
        cfg.mode = GPIO_MODE_OUTPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
        gpio_set_level(AUDIO_PA_CTRL_GPIO, 1);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC; /* 本板屏只写不读 */
        buscfg.sclk_io_num = DISPLAY_SCK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        /* RST 硬接 3.3V，传 NC 让驱动跳过复位时序 */
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
            }
            app.ToggleChatState();
        });
    }

public:
    Esp32S3Wroom1UN16R8() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializePaControl();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
    }

    /* 音频：PDM 数字麦 + I2S 标准功放。
     *
     * 用小智现成的 NoAudioCodecSimplexPdm，三个词各对应本板一个特征：
     *   - **NoAudioCodec**：本板**没有外挂 codec 芯片** —— NS4168 是纯 I2S DAC，
     *     没有 I2C 寄存器要配，直接用 ESP32 的 I2S 外设即可；
     *   - **Simplex**：收与发各占一个 I2S 控制器（实现里 duplex_ = false），
     *     与本板一致（原项目也是麦在 I2S0、喇叭在 I2S1）；
     *   - **Pdm**：输入走 PDM 模式 —— 本板麦克风 LMD2718T271 是**纯 PDM 麦**，
     *     物理上只有 CLK + DATA 两根线，不可能走 I2S 标准模式。
     *
     * ⚠️ slot 传 **BOTH**：NS4168 只采一个 slot，mono 音频必须同时写进左右两个
     *    slot，否则功放拿到的是静音（原项目 speaker.cpp 里踩过这个坑，
     *    注释写着「NS4168 物理线格式 stereo-both」）。 */
    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplexPdm audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            I2S_STD_SLOT_BOTH,
            AUDIO_PDM_MIC_GPIO_CLK, AUDIO_PDM_MIC_GPIO_DATA);
        return &audio_codec;
    }

    Display* GetDisplay() override { return display_; }

    /* 刻意不覆盖下面三个 —— 基类的默认值正好就是本板要的：
     *   GetLed()         → NoLed（本板没有独立状态 LED）
     *   GetBacklight()   → nullptr（屏的 BL 硬接 3.3V，没有 GPIO 可控）
     *   GetCamera()      → nullptr（本板用雷达、不接摄像头） */
};

DECLARE_BOARD(Esp32S3Wroom1UN16R8);
