#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "soc/gpio_sig_map.h"
#include "codec_es8388.h"
#include "audio_volume.h"
#include "math.h"

#define I2C_MASTER_NUM	0
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL I2C_MASTER_ACK
#define NACK_VAL I2C_MASTER_NACK

#define ES8388_ADDR 0b0010000

#define VOL_DEFAULT 70

#define SAMPLE_RATE     (44100)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (700)
#define PI              (3.14159265)

#define I2S_BCK_IO      (GPIO_NUM_13)
#define I2S_WS_IO       (GPIO_NUM_27)
#define I2S_DO_IO       (GPIO_NUM_14)
#define I2S_DI_IO       (GPIO_NUM_26)
#define IS2_MCLK_PIN	(GPIO_NUM_3)

#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_SCL_IO 18

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

#define SAMPLES			SAMPLE_PER_CYCLE	// Total number of samples left and right
#define	BUF_SAMPLES		SAMPLES * 4			// Size of DMA tx/rx buffer samples * left/right * 2 for 32 bit samples

#define ES8388_DAC_VOL_CFG_DEFAULT() {                      \
    .max_dac_volume = 0,                                    \
    .min_dac_volume = -96,                                  \
    .board_pa_gain = 0,                                     \
    .volume_accuracy = 0.5,                                 \
    .dac_vol_symbol = -1,                                   \
    .zero_volume_reg = 0,                                   \
    .reg_value = 0,                                         \
    .user_volume = 0,                                       \
    .offset_conv_volume = NULL,                             \
}

// DMA Buffers
int16_t rxBuf[BUF_SAMPLES];
int16_t txBuf[BUF_SAMPLES];
static codec_dac_volume_config_t *dac_vol_handle;

static const char *ES_TAG = "ES8388_DRIVER";

uint8_t i2c_write_bulk( uint8_t i2c_bus_addr, uint8_t reg, uint8_t bytes, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, &reg, 1, ACK_CHECK_EN);
    i2c_master_write(cmd, data, bytes, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin( I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}

uint8_t i2c_write( uint8_t i2c_bus_addr, uint8_t reg, uint8_t value)
{
    return i2c_write_bulk( i2c_bus_addr, reg, 1, &value );
}


uint8_t i2c_read( uint8_t i2c_bus_addr, uint8_t reg)
{
	   	uint8_t buffer[2];
	    //printf( "Addr: [%d] Reading register: [%d]\n", i2c_bus_addr, reg );

	    buffer[0] = reg;

	    int ret;
	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	    // Write the register address to be read
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	    i2c_master_write_byte(cmd, buffer[0], ACK_CHECK_EN);

	    // Read the data for the register from the slave
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	    i2c_master_read_byte(cmd, &buffer[0], NACK_VAL);
	    i2c_master_stop(cmd);

	    ret = i2c_master_cmd_begin( I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	    i2c_cmd_link_delete(cmd);

	    //printf( "Read: [%02x]=[%02x]\n", reg, buffer[0] );

	    return (buffer[0]);
}


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

static esp_err_t es_write_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t data)
{
    return i2c_write( ES8388_ADDR, reg_add, data );
}

static esp_err_t es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    *p_data = i2c_read( ES8388_ADDR, reg_add );
    return ESP_OK;
}

static int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        ESP_LOGW(ES_TAG, "Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, volume);
    }
    return res;
}

esp_err_t es8388_init( es_dac_output_t output, es_adc_input_t input )
{
    int res = 0;

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp

    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //normal all and power up all
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, ES_MODE_SLAVE ); //CODEC IN I2S SLAVE MODE

    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);   //vroi=0
    res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);          // 0db

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x19); // Set L1 R1 L2 R2 volume. 0x00: -30dB, 0x1E: 0dB, 0x21: 3dB
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x19);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0x19);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0x19);

    ESP_LOGE(ES_TAG, "Setting DAC Output: %02x", output );
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, output );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb); // MIC Left and Right channel PGA gain


    ESP_LOGE(ES_TAG, "Setting ADC Input: %02x", input );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, input);

    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x02);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0d); // Left/Right data, Left/Right justified mode, Bits length, I2S format
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
    //ALC for Microphone
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode

    codec_dac_volume_config_t vol_cfg = ES8388_DAC_VOL_CFG_DEFAULT();
    dac_vol_handle = audio_codec_volume_init(&vol_cfg);

    return res;
}

esp_err_t es8388_config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt )
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;

    // Set the Format
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Format\n");
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S DAC Format\n");
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
    }

    // Set the Sample bits length
    int bits = (int)bits_length;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Bits: %d\n", bits);
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGE(ES_TAG, "Setting I2S DAC Bits: %d\n", bits);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}


esp_err_t es8388_set_voice_mute(bool enable)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

esp_err_t es8388_start(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
    	printf( "Resetting State Machine\n");

        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   //start state machine
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up ADC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up DAC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= es8388_set_voice_mute(false);
    }

    return res;
}


esp_err_t es8388_set_voice_volume(int volume)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;

    reg = audio_codec_get_dac_reg_value(dac_vol_handle, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, reg);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, reg);
    ESP_LOGI(ES_TAG, "Set volume:%.2d reg_value:0x%.2x dB:%.1f", (int)dac_vol_handle->user_volume, reg,
            audio_codec_cal_dac_volume(dac_vol_handle));
    return res;
}

void es8388_config()
{
    es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;

    es8388_init(output, input);

    es_bits_length_t bits_length = BIT_LENGTH_16BITS;
    es_module_t module = ES_MODULE_DAC;
    es_format_t fmt = I2S_NORMAL;

    es8388_config_i2s(bits_length, ES_MODULE_ADC_DAC, fmt);
    es8388_set_voice_volume(70);
    es8388_start(module);
}

void i2s_init()
{
	i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .use_apll = true,
	    .tx_desc_auto_clear = true,
	    .fixed_mclk = 3,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = IS2_MCLK_PIN,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2);
}


static void setup_sine_waves16( int amplitude )
{
	double sin_float;
    size_t i2s_bytes_write = 0;

    for(int pos = 0; pos < BUF_SAMPLES; pos += 2) {
        sin_float = amplitude * sin( pos/2 * 2 * PI / SAMPLE_PER_CYCLE);

        int lval = sin_float;
        int rval = sin_float;

        txBuf[pos] = lval&0xFFFF;
        txBuf[pos+1] = rval&0xFFFF;
    }
}

void app_main(void)
{
    size_t i2s_bytes_write = 0;

    int amplitude = 8000;
    int start_dir = 50;
    int dir = start_dir;

    i2c_master_init();
	es8388_config();

	i2s_init();

    for (;;) {
/*    	amplitude -= dir;
    	if ( amplitude <= start_dir || amplitude >= 15000 )
    		dir *= -1;
*/
    	setup_sine_waves16( amplitude );

    	//i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	//printf( "Bytes: %d\n", i2s_bytes_write );
    	//vTaskDelay(10/portTICK_RATE_MS);
    }
}
