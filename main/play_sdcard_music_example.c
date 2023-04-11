/* Example of Voice Activity Detection (VAD)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "board.h"
#include "audio_common.h"
#include "audio_pipeline.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "filter_resample.h"
#include "esp_vad.h"
#include <esp_event.h>

#include "fatfs_stream.h"
#include "mp3_decoder.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "audio_idf_version.h"

static const char *TAG_AUDIO = "Audio Processing";
static const char *TAG_PLAY = "PLAY_SDCARD_MUSIC";

#define VAD_SAMPLE_RATE_HZ 16000
#define VAD_FRAME_LENGTH_MS 30
static int stop_bit = 0;

#define VAD_BUFFER_LENGTH (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)
EventGroupHandle_t audio_playback_event_group;
#define Play_BIT BIT0
/**
 * @brief Audio Processing Code
 * 
 * @param None
 */
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t fatfs_stream_reader, i2s_stream_writer, music_decoder;
    esp_periph_set_handle_t set;

void Audio_processing()
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG_AUDIO, ESP_LOG_INFO);

    audio_pipeline_handle_t pipeline_audio;
    audio_element_handle_t i2s_stream_reader, filter, raw_read;

    ESP_LOGI(TAG_AUDIO, "[ 2 ] Create audio pipeline_audio for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline_audio = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline_audio);

    ESP_LOGI(TAG_AUDIO, "[2.1] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.i2s_config.sample_rate = 48000;
    i2s_cfg.type = AUDIO_STREAM_READER;
#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    i2s_cfg.i2s_port = 1;
#if (ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(4, 0, 0))
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
#endif
#endif
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG_AUDIO, "[2.2] Create filter to resample audio data");
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 48000;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = VAD_SAMPLE_RATE_HZ;
    rsp_cfg.dest_ch = 1;
    filter = rsp_filter_init(&rsp_cfg);

    ESP_LOGI(TAG_AUDIO, "[2.3] Create raw to receive data");
    raw_stream_cfg_t raw_cfg = {
        .out_rb_size = 8 * 1024,
        .type = AUDIO_STREAM_READER,
    };
    raw_read = raw_stream_init(&raw_cfg);

    ESP_LOGI(TAG_AUDIO, "[ 3 ] Register all elements to audio pipeline_audio");
    audio_pipeline_register(pipeline_audio, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline_audio, filter, "filter");
    audio_pipeline_register(pipeline_audio, raw_read, "raw");

    ESP_LOGI(TAG_AUDIO, "[ 4 ] Link elements together [codec_chip]-->i2s_stream-->filter-->raw-->[VAD]");
    const char *link_tag[3] = {"i2s", "filter", "raw"};
    audio_pipeline_link(pipeline_audio, &link_tag[0], 3);

    ESP_LOGI(TAG_AUDIO, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline_audio);

    ESP_LOGI(TAG_AUDIO, "[ 6 ] Initialize VAD handle");
    vad_handle_t vad_inst = vad_create(VAD_MODE_4);

    int16_t *vad_buff = (int16_t *)malloc(VAD_BUFFER_LENGTH * sizeof(short));
    if (vad_buff == NULL) {
        ESP_LOGE(TAG_AUDIO, "Memory allocation failed!");
        goto abort_speech_detection;
    }

    while (1) {
        raw_stream_read(raw_read, (char *)vad_buff, VAD_BUFFER_LENGTH * sizeof(short));

        // Feed samples to the VAD process and get the result
        vad_state_t vad_state = vad_process(vad_inst, vad_buff,VAD_SAMPLE_RATE_HZ, VAD_FRAME_LENGTH_MS);
        if (vad_state == VAD_SPEECH) {
            ESP_LOGI(TAG_AUDIO, "Speech detected");
            stop_bit = 1;

            audio_pipeline_stop(pipeline);
            audio_pipeline_wait_for_stop(pipeline);
            audio_pipeline_terminate(pipeline);

            audio_element_set_uri(fatfs_stream_reader, "/sdcard/audio2.mp3");
            audio_pipeline_reset_ringbuffer(pipeline);
            audio_pipeline_reset_elements(pipeline);
            audio_pipeline_run(pipeline);
            stop_bit = 0;
            //Send vad_buff to cloud for processing
        }
    }

    free(vad_buff);
    vad_buff = NULL;

abort_speech_detection:

    ESP_LOGI(TAG_AUDIO, "[ 7 ] Destroy VAD");
    vad_destroy(vad_inst);

    ESP_LOGI(TAG_AUDIO, "[ 8 ] Stop audio_pipeline and release all resources");
    audio_pipeline_stop(pipeline_audio);
    audio_pipeline_wait_for_stop(pipeline_audio);
    audio_pipeline_terminate(pipeline_audio);

    /* Terminate the pipeline_audio before removing the listener */
    audio_pipeline_remove_listener(pipeline_audio);

    audio_pipeline_unregister(pipeline_audio, i2s_stream_reader);
    audio_pipeline_unregister(pipeline_audio, filter);
    audio_pipeline_unregister(pipeline_audio, raw_read);

    /* Release all resources */
    audio_pipeline_deinit(pipeline_audio);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(filter);
    audio_element_deinit(raw_read);

}

/**
 * @brief Audio Playback for SD card playback list
 * 
 */
void Audio_Playback()
{

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG_PLAY, ESP_LOG_INFO);

    ESP_LOGI(TAG_PLAY, "[3.0] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG_PLAY, "[3.1] Create fatfs stream to read data from sdcard");
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fatfs_stream_reader = fatfs_stream_init(&fatfs_cfg);

    ESP_LOGI(TAG_PLAY, "[3.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

#ifdef CONFIG_AUDIO_SUPPORT_MP3_DECODER
    ESP_LOGI(TAG_PLAY, "[3.3] Create mp3 decoder");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    music_decoder = mp3_decoder_init(&mp3_cfg);
#endif

    ESP_LOGI(TAG_PLAY, "[3.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, fatfs_stream_reader, "file");
    audio_pipeline_register(pipeline, music_decoder, "dec");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG_PLAY, "[3.5] Link it together [sdcard]-->fatfs_stream-->music_decoder-->i2s_stream-->[codec_chip]");
    const char *link_tag[3] = {"file", "dec", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

#ifdef CONFIG_AUDIO_SUPPORT_MP3_DECODER
    ESP_LOGI(TAG_PLAY, "[3.6] Set up uri: /sdcard/test.mp3 ");
    audio_element_set_uri(fatfs_stream_reader, "/sdcard/test.mp3");
#endif

    ESP_LOGI(TAG_PLAY, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG_PLAY, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG_PLAY, "[4.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGI(TAG_PLAY, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);
    // Example of linking elements into an audio pipeline -- END

    ESP_LOGI(TAG_PLAY, "[ 6 ] Listen for all pipeline events");
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_PLAY, "[ * ] Event interface error : %d", ret);
            continue;
        }
        if(stop_bit == 1)
        {
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) music_decoder
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(music_decoder, &music_info);

            ESP_LOGI(TAG_PLAY, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_setinfo(i2s_stream_writer, &music_info);
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
            continue;
        }

        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            ESP_LOGW(TAG_PLAY, "[ * ] Stop event received");
            //Restart playing the waiting song
            audio_element_set_uri(fatfs_stream_reader, "/sdcard/test.mp3");
            audio_pipeline_reset_ringbuffer(pipeline);
            audio_pipeline_reset_elements(pipeline);
            audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
            audio_pipeline_run(pipeline);
            continue;
        }
        
    }/*While Loop*/

    ESP_LOGI(TAG_PLAY, "[ 7 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, fatfs_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, music_decoder);

    /* Terminal the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Stop all periph before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(fatfs_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(music_decoder);
    esp_periph_set_destroy(set);
}


void app_main()
{
    ESP_LOGI(TAG_AUDIO, "[ 1 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

        ESP_LOGI(TAG_PLAY, "[ 1 ] Mount sdcard");
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    set = esp_periph_set_init(&periph_cfg);

    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_1_LINE);

    vTaskDelay(100 /portTICK_PERIOD_MS);
     xTaskCreate(&Audio_processing, "Audio_processing", 1024*9, NULL, 4, NULL);
     xTaskCreate(&Audio_Playback, "Audio_Playback", 1024*10, NULL, 5, NULL);
     
}