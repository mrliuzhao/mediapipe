// 将摄像头麦克风数据解码后重编码为MP3，保存到MP3文件中

#include <stdio.h>
#define __STDC_CONSTANT_MACROS

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libswresample/swresample.h>
#include <libavutil/channel_layout.h>
#include <libavutil/error.h>
#include <libavdevice/avdevice.h>
#include <libavutil/avassert.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
#include <libavutil/timestamp.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/avstring.h>
#include <libavutil/frame.h>
};

#define MAX_AUDIO_FRAME_SIZE 192000 // 1 second of 48khz 32bit audio

// a wrapper around a single output AVStream
typedef struct OutputStream {
    AVStream *st;
    /* 流的编码器上下文 */
    AVCodecContext *enc_ctx;
    /* 流的编码器 */
    AVCodec *enc;
    /* 流在OutputContext中的位置 */
    int stream_idx = -1;
    /* pts of the next frame that will be generated */
    // int64_t next_pts;
    int samples_count;
    AVFrame *frame;
    AVFrame *tmp_frame;
    /* 音频的转换器上下文 */
    struct SwrContext *swr_ctx;
} OutputStream;

/* 音频输入上下文 */
AVFormatContext* audio_in_ctx = nullptr;
/* 音频输入流的位置 */
int audio_stream_idx = -1;
/* 音频解码器上下文 */
AVCodecContext *audio_decode_ctx;

/* 输入的Packet，音视频公用 */
AVPacket input_pkt;
/* 输入的AVFrame，音视频公用 */
AVFrame *input_frame = NULL;
/* 音频专用的FIFO缓冲队列 */
AVAudioFifo *audio_fifo = NULL;

/* 输出音视频上下文 */
AVFormatContext *out_ctx;
/* 输出格式 */
AVOutputFormat *out_fmt;

// 初始化摄像头麦克风输入流，创建输入的解封装（AVFormatContext）以及解码器（AVCodecContext）
int initAudioInput(){
    int ret;

    // 通过alsa库打开默认音频设备
    audio_in_ctx = avformat_alloc_context();
    AVInputFormat *audio_in_fmt = av_find_input_format("alsa");
    char audioInputPath[]= "default";
    ret = avformat_open_input(&audio_in_ctx, audioInputPath, audio_in_fmt, NULL);
    if(ret != 0){
        printf("Couldn't open audio input stream default. return code: %d\n", ret);
        return -1;
    }
    ret = avformat_find_stream_info(audio_in_ctx, NULL);
    if(ret < 0){
        printf("Couldn't find stream information. return code: %d\n", ret);
        return -1;
    }
    for(int i=0; i < audio_in_ctx->nb_streams; i++)
    {
        if(audio_in_ctx->streams[i]->codecpar->codec_type == AVMediaType::AVMEDIA_TYPE_AUDIO){
            audio_stream_idx = i;
            printf("find audio stream id %d\n", audio_stream_idx);
            break;
        }
    }
    if(audio_stream_idx == -1){
        printf("Didn't find a audio stream.\n");
        return -1;
    }
    printf("---------- Audio Input Format ----------\n");
    av_dump_format(audio_in_ctx, audio_stream_idx, audioInputPath, false);
    printf("---------- End ----------\n");

    // 找到音频输入流的解码器
    AVStream *audio_st = audio_in_ctx->streams[audio_stream_idx];
    AVCodec *audio_dec = avcodec_find_decoder(audio_st->codecpar->codec_id);
    if (!audio_dec) {
        printf("Failed to find Audio Decoder.\n");
        return -1;
    }
    audio_decode_ctx = avcodec_alloc_context3(audio_dec);
    if (!audio_decode_ctx) {
        printf("Failed to allocate Audio Decoder Context.\n");
        return -1;
    }
    ret = avcodec_parameters_to_context(audio_decode_ctx, audio_st->codecpar);
    if (ret < 0) {
        printf("Failed to copy Audio Decoder parameters. return code: %d\n", ret);
        return -1;
    }
    ret = avcodec_open2(audio_decode_ctx, audio_dec, NULL);
    if (ret < 0) {
        printf("Failed to open Audio Decoder Context. return code: %d\n", ret);
        return -1;
    }
    return 0;
}


/* Add an output stream. */
int add_stream(OutputStream *ost, enum AVCodecID codec_id)
{
    AVCodecContext *enc_ctx;
    AVCodec *enc;
    /* find the encoder */
    enc = avcodec_find_encoder(codec_id);
    if (!enc) {
        printf("Could not find encoder for '%s'\n", avcodec_get_name(codec_id));
        return -1;
    }
    ost->st = avformat_new_stream(out_ctx, NULL);
    if (!ost->st) {
        printf("Could not allocate new output stream\n");
        return -1;
    }
    ost->st->id = out_ctx->nb_streams-1;
    ost->stream_idx = out_ctx->nb_streams-1;
    enc_ctx = avcodec_alloc_context3(enc);
    if (!enc_ctx) {
        printf("Could not allocate encoding context\n");
        return -1;
    }
    ost->enc_ctx = enc_ctx;
    ost->enc = enc;
    switch (enc->type) {
    case AVMEDIA_TYPE_AUDIO:
        enc_ctx->sample_fmt  = AV_SAMPLE_FMT_S16;
        if (enc->sample_fmts) {
            for (size_t i = 0; enc->sample_fmts[i] != -1; i++)
            {
                if (enc->sample_fmts[i] == AV_SAMPLE_FMT_S16P)
                    enc_ctx->sample_fmt = AV_SAMPLE_FMT_S16P;
            }
        }
        // enc_ctx->bit_rate    = 64000;
        enc_ctx->sample_rate = 48000;
        if (enc->supported_samplerates) {
            enc_ctx->sample_rate = enc->supported_samplerates[0];
            for (int i = 0; enc->supported_samplerates[i]; i++) {
                if (enc->supported_samplerates[i] == 48000)
                    enc_ctx->sample_rate = 48000;
            }
        }
        enc_ctx->channel_layout = AV_CH_LAYOUT_STEREO;
        if (enc->channel_layouts) {
            enc_ctx->channel_layout = enc->channel_layouts[0];
            for (int i = 0; enc->channel_layouts[i]; i++) {
                if (enc->channel_layouts[i] == AV_CH_LAYOUT_STEREO)
                    enc_ctx->channel_layout = AV_CH_LAYOUT_STEREO;
            }
        }
        enc_ctx->channels = av_get_channel_layout_nb_channels(enc_ctx->channel_layout);
        ost->st->time_base.num = 1;
        ost->st->time_base.den = enc_ctx->sample_rate;
        break;
    default:
        break;
    }
    /* Some formats want stream headers to be separate. */
    if (out_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    
    return 0;
}


/* audio output */
AVFrame *alloc_audio_frame(enum AVSampleFormat sample_fmt,
                            uint64_t channel_layout,
                            int sample_rate, int nb_samples)
{
    AVFrame *frame = av_frame_alloc();
    int ret;
    if (!frame) {
        printf("Error allocating an audio frame\n");
        exit(1);
    }
    frame->format = sample_fmt;
    frame->channel_layout = channel_layout;
    frame->sample_rate = sample_rate;
    frame->nb_samples = nb_samples;
    if (nb_samples) {
        ret = av_frame_get_buffer(frame, 0);
        if (ret < 0) {
            printf("Error allocating an audio buffer, return code: %d\n", ret);
            exit(1);
        }
    }
    return frame;
}
/* Open Audio output stream. */
int open_audio_output(OutputStream *ost)
{
    AVCodecContext *c;
    int ret;
    c = ost->enc_ctx;
    /* open it */
    ret = avcodec_open2(c, ost->enc, nullptr);
    if (ret < 0) {
        printf("Could not open audio encoder, return code: %d\n", ret);
        return -1;
    }

    int nb_samples;
    if (c->codec->capabilities & AV_CODEC_CAP_VARIABLE_FRAME_SIZE)
        nb_samples = 10240;
    else
        nb_samples = c->frame_size;
    ost->frame     = alloc_audio_frame(c->sample_fmt, c->channel_layout, c->sample_rate, nb_samples);
    // 为原始输入音频帧做准备
    ost->tmp_frame = NULL;
    // int64_t src_ch_layout = av_get_default_channel_layout(audio_decode_ctx->channels);
    // ost->tmp_frame = alloc_audio_frame(audio_decode_ctx->sample_fmt, src_ch_layout, audio_decode_ctx->sample_rate, audio_decode_ctx->frame_size);
    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        printf("Could not copy audio stream parameters, return code: %d\n", ret);
        return -1;
    }
    /* create resampler context */
    ost->swr_ctx = swr_alloc();
    if (!ost->swr_ctx) {
        printf("Could not allocate audio resampler context\n");
        return -1;
    }
    /* set options */
    ret = av_opt_set_int(ost->swr_ctx, "in_channel_count", audio_decode_ctx->channels, 0);
    if (ret < 0) {
        printf("Error set in_channel_count. Return Code: %d\n", ret);
    }
    ret = av_opt_set_int(ost->swr_ctx, "in_sample_rate", audio_decode_ctx->sample_rate, 0);
    if (ret < 0) {
        printf("Error set in_sample_rate. Return Code: %d\n", ret);
    }
    ret = av_opt_set_sample_fmt(ost->swr_ctx, "in_sample_fmt", audio_decode_ctx->sample_fmt, 0);
    if (ret < 0) {
        printf("Error set in_sample_fmt. Return Code: %d\n", ret);
    }
    ret = av_opt_set_int(ost->swr_ctx, "out_channel_count", c->channels, 0);
    if (ret < 0) {
        printf("Error set out_channel_count. Return Code: %d\n", ret);
    }
    ret = av_opt_set_int(ost->swr_ctx, "out_sample_rate", c->sample_rate, 0);
    if (ret < 0) {
        printf("Error set out_sample_rate. Return Code: %d\n", ret);
    }
    ret = av_opt_set_sample_fmt(ost->swr_ctx, "out_sample_fmt", c->sample_fmt, 0);
    if (ret < 0) {
        printf("Error set out_sample_fmt. Return Code: %d\n", ret);
    }
    int64_t src_ch_layout = av_get_default_channel_layout(audio_decode_ctx->channels);
    ret = av_opt_set_channel_layout(ost->swr_ctx, "in_channel_layout", src_ch_layout, 0);
    if (ret < 0) {
        printf("Error set in_channel_layout. Return Code: %d\n", ret);
    }
    ret = av_opt_set_channel_layout(ost->swr_ctx, "out_channel_layout", c->channel_layout, 0);
    if (ret < 0) {
        printf("Error set out_channel_layout. Return Code: %d\n", ret);
    }

    // 老式设置方式
    // ost->swr_ctx = swr_alloc_set_opts(ost->swr_ctx, c->channel_layout, c->sample_fmt, c->sample_rate, src_ch_layout, audio_decode_ctx->sample_fmt, audio_decode_ctx->sample_rate, 0, NULL);

    /* initialize the resampling context */
    if ((ret = swr_init(ost->swr_ctx)) < 0) {
        printf("Failed to initialize the resampling context, return code: %d\n", ret);
        return -1;
    }

    return 0;
}


/* Decode Audio and store to buffer. */
int decode_and_store(OutputStream *ost)
{
    /** Temporary storage for the converted input samples. */
    uint8_t **converted_input_samples = NULL;
    int ret;

    // 解码音频流
    ret = av_read_frame(audio_in_ctx, &input_pkt);
    if (ret < 0){
        printf("error read from audio\n"); 
        return -1;
    }
    if (input_pkt.stream_index == audio_stream_idx) {
        ret = avcodec_send_packet(audio_decode_ctx, &input_pkt);
        if (ret < 0) {
            printf( "Error decode audio frame at avcodec_send_packet, return value: %d\n", ret);
            return ret;
        }
        while (ret >= 0) {
            ret = avcodec_receive_frame(audio_decode_ctx, input_frame);
            if (ret < 0) {
                if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN))
                    return 0;
                printf( "Error decode audio frame at avcodec_receive_frame, return value: %d\n", ret);
                return -1;
            }
            // 处理每帧数据
            // 初始化转换后数据的空间
            converted_input_samples = (uint8_t **)calloc(ost->enc_ctx->channels, sizeof(*converted_input_samples));
            if (!converted_input_samples) {
                printf( "Could not allocate converted input sample pointers\n");
                return -1;
            }
            ret = av_samples_alloc(converted_input_samples, NULL, ost->enc_ctx->channels, input_frame->nb_samples, ost->enc_ctx->sample_fmt, 0);
            if (ret < 0) {
                printf( "Could not allocate converted input samples, return value: %d\n", ret);
                av_freep(&(*converted_input_samples)[0]);
                free(*converted_input_samples);
                return -1;
            }
            // 转换数据
            ret = swr_convert(ost->swr_ctx, converted_input_samples, input_frame->nb_samples, (const uint8_t**)input_frame->extended_data, input_frame->nb_samples);
            if (ret < 0) {
                printf("error converting audio, return value: %d\n", ret);
                return -1;
            }
            // 将数据加入FIFO队列
            ret = av_audio_fifo_realloc(audio_fifo, av_audio_fifo_size(audio_fifo) + input_frame->nb_samples);
            if (ret < 0) {
                printf( "Could not reallocate FIFO, return value: %d\n", ret);
                return -1;
            }
            /** Store the new samples in the FIFO buffer. */
            ret = av_audio_fifo_write(audio_fifo, (void **)converted_input_samples, input_frame->nb_samples);
            if (ret < (input_frame->nb_samples)) {
                printf( "Could not write data to FIFO\n");
                return -1;
            }
            // 资源清理
            if (converted_input_samples) {
                av_freep(&converted_input_samples[0]);
                free(converted_input_samples);
            }
            av_frame_unref(input_frame);
        }
    }

    return 0;
}

int64_t audio_pts = 0;
int load_encode_and_write(OutputStream *ost)
{
    int ret;
    // 保证从队列中读取每帧必须多的数据，1152个采样点
    if (av_audio_fifo_read(audio_fifo, (void **)ost->frame->data, ost->frame->nb_samples) < ost->frame->nb_samples) {
        printf("Could not read enough data from FIFO\n"); 
        return -1;
    }

    AVPacket output_packet;
    av_init_packet(&output_packet);
    output_packet.data = NULL;
    output_packet.size = 0;
    /** Set a timestamp based on the sample rate for the container. */
    if (ost->frame) {
        ost->frame->pts = audio_pts;
        audio_pts += ost->frame->nb_samples;
    }
    // 编码封装数据
    ret = avcodec_send_frame(ost->enc_ctx, ost->frame);
    if (ret < 0) {
        printf("Error sending a frame to the encoder, return code: %d\n", ret);
        return -1;
    }
    while (ret >= 0) {
        ret = avcodec_receive_packet(ost->enc_ctx, &output_packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0) {
            printf("Error encoding a frame, return code: %d\n", ret);
            av_packet_unref(&output_packet);
            return -1;
        }

        // av_packet_rescale_ts(&output_packet, in_st->time_base, out_st->time_base);

        ret = av_write_frame(out_ctx, &output_packet);
        if (ret < 0) {
            printf("Error while writing output packet, return code: %d\n", ret);
            av_packet_unref(&output_packet);
            return -1;
        }
    }
    av_packet_unref(&output_packet);

    return 0;
}


/* 释放输出流的各种资源 */
void close_output_stream(OutputStream *ost)
{
    avcodec_free_context(&ost->enc_ctx);
    av_frame_free(&ost->frame);
    av_frame_free(&ost->tmp_frame);
    swr_free(&ost->swr_ctx);
}


int main(int argc, char* argv[])
{
    int ret;

    // 注册全部插件
    av_register_all();
    avdevice_register_all();
    avformat_network_init();

    // 初始化摄像头的音视频输入
    ret = initAudioInput();
    if (ret < 0) {
        printf("Failed to Initialize Audio input from camera\n");
        return -1;
    }
    printf("Successfully open audio decoder context.\n");

    // 初始化输出
    // char out_filename[]= "/home/gdcc/audioEncode.mp3";
    char out_filename[]= "rtmp://192.168.20.2/live/demo3";
    OutputStream audio_out_st = { 0 };
    // 从文件名推断输出上下文
    // avformat_alloc_output_context2(&out_ctx, NULL, NULL, out_filename);
    avformat_alloc_output_context2(&out_ctx, NULL, "flv", out_filename);
    if (!out_ctx) {
        printf("Could not deduce output format from file extension.\n");
        return -1;
    }
    out_fmt = out_ctx->oformat;
    if (out_fmt->audio_codec == AV_CODEC_ID_NONE) {
        printf("There is NO default audio codec in Output Context.\n");
        return -1;
    }
    printf("Default audio_codec: %d - %s\n", out_fmt->audio_codec, avcodec_get_name(out_fmt->audio_codec));
    // Default audio_codec: 86017 - mp3

    // 初始化音频输出流，使用默认的编码器
    ret = add_stream(&audio_out_st, out_fmt->audio_codec);
    if (ret < 0) {
        printf("Error when adding audio output stream\n");
        return -1;
    }
    ret = open_audio_output(&audio_out_st);
    if (ret < 0) {
        printf("Error when opening audio output stream\n");
        return -1;
    }
    printf("---------- Output Format ----------\n");
    av_dump_format(out_ctx, 0, out_filename, 1);
    printf("---------- End ----------\n");
// Output #0, mp3, to '/home/gdcc/audioEncode.mp3':
    // Stream #0:0: Audio: mp3, 48000 Hz, stereo, s32p, 64 kb/s

    if (!(out_fmt->flags & AVFMT_NOFILE)) {
        ret = avio_open(&out_ctx->pb, out_filename, AVIO_FLAG_WRITE);
        if (ret < 0) {
            printf("Could not open output URL, return code: %d\n", ret);
            return -1;
        }
    }
    /* Write the stream header, if any. */
    printf("Before write header, audio_out_st.st->time_base: %d / %d; \n", audio_out_st.st->time_base.num, audio_out_st.st->time_base.den);
    ret = avformat_write_header(out_ctx, NULL);
    if (ret < 0) {
        printf("Error when writing header, return code: %d\n", ret);
        return -1;
    }
    printf("After write header, audio_out_st.st->time_base: %d / %d \n", audio_out_st.st->time_base.num, audio_out_st.st->time_base.den);

    printf("---------- Input stream info ----------\n");
    printf("stream.time_base: %d / %d; frame_size: %d; sample_rate: %d; channels: %d\n", audio_in_ctx->streams[audio_stream_idx]->time_base.num, audio_in_ctx->streams[audio_stream_idx]->time_base.den, audio_in_ctx->streams[audio_stream_idx]->codecpar->frame_size, audio_in_ctx->streams[audio_stream_idx]->codecpar->sample_rate, audio_in_ctx->streams[audio_stream_idx]->codecpar->channels);
// stream.time_base: 1 / 1000000; frame_size: 4; sample_rate: 48000; channels: 2
    printf("---------- Input Decoder info ----------\n");
    printf("decoder.time_base: %d / %d; frame_size: %d; sample_rate: %d; sample_fmt: %d; channels: %d; channel_layout: %lld\n", audio_decode_ctx->time_base.num, audio_decode_ctx->time_base.den, audio_decode_ctx->frame_size, audio_decode_ctx->sample_rate, audio_decode_ctx->sample_fmt, audio_decode_ctx->channels, audio_decode_ctx->channel_layout);
// decoder.time_base: 1 / 48000; frame_size: 4; sample_rate: 48000; sample_fmt: 1; channels: 2; channel_layout: 0
    printf("---------- Output Encoder Info ----------\n");
    printf("encoder.time_base: %d / %d; frame_size: %d; sample_rate: %d; sample_fmt: %d; channels: %d; channel_layout: %lld\n", audio_out_st.enc_ctx->time_base.num, audio_out_st.enc_ctx->time_base.den, audio_out_st.enc_ctx->frame_size, audio_out_st.enc_ctx->sample_rate, audio_out_st.enc_ctx->sample_fmt, audio_out_st.enc_ctx->channels, audio_out_st.enc_ctx->channel_layout);
// encoder.time_base: 1 / 48000; frame_size: 1152; sample_rate: 48000; sample_fmt: 7; channels: 2; channel_layout: 3
    if(audio_out_st.enc->supported_samplerates != NULL){
        printf("---------- encoder supported samplerates ----------\n");
        for (size_t i = 0; audio_out_st.enc->supported_samplerates[i]; i++)
        {
            printf("%d ", audio_out_st.enc->supported_samplerates[i]);
        }
        printf("\n");
    } else {
        printf("---------- NO encoder supported samplerates ----------\n");
    }
    if(audio_out_st.enc->sample_fmts != NULL){
        printf("---------- encoder supported sample formats ----------\n");
        for (size_t i = 0; audio_out_st.enc->sample_fmts[i] != -1; i++)
        {
            printf("%d ", audio_out_st.enc->sample_fmts[i]);
        }
        printf("\n");
    } else {
        printf("---------- NO encoder supported sample formats ----------\n");
    }
    if (audio_out_st.enc_ctx->codec->capabilities & AV_CODEC_CAP_VARIABLE_FRAME_SIZE)
        printf("Audio encoder supports receiving a different number of samples in each call\n");
    printf("---------- Output AVFrame Info ----------\n");
    printf("frame.format: %d; channel_layout: %d; sample_rate: %d; nb_samples: %d\n", audio_out_st.frame->format, audio_out_st.frame->channel_layout, audio_out_st.frame->sample_rate, audio_out_st.frame->nb_samples);
// frame.format: 7; channel_layout: 3; sample_rate: 48000; nb_samples: 1152

    // 准备音频缓冲队列
    // audio_fifo = av_audio_fifo_alloc(audio_out_st.enc_ctx->sample_fmt, audio_out_st.enc_ctx->channels, audio_out_st.enc_ctx->frame_size);
    audio_fifo = av_audio_fifo_alloc(audio_out_st.enc_ctx->sample_fmt, audio_out_st.enc_ctx->channels, 1);
    if (!audio_fifo) {
        printf("Error initialize Audio FIFO\n");
        return -1;
    }

    // 准备输入数据帧和数据包
    input_frame = av_frame_alloc();
    if (!input_frame) {
        printf("Error initialize input frame\n");
        return -1;
    }
    av_init_packet(&input_pkt);
    input_pkt.data = NULL;
    input_pkt.size = 0;

    // 开始读取->解码->编码->打包->推送循环
    bool keepOutput = true;
    int sendCount = 1000;
    int dst_nb_samples;
    // 记录输出流中每个Packet必须的采样点数量
    const int output_frame_size = audio_out_st.enc_ctx->frame_size;
    while (keepOutput) {
        // 先检查可否向输出流中写入数据
        if (av_frame_make_writable(audio_out_st.frame) < 0) {
            printf( "Audio Output stream frame cannot be written\n");
            keepOutput = false;
            continue;
        }

        // 队列中数据少于输出必须采样点数量时仅解码并储存在临时
        while (av_audio_fifo_size(audio_fifo) < output_frame_size) {
            ret = decode_and_store(&audio_out_st);
            if (ret < 0) {
                printf( "Error when decoding and store audio data\n");
                keepOutput = false;
                continue;
            }
            // 每次循环清理数据
            av_packet_unref(&input_pkt);
        }

        // 队列中数据多于输出必须采样点数量时开始写入输出流
        while (av_audio_fifo_size(audio_fifo) >= output_frame_size) {
            ret = load_encode_and_write(&audio_out_st);
            if (ret < 0) {
                printf( "Error when writing into file\n");
                keepOutput = false;
                continue;
            }
        }
        sendCount--;
        if (sendCount <= 0) {
            keepOutput = false;
        }
    }
    printf("Finish loop...\n");

    // Write trailer to output stream
    av_write_trailer(out_ctx);

    // Release Resouces
    if (audio_fifo)
        av_audio_fifo_free(audio_fifo);

    // close input
    avcodec_free_context(&audio_decode_ctx);
    avformat_close_input(&audio_in_ctx);

    av_packet_unref(&input_pkt);
    av_frame_free(&input_frame);

    /* Close output. */
    close_output_stream(&audio_out_st);
    if (!(out_fmt->flags & AVFMT_NOFILE))
        avio_closep(&out_ctx->pb);
    avformat_free_context(out_ctx);

    return 0;
}
