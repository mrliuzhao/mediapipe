// 推送摄像头视频数据到RTMP直播流

#include <stdio.h>
#define __STDC_CONSTANT_MACROS

extern "C"
{
#include <libavutil/avassert.h>
#include <libavutil/error.h>
#include <libavutil/channel_layout.h>
#include <libavformat/avio.h>
#include <libavutil/opt.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
#include <libavdevice/avdevice.h>
#include <libavutil/timestamp.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/avstring.h>
#include <libavutil/frame.h>
};


#define STREAM_DURATION   10.0
/* FPS */
#define STREAM_FRAME_RATE 24
/* default pix_fmt */
#define STREAM_PIX_FMT    AV_PIX_FMT_YUV420P
#define SCALE_FLAGS SWS_BICUBIC
#define FRAME_WIDTH 640;
#define FRAME_HEIGHT 480;

// a wrapper around a single output AVStream
typedef struct OutputStream {
    AVStream *st;
    /* 流的编码器上下文 */
    AVCodecContext *enc_ctx;
    /* 流的编码器 */
    AVCodec *enc;
    /* 流在OutputContext中的位置 */
    int stream_idx = -1;
    /* pts of the next frame that will be generated, related to st_tb */
    int64_t next_pts = 0;
    /* 当前帧的pts，仅供音频帧使用 */
    int64_t cur_pts = 0;
    /* 输出流应该遵循的时间基，视频为FPS的倒数，音频为采样率的倒数 */
    AVRational st_tb;
    AVFrame *frame;
    /* 图像的转换器上下文 */
    struct SwsContext *sws_ctx;
    /* 音频的转换器上下文 */
    struct SwrContext *swr_ctx;
} OutputStream;

// 输入的各种全局变量
/* 视频输入上下文 */
AVFormatContext* video_in_ctx = nullptr;
/* 音频输入上下文 */
AVFormatContext* audio_in_ctx = nullptr;
/* 视频输入流的位置 */
int video_stream_idx = -1;
/* 音频输入流的位置 */
int audio_stream_idx = -1;
/* 视频解码器上下文 */
AVCodecContext *video_decode_ctx;
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


// 初始化摄像头视频输入流，创建输入的解封装（AVFormatContext）以及解码器（AVCodecContext）
int initVideoInput(){
    int ret;
    // 初始化摄像头视频输入上下文
    video_in_ctx = avformat_alloc_context();
    char videoInputPath[]= "/dev/video0";
    AVInputFormat* video_in_fmt = av_find_input_format("video4linux2");
    if(!video_in_fmt)
    {
        printf("Can not find video input format\n");
        return -1;
    }
    ret = avformat_open_input(&video_in_ctx, videoInputPath, video_in_fmt, NULL);
    if(ret < 0){
        printf("Couldn't open video input stream. return code: %d \n", ret);
        return -1;
    }
    ret = avformat_find_stream_info(video_in_ctx, NULL);
    if(ret < 0){
        printf("can't find video input stream info. return code: %d \n", ret);
        return -1;
    }
    for(int i = 0; i < video_in_ctx->nb_streams; i++){
        if(video_in_ctx->streams[i]->codecpar->codec_type == AVMediaType::AVMEDIA_TYPE_VIDEO) {
            video_stream_idx = i;
            printf("find video stream id %d\n", video_stream_idx);
            break;
        }
    }
    if(video_stream_idx == -1){
        printf("Didn't find a video stream.\n");
        return -1;
    }
    printf("---------- Video Input Format ----------\n");
    av_dump_format(video_in_ctx, video_stream_idx, videoInputPath, false);
    printf("---------- End ----------\n");

    // 查找视频流的解码器
    AVStream *video_st = video_in_ctx->streams[video_stream_idx];
    AVCodec *video_dec = avcodec_find_decoder(video_st->codecpar->codec_id);
    if (!video_dec) {
        printf("Failed to find Video Decoder.\n");
        return -1;
    }
    video_decode_ctx = avcodec_alloc_context3(video_dec);
    if (!video_decode_ctx) {
        printf("Failed to allocate Video Decoder Context.\n");
        return -1;
    }
    ret = avcodec_parameters_to_context(video_decode_ctx, video_st->codecpar);
    if (ret < 0) {
        printf("Failed to copy Video Decoder parameters. return code: %d\n", ret);
        return -1;
    }
    ret = avcodec_open2(video_decode_ctx, video_dec, NULL);
    if (ret < 0) {
        printf("Failed to open Video Decoder Context. return code: %d\n", ret);
        return -1;
    }
    return 0;
}


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
        // 音频输出流时间基为采样率的倒数
        ost->st_tb = AVRational{1, enc_ctx->sample_rate};
        break;
    case AVMEDIA_TYPE_VIDEO:
        enc_ctx->codec_id = codec_id;
        // enc_ctx->bit_rate = 400000;
        // 默认与输入大小一致
        enc_ctx->width = video_in_ctx->streams[video_stream_idx]->codecpar->width;
        enc_ctx->height = video_in_ctx->streams[video_stream_idx]->codecpar->height;
        ost->st->time_base.num = 1;
        ost->st->time_base.den = STREAM_FRAME_RATE;
        enc_ctx->time_base = ost->st->time_base;
        enc_ctx->gop_size = 12; /* emit one intra frame every twelve frames at most */
        enc_ctx->pix_fmt = STREAM_PIX_FMT;
        // 视频输出流时间基为FPS的倒数
        ost->st_tb = AVRational{1, STREAM_FRAME_RATE};
        break;
    default:
        break;
    }
    /* Some formats want stream headers to be separate. */
    if (out_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    
    return 0;
}


/* video output */
AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
{
    AVFrame *picture;
    int ret;
    picture = av_frame_alloc();
    if (!picture)
        return NULL;
    picture->format = pix_fmt;
    picture->width  = width;
    picture->height = height;
    /* allocate the buffers for the frame data */
    ret = av_frame_get_buffer(picture, 0);
    if (ret < 0) {
        printf("Could not allocate frame data, return code: %d\n", ret);
        return NULL;
    }
    return picture;
}
/* Open Vido output stream. */
int open_video_output(OutputStream *ost)
{
    int ret;
    AVCodecContext *c = ost->enc_ctx;
    c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    //实时推流，零延迟
    av_opt_set(c->priv_data, "tune", "zerolatency", 0);
    //一组图片的数量
    c->gop_size = 2;
    //去掉B帧只留下 I帧和P帧
    c->max_b_frames = 0;
    /* open the codec */
    ret = avcodec_open2(c, ost->enc, nullptr);
    if (ret < 0) {
        printf("Could not open video encoder, return code: %d\n", ret);
        return -1;
    }
    /* allocate and init a re-usable frame */
    ost->frame = alloc_picture(c->pix_fmt, c->width, c->height);
    if (!ost->frame) {
        printf("Could not allocate video frame\n");
        return -1;
    }
    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        printf("Could not Copy output video stream parameters\n");
        return -1;
    }

    // 再初始化视频转码器
    ost->sws_ctx = sws_getContext(video_decode_ctx->width, video_decode_ctx->height, video_decode_ctx->pix_fmt, c->width, c->height, c->pix_fmt, SWS_BICUBIC, NULL, NULL, NULL);
    if (!ost->sws_ctx)
    {
        printf( "Error creating sws context (image converter), return code: %d\n", ret);
        return -1;
    }

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
        nb_samples = 10000;
    else
        nb_samples = c->frame_size;
    ost->frame     = alloc_audio_frame(c->sample_fmt, c->channel_layout, c->sample_rate, nb_samples);
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


/* Encode and Write frame into output stream. */
int write_frame(OutputStream *ost, AVStream *in_st)
{
    int ret;
    // send the frame to the encoder
    ret = avcodec_send_frame(ost->enc_ctx, ost->frame);
    if (ret < 0) {
        printf("Error sending a frame to the encoder, return code: %d\n", ret);
        return -1;
    }
    while (ret >= 0) {
        AVPacket pkt = { 0 };
        ret = avcodec_receive_packet(ost->enc_ctx, &pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0) {
            printf("Error encoding a frame, return code: %d\n", ret);
            return -1;
        }

        /* rescale output packet timestamp values from codec to stream timebase */
        av_packet_rescale_ts(&pkt, ost->st_tb, ost->st->time_base);

        pkt.stream_index = ost->st->index;
        ret = av_interleaved_write_frame(out_ctx, &pkt);
        // ret = av_write_frame(out_ctx, &pkt);
        av_packet_unref(&pkt);
        if (ret < 0) {
            printf("Error while writing output packet, return code: %d\n", ret);
            return -1;
        }
    }
    return 0;
}

/* 解码视频流数据，并转换为输出流适用的格式，最后写入输出流 */
int push_video(OutputStream *ost) {
    AVCodecContext *c = ost->enc_ctx;

    int ret;
    ret = av_read_frame(video_in_ctx, &input_pkt);
    if (ret < 0) {
        printf( "Error read video frame, return value: %d\n", ret);
        return -1;
    }
    if (input_pkt.stream_index == video_stream_idx)
    {
        // 解码输入视频流
        ret = avcodec_send_packet(video_decode_ctx, &input_pkt);
        if (ret < 0) {
            printf( "Error decode video frame at avcodec_send_packet, return value: %d\n", ret);
            return -1;
        }
        while(ret >= 0) {
            ret = avcodec_receive_frame(video_decode_ctx, input_frame);
            if (ret < 0) {
                if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN))
                    return 0;
                printf( "Error decode video frame at avcodec_receive_frame, return value: %d\n", ret);
                return -1;
            }
            // 转换格式
            sws_scale(ost->sws_ctx, (const uint8_t *const *)input_frame->data, input_frame->linesize, 0, c->height, (uint8_t *const *)ost->frame->data, ost->frame->linesize);

            // AVFrame中的pts暂存目前已发送的帧数
            ost->frame->pts = ost->next_pts++;

            // 发送视频帧
            ret = write_frame(ost, video_in_ctx->streams[video_stream_idx]);
            av_frame_unref(input_frame);
            if(ret < 0) {
                printf( "Error write video frame into output stream\n");
                return -1;
            }
        }
    }
    return 0;
}

/* 解码音频数据并存储于临时缓冲区 */
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
            // 初始化转换后数据的临时空间
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
            if (ret != input_frame->nb_samples) {
                printf("converted nb_samples (%d) is not equal to read nb_samples (%d), Change in Sample Rate!!!\n", ret, input_frame->nb_samples);
            }

            // 队列中没有数据时，记录当前的pts，作为该帧发送时间
            if (av_audio_fifo_size(audio_fifo) == 0) {
                ost->cur_pts = ost->next_pts;
            }
            // 下一帧时间往前推进，暂不考虑采样率变化
            // ost->next_pts += input_frame->nb_samples;

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
/* 从缓冲区读取音频数据并编码发送 */
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
        ost->frame->pts = ost->cur_pts;
        ost->next_pts += ost->frame->nb_samples;
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

        av_packet_rescale_ts(&output_packet, ost->st_tb, ost->st->time_base);

        output_packet.stream_index = ost->st->index;
        // ret = av_write_frame(out_ctx, &output_packet);
        ret = av_interleaved_write_frame(out_ctx, &output_packet);
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
    sws_freeContext(ost->sws_ctx);
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
    ret = initVideoInput();
    if (ret < 0) {
        printf("Failed to Initialize Video input from camera\n");
        return -1;
    }
    ret = initAudioInput();
    if (ret < 0) {
        printf("Failed to Initialize Audio input from camera\n");
        return -1;
    }

    OutputStream video_out_st = { 0 };
    OutputStream audio_out_st = { 0 };
    // 初始化推送数据的输出流上下文
    char out_filename[]= "rtmp://192.168.20.2/live/demo3";
    ret = avformat_alloc_output_context2(&out_ctx, NULL, "flv", out_filename); //RTMP
    if (!out_ctx) {
        printf("Could not allocate output context, return code: %d\n", ret);
        return -1;
    }
    out_fmt = out_ctx->oformat;

    printf("out_fmt->video_codec: %d - %s\n", out_fmt->video_codec, avcodec_get_name(out_fmt->video_codec));
    //out_fmt->video_codec: 22 - flv1
    printf("out_fmt->audio_codec: %d - %s\n", out_fmt->audio_codec, avcodec_get_name(out_fmt->audio_codec));
    // out_fmt->audio_codec: 86017 - mp3

    // 初始化视频输出流
    ret = add_stream(&video_out_st, out_fmt->video_codec);
    // ret = add_stream(&video_out_st, AV_CODEC_ID_H264);
    if (ret < 0) {
        printf("Error when adding video output stream\n");
        return -1;
    }
    ret = open_video_output(&video_out_st);
    if (ret < 0) {
        printf("Error when opening video output stream\n");
        return -1;
    }

    // 初始化音频输出流
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
/*
Output #0, flv, to 'rtmp://192.168.20.2/live/demo3':
    Stream #0:0: Video: h264, yuv420p, 640x480, q=2-31, 24 tbn
    Stream #0:1: Audio: mp3, 48000 Hz, stereo, s16p
*/

    if (!(out_fmt->flags & AVFMT_NOFILE)) {
        ret = avio_open(&out_ctx->pb, out_filename, AVIO_FLAG_WRITE);
        if (ret < 0) {
            printf("Could not open output URL, return code: %d\n", ret);
            return -1;
        }
    }
    /* Write the stream header, if any. */
    // 实时推流，零延迟
    out_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER | AV_CODEC_FLAG_LOW_DELAY;
    av_opt_set(out_ctx->priv_data, "tune", "zerolatency", 0);
    ret = avformat_write_header(out_ctx, NULL);
    if (ret < 0) {
        printf("Error when writing header, return code: %d\n", ret);
        return -1;
    }

    // 准备音频缓冲队列
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
    // 记录输出流中每个Packet必须的采样点数量
    const int output_frame_size = audio_out_st.enc_ctx->frame_size;
    while (keepOutput) {
        int cmp = av_compare_ts(video_out_st.next_pts, video_out_st.st_tb, audio_out_st.next_pts, audio_out_st.st_tb);
        if (cmp <= 0) {
            // 解析并发送视频帧
            if (av_frame_make_writable(video_out_st.frame) < 0) {
                printf( "Video Output stream frame cannot be written\n");
                keepOutput = false;
                continue;
            }
            ret = push_video(&video_out_st);
            if (ret < 0) {
                printf( "Push video error\n");
                keepOutput = false;
                continue;
            }
            av_packet_unref(&input_pkt);
        } else {
            // 解析并发送音频帧
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
                    break;
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
                    break;
                }
            }
        }
    }

    // Write trailer to output stream
    av_write_trailer(out_ctx);

    // Release Resouces
    // close input
    avcodec_free_context(&video_decode_ctx);
    avcodec_free_context(&audio_decode_ctx);
    avformat_close_input(&video_in_ctx);
    avformat_close_input(&audio_in_ctx);

    av_packet_unref(&input_pkt);
    av_frame_free(&input_frame);

    /* Close output. */
    close_output_stream(&video_out_st);
    close_output_stream(&audio_out_st);
    if (!(out_fmt->flags & AVFMT_NOFILE))
        avio_closep(&out_ctx->pb);
    avformat_free_context(out_ctx);

    return 0;
}


