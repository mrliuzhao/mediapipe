// 尝试整合所有内容进行Demo开发
#include <stdio.h>
#define __STDC_CONSTANT_MACROS

#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

#include <SimpleAmqpClient/SimpleAmqpClient.h>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <uuid/uuid.h>

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/gpu/gl_calculator_helper.h"
#include "mediapipe/gpu/gpu_buffer.h"
#include "mediapipe/gpu/gpu_shared_data_internal.h"
#include "CycleBuffer.h"

extern "C"
{
#include <libavdevice/avdevice.h>
#include <libavformat/avio.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavutil/avassert.h>
#include <libavutil/error.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
#include <libavutil/timestamp.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/avstring.h>
#include <libavutil/frame.h>

#include <SDL2/SDL.h>
};

#define M_PI 3.1415926535898
#define MAX_AUDIO_FRAME_SIZE 192000 // 1 second of 48khz 32bit audio

/* Footer文字的相关信息 */
typedef struct FooterTextContext {
    /* 文字模板 */
    boost::format fmt;
    /* 具体的文字内容 */
    std::string content;
    /* 文字在Footer中的位置 */
    cv::Point org;
} FooterTextContext;
/* Footer中各个部分的文字信息 */
FooterTextContext footer_time, footer_count, footer_left, footer_right;

// FFMPeg输出流封装
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
/* 推送的音视频输出流 */
OutputStream video_out_st, audio_out_st;

// FFMPeg输入流封装
typedef struct InputStream {
    /* 具体的输入流 */
    AVStream *st;
    /* 流的解码器上下文 */
    AVCodecContext *dec_ctx;
    /* 流的解码器 */
    AVCodec *dec;
    /* 流在整个输入中的位置 */
    int stream_idx = -1;
    /* 从流中读取的原始数据 */
    AVFrame *frame;
    /* 转换后的数据，用于展示或播放 */
    AVFrame *frameTrans;
    /* 图像的转换器上下文 */
    struct SwsContext *sws_ctx;
    /* 音频的转换器上下文 */
    struct SwrContext *swr_ctx;
} InputStream;
/* 拉取的音视频输入流 */
InputStream video_in_st, audio_in_st;

//定义摄像头图像大小常量
const int CAM_WIDTH = 640;
const int CAM_HEIGHT = 480;
const int CAM_FPS = 30;
/* Footer高度占据窗口的比例 */
const double FOOTER_PERCENT = 0.1;

// 定义各种输入输出流名称
constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream_Video[] = "output_video";
constexpr char kOutputStream_Landmarks[] = "pose_landmarks_every";
constexpr char kWindowName[] = "XRHealer";

/* 摄像头捕获器 */
cv::VideoCapture cam_cap;
/* 视频文件捕获器 */
cv::VideoCapture file_cap;
/* 视频文件全路径 */
std::string video_file_path;

/* 是否保持从文件中读取数据 */
bool keepPlayingVideo = false;
/* 是否保持从RabbitMQ中接收消息 */
bool keepReceiving = false;
/* 是否保持推送音视频 */
bool keepPushing = false;
/* 是否保持拉取音视频 */
bool keepPulling = false;
/* 是否由医生实时指导 */
bool guideByDoc = false;

/* SDL展示窗口大小 */
int window_width, window_height;
/* SDL图像部分高度，为窗口大小减去Footer高度 */
int content_height;

/* 视频文件播放的位置（窗口左半边） */
SDL_Rect left_dst_rect;
/* 实时通信画中画位置 */
SDL_Rect left_inside_dst_rect;
/* 摄像头视频播放位置（窗口右半边） */
SDL_Rect right_dst_rect;
/* Footer的位置（窗口最下方） */
SDL_Rect footer_dst_rect;

/* Footer角度文字模板 */
boost::format angle_fmt("%.1f");
/* Footer时间文字模板 */
boost::format time_fmt("%02d:%02d");
/* N/A文字表示 */
const std::string NASTR("N/A");

/* 计时起点 */
std::chrono::_V2::system_clock::time_point start;
/* 动作计数 */
int act_count = 0;
/* 动作起始判断 */
bool act_start = false;
/* 动作结束判断 */
bool act_end = false;

/* 摄像头具体展示数据 */
cv::Mat cam_disp_data;
/* 左半部分窗口具体展示数据 */
cv::Mat left_disp_data;
/* 左半部分画中画具体展示数据 */
cv::Mat left_inside_data;
/* Footer展示数据 */
cv::Mat footer_data;
/* 空白Footer */
cv::Mat blank_footer;
/* 摄像头推送视频数据 */
cv::Mat cam_push_data;

/* 音频播放缓冲区 */
CCycleBuffer *pSoundBuf;
/* 麦克风输入上下文 */
AVFormatContext* audio_in_ctx = nullptr;
/* 麦克风音频解码器上下文 */
AVCodecContext *audio_decode_ctx;
/* 输出音视频（推流）上下文 */
AVFormatContext *out_ctx;
/* 推送音频专用的FIFO缓冲队列 */
AVAudioFifo *audio_fifo = NULL;

/* 拉流的上下文 */
AVFormatContext* pull_ctx;

/* 音频播放缓冲区 */
uint8_t *copy_buf;
/* 拉流音频接收缓冲区 */
uint8_t *audio_out_buffer;

/* SDL音频播放回调函数 */
void fill_audio(void *udata, Uint8 *stream, int len){ 
    //SDL 2.0
    SDL_memset(stream, 0, len);

    int n = pSoundBuf->Read((char*)copy_buf, len);
    SDL_MixAudio(stream, copy_buf, n, SDL_MIX_MAXVOLUME);
}

/* 将cv::Mat转换为AVFrame，注意输入为BGR格式，输出为AV_PIX_FMT_YUV420P格式 */
int cvmat2avframe(cv::Mat mat, OutputStream *ost) {
    if (!mat.empty()) {
        cv::Mat yuv; // convert to yuv420p first
        cv::cvtColor(mat, yuv, cv::COLOR_BGR2YUV_I420);
        // calc frame size
        int frame_size = mat.cols * mat.rows;
        unsigned char *pdata = yuv.data;
        // fill yuv420
        ost->frame->data[0] = pdata; // fill y
        ost->frame->data[1] = pdata + frame_size; // fill u
        ost->frame->data[2] = pdata + frame_size * 5 / 4; // fill v
        return 0;
    }
    return -1;
}

/* 刷新Footer */
void refreshFooter(){
    // 清空Footer之前的内容
    blank_footer.copyTo(footer_data);
    // 分别格式化后输出
    footer_time.fmt % footer_time.content;
    footer_count.fmt % footer_count.content;
    footer_left.fmt % footer_left.content;
    footer_right.fmt % footer_right.content;
    cv::putText(footer_data, footer_time.fmt.str(), footer_time.org, cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255, 255, 255, 255), 1, cv::FILLED);
    cv::putText(footer_data, footer_count.fmt.str(), footer_count.org, cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255, 255, 255, 255), 1, cv::FILLED);
    cv::putText(footer_data, footer_left.fmt.str(), footer_left.org, cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255, 255, 255, 255), 1, cv::FILLED);
    cv::putText(footer_data, footer_right.fmt.str(), footer_right.org, cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255, 255, 255, 255), 1, cv::FILLED);
}

/* 从视频文件读取视频数据并刷新到left_disp_data */
void file_read(){
    int total_frames = file_cap.get(cv::CAP_PROP_FRAME_COUNT);
    int fps = file_cap.get(cv::CAP_PROP_FPS);
    int waitTime = 33;
    if (fps > 0) {
        waitTime = 1000 / fps;
    }
    int current_idx = 0;
    while(keepPlayingVideo) {
        cv::Mat file_frame_raw;
        file_cap >> file_frame_raw;
        if (file_frame_raw.empty()) {
            continue;
        }
        // 转换色彩空间
        cv::cvtColor(file_frame_raw, file_frame_raw, cv::COLOR_BGR2RGBA);
        // 再resize到合适的大小
        cv::resize(file_frame_raw, left_disp_data, cv::Size(left_dst_rect.w, left_dst_rect.h), 0, 0, cv::INTER_AREA);
        current_idx++;
        // 播放到最后回退到第一帧循环播放
        if (current_idx == total_frames) {
            file_cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            current_idx = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(waitTime));
    }
    file_cap.release();
    // 退出前刷新左侧内容为空白
    left_disp_data = cv::Mat(left_dst_rect.h, left_dst_rect.w, CV_8UC4, cv::Scalar(255, 255, 255, 255));
    printf("Out of Video File Play Loop\n");
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
    if (audio_in_ctx->nb_streams != 1) {
        printf("Incorrect number of streams (%d) in microphone!\n", audio_in_ctx->nb_streams);
        return -1;
    }

    // 找到音频输入流的解码器
    AVStream *audio_st = audio_in_ctx->streams[0];
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

    printf("---------- Audio Input Format ----------\n");
    printf("Audio number of streams: %d\n", audio_in_ctx->nb_streams);
    printf("stream index: %d; decoder: %s; sample format: %s; sample rate: %d; %d channels; bit rate: %lld", 0, avcodec_get_name(audio_in_ctx->audio_codec_id),
    av_get_sample_fmt_name(audio_decode_ctx->sample_fmt), audio_st->codecpar->sample_rate, audio_st->codecpar->channels, audio_st->codecpar->bit_rate);
    // av_dump_format(audio_in_ctx, 0, audioInputPath, 0);
    printf("---------- End ----------\n");

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
        // 默认与输入大小一致
        enc_ctx->width = CAM_WIDTH;
        enc_ctx->height = CAM_HEIGHT;
        ost->st->time_base.num = 1;
        ost->st->time_base.den = CAM_FPS;
        enc_ctx->time_base = ost->st->time_base;
        enc_ctx->gop_size = 12; /* emit one intra frame every twelve frames at most */
        enc_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
        // 视频输出流时间基为FPS的倒数
        ost->st_tb = AVRational{1, CAM_FPS};
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

    // 暂时不需要转换器，输入输出大小相同，且均为AV_PIX_FMT_YUV420P格式
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

/* 初始化推流相关 */
int init_push_output(const char* pushURL){
    video_out_st = { 0 };
    audio_out_st = { 0 };
    int ret;
    // 初始化推送数据的输出流上下文
    ret = avformat_alloc_output_context2(&out_ctx, NULL, "flv", pushURL);
    if (!out_ctx || ret < 0) {
        printf("Could not allocate output context, return code: %d\n", ret);
        return -1;
    }

    // 初始化视频输出流
    ret = add_stream(&video_out_st, AV_CODEC_ID_FLV1);
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
    ret = add_stream(&audio_out_st, AV_CODEC_ID_MP3);
    if (ret < 0) {
        printf("Error when adding audio output stream\n");
        return -1;
    }
    ret = open_audio_output(&audio_out_st);
    if (ret < 0) {
        printf("Error when opening audio output stream\n");
        return -1;
    }

    if (!(out_ctx->oformat->flags & AVFMT_NOFILE)) {
        printf("Going to Open output context\n");
        ret = avio_open(&out_ctx->pb, pushURL, AVIO_FLAG_WRITE);
        if (ret < 0) {
            printf("Could not open output URL, return code: %d\n", ret);
            return -1;
        }
    }
    /* Write the stream header, if any. */
    printf("Going to Write Header to output\n");
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

    printf("---------- Output Format ----------\n");
    for (size_t i = 0; i < out_ctx->nb_streams; i++)
    {
        AVStream *temp_st = out_ctx->streams[i];
        if (temp_st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            printf("Stream: %d: %s: encoder:%s; pixel format: %s; size: %dx%d\n", i, av_get_media_type_string(temp_st->codecpar->codec_type), avcodec_get_name(temp_st->codecpar->codec_id), av_get_pix_fmt_name((AVPixelFormat) temp_st->codecpar->format), temp_st->codecpar->width, temp_st->codecpar->height);
        }
        if (temp_st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            
            printf("Stream: %d: %s: encoder:%s; sample format: %s; sample rate: %d\n", i, av_get_media_type_string(temp_st->codecpar->codec_type), avcodec_get_name(temp_st->codecpar->codec_id), av_get_sample_fmt_name((AVSampleFormat) temp_st->codecpar->format), temp_st->codecpar->sample_rate);
        }
    }
    // av_dump_format(out_ctx, 0, pushURL, 1);
    printf("---------- End ----------\n");

    return 0;
}

/* Encode and Write frame into output stream. */
int write_frame(OutputStream *ost)
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
    ret = cvmat2avframe(cam_push_data, ost);
    if (ret < 0) {
        return 0;
    }
    // AVFrame中的pts暂存目前已发送的帧数
    ost->frame->pts = ost->next_pts++;

    // 发送视频帧
    ret = write_frame(ost);
    if(ret < 0) {
        printf("Error write video frame into output stream\n");
        return -1;
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
    AVFrame *input_frame = av_frame_alloc();
    AVPacket input_pkt;
    av_init_packet(&input_pkt);
    input_pkt.data = NULL;
    input_pkt.size = 0;
    ret = av_read_frame(audio_in_ctx, &input_pkt);
    if (ret < 0){
        printf("error read from audio\n"); 
        return -1;
    }
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
    av_packet_unref(&input_pkt);
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
    /** Set a timestamp based on the sample rate for the container. */
    if (ost->frame) {
        ost->frame->pts = ost->cur_pts;
        ost->next_pts += ost->frame->nb_samples;
    }

    ret = write_frame(ost);
    if(ret < 0) {
        printf( "Error write audio frame into output stream\n");
        return -1;
    }
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
/* 推流具体执行线程方法 */
void push_stream(){
    // 开始读取->解码->编码->打包->推送循环
    // 记录音频输出流中每个Packet必须的采样点数量
    const int output_frame_size = audio_out_st.enc_ctx->frame_size;
    int ret;
    while (keepPushing) {
        int cmp = av_compare_ts(video_out_st.next_pts, video_out_st.st_tb, audio_out_st.next_pts, audio_out_st.st_tb);
        if (cmp <= 0) {
            // 解析并发送视频帧
            if (av_frame_make_writable(video_out_st.frame) < 0) {
                printf( "Video Output stream frame cannot be written\n");
                keepPushing = false;
                continue;
            }
            ret = push_video(&video_out_st);
            if (ret < 0) {
                printf( "Push video error\n");
                keepPushing = false;
                continue;
            }
        } else {
            // 解析并发送音频帧
            if (av_frame_make_writable(audio_out_st.frame) < 0) {
                printf( "Audio Output stream frame cannot be written\n");
                keepPushing = false;
                continue;
            }
            // 队列中数据少于输出必须采样点数量时仅解码并储存在临时缓冲区
            while (av_audio_fifo_size(audio_fifo) < output_frame_size) {
                ret = decode_and_store(&audio_out_st);
                if (ret < 0) {
                    printf( "Error when decoding and store audio data\n");
                    keepPushing = false;
                    break;
                }
            }

            // 队列中数据多于输出必须采样点数量时开始写入输出流
            while (av_audio_fifo_size(audio_fifo) >= output_frame_size) {
                ret = load_encode_and_write(&audio_out_st);
                if (ret < 0) {
                    printf( "Error when writing into file\n");
                    keepPushing = false;
                    break;
                }
            }
        }
    }
    printf("Out of Pushing Stream Loop\n");

    // Write trailer to output stream
    av_write_trailer(out_ctx);

    // Release Resouces
    // close input
    avcodec_free_context(&audio_decode_ctx);
    avformat_close_input(&audio_in_ctx);

    /* Close output. */
    close_output_stream(&video_out_st);
    close_output_stream(&audio_out_st);
    if (!(out_ctx->oformat->flags & AVFMT_NOFILE))
        avio_closep(&out_ctx->pb);
    avformat_free_context(out_ctx);
}

/* 初始化拉流相关 */
int init_pull_input(const char* pullURL) {
    video_in_st = { 0 };
    audio_in_st = { 0 };

    int ret;
    // 分配内存
    pull_ctx = avformat_alloc_context();
    
    // 打开输入流
    AVDictionary* options = NULL;
    av_dict_set(&options, "fflags", "nobuffer", 0);
    ret = avformat_open_input(&pull_ctx, pullURL, NULL, &options);
    if(ret != 0){
        printf("Couldn't open Pull stream.\n");
        return -1;
    }
    ret = avformat_find_stream_info(pull_ctx, NULL);
    if(ret < 0){
        printf("Can't find Pull stream info ");
        return -1;
    }

    //查找视频流和音频流的编号
    int find_n = 0;
    printf("---------- Input Pull Stream Format ----------\n");
    for(int i=0; i < pull_ctx->nb_streams; i++) {
        AVStream *temp_st = pull_ctx->streams[i];
        if(temp_st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_in_st.stream_idx = i;
            video_in_st.st = pull_ctx->streams[i];
            find_n ++;
            printf("Stream: %d: %s: encoder:%s; pixel format: %s; size: %dx%d\n", i, av_get_media_type_string(temp_st->codecpar->codec_type), avcodec_get_name(temp_st->codecpar->codec_id), av_get_pix_fmt_name((AVPixelFormat) temp_st->codecpar->format), temp_st->codecpar->width, temp_st->codecpar->height);
        }
        if(temp_st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            audio_in_st.stream_idx = i;
            audio_in_st.st = pull_ctx->streams[i];
            find_n ++;
            printf("Stream: %d: %s: encoder:%s; sample format: %s; sample rate: %d\n", i, av_get_media_type_string(temp_st->codecpar->codec_type), avcodec_get_name(temp_st->codecpar->codec_id), av_get_sample_fmt_name((AVSampleFormat) temp_st->codecpar->format), temp_st->codecpar->sample_rate);
        }
        if(find_n >= 2){
            break;
        }
    }
    printf("---------- End ----------\n");
//---------- Input Pull Stream Format ----------
// Stream: 0: audio: encoder:aac; sample format: fltp; sample rate: 44100
// Stream: 1: video: encoder:h264; pixel format: yuv420p; size: 480x640
// ---------- End ----------
    if(find_n < 2) {
        printf("Didn't find a video stream.\n");
        return -1;
    }

    //获取解码上下文和解码器
    video_in_st.dec = avcodec_find_decoder(video_in_st.st->codecpar->codec_id);
    audio_in_st.dec = avcodec_find_decoder(audio_in_st.st->codecpar->codec_id);
    if (video_in_st.dec == NULL || audio_in_st.dec == NULL) {
        printf("Cannot Find Suitable Decoder for pull stream.\n");
        return -1;
    }
    AVCodecContext *video_dec_ctx, *audio_dec_ctx;
    video_dec_ctx = avcodec_alloc_context3(video_in_st.dec);
    audio_dec_ctx = avcodec_alloc_context3(audio_in_st.dec);
    if (video_dec_ctx == NULL || audio_dec_ctx == NULL) {
        printf("Cannot Allocate Decoder context.\n");
        return -1;
    }
    ret = avcodec_parameters_to_context(video_dec_ctx, video_in_st.st->codecpar);
    if (ret < 0) {
        printf("Could not Copy Input video stream parameters\n");
        return -1;
    }
    ret = avcodec_parameters_to_context(audio_dec_ctx, audio_in_st.st->codecpar);
    if (ret < 0) {
        printf("Could not Copy Input audio stream parameters\n");
        return -1;
    }
    ret = avcodec_open2(video_dec_ctx, video_in_st.dec, NULL);
    if (ret < 0) {
        printf("Cannot Open Video Decoder for Pull Stream.\n");
        return -1;
    }
    ret = avcodec_open2(audio_dec_ctx, audio_in_st.dec, NULL);
    if (ret < 0) {
        printf("Cannot Open Audio Decoder for Pull Stream.\n");
        return -1;
    }
    video_in_st.dec_ctx = video_dec_ctx;
    audio_in_st.dec_ctx = audio_dec_ctx;

    // 初始化可复用的Frame以接收数据
    video_in_st.frame = alloc_picture(video_dec_ctx->pix_fmt, video_dec_ctx->width, video_dec_ctx->height);
    // 展示数据指定为 AV_PIX_FMT_RGBA 格式，方便转换为cv::Mat
    AVPixelFormat outputFormat = AV_PIX_FMT_RGBA;
    video_in_st.frameTrans = alloc_picture(outputFormat, video_dec_ctx->width, video_dec_ctx->height);
    if ((!video_in_st.frame) || (!video_in_st.frameTrans)) {
        printf("Could not allocate video frame\n");
        return -1;
    }
    // 创建视频大小格式转换器
    video_in_st.sws_ctx = sws_getContext(video_dec_ctx->width, video_dec_ctx->height, video_dec_ctx->pix_fmt, video_dec_ctx->width, video_dec_ctx->height, outputFormat, SWS_BICUBIC, NULL, NULL, NULL); 
    if (!video_in_st.sws_ctx)
    {
        printf("Error creating sws context (image converter) for pull stream, return code: %d\n", ret);
        return -1;
    }

    // 为音频帧预分配内存
    audio_out_buffer = (uint8_t *)av_malloc(MAX_AUDIO_FRAME_SIZE * 2);
    copy_buf = (uint8_t *)av_malloc(MAX_AUDIO_FRAME_SIZE * 2);

    // 创建音频转换器
    audio_in_st.swr_ctx = swr_alloc();
    if (!audio_in_st.swr_ctx) {
        printf("Could not allocate audio resampler context for pull stream\n");
        return -1;
    }
    /* set options */
    ret = av_opt_set_int(audio_in_st.swr_ctx, "in_channel_count", audio_dec_ctx->channels, 0);
    ret = av_opt_set_int(audio_in_st.swr_ctx, "in_sample_rate", audio_dec_ctx->sample_rate, 0);
    ret = av_opt_set_sample_fmt(audio_in_st.swr_ctx, "in_sample_fmt", audio_dec_ctx->sample_fmt, 0);
    int64_t src_ch_layout = av_get_default_channel_layout(audio_dec_ctx->channels);
    ret = av_opt_set_channel_layout(audio_in_st.swr_ctx, "in_channel_layout", src_ch_layout, 0);
    uint64_t out_channel_layout = AV_CH_LAYOUT_STEREO;
    int out_channels = av_get_channel_layout_nb_channels(out_channel_layout);
    ret = av_opt_set_int(audio_in_st.swr_ctx, "out_channel_count", out_channels, 0);
    ret = av_opt_set_int(audio_in_st.swr_ctx, "out_sample_rate", audio_dec_ctx->sample_rate, 0);
    ret = av_opt_set_sample_fmt(audio_in_st.swr_ctx, "out_sample_fmt", AV_SAMPLE_FMT_S16, 0);
    ret = av_opt_set_channel_layout(audio_in_st.swr_ctx, "out_channel_layout", out_channel_layout, 0);
    if ((ret = swr_init(audio_in_st.swr_ctx)) < 0) {
        printf("Failed to initialize the resampling context for pull stream, return code: %d\n", ret);
        return -1;
    }

    // 初始化SDL音频相关，SDL_AudioSpec
    SDL_AudioSpec wanted_spec;
    wanted_spec.freq = audio_dec_ctx->sample_rate; 
    wanted_spec.format = AUDIO_S16SYS; 
    wanted_spec.channels = out_channels; 
    wanted_spec.silence = 0; 
    wanted_spec.samples = audio_dec_ctx->frame_size; 
    wanted_spec.callback = fill_audio; 
    wanted_spec.userdata = audio_dec_ctx; 
    if (SDL_OpenAudio(&wanted_spec, NULL)<0){ 
        printf("Can't open SDL audio.\n"); 
        return -1; 
    }

    return 0;
}
/* 拉流具体执行线程方法 */
void pull_stream(){
    // 提前计算实时指导时应该对视频流的处理
    cv::Rect pull_roi_rect;
    double left_ratio = 1.0 * left_dst_rect.w / left_dst_rect.h;
    pull_roi_rect.x = (video_in_st.frame->width - (left_ratio * video_in_st.frame->height)) / 2.0;
    pull_roi_rect.y = 0;
    pull_roi_rect.width = video_in_st.frame->width - (2 * pull_roi_rect.x);
    pull_roi_rect.height = video_in_st.frame->height;
    AVPacket input_pkt;
    av_init_packet(&input_pkt);
    input_pkt.data = NULL;
    input_pkt.size = 0;
    AVFrame *audioFrame;
    audioFrame = av_frame_alloc();
    // SDL_PauseAudio(0);
    int ret;
    while(keepPulling) {
        SDL_PauseAudio(0);
        while(true) {
            ret = av_read_frame(pull_ctx, &input_pkt);
            if (ret < 0){
                printf("error read from pull stream\n"); 
                keepPulling = false;
                break;
            }

            // 解析视频流
            if (input_pkt.stream_index == video_in_st.stream_idx) {
                int got;
                ret = avcodec_decode_video2(video_in_st.dec_ctx, video_in_st.frame, &got, &input_pkt);
                if(ret < 0){
                    printf( "Error decode video frame at avcodec_send_packet, return value: %d\n", ret);
                    keepPulling = false;
                    break;
                }
                if(got){
                    sws_scale(video_in_st.sws_ctx, (const unsigned char* const*)video_in_st.frame->data, video_in_st.frame->linesize, 0, video_in_st.frame->height, video_in_st.frameTrans->data, video_in_st.frameTrans->linesize);
                    // 直接简单粗暴地通过指针转换为cv::Mat
                    cv::Mat temp = cv::Mat(cv::Size(video_in_st.frameTrans->width, video_in_st.frameTrans->height), CV_8UC4);
                    temp.data = video_in_st.frameTrans->data[0];

                    // 实时指导时将视频流渲染在左半部分
                    if (guideByDoc) {
                        cv::Mat roi = temp(pull_roi_rect);
                        cv::resize(roi, left_disp_data, cv::Size(left_dst_rect.w, left_dst_rect.h), 0, 0, cv::INTER_AREA);
                    } 
                    else  // 非实时指导时视频流渲染在画中画
                    {
                        cv::resize(temp, left_inside_data, cv::Size(left_inside_dst_rect.w, left_inside_dst_rect.h), 0, 0, cv::INTER_AREA);
                    }
                    break;
                }
            }
            // 解析音频流
            else if (input_pkt.stream_index == audio_in_st.stream_idx) {
                int got;
                ret = avcodec_decode_audio4(audio_in_st.dec_ctx, audioFrame, &got, &input_pkt);
                if(got > 0){
                    int rr = swr_convert(audio_in_st.swr_ctx, &audio_out_buffer, MAX_AUDIO_FRAME_SIZE, (const uint8_t **)audioFrame->data , audioFrame->nb_samples);
                    // printf("audioFrame->nb_samples: %d\n", audioFrame->nb_samples); // 1024
                    pSoundBuf->Write((char*)audio_out_buffer, rr * 4);
                }
            }
            av_packet_unref(&input_pkt);
        }
        SDL_Delay(30);
    }
    printf("Out of Pull Stream loop\n"); 

    // Close Converter
    sws_freeContext(video_in_st.sws_ctx);
    swr_free(&audio_in_st.swr_ctx);

    // Close SDL
    SDL_CloseAudio();

    // Free AVFrame
    av_frame_free(&audioFrame);

    // Free av_malloc
    av_free(audio_out_buffer);
    av_free(copy_buf);

    // close Codec
    avcodec_close(video_in_st.dec_ctx);
    avcodec_close(audio_in_st.dec_ctx);

    // close input
    avformat_close_input(&pull_ctx);

    // 退出前刷新左侧内容为空白
    left_disp_data = cv::Mat(left_dst_rect.h, left_dst_rect.w, CV_8UC4, cv::Scalar(255, 255, 255, 255));
    printf("Pull Stream subthread over\n"); 
}

/* 从RabbitMQ中持续读取消息的线程方法 */
void rabbit_receive(){
    std::string receive_queue = "com.gdcccn.rer.device";
    AmqpClient::Channel::ptr_t channel = AmqpClient::Channel::Create("192.168.20.2", 5672, "rer", "gdcc@2021", "rer");
    channel->DeclareQueue(receive_queue, false, true, false, false);
    std::string consumer_tag = channel->BasicConsume(receive_queue, "");

    std::thread file_t;
    std::thread push_t;
    std::thread pull_t;
    while (keepReceiving) {
#pragma region ReceiveAndParse
        AmqpClient::Envelope::ptr_t envelope;
        bool got_msg = channel->BasicConsumeMessage(consumer_tag, envelope, 30);
        if (!got_msg) {
            continue;
        }
        std::string receive_str = envelope->Message()->Body();
        std::cout << "Receive Message from RabbitMQ: " << receive_str << std::endl;
        // Parse String to JSON ptree
        std::stringstream receive_st(receive_str);
        boost::property_tree::ptree receive_json;
        boost::property_tree::ptree msgData;
        std::string command;
        std::string msgType;
        std::string msgId;
        std::string replyQueue;
        try {
            boost::property_tree::json_parser::read_json(receive_st, receive_json);
            command = receive_json.get<std::string>("command");
            msgType = receive_json.get<std::string>("msgType");
            msgId = receive_json.get<std::string>("msgId");
            replyQueue = receive_json.get<std::string>("replyQueue");
            msgData = receive_json.get_child("msgData");
        }
        catch (boost::property_tree::ptree_error &e) {
            std::cout << "Error Occur when parsing JSON" << std::endl;
            continue;
        }
#pragma endregion ReceiveAndParse

#pragma region PrepareReply
        // 根据报文进行相关操作
        std::cout << "Parse Result: " << std::endl;
        std::cout << "    command: " << command << std::endl;
        std::cout << "    msgType: " << msgType << std::endl;
        std::cout << "    msgId: " << msgId << std::endl;
        std::cout << "    replyQueue: " << replyQueue << std::endl;
        boost::property_tree::ptree reply_json;
        boost::property_tree::ptree reply_msg_data;
        std::stringstream reply_st;
        reply_json.put("command", command);
        reply_json.put("msgType", "rpc_response");
        reply_json.put("msgId", msgId);
#pragma endregion PrepareReply

        // 开始训练
        if (command == "startTraining") {
            std::string teachingVideoId = msgData.get<std::string>("teachingVideoId");
            std::cout << "Going to Start Training, teachingVideoId is " << teachingVideoId << std::endl;
            if (!video_file_path.empty()) {
                std::string errorMsg = "目前已有训练视频播放中，训练视频：";
                errorMsg += video_file_path;
                std::cout << errorMsg << std::endl;
                reply_msg_data.put("status", 0);
                reply_msg_data.put("msg", errorMsg);
                reply_json.put_child("msgData", reply_msg_data);
                boost::property_tree::json_parser::write_json(reply_st, reply_json);
                channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
                continue;
            }
            video_file_path = "/home/gdcc/anim2.mp4";
            // 打开视频文件
            std::cout << "Going to open video file: " << video_file_path << std::endl;
            if(!file_cap.open(video_file_path)){
                std::string errorMsg = "视频文件打开失败";
                std::cout << errorMsg << std::endl;
                reply_msg_data.put("status", 0);
                reply_msg_data.put("msg", errorMsg);
                reply_json.put_child("msgData", reply_msg_data);
                boost::property_tree::json_parser::write_json(reply_st, reply_json);
                channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
                continue;
            }
            // 根据视频文件的大小适配窗口
            int file_width = file_cap.get(cv::CAP_PROP_FRAME_WIDTH);
            int file_height = file_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
            double file_ratio = 1.0 * file_width / file_height;
            printf("File Size: %d x %d; File Ratio: %.4f\n", file_width, file_height, file_ratio);

            // 若当前正在实时指导，将医生的视频窗口缩小
            guideByDoc = false;

            // 创建一个线程读取文件数据
            keepPlayingVideo = true;
            act_count = 0;
            start = std::chrono::system_clock::now();  // 记录训练起始时间
            file_t = std::thread(file_read);
            reply_msg_data.put("status", 1);
            reply_json.put_child("msgData", reply_msg_data);
            boost::property_tree::json_parser::write_json(reply_st, reply_json);
            channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
        }

        // 结束训练
        if (command == "stopTraining") {
            std::cout << "Going to Stop Training..." << std::endl;
            if((!keepPlayingVideo) || video_file_path.empty()) {
                std::string errorMsg = "目前还没有播放训练视频";
                std::cout << errorMsg << std::endl;
                reply_msg_data.put("status", 0);
                reply_msg_data.put("msg", errorMsg);
                reply_json.put_child("msgData", reply_msg_data);
                boost::property_tree::json_parser::write_json(reply_st, reply_json);
                channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
                continue;
            }
            keepPlayingVideo = false;
            video_file_path = "";
            if (file_t.joinable()) {
                file_t.join();
            }
            reply_msg_data.put("status", 1);
            reply_json.put_child("msgData", reply_msg_data);
            boost::property_tree::json_parser::write_json(reply_st, reply_json);
            channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
        }

        // 开始推流
        if (command == "startPushStream") {
            std::string pullURL = msgData.get<std::string>("doctorPullSteamUrl");
            std::string pushURL = msgData.get<std::string>("patientPushStreamUrl");
            std::cout << "Going to Push Stream. Push Stream URL: " << pushURL << "; Pull Stream URL: " << pullURL << std::endl;
            int ret;

#pragma region PushStart
            // 初始化摄像头的音频输入
            ret = initAudioInput();
            if (ret < 0) {
                std::string errorMsg = "无法初始化麦克风输入";
                std::cout << errorMsg << std::endl;
                reply_msg_data.put("status", 0);
                reply_msg_data.put("msg", errorMsg);
                reply_json.put_child("msgData", reply_msg_data);
                boost::property_tree::json_parser::write_json(reply_st, reply_json);
                channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
                continue;
            }

            ret = init_push_output(pushURL.c_str());
            if (ret < 0) {
                std::string errorMsg = "初始化推流失败";
                std::cout << errorMsg << std::endl;
                reply_msg_data.put("status", 0);
                reply_msg_data.put("msg", errorMsg);
                reply_json.put_child("msgData", reply_msg_data);
                boost::property_tree::json_parser::write_json(reply_st, reply_json);
                channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
                continue;
            }
#pragma endregion PushStart

#pragma region PullStart
            ret = init_pull_input(pullURL.c_str());
            if (ret < 0) {
                std::string errorMsg = "初始化拉流失败";
                std::cout << errorMsg << std::endl;
                reply_msg_data.put("status", 0);
                reply_msg_data.put("msg", errorMsg);
                reply_json.put_child("msgData", reply_msg_data);
                boost::property_tree::json_parser::write_json(reply_st, reply_json);
                channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
                continue;
            }
#pragma endregion PullStart

            // 创建线程开始推送音视频数据
            keepPushing = true;
            push_t = std::thread(push_stream);

            // 创建线程开始推送音视频数据
            keepPulling = true;
            pull_t = std::thread(pull_stream);

            reply_msg_data.put("status", 1);
            reply_json.put_child("msgData", reply_msg_data);
            boost::property_tree::json_parser::write_json(reply_st, reply_json);
            channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
        }

        // 停止推流
        if (command == "stopPushStream") {
            std::cout << "Stop Push Stream." << std::endl;
            // 停止推流
            keepPushing = false;
            // 停止拉流
            keepPulling = false;
            // 重置实时指导flag
            guideByDoc = false;
            
            // 返回消息后再等待线程结束
            if (push_t.joinable()) {
                push_t.join();
            }
            std::cout << "Wait push thread over." << std::endl;
            if (pull_t.joinable()) {
                pull_t.join();
            }
            reply_msg_data.put("status", 1);
            reply_json.put_child("msgData", reply_msg_data);
            boost::property_tree::json_parser::write_json(reply_st, reply_json);
            channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));

            std::cout << "Wait pull thread over." << std::endl;
        }

        // 开始实时指导
        if (command == "guidance") {
            std::cout << "Start Real Time Guidence." << std::endl;
            // 设置实时指导
            guideByDoc = true;
            reply_msg_data.put("status", 1);
            reply_json.put_child("msgData", reply_msg_data);
            boost::property_tree::json_parser::write_json(reply_st, reply_json);
            channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(reply_st.str()));
        }
    }

    channel->BasicCancel(consumer_tag);

    // 检查子线程是否结束
    if (keepPlayingVideo && file_t.joinable()) {
        keepPlayingVideo = false;
        file_t.join();
    }
    if (keepPushing && push_t.joinable()) {
        keepPushing = false;
        push_t.join();
    }
    if (keepPulling && pull_t.joinable()) {
        keepPulling = false;
        pull_t.join();
    }

}

/* 从文件中读取姿势识别的标准特征 */
std::vector<double> parse_crit_embedding(std::string fileName){
    std::ifstream inFile(fileName, std::ios::in);
    if (!inFile)
    {
        std::cout << "Fail to open embedding file: " << fileName << std::endl;
        exit(1);
    }

    std::cout << "Parsing embedding file: " << fileName << std::endl;
    int line_count = 0;
    std::string line;
    std::string field;
    std::vector<double> col_sum;
    while (getline(inFile, line))
    {
        // 忽略第一行header
        if(line_count > 0) {
            std::istringstream sin(line);
            // 目前确定17列，忽略第一列ID
            for (size_t i = 0; i < 17; i++)
            {
                std::string field;
                getline(sin, field, ',');
                if(i > 0) {
                    double temp = atof(field.c_str());
                    if (col_sum.size() < 16) {
                        col_sum.push_back(temp);
                    }
                    else {
                        col_sum[i - 1] += temp;
                    }
                }
            }
        }
        line_count++;
    }
    inFile.close();
    std::cout << "Read " << (line_count - 1) << " lines" << std::endl;

    std::vector<double> col_mean;
    for (size_t i = 0; i < col_sum.size(); i++)
    {
        col_mean.push_back(col_sum[i] / (line_count - 1));
    }
    return col_mean;
}

/* 计算关节点之间距离 */
double get_distance(const mediapipe::NormalizedLandmark& jointA, const mediapipe::NormalizedLandmark& jointB, double norm_scale) {
    double x1 = jointA.x() * CAM_WIDTH;
    double y1 = jointA.y() * CAM_HEIGHT;
    double x2 = jointB.x() * CAM_WIDTH;
    double y2 = jointB.y() * CAM_HEIGHT;
    return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0)) / norm_scale;
}
/* 计算关节点A到关节点B的向量 */
void get_vector(const mediapipe::NormalizedLandmark& jointA, const mediapipe::NormalizedLandmark& jointB, bool normalize, double vec[2]){
    double x1 = jointA.x() * CAM_WIDTH;
    double y1 = jointA.y() * CAM_HEIGHT;
    double x2 = jointB.x() * CAM_WIDTH;
    double y2 = jointB.y() * CAM_HEIGHT;
    vec[0] = x2 - x1;
    vec[1] = y2 - y1;
    if (normalize) {
        double vec_len = sqrt(pow(vec[0], 2.0) + pow(vec[1], 2.0));
        vec[0] = vec[0] / vec_len;
        vec[1] = vec[1] / vec_len;
    }
}
/* 关节点转换为姿势识别特征向量 */
int get_pose_embedding(mediapipe::NormalizedLandmarkList landmarks, std::vector<double> &pose_embedding) {
    // 检查涉及到的关节点是否可见
    int used_joint_idx[] = {11, 12, 13, 14, 15, 16, 23, 24};
    for (size_t i = 0; i < 8; i++) {
        auto& lm = landmarks.landmark(used_joint_idx[i]);
        if (lm.visibility() < 0.5) {
            return -1;
        }
    }

    // 关节点距离以两肩距离进行归一化
    double norm_scale = get_distance(landmarks.landmark(11), landmarks.landmark(12), 1.0);
    // 右腕（16） - 右肘（14） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(16), landmarks.landmark(14), norm_scale));
    // 右肘（14） - 右肩（12） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(14), landmarks.landmark(12), norm_scale));
    // 右腕（16） - 右胯（24） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(16), landmarks.landmark(24), norm_scale));
    // 右肘（14） - 右胯（24） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(14), landmarks.landmark(24), norm_scale));
    // 左腕（15） - 左肘（13） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(15), landmarks.landmark(13), norm_scale));
    // 左肘（13） - 左肩（11） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(13), landmarks.landmark(11), norm_scale));
    // 左腕（15） - 左胯（23） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(15), landmarks.landmark(23), norm_scale));
    // 左肘（13） - 左胯（23） Norm距离
    pose_embedding.push_back(get_distance(landmarks.landmark(13), landmarks.landmark(23), norm_scale));
    // 右肩（12） -> 右肘（14） Norm向量坐标 x, y
    double temp_arr[2] = {0.0, 0.0};
    get_vector(landmarks.landmark(12), landmarks.landmark(14), true, temp_arr);
    pose_embedding.push_back(temp_arr[0]);
    pose_embedding.push_back(temp_arr[1]);
    // 右肘（14） -> 右腕（16） Norm向量坐标 x, y
    get_vector(landmarks.landmark(14), landmarks.landmark(16), true, temp_arr);
    pose_embedding.push_back(temp_arr[0]);
    pose_embedding.push_back(temp_arr[1]);
    // 左肩（11） -> 左肘（13） Norm向量坐标 x, y
    get_vector(landmarks.landmark(11), landmarks.landmark(13), true, temp_arr);
    pose_embedding.push_back(temp_arr[0]);
    pose_embedding.push_back(temp_arr[1]);
    // 左肘（13） -> 左腕（15） Norm向量坐标 x, y
    get_vector(landmarks.landmark(13), landmarks.landmark(15), true, temp_arr);
    pose_embedding.push_back(temp_arr[0]);
    pose_embedding.push_back(temp_arr[1]);

    return 0;
}
/* 检查姿势特征是否符合某个标准 */
bool check_state(const std::vector<double> pose_embedding, const std::vector<double> crit_embedding, double threshold) {
    if (pose_embedding.size() != crit_embedding.size()) {
        return false;
    }
    double pow_sum = 0.0;
    for (size_t i = 0; i < pose_embedding.size(); i++)
    {
        pow_sum += pow(pose_embedding[i] - crit_embedding[i], 2.0);
    }
    double dist = sqrt(pow_sum);
    return dist < threshold;
}

int main(int argc, char** argv) {
    int ret;

    // 注册全部FFMpeg插件
    av_register_all();
    avformat_network_init();
    avdevice_register_all();

    pSoundBuf = new CCycleBuffer(192000 * 10);

#pragma region CritInit
    std::vector<double> ce_start_crit = parse_crit_embedding("/home/gdcc/PoseEmbedding/ChestExpansion/start.csv");
    std::vector<double> ce_end_crit = parse_crit_embedding("/home/gdcc/PoseEmbedding/ChestExpansion/end.csv");
    printf("ce_start_crit.size(): %d; ce_end_crit.size(): %d\n", ce_start_crit.size(), ce_end_crit.size());
#pragma endregion

#pragma region MediaPipeInit
    printf("Initialize the calculator graph...\n");
    std::string calculator_graph_config_contents = R"(
        input_stream: "input_video"
        output_stream: "output_video"
        output_stream: "pose_landmarks_every"

        node {
        calculator: "FlowLimiterCalculator"
        input_stream: "input_video"
        input_stream: "FINISHED:output_video"
        input_stream_info: {
            tag_index: "FINISHED"
            back_edge: true
        }
        output_stream: "throttled_input_video"
        }

        node {
        calculator: "PoseLandmarkGpu"
        input_stream: "IMAGE:throttled_input_video"
        output_stream: "LANDMARKS:pose_landmarks"
        output_stream: "DETECTION:pose_detection"
        output_stream: "ROI_FROM_LANDMARKS:roi_from_landmarks"
        }

        node {
        calculator: "ImagePropertiesCalculator"
        input_stream: "IMAGE_GPU:throttled_input_video"
        output_stream: "SIZE:image_size"
        }

        node {
        calculator: "LandmarksSmoothingCalculatorV2"
        input_stream: "NORM_LANDMARKS:pose_landmarks"
        input_stream: "IMAGE_SIZE:image_size"
        output_stream: "NORM_FILTERED_LANDMARKS:pose_landmarks_smoothed"
        output_stream: "NORM_FILTERED_LANDMARKS_V2:pose_landmarks_every"
        node_options: {
            [type.googleapis.com/mediapipe.LandmarksSmoothingCalculatorOptions] {
            velocity_filter: {
                window_size: 5
                velocity_scale: 10.0
            }
            }
        }
        }

        node {
        calculator: "PoseRendererGpu"
        input_stream: "IMAGE:throttled_input_video"
        input_stream: "LANDMARKS:pose_landmarks_smoothed"
        input_stream: "ROI:roi_from_landmarks"
        input_stream: "DETECTION:pose_detection"
        output_stream: "IMAGE:output_video"
        }
    )";
    mediapipe::CalculatorGraphConfig config =
        mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(calculator_graph_config_contents);
    mediapipe::Status mediaRet;
    mediapipe::CalculatorGraph graph;
    mediaRet = graph.Initialize(config);
    if (!mediaRet.ok())
    {
        printf("Fail to Initialize Mediapipe Graph! Error Message: %s\n", mediaRet.ToString().c_str());
        return -1;
    }

    printf("Initialize the GPU...\n");
    auto status_or_gpu = mediapipe::GpuResources::Create();
    if (!status_or_gpu.ok()) {
        printf("Fail to assign GPU Resources! Error Message: %s\n", mediaRet.ToString().c_str());
        return -1;
    }
    mediaRet = graph.SetGpuResources(std::move(status_or_gpu).ValueOrDie());
    if (!mediaRet.ok())
    {
        printf("Fail to Initialize Mediapipe Graph! Error Message: %s\n", mediaRet.ToString().c_str());
        return -1;
    }
    mediapipe::GlCalculatorHelper gpu_helper;
    gpu_helper.InitializeForTest(graph.GetGpuResources().get());

    mediapipe::StatusOrPoller status_or_poller_landmarks = graph.AddOutputStreamPoller(kOutputStream_Landmarks);
    if (!status_or_poller_landmarks.ok()) {
        printf("Fail to assign landmarks output stream poller! Error Message: %s\n", mediaRet.ToString().c_str());
        return -1;
    }
    mediapipe::OutputStreamPoller poller_landmark = std::move(status_or_poller_landmarks.ValueOrDie());

    mediapipe::StatusOrPoller status_or_poller_video = graph.AddOutputStreamPoller(kOutputStream_Video);
    if (!status_or_poller_video.ok()) {
        printf("Fail to assign video output stream poller! Error Message: %s\n", mediaRet.ToString().c_str());
        return -1;
    }
    mediapipe::OutputStreamPoller poller_video = std::move(status_or_poller_video.ValueOrDie());

    printf("Start running the calculator graph...\n");
    mediaRet = graph.StartRun({});
    if (!mediaRet.ok())
    {
        printf("Fail to Run Mediapipe Graph! Error Message: %s\n", mediaRet.ToString().c_str());
        return -1;
    }
#pragma endregion MediaPipeInit

#pragma region SDLInit
    // 初始化SDL
    ret = SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER);
    if(ret) {
        printf( "Could not initialize SDL: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    } 
    // 初始化SDL窗口
    SDL_Window* displayWindow = NULL;
    // displayWindow = SDL_CreateWindow(kWindowName, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 960, 540, SDL_WINDOW_OPENGL);
    displayWindow = SDL_CreateWindow(kWindowName, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1280, 720, SDL_WINDOW_OPENGL);
    if(!displayWindow) {
        printf("Could not create SDL window: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_GetWindowSize(displayWindow, &window_width, &window_height);
    // 把底部指定比例为footer所用
    content_height = window_height * (1.0 - FOOTER_PERCENT);
    double content_ratio = 1.0 * window_width / content_height;
    printf("Window Size: %d x %d; Content Ratio: %.4f\n", window_width, window_height, content_ratio);

    // 设置视频播放位置（左半边）
    left_dst_rect.x = 0;
    left_dst_rect.y = 0;
    left_dst_rect.w = window_width / 2;
    left_dst_rect.h = content_height;
    // 设置实时视频画中画位置
    left_inside_dst_rect.w = left_dst_rect.w * 0.2;
    // left_inside_dst_rect.h = left_dst_rect.h * 0.2;
    left_inside_dst_rect.h = left_inside_dst_rect.w * 480.0 / 640.0;  // 暂时固定比例 640x480
    left_inside_dst_rect.x = (window_width / 2) - left_inside_dst_rect.w;
    left_inside_dst_rect.y = 0;
    // 设置摄像头播放位置（右半边）
    right_dst_rect.x = window_width / 2;
    right_dst_rect.y = 0;
    right_dst_rect.w = window_width / 2;
    right_dst_rect.h = content_height;
    // 设置footer位置（最下方）
    footer_dst_rect.x = 0;
    footer_dst_rect.y = content_height;
    footer_dst_rect.w = window_width;
    footer_dst_rect.h = window_height - content_height;

    // 初始化Renderer和Texture，注意Texture已经是要填充的大小
    SDL_Renderer* sdlRenderer = SDL_CreateRenderer(displayWindow, -1, SDL_RENDERER_ACCELERATED);
    if(!sdlRenderer) {
        printf("Could not initialize SDL Renderer: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_Texture* rightContentTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, right_dst_rect.w, right_dst_rect.h);
    if(!rightContentTexture) {
        printf("Could not initialize Right Content SDL Texture: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_Texture* leftContentTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, left_dst_rect.w, left_dst_rect.h);
    if(!leftContentTexture) {
        printf("Could not initialize Left Content SDL Texture: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_Texture* leftInsideTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, left_inside_dst_rect.w, left_inside_dst_rect.h);
    if(!leftContentTexture) {
        printf("Could not initialize Left Inside Content SDL Texture: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_Texture* footTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, footer_dst_rect.w, footer_dst_rect.h);
    if(!footTexture) {
        printf("Could not initialize Footer SDL Texture: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    // 初始化纯白色展示视频
    cam_disp_data = cv::Mat(right_dst_rect.h, right_dst_rect.w, CV_8UC4, cv::Scalar(255, 255, 255, 255));
    left_disp_data = cv::Mat(left_dst_rect.h, left_dst_rect.w, CV_8UC4, cv::Scalar(255, 255, 255, 255));
    left_inside_data = cv::Mat(left_inside_dst_rect.h, left_inside_dst_rect.w, CV_8UC4, cv::Scalar(255, 255, 255, 255));
    blank_footer = cv::Mat(footer_dst_rect.h, footer_dst_rect.w, CV_8UC4, cv::Scalar(94, 133, 155, 160));

    // 设置footer每部分文字内容和位置
    footer_time.fmt = boost::format("Training Time: %s");
    footer_count.fmt = boost::format("Action Count: %s");
    footer_left.fmt = boost::format("Left Abduction: %s");
    footer_right.fmt = boost::format("Right Abduction: %s");
    footer_time.content = NASTR;
    footer_count.content = NASTR;
    footer_left.content = NASTR;
    footer_right.content = NASTR;
    int sideblank = window_width * 0.025;  // 旁边5%为空白
    int part_width = (window_width - (2.0 * sideblank)) / 4.0;  // 剩下的四部分平均分
    footer_time.org = cv::Point(sideblank, footer_dst_rect.h * 0.6);
    footer_count.org = cv::Point(sideblank + part_width, footer_dst_rect.h * 0.6);
    footer_left.org = cv::Point(sideblank + part_width + part_width, footer_dst_rect.h * 0.6);
    footer_right.org = cv::Point(sideblank + part_width + part_width + part_width, footer_dst_rect.h * 0.6);
    refreshFooter();
#pragma endregion SDLInit

#pragma region CameraInit
    // 开启摄像头
    printf("Going to open Camera...\n");
    if(cam_cap.open(0)){
        printf("Successfully open Camera\n");
    } else
    {
        printf("Fail to open Camera\n");
        return EXIT_FAILURE;
    }
    cam_cap.set(cv::CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    cam_cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
    cam_cap.set(cv::CAP_PROP_FPS, CAM_FPS);
    // int cam_w = cam_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    // int cam_h = cam_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    // int cam_fps = cam_cap.get(cv::CAP_PROP_FPS);
    // if ((cam_w != CAM_WIDTH) || (cam_h != CAM_HEIGHT) || (cam_fps != CAM_FPS)) {
    //     printf("Camera is not supported for %dx%d @%d!\n", CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
    //     return EXIT_FAILURE;
    // }
    double cam_ratio = 1.0 * CAM_WIDTH / CAM_HEIGHT;
    printf("Camera Size: %d x %d; Camera Ratio: %.4f; Camera FPS: %d\n", CAM_WIDTH, CAM_HEIGHT, cam_ratio, CAM_FPS);

    // 适配摄像头和窗口
    SDL_Rect cam_src_rect;  // 决定了原图像的ROI位置
    SDL_Rect cam_dst_rect;  // 决定了截图拉伸后在 cam_disp_data 中的大小和位置
    double right_ratio = 1.0 * right_dst_rect.w / right_dst_rect.h;
    if(cam_ratio > right_ratio) {  // 摄像头图像过宽，裁剪两边，左上角对齐拉伸
        cam_src_rect.x = (CAM_WIDTH - (right_ratio * CAM_HEIGHT)) / 2.0;
        cam_src_rect.y = 0;
        cam_src_rect.w = CAM_WIDTH - (2 * cam_src_rect.x);
        cam_src_rect.h = CAM_HEIGHT;
        cam_dst_rect.x = 0;
        cam_dst_rect.y = 0;
        cam_dst_rect.w = right_dst_rect.w;
        cam_dst_rect.h = right_dst_rect.h;
    } else {  // 摄像头图像过高，不做裁剪，拉伸后右移
        cam_src_rect.x = 0;
        cam_src_rect.y = 0;
        cam_src_rect.w = CAM_WIDTH;
        cam_src_rect.h = CAM_HEIGHT;
        cam_dst_rect.x = (right_dst_rect.w - (right_dst_rect.h * cam_ratio)) / 2.0;
        cam_dst_rect.y = 0;
        cam_dst_rect.w = right_dst_rect.w - (2 * cam_dst_rect.x);
        cam_dst_rect.h = right_dst_rect.h;
    }
#pragma endregion CameraInit

    // 开启RabbitMQ消息接收线程
    keepReceiving = true;
    std::thread rabbit_recv_t = std::thread(rabbit_receive);
    
    // 初始化RabbitMQ发送队列
    std::string send_queue = "com.gdcccn.rer.doctor";
    AmqpClient::Channel::ptr_t channel = AmqpClient::Channel::Create("192.168.20.2", 5672, "rer", "gdcc@2021", "rer");

    while (true) {

#pragma region RefreshCameraDisplay
        // 始终读取摄像头数据
        cv::Mat camera_frame_raw;
        cam_cap >> camera_frame_raw;
        if (!camera_frame_raw.empty()) {
            cv::Mat camera_frame;
            cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGBA);
            cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);

            // Wrap Mat into an ImageFrame.
            auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
                mediapipe::ImageFormat::SRGBA, camera_frame.cols, camera_frame.rows,
                mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
            cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
            camera_frame.copyTo(input_frame_mat);

            // Prepare and add graph input packet.
            size_t frame_timestamp_us =
                (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
            mediaRet = gpu_helper.RunInGlContext([&input_frame, &frame_timestamp_us, &graph, &gpu_helper]() -> ::mediapipe::Status {
                    // Convert ImageFrame to GpuBuffer.
                    auto texture = gpu_helper.CreateSourceTexture(*input_frame.get());
                    auto gpu_frame = texture.GetFrame<mediapipe::GpuBuffer>();
                    glFlush();
                    texture.Release();
                    // Send GPU image packet into the graph.
                    MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
                        kInputStream, mediapipe::Adopt(gpu_frame.release()).At(mediapipe::Timestamp(frame_timestamp_us))));
                    return mediapipe::OkStatus();
                });
            if (!mediaRet.ok())
            {
                printf("Fail to add graph input packet! Error Message: %s\n", mediaRet.ToString().c_str());
                break;
            }

            // Get the graph result packet, or stop if that fails.
            mediapipe::Packet packet_video;
            if (!poller_video.Next(&packet_video)) break;
            std::unique_ptr<mediapipe::ImageFrame> output_frame;
            // Convert GpuBuffer to ImageFrame.
            mediaRet = gpu_helper.RunInGlContext(
                [&packet_video, &output_frame, &gpu_helper]() -> ::mediapipe::Status {
                    auto& gpu_frame = packet_video.Get<mediapipe::GpuBuffer>();
                    auto texture = gpu_helper.CreateSourceTexture(gpu_frame);
                    output_frame = absl::make_unique<mediapipe::ImageFrame>(
                        mediapipe::ImageFormatForGpuBufferFormat(gpu_frame.format()),
                        gpu_frame.width(), gpu_frame.height(),
                        mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
                    gpu_helper.BindFramebuffer(texture);
                    const auto info =
                        mediapipe::GlTextureInfoForGpuBufferFormat(gpu_frame.format(), 0);
                    glReadPixels(0, 0, texture.width(), texture.height(), info.gl_format,
                                info.gl_type, output_frame->MutablePixelData());
                    glFlush();
                    texture.Release();
                    return mediapipe::OkStatus();
                });
            if (!mediaRet.ok())
            {
                printf("Fail to Convert GpuBuffer to ImageFrame! Error Message: %s\n", mediaRet.ToString().c_str());
                break;
            }
            // Convert back to opencv.
            cv::Mat output_frame_mat = mediapipe::formats::MatView(output_frame.get());
            cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2RGBA);
            // 对输出图像进行裁剪和覆盖
            cv::Mat src_roi = output_frame_mat(cv::Rect(cam_src_rect.x, cam_src_rect.y, cam_src_rect.w, cam_src_rect.h));
            // cv::Mat dst_roi = cam_disp_data(cv::Rect(cam_dst_rect.x, cam_dst_rect.y, cam_dst_rect.w, cam_dst_rect.h));
            // cv::Mat temp;
            // double factor = 1.0 * cam_dst_rect.h / cam_src_rect.h;
            // cv::resize(src_roi, temp, cv::Size(), factor, factor, cv::INTER_AREA);
            // temp.copyTo(dst_roi);
            cv::resize(src_roi, cam_disp_data, cv::Size(right_dst_rect.w, right_dst_rect.h), 0, 0, cv::INTER_AREA);

            // 推送数据时拷贝并转换色彩空间到推送数据
            if(keepPushing) {
                cv::cvtColor(output_frame_mat, cam_push_data, cv::COLOR_RGB2BGR);
            }

            // retrieve landmarks
            mediapipe::Packet packet_landmark;
            if (!poller_landmark.Next(&packet_landmark)) break;
            auto& output_list = packet_landmark.Get<mediapipe::NormalizedLandmarkList>();
            if (output_list.landmark_size() > 0) {  // 检测到人时，计算各种角度并更新footer文字
                auto& left_shoulder = output_list.landmark(11);
                auto& right_shoulder = output_list.landmark(12);
                auto& left_elbow = output_list.landmark(13);
                auto& right_elbow = output_list.landmark(14);
                // 仅对可见的手臂进行计算，注意左右手相反！
                if ((left_shoulder.has_visibility() && left_shoulder.visibility() < 0.6)
                || (left_elbow.has_visibility() && left_elbow.visibility() < 0.6)) {
                    footer_right.content = NASTR;
                } else {
                    double ls_x = left_shoulder.x() * CAM_WIDTH;
                    double ls_y = left_shoulder.y() * CAM_HEIGHT;
                    double le_x = left_elbow.x() * CAM_WIDTH;
                    double le_y = left_elbow.y() * CAM_HEIGHT;
                    double cos_left = (le_y - ls_y) / sqrt(pow((le_x - ls_x), 2.0) + pow((le_y - ls_y), 2.0));
                    // 防止underflow
                    if (abs(cos_left - 1.0) < 0.0001)
                        cos_left = 1.0;
                    if (abs(cos_left + 1.0) < 0.0001)
                        cos_left = -1.0;
                    
                    double left_abd = acos(cos_left) * 180.0 / M_PI;
                    angle_fmt % left_abd;
                    footer_right.content = angle_fmt.str();
                }

                if ((right_shoulder.has_visibility() && right_shoulder.visibility() < 0.6)
                || (right_elbow.has_visibility() && right_elbow.visibility() < 0.6)) {
                    footer_left.content = NASTR;
                } else {
                    double rs_x = right_shoulder.x() * CAM_WIDTH;
                    double rs_y = right_shoulder.y() * CAM_HEIGHT;
                    double re_x = right_elbow.x() * CAM_WIDTH;
                    double re_y = right_elbow.y() * CAM_HEIGHT;
                    double cos_right = (re_y - rs_y) / sqrt(pow((re_x - rs_x), 2.0) + pow((re_y - rs_y), 2.0));
                    // 防止underflow
                    if (abs(cos_right - 1.0) < 0.0001)
                        cos_right = 1.0;
                    if (abs(cos_right + 1.0) < 0.0001)
                        cos_right = -1.0;
                    
                    double right_abd = acos(cos_right) * 180.0 / M_PI;
                    angle_fmt % right_abd;
                    footer_left.content = angle_fmt.str();
                }

                // 训练期间动作计数
                if(keepPlayingVideo) {
                    std::vector<double> pose_embedding;
                    ret = get_pose_embedding(output_list, pose_embedding);
                    if(pose_embedding.size() > 0) {
                        bool start_now = check_state(pose_embedding, ce_start_crit, 0.5);
                        if (!start_now) {  // 没有在起始状态再检查是否在结束状态
                            bool end_now = check_state(pose_embedding, ce_end_crit, 1.0);
                            if (end_now) {
                                act_start = false;
                            }
                        } else {
                            if(!act_start) {
                                act_start = true;
                                act_count++;
                            }
                        }
                    }
                }
            } else {  // 没有人时，同样更新各种状态
                footer_left.content = NASTR;
                footer_right.content = NASTR;
                act_start = false;
                act_end = false;
            }
            // 将测量结果发送至RabbitMQ
            boost::property_tree::ptree sport_data_json;
            boost::property_tree::ptree sport_data;
            std::stringstream sport_st;
            sport_data_json.put("command", "receiveTrainingData");
            sport_data_json.put("msgType", "rpc_response");
            uuid_t uuid;
            char str[36];
            uuid_generate(uuid);
            uuid_unparse(uuid, str);
            sport_data_json.put("msgId", str);
            sport_data.put("leftArmAbduction", footer_left.content);
            sport_data.put("rightArmArbduction", footer_right.content);
            sport_data_json.put_child("msgData", sport_data);
            boost::property_tree::json_parser::write_json(sport_st, sport_data_json);
            channel->BasicPublish("", send_queue, AmqpClient::BasicMessage::Create(sport_st.str()));
        }
#pragma endregion RefreshCameraDisplay

        // 训练期间计时
        if(keepPlayingVideo) {
            const auto now = std::chrono::system_clock::now();
            const auto sec_d = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
            int sec = sec_d % 60;
            int min = sec_d / 60;
            time_fmt % min % sec;
            footer_time.content = time_fmt.str();
            std::stringstream tempSS;  
            tempSS << act_count; 
            footer_count.content = tempSS.str();
        } else {
            footer_time.content = NASTR;
            footer_count.content = NASTR;
        }

#pragma region UpdateAndRender
        // 渲染前先刷新footer数据
        refreshFooter();
        // 开始渲染工作
        ret = SDL_UpdateTexture(footTexture, NULL, footer_data.ptr(0), (footer_dst_rect.w * footer_data.elemSize()));
        if(ret) {
            printf( "Could not Update Footer Texture: %s\n", SDL_GetError());
            break;
        }
        ret = SDL_UpdateTexture(rightContentTexture, NULL, cam_disp_data.ptr(0), (right_dst_rect.w * cam_disp_data.elemSize()));
        if(ret) {
            printf( "Could not Update Camera Texture: %s\n", SDL_GetError());
            break;
        } 
        ret = SDL_UpdateTexture(leftContentTexture, NULL, left_disp_data.ptr(0), (left_dst_rect.w * left_disp_data.elemSize()));
        if(ret) {
            printf( "Could not Update Left Content Texture: %s\n", SDL_GetError());
            break;
        }

        ret = SDL_RenderCopy(sdlRenderer, footTexture, NULL, &footer_dst_rect);
        if(ret) {
            printf("Could not Copy Footer data to Renderer: %s\n", SDL_GetError());
            break;
        } 
        ret = SDL_RenderCopy(sdlRenderer, leftContentTexture, NULL, &left_dst_rect);
        if(ret) {
            printf("Could not Copy Left Content data to Renderer: %s\n", SDL_GetError());
            break;
        } 
        ret = SDL_RenderCopy(sdlRenderer, rightContentTexture, &cam_src_rect, &right_dst_rect);
        if(ret) {
            printf("Could not Copy Camera data to Renderer: %s\n", SDL_GetError());
            break;
        } 

        // 拉流并且非实时指导时才展示画中画
        if (keepPulling && (!guideByDoc)) {
            ret = SDL_UpdateTexture(leftInsideTexture, NULL, left_inside_data.ptr(0), (left_inside_dst_rect.w * left_inside_data.elemSize()));
            if(ret) {
                printf( "Could not Update Left Inside Content Texture: %s\n", SDL_GetError());
                break;
            }
            ret = SDL_RenderCopy(sdlRenderer, leftInsideTexture, NULL, &left_inside_dst_rect);
            if(ret) {
                printf("Could not Copy Left Inside Content data to Renderer: %s\n", SDL_GetError());
                break;
            }
        }

        SDL_RenderPresent(sdlRenderer);
#pragma endregion UpdateAndRender


        SDL_Delay(30);
        SDL_Event e;
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                break;
            }
        }
    }

    printf("Loop over...\n");

    // Close Mediapipe Graph
    mediaRet = graph.CloseInputStream(kInputStream);
    if (!mediaRet.ok())
    {
        printf("Fail to Close Graph Input Stream! Error Message: %s\n", mediaRet.ToString().c_str());
    }

    mediaRet = graph.WaitUntilDone();
    if (!mediaRet.ok())
    {
        printf("Fail to Wait Until Graph Done! Error Message: %s\n", mediaRet.ToString().c_str());
    }
    printf("MediaPipe Graph finish running\n");

    // 关闭线程
    keepReceiving = false;
    if (rabbit_recv_t.joinable()){
        rabbit_recv_t.join();
    }
    printf("RabbitMQ Receiving Subthread finish running\n");

    // Close and destroy all SDL Objects
    SDL_DestroyTexture(rightContentTexture);
    SDL_DestroyTexture(leftContentTexture);
    SDL_DestroyTexture(leftInsideTexture);
    SDL_DestroyRenderer(sdlRenderer);
    SDL_DestroyWindow(displayWindow);

    // Close SDL
    SDL_Quit();

    return EXIT_SUCCESS;
}
