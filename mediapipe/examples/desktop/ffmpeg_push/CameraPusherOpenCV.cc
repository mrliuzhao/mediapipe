// 推送摄像头数据到RTMP直播流

#include <stdio.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
// #include <libavdevice/avdevice.h>
};

#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"

AVFormatContext *out_ctx = nullptr;
AVCodecContext *encodeContext;

AVPacket* EncodeVideo(AVFrame* frame)
{
    int ret = 0;

    ret = avcodec_send_frame(encodeContext, frame);
    if (ret < 0) nullptr;

    AVPacket * packet = new AVPacket;
    av_init_packet(packet);
    ret = avcodec_receive_packet(encodeContext, packet);
    if (ret >= 0) return packet;
    av_packet_unref(packet);
    delete packet;
    packet = nullptr;
    return nullptr;
}

// int WritePacket(AVPacket *packet)
// {
//     auto inputStream = in_ctx->streams[packet->stream_index];
//     auto outputStream = out_ctx->streams[packet->stream_index];
//     av_packet_rescale_ts(packet, inputStream->time_base, outputStream->time_base);
//     return av_write_frame(out_ctx, packet);
// }

AVFrame *cvmat2avframe(cv::Mat mat) {
    // alloc avframe
    AVFrame *avframe = av_frame_alloc();
    if (avframe && !mat.empty()) {
        avframe->format = AV_PIX_FMT_YUV420P;
        avframe->width = mat.cols;
        avframe->height = mat.rows;
        av_frame_get_buffer(avframe, 0);
        av_frame_make_writable(avframe);
        cv::Mat yuv; // convert to yuv420p first
        cv::cvtColor(mat, yuv, cv::COLOR_BGR2YUV_I420);
        // calc frame size
        int frame_size = mat.cols * mat.rows;
        unsigned char *pdata = yuv.data;
        // fill yuv420
        // yyy yyy yyy yyy
        // uuu
        // vvv
        avframe->data[0] = pdata; // fill y
        avframe->data[1] = pdata + frame_size; // fill u
        avframe->data[2] = pdata + frame_size * 5 / 4; // fill v
    }
    return avframe;
}


//定义图像大小常量
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int FRAME_FPS = 30;

int main(int argc, char* argv[])
{
    int ret;

    // 注册全部插件
    av_register_all();
    // avdevice_register_all();
    avformat_network_init();

    // 初始化视频流编码器——用于输出前编码
    AVCodec *picCodec;
    // auto inputStream = in_ctx->streams[video_stream_idx];
    picCodec = avcodec_find_encoder(AV_CODEC_ID_H264);
    encodeContext = avcodec_alloc_context3(picCodec);
    encodeContext->codec_id = picCodec->id;
    encodeContext->time_base.num = 1;
    encodeContext->time_base.den = FRAME_FPS;
    encodeContext->pix_fmt = *picCodec->pix_fmts;
    encodeContext->width = FRAME_WIDTH;
    encodeContext->height = FRAME_HEIGHT;
    encodeContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    ret = avcodec_open2(encodeContext, picCodec, nullptr);
    if (ret < 0)
    {
        printf("Could not open video encoder.\n");
        return -1;
    }

    // 初始化推送数据的输出流上下文
    char out_filename[]= "rtmp://192.168.20.2/live/demo3";
    ret = avformat_alloc_output_context2(&out_ctx, NULL, "flv", out_filename); //RTMP
    //avformat_alloc_output_context2(&out_ctx, NULL, "mpegts", out_filename);//UDP
    if (ret < 0 || !out_ctx) {
        printf("Could not create output context\n");
        return -1;
    }
    //打开输出URL（Open output URL）
    ret = avio_open2(&out_ctx->pb, out_filename, AVIO_FLAG_READ_WRITE, nullptr, nullptr);  
    if(ret < 0)
    {
        printf( "Could not open output URL '%s'\n", out_filename);
        return -1;
    }
    // 在输出上下文下初始化具体输出流，并指明编码器
    AVStream * out_stream = avformat_new_stream(out_ctx, nullptr);
    if (!out_stream) {
        printf( "Failed allocating output stream\n");
        return -1;
    }
    out_stream->codecpar->codec_tag = 0;
    ret = avcodec_parameters_from_context(out_stream->codecpar, encodeContext);
    if (ret < 0) {
        printf( "Failed to set encoder to output stream\n");
        return -1;
    }
    // out_ctx->max_interleave_delta = 0;
    printf("---------- Output Format ----------\n");
    av_dump_format(out_ctx, 0, out_filename, 1);
    printf("---------- End ----------\n");

    //写文件头（Write file header）
    ret = avformat_write_header(out_ctx, NULL);
    if (ret < 0) {
        printf( "Error occurred when Write Header\n");
        return -1;
    }

    // 初始化格式化工具
    struct SwsContext* pSwsContext = nullptr;
    pSwsContext = sws_getContext(FRAME_WIDTH, FRAME_HEIGHT, AV_PIX_FMT_YUV420P, encodeContext->width, encodeContext->height, encodeContext->pix_fmt, SWS_BICUBIC, NULL, NULL, NULL);
    if (pSwsContext == nullptr)
    {
        printf( "Error creating sws context (image converter)\n");
        return -1;
    }

    // 初始化输出视频帧
    AVFrame *pSendFrame = av_frame_alloc();
    uint8_t *pSwpBuffer = nullptr;
    int numBytes = av_image_get_buffer_size(encodeContext->pix_fmt, encodeContext->width, encodeContext->height, 1);
    pSwpBuffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
    av_image_fill_arrays(pSendFrame->data, pSendFrame->linesize, pSwpBuffer, encodeContext->pix_fmt, encodeContext->width, encodeContext->height, 1);
    pSendFrame->width = encodeContext->width;
    pSendFrame->height = encodeContext->height;
    pSendFrame->format = encodeContext->pix_fmt;

    printf( "Going to open Camera...\n");
    cv::VideoCapture capture;
    if(capture.open(0)){
        printf( "Successfully open Camera\n");
    } else
    {
        printf( "Fail to open Camera\n");
        return -1;
    }
    
#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    capture.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    capture.set(cv::CAP_PROP_FPS, FRAME_FPS);
#endif

    int64_t start_time = av_gettime();
    int frameCount = 0;
    int frame_idx = 0;
    while (true) {
        cv::Mat camera_frame_raw;
        capture >> camera_frame_raw;
        if (camera_frame_raw.empty()) {
            continue;
        }

        // 转换cv::Mat为AVFrame
        AVFrame* frame = cvmat2avframe(camera_frame_raw);
        // 转换格式
        sws_scale(pSwsContext, (const uint8_t *const *)frame->data, frame->linesize, 0, FRAME_HEIGHT, (uint8_t *const *)pSendFrame->data, pSendFrame->linesize);
        pSendFrame->pts = frame_idx;
        frame_idx++;
        // 重编码
        AVPacket * packetEncode = EncodeVideo(pSendFrame);
        if (packetEncode)
        {
            // 将重编码后的Packet写入直播流
            // ret = WritePacket(packetEncode);
            ret = av_write_frame(out_ctx, packetEncode);
            frameCount++;
            if (frameCount > 1200) {
                printf( "Already Push 1200 frames, shutting down...\n");
                break;
            }
        }

    }

    av_frame_unref(pSendFrame);
    delete pSendFrame;

    //写文件尾（Write file trailer）
    av_write_trailer(out_ctx);
    avformat_free_context(out_ctx);

    // close input
    // avformat_close_input(&in_ctx);

    return 0;
}


