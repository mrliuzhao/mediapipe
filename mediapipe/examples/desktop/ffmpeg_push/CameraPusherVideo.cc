// 推送摄像头视频数据到RTMP直播流

#include <stdio.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
#include <libavdevice/avdevice.h>
};

int64_t lastReadPacketTime = 0;
AVFormatContext *in_ctx = nullptr;
AVFormatContext *out_ctx = nullptr;
AVCodecContext *encodeContext;
AVCodecContext *decodeContext;

static int interrupt_cb(void *ctx)
{
    int timeout = 10;
    if(av_gettime() - lastReadPacketTime > timeout * 1000 * 1000)
    {
        return -1;
    }
    return 0;
}

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

int WritePacket(AVPacket *packet)
{
    auto inputStream = in_ctx->streams[packet->stream_index];
    auto outputStream = out_ctx->streams[packet->stream_index];
    av_packet_rescale_ts(packet, inputStream->time_base, outputStream->time_base);
    return av_write_frame(out_ctx, packet);
}

int main(int argc, char* argv[])
{
    int ret;
    // 注册全部插件
    av_register_all();
    avdevice_register_all();
    avformat_network_init();

    // 分配内存
    in_ctx = avformat_alloc_context();
    in_ctx->interrupt_callback.callback = interrupt_cb;
    lastReadPacketTime = av_gettime();

    // 初始化摄像头输入流
    char inputFilePath[]= "/dev/video0";
    AVInputFormat* infmt = av_find_input_format("video4linux2");
    if(infmt == nullptr)
    {
        printf("Can not find input format\n");
        return -2;
    }
    // AVDictionary* options = NULL;
    // av_dict_set(&options, "fflags", "nobuffer", 0);
    // ret = avformat_open_input(&in_ctx, inputFilePath, infmt, &options);
    ret = avformat_open_input(&in_ctx, inputFilePath, infmt, NULL);
    if(ret != 0){
        printf("Couldn't open input stream. return code: %d \n", ret);
        return -1;
    }
    ret = avformat_find_stream_info(in_ctx, NULL);
    if(ret < 0){
        printf("can't find stream info ");
        return -1;
    }
    // in_ctx->max_interleave_delta = 0;
    printf("---------- Input Format ----------\n");
    av_dump_format(in_ctx, 0, inputFilePath, false);
    printf("in_ctx->max_interleave_delta: %d\n", in_ctx->max_interleave_delta);
    printf("---------- End ----------\n");

    //查找视频流的编号
    int video_stream_idx = -1;
    for(int i = 0; i < in_ctx->nb_streams; i++){
        if(in_ctx->streams[i]->codecpar->codec_type == AVMediaType::AVMEDIA_TYPE_VIDEO) {
            video_stream_idx = i;
            printf("find video stream id %d\n", video_stream_idx);
            break;
        }
    }
     if(video_stream_idx == -1){
        printf("Didn't find a video stream.\n");
        return -1;
    }

    // 初始化视频流解码器——用于解析输入
    AVCodec *video_codec =  new AVCodec;
    ret = av_find_best_stream(in_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &video_codec, 0);
    if (ret < 0)
    {
        printf("Find inform failed.\n");
        return -1;
    }
    decodeContext = avcodec_alloc_context3(video_codec);
    avcodec_parameters_to_context(decodeContext, in_ctx->streams[video_stream_idx]->codecpar);
    ret = avcodec_open2(decodeContext, video_codec, nullptr);
    if(ret < 0)
    {
        printf("Could not open video decoder.\n");
        return -1;
    }

    // 初始化视频流编码器——用于输出前编码
    AVCodec *picCodec;
    auto inputStream = in_ctx->streams[video_stream_idx];
    picCodec = avcodec_find_encoder(AV_CODEC_ID_H264);
    encodeContext = avcodec_alloc_context3(picCodec);
    encodeContext->codec_id = picCodec->id;
    encodeContext->time_base.num = inputStream->time_base.num;
    encodeContext->time_base.den = inputStream->time_base.den;
    printf("!!!!! inputStream->time_base.num=%d ; inputStream->time_base.den=%d\n", inputStream->time_base.num, inputStream->time_base.den);
    encodeContext->pix_fmt = *picCodec->pix_fmts;
    encodeContext->width = inputStream->codecpar->width;
    encodeContext->height = inputStream->codecpar->height;
    encodeContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    ret = avcodec_open2(encodeContext, picCodec, nullptr);
    if (ret < 0)
    {
        printf("Could not open video encoder.\n");
        return -1;
    }

    // 初始化推送数据的输出流上下文
    // char out_filename[]= "rtmp://192.168.20.2/live/demo3";
    char out_filename[]= "/home/gdcc/push.flv";
    ret = avformat_alloc_output_context2(&out_ctx, NULL, "flv", out_filename); //RTMP
    //avformat_alloc_output_context2(&out_ctx, NULL, "mpegts", out_filename);//UDP
    if (ret < 0 || !out_ctx) {
        printf("Could not create output context\n");
        return -1;
    }
    // 在输出上下文下初始化具体输出流，并指明编码器
    for (int i = 0; i < in_ctx->nb_streams; i++) {
        if (in_ctx->streams[i]->codecpar->codec_type == AVMediaType::AVMEDIA_TYPE_VIDEO) {
            AVStream * stream = avformat_new_stream(out_ctx, nullptr);
            if (!stream) {
                printf( "Failed allocating output stream\n");
                return -1;
            }
            stream->codecpar->codec_tag = 0;
            ret = avcodec_parameters_from_context(stream->codecpar, encodeContext);
            if (ret < 0) {
                printf( "Failed to set encoder to output stream\n");
                return -1;
            }
        }
        else continue;
    }
    // out_ctx->max_interleave_delta = 0;
    printf("---------- Output Format ----------\n");
    av_dump_format(out_ctx, 0, out_filename, 1);
    printf("out_ctx->max_interleave_delta: %d\n", out_ctx->max_interleave_delta);
    printf("---------- End ----------\n");

    //打开输出URL（Open output URL）
    ret = avio_open2(&out_ctx->pb, out_filename, AVIO_FLAG_READ_WRITE, nullptr, nullptr);  
    if(ret < 0)
    {
        printf( "Could not open output URL '%s'\n", out_filename);
        return -1;
    }
    //写文件头（Write file header）
    ret = avformat_write_header(out_ctx, NULL);
    if (ret < 0) {
        printf( "Error occurred when Write Header\n");
        return -1;
    }

    // 初始化格式化工具
    struct SwsContext* pSwsContext = nullptr;
    pSwsContext = sws_getContext(decodeContext->width, decodeContext->height, decodeContext->pix_fmt, encodeContext->width, encodeContext->height, encodeContext->pix_fmt, SWS_BICUBIC, NULL, NULL, NULL);
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

    AVFrame *frame = av_frame_alloc();
    int64_t start_time = av_gettime();
    int frameCount = 0;
    while (true) {
        AVPacket *packet = new AVPacket;
        av_init_packet(packet);
        lastReadPacketTime = av_gettime();
        ret = av_read_frame(in_ctx, packet);
        if (ret < 0) {
            av_packet_unref(packet);
            delete packet;
            continue;
        }
        if (packet->stream_index == video_stream_idx)
        {
            // 解码输入视频流
            ret = avcodec_send_packet(decodeContext, packet);
            if (ret < 0) {
                printf( "Error decode frame at avcodec_send_packet, return value: %d\n", ret);
                return -1;
            }
            av_packet_unref(packet);
            delete packet;
            packet = nullptr;
            ret = avcodec_receive_frame(decodeContext, frame);
            if (ret < 0) {
                printf( "Error decode frame at avcodec_receive_frame, return value: %d\n", ret);
                return -1;
            }
            // 转换格式
            sws_scale(pSwsContext, (const uint8_t *const *)frame->data, frame->linesize, 0, in_ctx->streams[0]->codecpar->height, (uint8_t *const *)pSendFrame->data, pSendFrame->linesize);
            pSendFrame->pts = frame->pts;
            // printf("!!!!! frame->pts=%lld\n", frame->pts);
            // 重编码
            AVPacket * packetEncode = EncodeVideo(pSendFrame);
            if (packetEncode)
            {
                // 将重编码后的Packet写入直播流
                ret = WritePacket(packetEncode);
                frameCount++;
                if (frameCount > 1200) {
                    printf( "Already Push 1200 frames, shutting down...\n");
                    break;
                }
            }
        }
    }

    av_frame_unref(pSendFrame);
    delete pSendFrame;

    //写文件尾（Write file trailer）
    av_write_trailer(out_ctx);
    
    avio_closep(&out_ctx->pb);
    avformat_free_context(out_ctx);

    // close input
    avformat_close_input(&in_ctx);


    return 0;
}


