// 推送识别关节点后的摄像头数据到RTMP直播流

#include <stdio.h>
#include <cstdlib>

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

// 定义各种输入输出流名称
constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream_Video[] = "output_video";
constexpr char kOutputStream_Landmarks[] = "pose_landmarks_every";
constexpr char kWindowName[] = "Pose Tracking Data";

int main(int argc, char* argv[])
{
    int ret;

    // 注册全部FFMpeg插件
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
    printf("Error occurred when Write Header\n");
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
        printf("Fail to Initialize Mediapipe Graph! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
        return -1;
    }

    printf("Initialize the GPU...\n");
    auto status_or_gpu = mediapipe::GpuResources::Create();
    if (!status_or_gpu.ok()) {
        printf("Fail to assign GPU Resources! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
        return -1;
    }
    mediaRet = graph.SetGpuResources(std::move(status_or_gpu).ValueOrDie());
    if (!mediaRet.ok())
    {
        printf("Fail to Initialize Mediapipe Graph! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
        return -1;
    }
    mediapipe::GlCalculatorHelper gpu_helper;
    gpu_helper.InitializeForTest(graph.GetGpuResources().get());

    mediapipe::StatusOrPoller status_or_poller_landmarks = graph.AddOutputStreamPoller(kOutputStream_Landmarks);
    if (!status_or_poller_landmarks.ok()) {
        printf("Fail to assign landmarks output stream poller! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
        return -1;
    }
    mediapipe::OutputStreamPoller poller_landmark = std::move(status_or_poller_landmarks.ValueOrDie());

    mediapipe::StatusOrPoller status_or_poller_video = graph.AddOutputStreamPoller(kOutputStream_Video);
    if (!status_or_poller_video.ok()) {
        printf("Fail to assign video output stream poller! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
        return -1;
    }
    mediapipe::OutputStreamPoller poller_video = std::move(status_or_poller_video.ValueOrDie());

    printf("Start running the calculator graph...\n");
    mediaRet = graph.StartRun({});
    if (!mediaRet.ok())
    {
        printf("Fail to Run Mediapipe Graph! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
        return -1;
    }

    printf("Initialize camera...\n");
    cv::VideoCapture capture;
    if(!capture.open(0)){
        printf( "Fail to open Camera\n");
        return -1;
    } 

#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    capture.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    capture.set(cv::CAP_PROP_FPS, FRAME_FPS);
#endif

    printf("Start grabbing and processing frames...\n");
    int frameCount = 0;
    int frame_idx = 0;
    double last_timestamp = 0.0;
    while (true) {
        cv::Mat camera_frame_raw;
        capture >> camera_frame_raw;
        if (camera_frame_raw.empty()) {
            continue;
        }
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
            printf("Fail to add graph input packet! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
            return -1;
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
            printf("Fail to Convert GpuBuffer to ImageFrame! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
            return -1;
        }
        // Convert back to opencv.
        cv::Mat output_frame_mat = mediapipe::formats::MatView(output_frame.get());
        cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);

        // retrieve landmarks
        mediapipe::Packet packet_landmark;
        if (!poller_landmark.Next(&packet_landmark)) break;
        auto& output_list = packet_landmark.Get<mediapipe::NormalizedLandmarkList>();
        if (output_list.landmark_size() > 0) {
            // Show output list.
            auto& nose = output_list.landmark(0);
            int x_int = static_cast<int>(nose.x() * output_frame_mat.cols);
            int y_int = static_cast<int>(nose.y() * output_frame_mat.rows);
            cv::Point point_to_draw(x_int + 10, y_int);
            cv::putText(output_frame_mat, "nose!!!", point_to_draw, cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, cv::FILLED);
        }

        double now = (double)cv::getTickCount() / (double)cv::getTickFrequency();
        if (last_timestamp > 0) {
            double fps = 1.0 / (now - last_timestamp);
            cv::Point text_org(0, 50);
            std::string text = "FPS: " + std::to_string(fps);
            cv::putText(output_frame_mat, text, text_org, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2, cv::FILLED);
        }
        last_timestamp = now;

        // 转换cv::Mat为AVFrame
        AVFrame* frame = cvmat2avframe(output_frame_mat);
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

    // Close Mediapipe Graph
    mediaRet = graph.CloseInputStream(kInputStream);
    if (!mediaRet.ok())
    {
        printf("Fail to Close Graph Input Stream! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
    }

    mediaRet = graph.WaitUntilDone();
    if (!mediaRet.ok())
    {
        printf("Fail to Wait Until Graph Done! Error code: %d; Error Message: %s\n", mediaRet.code(), mediaRet.message());
    }

    return 0;
}


