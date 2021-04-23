// 音视频结合的播放器，第一个参数为输入，可以为RTMP直播流，默认为四川卫视直播

#include <stdio.h>
#define __STDC_CONSTANT_MACROS

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavutil/imgutils.h>
#include <libavdevice/avdevice.h>

#include <SDL2/SDL.h>
};

#include "CycleBuffer.h"

//Refresh Event
#define SFM_REFRESH_EVENT  (SDL_USEREVENT + 1)
#define SFM_BREAK_EVENT  (SDL_USEREVENT + 2)

#define MAX_AUDIO_FRAME_SIZE 192000 // 1 second of 48khz 32bit audio
 
int thread_exit = 0;
//Thread
int sfp_refresh_thread(void *opaque)
{
    while (!thread_exit) {
        SDL_Event event;
        event.type = SFM_REFRESH_EVENT;
        SDL_PushEvent(&event);
        //Wait 20 ms
        SDL_Delay(20);
    }

    //Break
    SDL_Event event;
    event.type = SFM_BREAK_EVENT;
    SDL_PushEvent(&event);
    return 0;
}

//Buffer:
//|-----------|-------------|
//chunk-------pos---len-----|
static  Uint8  *audio_chunk; 
static  Uint32  audio_len; 
static  Uint8  *audio_pos; 
 
uint8_t         *copy_buf;
CCycleBuffer    *pSoundBuf;
 
/* The audio function callback takes the following parameters: 
 * stream: A pointer to the audio buffer to be filled 
 * len: The length (in bytes) of the audio buffer 
*/ 
void  fill_audio(void *udata, Uint8 *stream, int len){ 
    //SDL 2.0
    SDL_memset(stream, 0, len);

    int n = pSoundBuf->Read((char*)copy_buf, len);
    SDL_MixAudio(stream, copy_buf, n, SDL_MIX_MAXVOLUME);
} 

AVFormatContext* video_in_ctx = nullptr;
AVFormatContext* audio_in_ctx = nullptr;
int video_stream_idx = -1;
int audio_stream_idx = -1;
AVCodecContext *video_decode_ctx;
AVCodecContext *audio_decode_ctx;

int initVideoInputStream(){
    int ret;
    // 初始化摄像头视频输入流
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
    // 尝试使用新接口查找最符合要求的流
    // ret = av_find_best_stream(video_in_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    // if (ret < 0) {
    //     printf("can't find video input stream info. return code: %d \n", ret);
    //     return -1;
    // }
    // video_stream_idx = ret;
    printf("---------- Video Input Format ----------\n");
    av_dump_format(video_in_ctx, video_stream_idx, videoInputPath, false);
    printf("---------- End ----------\n");
    return 0;
}

int initAudioInputStream(){
    int ret;
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
    return 0;
}

int openVideoDecoder(){
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
    int ret;
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

int openAudioDecoder(){
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
    int ret;
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

int main(int argc, char* argv[])
{
    pSoundBuf = new CCycleBuffer(192000 * 10);
    int ret;
    // 注册全部插件
    av_register_all();
    avdevice_register_all();
    avformat_network_init();

    // 初始化摄像头视频输入流
    ret = initVideoInputStream();
    if (ret < 0) {
        printf("Cannot initialize video input stream.\n");
        return -1;
    }
    // 初始化摄像头麦克风音频输入流
    ret = initAudioInputStream();
    if (ret < 0) {
        printf("Cannot initialize audio input stream.\n");
        return -1;
    }

    // 初始化视频解码器
    ret = openVideoDecoder();
    if (ret < 0) {
        printf("Cannot open video decoder context.\n");
        return -1;
    }
    printf("Successfully open video decoder context.\n");
    ret = openAudioDecoder();
    if (ret < 0) {
        printf("Cannot open audio decoder context.\n");
        return -1;
    }
    printf("Successfully open audio decoder context.\n");

    //初始化frame
    AVFrame    *pFrame, *pFrameYUV, *audioFrame;
    AVPacket   *packet;
    pFrame = av_frame_alloc();
    audioFrame = av_frame_alloc();
    pFrameYUV = av_frame_alloc();
    packet = (AVPacket *)av_malloc(sizeof(AVPacket));
    av_init_packet(packet);
    // initialize packet, set data to NULL, let the demuxer fill it
    // av_init_packet(packet);
    // packet->data = NULL;
    // packet->size = 0;
    printf("Successfully initialize frames.\n");

    // 为视频帧预分配内存
    unsigned char *video_out_buffer = (unsigned char *)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P, video_decode_ctx->width, video_decode_ctx->height, 1));
    av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, video_out_buffer, AV_PIX_FMT_YUV420P, video_decode_ctx->width, video_decode_ctx->height, 1);
    // 创建视频大小格式转换器
    struct SwsContext *img_convert_ctx = sws_getContext(video_decode_ctx->width, video_decode_ctx->height, video_decode_ctx->pix_fmt, video_decode_ctx->width, video_decode_ctx->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL); 

    // 为音频帧预分配内存
    uint64_t out_channel_layout = AV_CH_LAYOUT_STEREO;
    int out_nb_samples = audio_decode_ctx->frame_size;
    AVSampleFormat out_sample_fmt = AV_SAMPLE_FMT_S16;
    int out_sample_rate = 44100;
    int out_channels = av_get_channel_layout_nb_channels(out_channel_layout);
    //Out Buffer Size
    int out_buffer_size = av_samples_get_buffer_size(NULL, out_channels, out_nb_samples, out_sample_fmt, 1);
    uint8_t *audio_out_buffer = (uint8_t *)av_malloc(MAX_AUDIO_FRAME_SIZE * 2);
    copy_buf = (uint8_t *)av_malloc(MAX_AUDIO_FRAME_SIZE * 2);

    // 创建音频格式转换器，Swr
    int64_t in_channel_layout = av_get_default_channel_layout(audio_decode_ctx->channels);
    struct SwrContext *au_convert_ctx = swr_alloc();
    au_convert_ctx = swr_alloc_set_opts(au_convert_ctx, out_channel_layout, out_sample_fmt, out_sample_rate, in_channel_layout, audio_decode_ctx->sample_fmt , audio_decode_ctx->sample_rate, 0, NULL);
    swr_init(au_convert_ctx);

    //初始化SDL
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {  
        printf( "Could not initialize SDL - %s\n", SDL_GetError()); 
        return -1;
    } 
 
    // 初始化SDL显示相关，按照输入流原始大小展示
    int screen_w = video_decode_ctx->width;
    int screen_h = video_decode_ctx->height;
    SDL_Window *screen = SDL_CreateWindow("Camera Display", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screen_w, screen_h, SDL_WINDOW_OPENGL);
    if(!screen) {  
        printf("SDL: could not create window - exiting:%s\n",SDL_GetError());  
        return -1;
    }
    SDL_Renderer* sdlRenderer = SDL_CreateRenderer(screen, -1, 0);
    SDL_Texture* sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, screen_w, screen_h);

    // 初始化SDL音频相关，SDL_AudioSpec
    SDL_AudioSpec wanted_spec;
    wanted_spec.freq = out_sample_rate; 
    wanted_spec.format = AUDIO_S16SYS;
    // wanted_spec.format = AUDIO_S16LSB; 
    wanted_spec.channels = out_channels; 
    wanted_spec.silence = 0; 
    wanted_spec.samples = out_nb_samples; 
    wanted_spec.callback = fill_audio; 
    // wanted_spec.userdata = audio_decode_ctx; 
    if (SDL_OpenAudio(&wanted_spec, NULL)<0){ 
        printf("can't open SDL audio - %s\n", SDL_GetError()); 
        return -1; 
    }

    //创建消息线程
    SDL_Thread *video_tid = SDL_CreateThread(sfp_refresh_thread, NULL, NULL);

    //消息循环
    SDL_Event e;
    int got;
    bool grab_frames = true;
    while(grab_frames) {
        SDL_WaitEvent(&e);
        if(e.type == SFM_REFRESH_EVENT){
            SDL_PauseAudio(0);
            ret = av_read_frame(video_in_ctx, packet);
            if (ret < 0){
                printf("error read from video\n"); 
                grab_frames = false;
            } else {
                if (packet->stream_index == video_stream_idx) {
                    ret = avcodec_decode_video2(video_decode_ctx, pFrame, &got, packet);
                    if(ret < 0){
                        printf("Decode Video Error.\n");
                        grab_frames = false;
                    }
                    if(got){
                        sws_scale(img_convert_ctx, (const unsigned char* const*)pFrame->data, pFrame->linesize, 0, video_decode_ctx->height, pFrameYUV->data, pFrameYUV->linesize);
                        //SDL---------------------------
                        SDL_UpdateTexture(sdlTexture, NULL, pFrameYUV->data[0], pFrameYUV->linesize[0] );  
                        SDL_RenderClear(sdlRenderer);  
                        SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, NULL);  
                        SDL_RenderPresent(sdlRenderer);  
                        //SDL End-----------------------
                        // break;
                    }
                }
                av_free_packet(packet);
            }

            ret = av_read_frame(audio_in_ctx, packet);
            if (ret < 0){
                printf("error read from audio\n"); 
                grab_frames = false;
            } else {
                if (packet->stream_index == audio_stream_idx) {
                    ret = avcodec_decode_audio4(audio_decode_ctx, audioFrame, &got, packet);
                    if(got > 0){
                        int rr = swr_convert(au_convert_ctx, &audio_out_buffer, MAX_AUDIO_FRAME_SIZE, (const uint8_t **)audioFrame->data, audioFrame->nb_samples);
                        pSoundBuf->Write((char*)audio_out_buffer, rr*4);
                        // pSoundBuf->Write((char*)audioFrame->data, audioFrame->nb_samples * 4);
                    }
                }
                av_free_packet(packet);
            }
        
        } else if (e.type == SDL_QUIT) {
            thread_exit = 1;
        } else if(e.type == SFM_BREAK_EVENT){
            break;
        }
    }
    printf("exit while loop.\n");

    // Close Converter
    sws_freeContext(img_convert_ctx);
    swr_free(&au_convert_ctx);

    // Close SDL
    SDL_CloseAudio();
    SDL_Quit();

    // Free AVFrame
    av_frame_free(&pFrameYUV);
    av_frame_free(&pFrame);
    av_frame_free(&audioFrame);

    // Free av_malloc
    av_free(audio_out_buffer);
    av_free(video_out_buffer);
    av_free(copy_buf);

    avcodec_free_context(&video_decode_ctx);
    avcodec_free_context(&audio_decode_ctx);
    avformat_close_input(&video_in_ctx);
    avformat_close_input(&audio_in_ctx);

    return 0;
}
