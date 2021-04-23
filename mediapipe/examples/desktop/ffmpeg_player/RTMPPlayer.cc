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
#include <libavutil/opt.h>

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
/*
    if(audio_len==0)        
            return; 
    len=(len>audio_len?audio_len:len);     
    SDL_MixAudio(stream,audio_pos,len,SDL_MIX_MAXVOLUME);
    audio_pos += len; 
    audio_len -= len; 
*/
    
    int n = pSoundBuf->Read((char*)copy_buf, len);
    SDL_MixAudio(stream, copy_buf, n, SDL_MIX_MAXVOLUME);
} 
 
int main(int argc, char* argv[])
{
    std::string filepathStr = std::string("rtmp://58.200.131.2:1935/livetv/sctv");
    if (argc > 1) {
        filepathStr = std::string(argv[1]);
    }
    // char filepath[]= "rtmp://58.200.131.2:1935/livetv/sctv";
    // char filepath[]= "rtmp://192.168.20.2/live/demo";
    const char *filepath = filepathStr.c_str();
    printf("Going to read stream from %s.\n", filepath);

    pSoundBuf=new CCycleBuffer(192000 * 10);
    int ret;
    // 注册全部插件
    av_register_all();
 
    // 分配内存
    AVFormatContext* fctx = avformat_alloc_context();
    
    // 打开输入流
    AVFrame* f = av_frame_alloc();
    avformat_network_init();
    printf("%s", avcodec_configuration());

    AVDictionary* options = NULL;
    av_dict_set(&options, "fflags", "nobuffer", 0);
    ret = avformat_open_input(&fctx, filepath, NULL, &options);
    if(ret != 0){
        printf("Couldn't open input stream.\n");
        exit(2);
    }
    ret = avformat_find_stream_info(fctx, NULL);
    if(ret < 0){
        printf("can't find stream info ");
        exit(3);
    }
    av_dump_format(fctx, 0, filepath, false);
 
    //查找视频流和音频流的编号
    int video_stream = -1;
    int audio_stream = -1;
    int find_n = 0;
    for(int i=0; i < fctx->nb_streams; i++){
        if(fctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream = i;
            find_n ++;
            printf("find video stream id %d",video_stream);
        }
        if(fctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO) {
            audio_stream = i;
            find_n ++;
            printf("find audio stream id %d",audio_stream);
        }
        if(find_n >= 2){
            break;
        }
    }
     if(video_stream == -1){
        printf("Didn't find a video stream.\n");
        return -1;
    }
     if(audio_stream == -1){
        printf("Didn't find an audio stream.\n");
        return -1;
    }

    //获取解码上下文和解码器
    AVCodecContext     *video_codec_ctx = fctx->streams[video_stream]->codec;
    AVCodec            *video_codec = avcodec_find_decoder(video_codec_ctx->codec_id);;
    AVCodecContext     *audio_codec_ctx = fctx->streams[audio_stream]->codec;
    AVCodec            *audio_codec = avcodec_find_decoder(audio_codec_ctx->codec_id);
    if(avcodec_open2(video_codec_ctx, video_codec,NULL)<0)
    {
        printf("Could not open video codec.\n");
        return -1;
    }
    if(avcodec_open2(audio_codec_ctx, audio_codec,NULL)<0)
    {
        printf("Could not open audio codec.\n");
        return -1;
    }
    
    //初始化frame
    AVFrame    *pFrame, *pFrameYUV, *audioFrame;
    AVPacket   *packet;
    pFrame = av_frame_alloc();
    audioFrame = av_frame_alloc();
    pFrameYUV = av_frame_alloc();
    packet = (AVPacket *)av_malloc(sizeof(AVPacket));
    av_init_packet(packet);

    // 为视频帧预分配内存
    unsigned char *video_out_buffer = (unsigned char *)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P, video_codec_ctx->width, video_codec_ctx->height, 1));
    av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, video_out_buffer, AV_PIX_FMT_YUV420P, video_codec_ctx->width, video_codec_ctx->height, 1);
    // 创建视频大小格式转换器
    struct SwsContext *img_convert_ctx = sws_getContext(video_codec_ctx->width, video_codec_ctx->height, video_codec_ctx->pix_fmt, video_codec_ctx->width, video_codec_ctx->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL); 

    // 为音频帧预分配内存
    uint64_t out_channel_layout = AV_CH_LAYOUT_STEREO;
    //nb_samples: AAC-1024 MP3-1152
    int out_nb_samples = audio_codec_ctx->frame_size;
    AVSampleFormat out_sample_fmt = AV_SAMPLE_FMT_S16;
    // int out_sample_rate = 44100;
    int out_channels = av_get_channel_layout_nb_channels(out_channel_layout);
    //Out Buffer Size
    // int out_buffer_size = av_samples_get_buffer_size(NULL, out_channels, out_nb_samples, out_sample_fmt, 1);
    uint8_t *audio_out_buffer = (uint8_t *)av_malloc(MAX_AUDIO_FRAME_SIZE*2);
    copy_buf = (uint8_t *)av_malloc(MAX_AUDIO_FRAME_SIZE*2);

    // 创建音频格式转换器，Swr
    int64_t in_channel_layout=av_get_default_channel_layout(audio_codec_ctx->channels);
    struct SwrContext *au_convert_ctx = swr_alloc();
    // au_convert_ctx = swr_alloc_set_opts(au_convert_ctx, out_channel_layout, out_sample_fmt, out_sample_rate, in_channel_layout, audio_codec_ctx->sample_fmt , audio_codec_ctx->sample_rate, 0, NULL);
    // swr_init(au_convert_ctx);
    ret = av_opt_set_int(au_convert_ctx, "in_channel_count", audio_codec_ctx->channels, 0);
    ret = av_opt_set_int(au_convert_ctx, "in_sample_rate", audio_codec_ctx->sample_rate, 0);
    ret = av_opt_set_sample_fmt(au_convert_ctx, "in_sample_fmt", audio_codec_ctx->sample_fmt, 0);
    ret = av_opt_set_channel_layout(au_convert_ctx, "in_channel_layout", in_channel_layout, 0);
    ret = av_opt_set_int(au_convert_ctx, "out_channel_count", out_channels, 0);
    ret = av_opt_set_int(au_convert_ctx, "out_sample_rate", audio_codec_ctx->sample_rate, 0);
    ret = av_opt_set_sample_fmt(au_convert_ctx, "out_sample_fmt", AV_SAMPLE_FMT_S16, 0);
    ret = av_opt_set_channel_layout(au_convert_ctx, "out_channel_layout", out_channel_layout, 0);
    if ((ret = swr_init(au_convert_ctx)) < 0) {
        printf("Failed to initialize the resampling context for pull stream, return code: %d\n", ret);
        return -1;
    }

    //初始化SDL
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {  
        printf( "Could not initialize SDL - %s\n", SDL_GetError()); 
        return -1;
    } 
 
    // 初始化SDL显示相关，按照输入流原始大小展示
    int screen_w = video_codec_ctx->width;
    int screen_h = video_codec_ctx->height;
    SDL_Window *screen = SDL_CreateWindow("Simplest ffmpeg player's Window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screen_w, screen_h, SDL_WINDOW_OPENGL);
    if(!screen) {  
        printf("SDL: could not create window - exiting:%s\n",SDL_GetError());  
        return -1;
    }
    SDL_Renderer* sdlRenderer = SDL_CreateRenderer(screen, -1, 0);
    SDL_Texture* sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, screen_w, screen_h);
    SDL_Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.w = screen_w;
    rect.h = screen_h;

    // 初始化SDL音频相关，SDL_AudioSpec
    SDL_AudioSpec wanted_spec;
    // wanted_spec.freq = out_sample_rate; 
    wanted_spec.freq = audio_codec_ctx->sample_rate; 
    wanted_spec.format = AUDIO_S16SYS; 
    wanted_spec.channels = out_channels; 
    wanted_spec.silence = 0; 
    wanted_spec.samples = out_nb_samples; 
    wanted_spec.callback = fill_audio; 
    wanted_spec.userdata = audio_codec_ctx; 
    if (SDL_OpenAudio(&wanted_spec, NULL)<0){ 
        printf("can't open audio.\n"); 
        return -1; 
    }

    //创建消息线程
    SDL_Thread *video_tid = SDL_CreateThread(sfp_refresh_thread, NULL, NULL);

    //消息循环
    SDL_Event e;
    int got;
    while(true) {
        //Wait
        SDL_WaitEvent(&e);
        if(e.type == SFM_REFRESH_EVENT){
            SDL_PauseAudio(0);
            while(true){
                if(av_read_frame(fctx, packet) >= 0) {
                    if(packet->stream_index == video_stream) {
                        ret = avcodec_decode_video2(video_codec_ctx, pFrame, &got, packet);
                        if(ret < 0){
                            printf("Decode Error.\n");
                            return -1;
                        }
                        if(got){
                            sws_scale(img_convert_ctx, (const unsigned char* const*)pFrame->data, pFrame->linesize, 0, video_codec_ctx->height, pFrameYUV->data, pFrameYUV->linesize);
                            //SDL---------------------------
                            SDL_UpdateTexture(sdlTexture, NULL, pFrameYUV->data[0], pFrameYUV->linesize[0] );  
                            SDL_RenderClear(sdlRenderer);  
                            //SDL_RenderCopy( sdlRenderer, sdlTexture, &sdlRect, &sdlRect);  
                            SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, NULL);  
                            SDL_RenderPresent(sdlRenderer);  
                            //SDL End-----------------------
                            break;
                        }
                    } else if(packet->stream_index == audio_stream){
                        ret = avcodec_decode_audio4(audio_codec_ctx, audioFrame, &got, packet);
                        if(got > 0){
                            int rr = swr_convert(au_convert_ctx, &audio_out_buffer, MAX_AUDIO_FRAME_SIZE, (const uint8_t **)audioFrame->data , audioFrame->nb_samples);
                            pSoundBuf->Write((char*)audio_out_buffer, rr*4);
                        }
                    }
                    av_free_packet(packet);
                } else {
                    //Exit Thread
                    thread_exit = 1;
                    break;
                }
            }
        } else if (e.type == SDL_QUIT) {
            thread_exit = 1;
        } else if(e.type == SFM_BREAK_EVENT){
            break;
        }
    }

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

    // close Codec
    avcodec_close(video_codec_ctx);
    avcodec_close(audio_codec_ctx);

    // close input
    avformat_close_input(&fctx);

    return 0;
}
