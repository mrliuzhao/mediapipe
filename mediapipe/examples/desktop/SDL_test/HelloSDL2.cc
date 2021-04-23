#include <iostream>
#include <SDL2/SDL.h>

//定义窗口尺寸常量
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

int main()
{
    int ret_sdl = 0;
    // 初始化SDL，返回0时成功，否则返回小于0，具体错误调用SDL_GetError查看
    // 参数为子系统的Flag，主要有以下几条：
    // - SDL_INIT_TIMER timer subsystem
    // - SDL_INIT_AUDIO audio subsystem
    // - SDL_INIT_VIDEO video subsystem; automatically initializes the events subsystem
    // - SDL_INIT_JOYSTICK joystick subsystem; automatically initializes the events subsystem
    // - SDL_INIT_HAPTIC haptic (force feedback) subsystem
    // - SDL_INIT_GAMECONTROLLER controller subsystem; automatically initializes the joystick subsystem
    // - SDL_INIT_EVENTS events subsystem
    // - SDL_INIT_EVERYTHING all of the above subsystems
    // - SDL_INIT_NOPARACHUTE compatibility; this flag is ignored
    ret_sdl = SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    if(ret_sdl) {
        printf( "Could not initialize SDL: %s\n", SDL_GetError());
        return -1;
    } 

    // 将要渲染的窗口
    SDL_Window* window = NULL;
    // 窗口下的surface，包含具体的图像数据
    SDL_Surface* screenSurface = NULL;

    // 初始化SDL窗口，成功返回窗口指针，失败返回NULL
    // 参数依次为：标题，窗口x坐标，窗口y坐标，窗口width，窗口height，SDL_WindowFlags
    window = SDL_CreateWindow("Simplest ffmpeg player's Window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_OPENGL);
    if(!window) {  
        printf("SDL: could not create window:%s\n", SDL_GetError());
        return -1;
    }

    //获取 window surface
    screenSurface = SDL_GetWindowSurface(window);
    if(!screenSurface) {
        printf("SDL: could not get window surface: %s\n", SDL_GetError());
        return -1;
    }

    // 用纯白色填充surface，参数依次为：surface指针、SDL_Rect指明渲染位置、颜色
    // 往往通过SDL_MapRGB，获取RGB空间下的颜色在surface对应的格式下的颜色值
    ret_sdl = SDL_FillRect(screenSurface, NULL, SDL_MapRGB(screenSurface->format, 0xFF, 0xFF, 0xFF));
    if(ret_sdl) {
        printf( "Could not initialize SDL: %s\n", SDL_GetError());
        return -1;
    } 
    // 更新surface
    ret_sdl = SDL_UpdateWindowSurface(window);
    if(ret_sdl) {
        printf( "Error when update window: %s\n", SDL_GetError());
        return -1;
    }

    //延迟
    SDL_Delay(5000);

    // 读取图像
    SDL_Surface* imgSurface = SDL_LoadBMP("/home/gdcc/test.bmp");
    if(!imgSurface)
    {
        printf( "Unable to load image, SDL Error: %s\n", SDL_GetError());
        return -1;
    }
    // 将读取的图像数据发送至屏幕的surface
    ret_sdl = SDL_BlitSurface(imgSurface, NULL, screenSurface, NULL);
    if(ret_sdl) {
        printf("Error when blitting image: %s\n", SDL_GetError());
        return -1;
    }

    // 更新surface
    ret_sdl = SDL_UpdateWindowSurface(window);
    if(ret_sdl) {
        printf( "Error when update window: %s\n", SDL_GetError());
        return -1;
    }

    //延迟
    SDL_Delay(5000);

    // SDL_Renderer* sdlRenderer = SDL_CreateRenderer(screen, -1, 0);
    // SDL_Texture* sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, screen_w, screen_h);
    // SDL_Rect rect;
    // rect.x = 0;
    // rect.y = 0;
    // rect.w = screen_w;
    // rect.h = screen_h;

    //释放 surface
    SDL_FreeSurface(imgSurface);
    imgSurface = NULL;

    // Close and destroy the window
    SDL_DestroyWindow(window);
    window = NULL;

    // Close SDL
    SDL_Quit();

    return 0;
}



