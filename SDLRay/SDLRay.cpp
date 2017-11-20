// SDLRay.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "SDL.h"
#include <chrono>
#include <thread>
#include <vector>
#include <Windows.h>
#include <fstream>

#include <direct.h>
#define GetCurrentDir _getcwd

typedef unsigned long DWORD;
typedef unsigned char BYTE;
typedef unsigned short WORD;

double Now()
{
    return SDL_GetPerformanceCounter() / (double)SDL_GetPerformanceFrequency();
}

typedef void(__stdcall *TraceFunc)(void*& worldData, __m128* output, int width, int height, int workers);
typedef void(__stdcall *TracePixelFunc)(void*& worldData, int width, int height, int workers, int x, int y);
typedef void(__stdcall *EntryFunc)(void*& worldData);
typedef void(__stdcall *ExitFunc)(void*& worldData);

// Stores handles into the DLL
struct DLLInterface
{
    TraceFunc m_TraceFunc;
    TracePixelFunc m_TracePixelFunc;
    HINSTANCE m_DLLInst;
    time_t m_LastModified;
};

void UnloadDLL(DLLInterface& dllInterface, void*& worldData)
{
    ExitFunc exitFunc = (ExitFunc)GetProcAddress(dllInterface.m_DLLInst, "Exit");
    if (exitFunc != nullptr)
    {
        exitFunc(worldData);
    }
    FreeLibrary(dllInterface.m_DLLInst);
}

void CopyFile(char const* sourcePath, char const* destPath)
{
    std::ifstream  src(sourcePath, std::ios::binary);
    std::ofstream  dst(destPath, std::ios::binary);

    dst << src.rdbuf();
}

void GetTheFileTime(time_t& lastWriteTime)
{
    struct stat result;
    int funcRes = stat("..\\..\\RayCode\\x64\\Debug\\RayCodeDLL.dll", &result);
    lastWriteTime = result.st_mtime;
}

void LoadDLL(DLLInterface& dllInterface, void*& worldData)
{
    char buf[256];
    GetCurrentDir(buf, 256);
    // DLLs are copied into a folder "DLLStore" so that when program locks and runs them, the originals can still be
    // modified by a re-build.
    CreateDirectoryA("DLLStore", nullptr);
#ifdef _DEBUG
    CopyFile("..\\..\\RayCode\\x64\\Debug\\RayCodeDLL.dll", "DLLStore\\RayCodeDLL.dll");
    CopyFile("..\\..\\RayCode\\x64\\Debug\\RayCodeDLL.pdb", "DLLStore\\RayCodeDLL.pdb");
#else
    CopyFile("..\\..\\RayCode\\x64\\Release\\RayCodeDLL.dll", "DLLStore\\RayCodeDLL.dll");
    CopyFile("..\\..\\RayCode\\x64\\Release\\RayCodeDLL.pdb", "DLLStore\\RayCodeDLL.pdb");
#endif

    GetTheFileTime(dllInterface.m_LastModified);

    // Grab our handles and call the Entry point
    dllInterface.m_DLLInst = LoadLibrary(L"DLLStore\\RayCodeDLL.dll");
    dllInterface.m_TraceFunc = (TraceFunc)GetProcAddress(dllInterface.m_DLLInst, "Trace");
    dllInterface.m_TracePixelFunc = (TracePixelFunc)GetProcAddress(dllInterface.m_DLLInst, "TracePixel");
    EntryFunc entryFunc = (EntryFunc)GetProcAddress(dllInterface.m_DLLInst, "Entry");
    if (entryFunc)
    {
        entryFunc(worldData);
    }
}

bool HasDLLModified(DLLInterface& dllInterface)
{
    time_t lastModified = 0;
    GetTheFileTime(lastModified);
    return lastModified != dllInterface.m_LastModified;
}

void CopyToSurface(SDL_Surface* surface, int width, int height, __m128* samplesToCopy)
{
    SDL_LockSurface(surface);

    unsigned char* pixels = (unsigned char*)surface->pixels;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // Convert from 0-1 to 0-255 and clamp
            __m128 res = _mm_min_ps(_mm_mul_ps(samplesToCopy[y * width + x], _mm_set_ps1(255.0f)), _mm_set_ps1(255.0f));
            unsigned char* pixel = &pixels[(y * width + x) * surface->format->BytesPerPixel];
            pixel[0] = (unsigned char)res.m128_f32[0];
            pixel[1] = (unsigned char)res.m128_f32[1];
            pixel[2] = (unsigned char)res.m128_f32[2];
        }
    }

    SDL_UnlockSurface(surface);
}

int APIENTRY _tWinMain(HINSTANCE hInstance,
    HINSTANCE hPrevInstance,
    LPTSTR    lpCmdLine,
    int       nCmdShow)
{
    //Initialize SDL
    SDL_Init(SDL_INIT_VIDEO);

    int width = 1024;
    int height = 768;

#ifdef _DEBUG
    width /= 4;
    height /= 4;
#endif

    int bpp = 3;

    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_CreateWindowAndRenderer(width, height, SDL_WINDOW_SHOWN, &window, &renderer);
    //SDL_HideWindow(window);

    auto imageSurface = SDL_CreateRGBSurface(0,
        width,
        height,
        sizeof(unsigned char) * bpp * 8,       // depth
        0x000000ff,                            // red mask
        0x0000ff00,                            // green mask
        0x00ff0000,                            // blue mask
        0);                                    // alpha mask

    std::vector<__m128> backBuffer;
    backBuffer.resize(width * height);

    void* worldData = nullptr;

    DLLInterface dllInterface;
    LoadDLL(dllInterface, worldData);

    bool done = false;
    while (!done)
    {
        bool reload = HasDLLModified(dllInterface);
        if (reload)
        {
            UnloadDLL(dllInterface, worldData);
            LoadDLL(dllInterface, worldData);
        }

        double start = Now();

        dllInterface.m_TraceFunc(worldData, &backBuffer[0], width, height, 15);

        double end = Now();

        char msg[256];
        sprintf_s(msg, "Took %f to render\n", end - start);
        OutputDebugStringA(msg);

        CopyToSurface(imageSurface, width, height, &backBuffer[0]);

        // Copy to the window surface.
        // TODO: Do we need a imageSurface or can we copy directly to the window surface?
        auto windowSurface = SDL_GetWindowSurface(window);
        SDL_BlitSurface(imageSurface, NULL, windowSurface, NULL);
        SDL_UpdateWindowSurface(window);

        // Event loop
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                {
                    done = true;
                    break;
                }

            case SDL_MOUSEBUTTONDOWN:
                {
                    // Useful break point if we want to debug a specific pixel
                    int mouseX, mouseY;
                    SDL_GetMouseState(&mouseX, &mouseY);

                    dllInterface.m_TracePixelFunc(worldData, width, height, 1, mouseX, mouseY);
                    break;
                }

             case SDL_KEYDOWN:
                {
                    if (event.key.keysym.sym == SDLK_ESCAPE)
                    {
                        done = true;
                        break;
                    }
                }
            }
        }

#ifdef _DEBUG
        // Debug builds aren't real time, no need to hog CPU
        std::chrono::milliseconds timespan(1000);
        std::this_thread::sleep_for(timespan);
#endif
    }

    // Give the dll chance to unwind nicely
    UnloadDLL(dllInterface, worldData);

    return 0;
}

