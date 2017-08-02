// SDLRay.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "SDL.h"
//#include "Core.h"
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

#define RGB(r,g,b)          ((DWORD)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

struct WorldData;

double GetInvFrequency()
{
    static double s_Frequency = 0;
    static double s_InvFrequency = 0;
    if (s_Frequency == 0)
    {
        LARGE_INTEGER frequency;
        QueryPerformanceFrequency(&frequency);
        s_Frequency = static_cast<unsigned long long>(frequency.QuadPart);
        s_InvFrequency = 1.0 / static_cast<double>(frequency.QuadPart);
    }
    return s_InvFrequency;
}

double Now()
{
    LARGE_INTEGER time;
    time.QuadPart = 1;
    QueryPerformanceCounter(&time);
    return (double)time.QuadPart * GetInvFrequency();
}

typedef void(__stdcall *TraceFunc)(WorldData*& worldData, __m128* output, int width, int height, int workers);
typedef void(__stdcall *TracePixelFunc)(WorldData*& worldData, int width, int height, int workers, int x, int y);
typedef void(__stdcall *EntryFunc)(WorldData*& worldData);
typedef void(__stdcall *ExitFunc)(WorldData*& worldData);

struct DLLInterface
{
    TraceFunc m_TraceFunc;
    TracePixelFunc m_TracePixelFunc;
    HINSTANCE m_DLLInst;
    time_t m_LastModified;
};

void UnloadDLL(DLLInterface& dllInterface, WorldData*& worldData)
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
    int funcRes = stat("..\\..\\RayCode\\x64\\Debug\\RayCode.dll", &result);
    lastWriteTime = result.st_mtime;
}

void LoadDLL(DLLInterface& dllInterface, WorldData*& worldData)
{
    CreateDirectoryA("DLLStore", nullptr);
#ifdef _DEBUG
    CopyFile("..\\..\\RayCode\\x64\\Debug\\RayCode.dll", "DLLStore\\RayCode.dll");
    CopyFile("..\\..\\RayCode\\x64\\Debug\\RayCode.pdb", "DLLStore\\RayCode.pdb");
#else
    CopyFile("..\\..\\RayCode\\x64\\Release\\RayCode.dll", "DLLStore\\RayCode.dll");
    CopyFile("..\\..\\RayCode\\x64\\Release\\RayCode.pdb", "DLLStore\\RayCode.pdb");
#endif

    GetTheFileTime(dllInterface.m_LastModified);
    dllInterface.m_DLLInst = LoadLibrary(L"DLLStore\\RayCode.dll");
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

	bool done = false;

	unsigned char* pixels = new unsigned char[bpp * width * height];
	// ... all pixels are filled with white color

	auto imageSurface = SDL_CreateRGBSurfaceFrom(pixels,
		width,
		height,
		sizeof(unsigned char) * bpp * 8,       // depth
		width * bpp,               // pitch (row length * BPP)
		0x000000ff,                                   // red mask
		0x0000ff00,                                   // green mask
		0x00ff0000,                                   // blue mask
		0);                                           // alpha mask

	std::vector<__m128> buffer;
	buffer.resize(width * height);

	SDL_GetPerformanceCounter();

	//CreateWorkers(15);

    WorldData* worldData = nullptr;

    DLLInterface dllInterface;
    LoadDLL(dllInterface, worldData);

    while (!done)
    {
        bool reload = HasDLLModified(dllInterface);
        if (reload)
        {
            UnloadDLL(dllInterface, worldData);
            LoadDLL(dllInterface, worldData);
        }

        double start = Now();

        dllInterface.m_TraceFunc(worldData, &buffer[0], width, height, 15);

        double end = Now();

        char msg[256];
        sprintf_s(msg, "Took %f to render\n", end - start);
        OutputDebugStringA(msg);

		SDL_LockSurface(imageSurface);

		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				unsigned char* pixels = (unsigned char*)imageSurface->pixels;
				__m128 res = _mm_min_ps(_mm_mul_ps(buffer[y * width + x], _mm_set_ps1(255.0f)), _mm_set_ps1(255.0f));
				unsigned char* pixel = &pixels[(y * width + x) * bpp];
				pixel[0] = res.m128_f32[0];
				pixel[1] = res.m128_f32[1];
				pixel[2] = res.m128_f32[2];
			}
		}

		SDL_UnlockSurface(imageSurface);

		SDL_Rect rect;
		rect.h = height;
		rect.w = width;
		rect.x = 0;
		rect.y = 0;

		auto windowSurface = SDL_GetWindowSurface(window);

		SDL_BlitSurface(imageSurface, NULL, windowSurface, NULL);
		/*SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		for (int i = 0; i < width; ++i)
			SDL_RenderDrawPoint(renderer, i, i);*/
		SDL_UpdateWindowSurface(window);

		SDL_Event event;
		/* Check for new events */
		while (SDL_PollEvent(&event))
		{
			/* If a quit event has been sent */
			if (event.type == SDL_QUIT)
			{
				/* Quit the application */
				done = 1;
			}
            else if (event.type == SDL_MOUSEBUTTONDOWN)
            {
                if (event.button.button == SDL_BUTTON_RIGHT)
                {
                    int mouseX, mouseY;
                    SDL_GetMouseState(&mouseX, &mouseY);

                    dllInterface.m_TracePixelFunc(worldData, width, height, 1, mouseX, mouseY);
                }
            }
		}

		std::chrono::milliseconds timespan(1000); // or whatever
		std::this_thread::sleep_for(timespan);
	}

	UnloadDLL(dllInterface, worldData);

    return 0;
}

