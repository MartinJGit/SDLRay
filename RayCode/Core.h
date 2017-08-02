#pragma once

#define SIMD

#ifdef SIMD

#include <xmmintrin.h>



typedef __m128 Color;
#else

#endif

extern "C"
{
    __declspec(dllexport) void Trace(void*& worldData, Color* buffer, int width, int height, int workerCount);
    __declspec(dllexport) void TracePixel(void*& worldData, int width, int height, int workerCount, int x, int y);
    __declspec(dllexport) void Entry(void*& worldData);
    __declspec(dllexport) void Exit(void*& worldData);
}

double Now();

void DebugTrace(int width, int height, int x, int y);

void CreateWorkers(int workerCount);
void DestroyWorkers();

double Now();
