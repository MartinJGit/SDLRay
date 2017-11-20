#pragma once

#include "Core.h"

extern "C"
{
    // External facing API that the main app will call into
    __declspec(dllexport) void Trace(WorldData*& worldData, Color* buffer, int width, int height, int workerCount);
    __declspec(dllexport) void TracePixel(WorldData*& worldData, int width, int height, int workerCount, int x, int y);
    __declspec(dllexport) void Entry(WorldData*& worldData);
    __declspec(dllexport) void Exit(WorldData*& worldData);
}
