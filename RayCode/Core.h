#pragma once

#include <xmmintrin.h>

typedef __m128 Color;


struct WorldData;

void CreateWorkers(int workerCount);
void DestroyWorkers();

double Now();

namespace RayCode
{
    // External facing API that the main app will call into
    void Trace(WorldData*& worldData, Color* buffer, int width, int height, int workerCount);
    void TracePixel(WorldData*& worldData, int width, int height, int workerCount, int x, int y);
    void Entry(WorldData*& worldData);
    void Exit(WorldData*& worldData);
}