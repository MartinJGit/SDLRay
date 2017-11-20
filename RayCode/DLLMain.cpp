#include "DLLMain.h"

void Trace(WorldData*& worldData, Color* buffer, int width, int height, int workerThreads)
{
    RayCode::Trace(worldData, buffer, width, height, workerThreads);
}

void TracePixel(WorldData*& worldData, int width, int height, int workerCount, int x, int y)
{
    RayCode::TracePixel(worldData, width, height, workerCount, x, y);
}

void Entry(WorldData*& worldData)
{
    RayCode::Entry(worldData);
}

void Exit(WorldData*& worldData)
{
    RayCode::Exit(worldData);
}