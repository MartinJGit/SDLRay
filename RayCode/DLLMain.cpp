#include "DLLMain.h"

void Trace(WorldData*& worldData, Color* buffer, int width, int height, int workerThreads)
{
    RayCode::Trace(worldData, buffer, width, height, workerThreads);
}

void TracePixel(WorldData*& worldData, int width, int height, int workerCount, int x, int y)
{
    RayCode::TracePixel(worldData, width, height, workerCount, x, y);
}

void Entry(std::unique_ptr<WorldData>& worldData)
{
    RayCode::Entry(worldData);
}

void Exit(std::unique_ptr<WorldData>& worldData)
{
    RayCode::Exit(worldData);
}