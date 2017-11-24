#include "math.h"
#include "Core.h"
#include <array>
#include <vector>
#include <concurrent_queue.h>
#include <thread>
#include "Windows.h"
#include "Vector.h"
#include "Intersection.h"
#include "BVH.h"

#ifdef _DEBUG
#include "gtest\gtest.h"
#endif
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

float xRot = 0.0f;

double GetInvFrequency()
{
    static double s_Frequency = 0;
    static double s_InvFrequency = 0;
    if (s_Frequency == 0)
    {
        LARGE_INTEGER frequency;
        QueryPerformanceFrequency(&frequency);
        s_Frequency = static_cast<double>(frequency.QuadPart);
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


struct IntersectionResult
{
    float m_Dist;
    Vec3 m_Normal;
};

struct Ray
{
    Vec3 m_Pos;
    Vec3 m_Dir;
    Vec3 m_InvDir;
    Vec3 m_Sign;
};

struct MeshInstance
{
    inline NVec FindClosest(WVec rayP, WVec rayD, WVec invRayD, WVec invSign, NVec rayLength, WVec& normal) const
    {
        WVec localRayP = rayP - m_TranslationW;
        //return m_BVH->FindClosest(localRayP, rayD, invRayD, invSign, rayLength, normal);

        NVec distSource = NVecMake(FLT_MAX);
        NVec dist = distSource;
        AABB aabb = { m_BVH->Min(), m_BVH->Max() };
        RayAABB(localRayP, invRayD, aabb, dist);

        NVec intersects = NVecLT(dist, distSource);
        if (NVecMoveMask(intersects) > 0)
        {
            return m_BVH->FindClosest(localRayP, rayD, invRayD, invSign, rayLength, normal);
        }

        return distSource;
    }

    NVec m_Translation;
    WVec m_TranslationW;
    std::shared_ptr<BVH::Root> m_BVH;
};

struct WorldData
{
    std::vector<Vec3> m_Spheres;
    std::vector<AABB> m_AABB;

    std::vector<MeshInstance> m_Meshes;
};

int gBreak = 0;

void RayCode::Entry(std::unique_ptr<WorldData>& worldData)
{
#ifdef _DEBUG
    int argc = 1;
    char* nullPtr[] = { "GTest" };
    ::testing::InitGoogleTest(&argc, nullPtr);

    RUN_ALL_TESTS();
#else
    CreateWorkers(15);
#endif

    if (worldData == nullptr)
    {
        worldData = std::make_unique<WorldData>();

        for (int z = 0; z < 1; ++z)
        {
            for (int y = 0; y < 3; ++y)
            {
                for (int x = 0; x < 3; ++x)
                {
                    float xInterp = x / (3.0f - 1);
                    float yInterp = y / (3.0f - 1);
                    float zInterp = z / (3.0f - 1);
                    Vec3 sphereC = Vec3Make(6.0f * xInterp, 6.0f * yInterp, 6.0f * zInterp) - Vec3Make(3.0f, 3.0f, 9.0f);
                    //worldData->m_Spheres.push_back(sphereC);
                }
            }
        }

        for (int y = 0; y < 3; ++y)
        {
            for (int x = 0; x < 3; ++x)
            {
                float xInterp = x / (3.0f - 1);
                float yInterp = y / (3.0f - 1);
                Vec3 sphereC = Vec3Make(6.0f * xInterp, 6.0f * yInterp, 0.0f) - Vec3Make(3.0f, 3.0f, 0.0f);
                AABB aab;
                aab.m_Min = sphereC - Vec3Make(0.5f);
                aab.m_Max = sphereC + Vec3Make(0.5f);
                worldData->m_AABB.push_back(aab);
            }
        }

        char const* cachePath = "..\\..\\happy\\bvh.cache";

        bool loaded = false;

        std::ifstream file(cachePath, std::ios::binary | std::ios::ate);
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::shared_ptr<BVH::Root> bvh = std::make_shared<BVH::Root>();

        if (size > 0)
        {
            std::vector<char> buffer(size);
            if (file.read(buffer.data(), size))
            {
                loaded = bvh->Deserialise(&buffer[0], (int)buffer.size());
            }
        }
        
        if (!loaded)
        {
            char const* ply[] =
            {
                "..\\..\\happy\\buddha.obj",
                /*"..\\..\\happy\\happyBottomFill1_0.ply",
                "..\\..\\happy\\happyBottomFill2_0.ply",
                "..\\..\\happy\\happyTopFill1_0.ply",
                "..\\..\\happy\\happySideRight_0.ply",
                "..\\..\\happy\\happySideRight_24.ply",
                "..\\..\\happy\\happySideRight_48.ply",
                "..\\..\\happy\\happySideRight_72.ply",
                "..\\..\\happy\\happySideRight_96.ply",
                "..\\..\\happy\\happySideRight_120.ply",
                "..\\..\\happy\\happySideRight_144.ply",
                "..\\..\\happy\\happySideRight_168.ply",
                "..\\..\\happy\\happySideRight_192.ply",
                "..\\..\\happy\\happySideRight_216.ply",
                "..\\..\\happy\\happySideRight_240.ply",
                "..\\..\\happy\\happySideRight_264.ply",
                "..\\..\\happy\\happySideRight_288.ply",
                "..\\..\\happy\\happySideRight_312.ply",
                "..\\..\\happy\\happySideRight_336.ply",*/
            };
            //worldData->m_BVH.resize(_ARRAYSIZE(ply));

            std::vector<BVH::BVHPoly> polys;
            std::vector<BVH::BVHVert> verts;

            for (int fileI = 0; fileI < _ARRAYSIZE(ply); ++fileI)
            {
                int vertOffset = (int)verts.size();

                std::ifstream stream(ply[fileI]);

                if (ply[fileI][strlen(ply[fileI]) - 1] == 'j')
                {
                    int vertCount = 0;
                    std::string line;
                    while (std::getline(stream, line))
                    {
                        if (!line.empty() && line[0] == 'v')
                        {
                            std::stringstream ss(line);

                            BVH::BVHVert vert;
                            char lead;
                            ss >> lead;
                            ss >> vert.m128_f32[0];
                            ss >> vert.m128_f32[1];
                            ss >> vert.m128_f32[2];
                            verts.push_back(vert);
                        }
                        else if (!line.empty() && line[0] == 'f')
                        {
                            std::stringstream ss(line);

                            BVH::BVHPoly poly;
                            char lead;
                            ss >> lead;
                            ss >> poly.m_VertexIndices[0];
                            ss >> poly.m_VertexIndices[1];
                            ss >> poly.m_VertexIndices[2];
                            --poly.m_VertexIndices[0];
                            --poly.m_VertexIndices[1];
                            --poly.m_VertexIndices[2];
                            polys.push_back(poly);
                        }
                    }
                }
                else
                {
                    int vertCount = 0;
                    std::string line;
                    while (std::getline(stream, line))
                    {
                        if (line.substr(0, 14) == "element vertex")
                        {
                            vertCount = atoi(line.substr(14).c_str());
                            break;
                        }
                    }

                    verts.reserve(verts.size() + vertCount);
                    polys.reserve(polys.size() + vertCount);

                    if (vertCount > 0)
                    {
                        while (std::getline(stream, line) && line != "end_header")
                        {
                        }

                        int vertI = 0;
                        while (std::getline(stream, line) && vertI < vertCount)
                        {
                            std::stringstream ss(line);

                            if (vertI % 3 == 2)
                            {
                                BVH::BVHPoly poly;
                                poly.m_VertexIndices[0] = (u32)verts.size() - 2;
                                poly.m_VertexIndices[2] = (u32)verts.size() - 1;
                                poly.m_VertexIndices[1] = (u32)verts.size();
                                polys.push_back(poly);
                            }

                            BVH::BVHVert vert;
                            ss >> vert.m128_f32[0];
                            ss >> vert.m128_f32[2];
                            ss >> vert.m128_f32[1];
                            verts.push_back(vert * 100);
                            vertI++;
                        }
                    }
                }
            }

            if (!verts.empty())
            {
                for (BVH::BVHPoly& poly : polys)
                {
                    Vec3 edge1 = Vec3Normalise(verts[poly.m_VertexIndices[1]] - verts[poly.m_VertexIndices[0]]);
                    Vec3 edge2 = Vec3Normalise(verts[poly.m_VertexIndices[2]] - verts[poly.m_VertexIndices[0]]);
                    poly.m_Norm = Vec3Cross(edge1, edge2);

                    Vec3 min = Vec3Min(Vec3Min(verts[poly.m_VertexIndices[0]], verts[poly.m_VertexIndices[1]]), verts[poly.m_VertexIndices[2]]);
                    Vec3 max = Vec3Max(Vec3Max(verts[poly.m_VertexIndices[0]], verts[poly.m_VertexIndices[1]]), verts[poly.m_VertexIndices[2]]);

                    Vec3 size = max - min;
                    static float tolerance = 10.0f;
                    if ((size.m128_f32[0] + size.m128_f32[1] + size.m128_f32[2]) > tolerance)
                    {
                        gBreak = 1;
                    }
                }

                bvh->Build(&polys[0], (u32)polys.size(), &verts[0], (u32)verts.size(), 5);

                std::unique_ptr<char> dataStream;
                int streamLength = 0;
                bvh->Serialise(dataStream, streamLength);
                std::ofstream outfile(cachePath, std::ofstream::binary);
                outfile.write(dataStream.get(), streamLength);

                loaded = true;
            }

        }

        if (loaded)
        {
            //worldData->m_Meshes.push_back({ WVecMake2(-5.0f, -4.0f, 0.0f), bvh });
            worldData->m_Meshes.push_back({ { 0.0f, -4.0f, 0.0f, 0.0f }, WVecMake2(0.0f, -4.0f, 0.0f), bvh });
            //worldData->m_Meshes.push_back({ WVecMake2(5.0f, -4.0f, 0.0f), bvh });
        }

        {
            BVH::BVHVert verts[4];
            float scale = 1000.0f;
            verts[0] = Vec3Make(-1.0f, 0.0f, -1.0f) * scale;
            verts[1] = Vec3Make(-1.0f, 0.0f, 1.0f) * scale;
            verts[2] = Vec3Make(1.0f, 0.0f, 1.0f) * scale;
            verts[3] = Vec3Make(1.0f, 0.0f, -1.0f) * scale;
            BVH::BVHPoly polys[2];
            polys[0].m_VertexIndices[0] = 0;
            polys[0].m_VertexIndices[1] = 1;
            polys[0].m_VertexIndices[2] = 2;
            polys[1].m_VertexIndices[0] = 0;
            polys[1].m_VertexIndices[1] = 2;
            polys[1].m_VertexIndices[2] = 3;

            std::shared_ptr<BVH::Root> groundPlane = std::make_shared<BVH::Root>();
            groundPlane->Build(polys, 2, verts, 4, 10);
            //worldData->m_Meshes.push_back({ {0.0f, -10.0f, 0.0f }, WVecMake2(0.0f, -10.0f, 0.0f), groundPlane });
        }
    }
}

void RayCode::Exit(std::unique_ptr<WorldData>& worldData)
{
    DestroyWorkers();
}

#if 0
void RaySphereIntersectionTest(Ray r, Vec3 pos, Vec3 radiusSq, IntersectionResult& res)
{
    // Find the closest point on the ray to the sphere position, if the distance is less than the radius, we have a collision.
    float collisionDistance = FLT_MAX;

    Vec3 rayStartToSphereCentre = pos - r.m_Pos;
    Vec3 dot = Dot(rayStartToSphereCentre, r.m_Dir);
    if (dot.m128_f32[0] > 0.0f)
    {
        Vec3 closestPoint = r.m_Pos + r.m_Dir * dot;

        Vec3 pointToSphere = closestPoint - pos;
        Vec3 lenthSq = VecLengthSq(pointToSphere);
        Vec3 collided = _mm_cmple_ps(lenthSq, radiusSq);
        if (lenthSq.m128_f32[0] >= 0.0f && lenthSq.m128_f32[0] < radiusSq.m128_f32[0])
        {
            collisionDistance = dot.m128_f32[0];
            if (pointToSphere.m128_f32[0] > 0.000001f)
            {
                res.m_Normal = Vec3Normalise(pointToSphere);
            }
            else
            {
                res.m_Normal = -r.m_Dir;
            }
        }
    }

    res.m_Dist = collisionDistance;
}
#else

#endif
#if 1


#else
void RaySphereIntersection4(Vec3 pX, Vec3 pY, Vec3 pZ, Vec3 dX, Vec3 dY, Vec3 dZ, Vec3 pos, Vec3 radiusSq, Vec3& dist, Vec3& normalsX, Vec3& normalsY, Vec3& normalsZ)
{
    dist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };

    Vec3 posX = { pos.m128_f32[0], pos.m128_f32[0] , pos.m128_f32[0] , pos.m128_f32[0] };
    Vec3 posY = { pos.m128_f32[1], pos.m128_f32[1] , pos.m128_f32[1] , pos.m128_f32[1] };
    Vec3 posZ = { pos.m128_f32[2], pos.m128_f32[2] , pos.m128_f32[2] , pos.m128_f32[2] };

    Vec3 rayToSphereX = posX - pX;
    Vec3 rayToSphereY = posY - pY;
    Vec3 rayToSphereZ = posZ - pZ;

    Vec3 a = rayToSphereX * rayToSphereX + rayToSphereY * rayToSphereY + rayToSphereZ * rayToSphereZ;

    Vec3 aGreaterThanZero = Vec3GT(a, Vec3Make(0.0f, 0.0f, 0.0f, 0.0f));

    Vec3 b = rayToSphereX * dX + rayToSphereY * dY + rayToSphereZ * dZ;
    Vec3 d = radiusSq - (a - (b * b));


    Vec3 minD = b - Vec3Sqrt(d);

    dist = Vec3Select(minD, dist, aGreaterThanZero);
    normalsX = (pX + dX * minD) - posX;
    normalsY = (pY + dY * minD) - posY;
    normalsZ = (pZ + dZ * minD) - posZ;

    Vec3 length = Vec3Sqrt(normalsX * normalsX + normalsY * normalsY + normalsZ * normalsZ);

    normalsX = normalsX / length;
    normalsY = normalsY / length;
    normalsZ = normalsZ / length;
}

#endif

#ifdef SIMD
void VECTOR_CALL TraceRay4(WVec rayP, WVec rayD, WVec invRayD, WVec rayDSign, WorldData const& world, int depth, WVec& results)
{
    NVec sphereRadius = NVecMake(1.0f);

    Vec3 lightDir = Vec3Normalise(Vec3Make(cosf(xRot), sinf(xRot), 0.0f, 0.0f));
    lightDir = -lightDir;

    WVec lightColor = WVecMake2(0.5f, 0.5f, 0.5f);

    WVec lightDirExp = WVecMake2(
        lightDir.m128_f32[0],
        lightDir.m128_f32[1],
        lightDir.m128_f32[2]);

    NVec closestIntersection = NVecMake(FLT_MAX);
    WVec closestNormal = WVecMake(0.0f);

    for (auto sphere : world.m_Spheres)
    {
        WVec normal;

        NVec intersectionDist = NVecMake(FLT_MAX);
        RaySphereIntersection(rayP, rayD, sphere, sphereRadius, intersectionDist, normal);

        NVec useNewNormal = NVecLT(intersectionDist, closestIntersection);
        closestIntersection = NVecSelect(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = WVecSelect(normal, closestNormal, useNewNormal);
    }

    for (auto& mesh : world.m_Meshes)
    {
        NVec rayLength = NVecMake(1000.0f);
        WVec bvhNormal;
        NVec bvhDist = mesh.FindClosest(rayP, rayD, invRayD, rayDSign, rayLength, bvhNormal);
        NVec useBVH = NVecLT(bvhDist, closestIntersection);
        closestIntersection = NVecSelect(bvhDist, closestIntersection, useBVH);
        closestNormal = WVecSelect(bvhNormal, closestNormal, useBVH);
    }

#if 0
    for (auto aabb : world.m_AABB)
    {
        WVec normal;

        NVec intersectionDist = NVecMake(0.0f);
        RayAABB(rayP, invRayD, rayDSign, aabb, intersectionDist, normal);

        NVec useNewNormal = NVecLT(intersectionDist, closestIntersection);
        closestIntersection = NVecSelect(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = WVecSelect(normal, closestNormal, useNewNormal);
    }
#endif


    NVec litColor = NVecMax(WVecDot(closestNormal, lightDirExp), NVecMake(0.0f));
    WVec rayColors = lightColor * litColor + WVecMake(0.2f) + WVecMake2(0.5f, 0.5f, 0.5f);
    NVec foundIntersection = NVecLT(closestIntersection, NVecMake(FLT_MAX));
    results = WVecSelect(rayColors, results, foundIntersection);
    /*
    if (NVecMoveMask(foundIntersection) > 0 && depth > 0)
    {
        WVec rayDir = WVecReflect(rayD, closestNormal);
        WVec rayPos = rayP + rayD * closestIntersection;

        WVec rayDSign = {
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.x, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.y, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.z, NVecMake(0.0f))),
        };

        WVec invRayDir = {
            NVecMake(1.0f) / rayDir.x,
            NVecMake(1.0f) / rayDir.y,
            NVecMake(1.0f) / rayDir.z,
        };

        WVec reflectionRes = WVecMake(0.0f);
        //TraceRay4(rayPos, rayDir, invRayDir, rayDSign, world, depth - 1, reflectionRes);

        results = results + reflectionRes * NVecMake(0.25f);
    }*/
}

#ifdef AVX
#if 0
void VECTOR_CALL TraceRay8(Vec83 rayP, Vec83 rayD, Vec83 invRayD, Vec83 rayDSign, WorldData const& world, int depth, Vec83& results)
{
    Vec3 sphereRadius = { 1.0f, 1.0f, 1.0f, 1.0f };

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f, 0.0f));
    lightDir = -lightDir;

    Vec83 lightColor = { Vec3Make(0.5f), Vec3Make(0.5f), Vec3Make(0.5f) };

    Vec83 lightDirExp = {
        Vec3Make(lightDir.m128_f32[0]),
        Vec3Make(lightDir.m128_f32[1]),
        Vec3Make(lightDir.m128_f32[2]) };

    Vec3 closestIntersection = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
    Vec83 closestNormal = { Vec3Make(0.0f), Vec3Make(0.0f), Vec3Make(0.0f) };

    for (auto sphere : world.m_Spheres)
    {
        Vec83 normal;

        Vec8 intersectionDist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
        RaySphereIntersection4(rayP, rayD, sphere, sphereRadius, intersectionDist, normal);

        Vec3 useNewNormal = Vec3LT(intersectionDist, closestIntersection);
        closestIntersection = Vec3Select(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = Vec43Select(normal, closestNormal, useNewNormal);
    }

    for (auto aabb : world.m_AABB)
    {
        Vec83 normal;

        Vec8 intersectionDist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
        RayAABB4(rayP, invRayD, rayDSign, aabb, intersectionDist, normal);

        Vec3 useNewNormal = Vec3LT(intersectionDist, closestIntersection);
        closestIntersection = Vec3Select(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = Vec43Select(normal, closestNormal, useNewNormal);
    }


    Vec3 litColor = Vec3Max(Vec43Dot(closestNormal, lightDirExp), Vec3Make(0.0f));
    Vec83 rayColors = lightColor * litColor + Vec43Make(Vec3Make(0.2f, 0.2f, 0.2f, 0.0f));
    Vec3 foundIntersection = Vec3LT(closestIntersection, { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX });
    results = Vec43Select(rayColors, results, foundIntersection);

    if (Vec3MoveMask(foundIntersection) > 0 && depth > 0)
    {
        Vec83 rayDir = Vec43Reflect(rayD, closestNormal);
        Vec83 rayPos = rayP + rayD * closestIntersection;

        Vec83 rayDSign = {
            Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.x, Vec3Make(0.0f))),
            Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.y, Vec3Make(0.0f))),
            Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.z, Vec3Make(0.0f))),
        };

        Vec83 invRayDir = {
            Vec3Make(1.0f) / rayDir.x,
            Vec3Make(1.0f) / rayDir.y,
            Vec3Make(1.0f) / rayDir.z,
        };

        Vec83 reflectionRes = { Vec3Make(0.0f), Vec3Make(0.0f), Vec3Make(0.0f) };
        TraceRay4(rayPos, rayDir, invRayDir, rayDSign, world, depth - 1, reflectionRes);

        results = results + reflectionRes * Vec3Make(0.25f, 0.25f, 0.25f, 0.25f);
    }
}
#endif
#endif
#else

Color VECTOR_CALL TraceRay(Ray ray, WorldData const& world, int depth)
{
    Color result = { 0.0f, 0.0f, 0.0f, 1.0f };

    float sphereRadius = 1.0f;

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f));

    IntersectionResult closestIntersection;
    closestIntersection.m_Dist = FLT_MAX;

    for (auto sphere : world.m_Spheres)
    {
        IntersectionResult res;
        res.m_Dist = FLT_MAX;
        RaySphereIntersection(ray.m_Pos, ray.m_Dir, sphere, sphereRadius, res.m_Dist, res.m_Normal);
        if (res.m_Dist < closestIntersection.m_Dist)
        {
            closestIntersection = res;
        }
    }

    for (auto aabb : world.m_AABB)
    {
        IntersectionResult res;
        res.m_Dist = FLT_MAX;
        RayAABB(ray.m_Pos, ray.m_InvDir, ray.m_Sign, aabb, res.m_Dist, res.m_Normal);
        if (res.m_Dist < closestIntersection.m_Dist)
        {
            closestIntersection = res;
        }
    }

    if (closestIntersection.m_Dist != FLT_MAX)
    {
        Vec3 lightDir = Vec3Normalise({ -1.0f, -1.0f, 0.0f });
        lightDir = -lightDir;

        Vec3 lightColor = Vec3Make(0.5f);


        float lightIntensity = MAX(Dot(closestIntersection.m_Normal, lightDir), 0.0f);
        Vec3 rayColors = lightColor * lightIntensity + Vec3Make(0.2f, 0.2f, 0.2f);
        result.m128_f32[0] = rayColors.x;
        result.m128_f32[1] = rayColors.y;
        result.m128_f32[2] = rayColors.z;

        if (depth > 0)
        {
            Ray reflectedRay;
            reflectedRay.m_Dir = Vec3Reflect(ray.m_Dir, closestIntersection.m_Normal);
            reflectedRay.m_Pos = ray.m_Pos + ray.m_Dir * closestIntersection.m_Dist;

            reflectedRay.m_Sign = {
                reflectedRay.m_Dir.x > 0.0f ? 1.0f : -1.0f,
                reflectedRay.m_Dir.y > 0.0f ? 1.0f : -1.0f,
                reflectedRay.m_Dir.z > 0.0f ? 1.0f : -1.0f };

            reflectedRay.m_InvDir = {
                1.0f / reflectedRay.m_Dir.x,
                1.0f / reflectedRay.m_Dir.y,
                1.0f / reflectedRay.m_Dir.z,
            };

            Color reflectedRes = TraceRay(reflectedRay, world, depth - 1);
            result.m128_f32[0] += reflectedRes.m128_f32[0] * 0.25f;
            result.m128_f32[1] += reflectedRes.m128_f32[1] * 0.25f;
            result.m128_f32[2] += reflectedRes.m128_f32[2] * 0.25f;
        }
    }

    return result;
}
#endif

struct RowTrace
{
    Color* m_Buffer;
    int m_Width;
    int m_Height;
    int m_Row;
    WorldData* m_World;
};

Concurrency::concurrent_queue<RowTrace> gRows;
std::atomic<int> gRowCount;

void TraceRow(RowTrace row)
{
    Vec3 cameraPosition = Vec3Make(0.0f);

    cameraPosition = Vec3Make(sinf(xRot) * 10.0f, 0.0f, cosf(xRot) * 10.0f);

    cameraPosition = Vec3Make(0.0f, 0.0f, 10.0f);

    Vec3 cameraDir = Vec3Normalise(-cameraPosition);// TransformByMatrix(Vec3Make(0.0f, 0.0f, -1.0f), CreateRotationMatrixX(xRot) * CreateRotationMatrixZ(zRot));
#if 0
    Vec3 worldUp = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    Vec3 cameraRight = Vec3Normalise(Vec3Cross(worldUp, cameraPosition));

    Vec3 cameraUp = Vec3Cross(cameraDir, cameraRight);
#endif

    Vec3 topLeft = { -0.5f, 0.5f, cameraPosition.m128_f32[2] - 1.0f };
    Vec3 topRight{ 0.5f, 0.5f, cameraPosition.m128_f32[2] - 1.0f };

    Vec3 bottomLeft = { -0.5f, -0.5f, cameraPosition.m128_f32[2] - 1.0f };
    Vec3 bottomRight{ 0.5f, -0.5f, cameraPosition.m128_f32[2] - 1.0f };

#ifdef SIMD

    WVec topLeftS = WVecMake2(-0.5f, 0.5f, cameraPosition.m128_f32[2] - 1.0f);
    WVec topRightS = WVecMake2(0.5f, 0.5f, cameraPosition.m128_f32[2] - 1.0f);
    WVec bottomLeftS = WVecMake2(-0.5f, -0.5f, cameraPosition.m128_f32[2] - 1.0f);
    WVec bottomRightS = WVecMake2(0.5f, -0.5f, cameraPosition.m128_f32[2] - 1.0f);

    WVec rayPos = WVecMake2(cameraPosition.m128_f32[0], cameraPosition.m128_f32[1], cameraPosition.m128_f32[2]);

    NVec yNorm = NVecMake(((row.m_Height - 1.0f) - row.m_Row) / (row.m_Height - 1.0f));

    int const rayWidth = sizeof(NVec) / sizeof(float);

    for (int x = 0; x < row.m_Width; x += rayWidth)
    {
#ifdef SIMD
        NVec xNorm = { x / (row.m_Width - 1.0f), (x + 1) / (row.m_Width - 1.0f), (x + 2) / (row.m_Width - 1.0f), (x + 3) / (row.m_Width - 1.0f) };
#else
        NVec xNorm = Vec8Make(
            x / (row.m_Width - 1.0f), 
            (x + 1) / (row.m_Width - 1.0f), 
            (x + 2) / (row.m_Width - 1.0f), 
            (x + 3) / (row.m_Width - 1.0f),
            (x + 4) / (row.m_Width - 1.0f),
            (x + 5) / (row.m_Width - 1.0f),
            (x + 6) / (row.m_Width - 1.0f),
            (x + 7) / (row.m_Width - 1.0f));
#endif

        WVec rayPoint = WVecLerp(WVecLerp(bottomLeftS, bottomRightS, xNorm), WVecLerp(topLeftS, topRightS, xNorm), yNorm);
        WVec rayDir = WVecNormalise(rayPoint - rayPos);
        WVec color = WVecMake(0.0f);

        WVec invRayDir = {
            NVecMake(1.0f) / rayDir.x,
            NVecMake(1.0f) / rayDir.y,
            NVecMake(1.0f) / rayDir.z,
        };

        WVec rayDSign = {
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.x, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.y, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.z, NVecMake(0.0f))),
        };

        TraceRay4(rayPoint, rayDir, invRayDir, rayDSign, *row.m_World, 1, color);

#ifdef AVX
        row.m_Buffer[row.m_Row * row.m_Width + x + 0] = { color.x.m256_f32[0], color.y.m256_f32[0], color.z.m256_f32[0], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 1] = { color.x.m256_f32[1], color.y.m256_f32[1], color.z.m256_f32[1], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 2] = { color.x.m256_f32[2], color.y.m256_f32[2], color.z.m256_f32[2], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 3] = { color.x.m256_f32[3], color.y.m256_f32[3], color.z.m256_f32[3], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 4] = { color.x.m256_f32[4], color.y.m256_f32[4], color.z.m256_f32[4], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 5] = { color.x.m256_f32[5], color.y.m256_f32[5], color.z.m256_f32[5], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 6] = { color.x.m256_f32[6], color.y.m256_f32[6], color.z.m256_f32[6], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 7] = { color.x.m256_f32[7], color.y.m256_f32[7], color.z.m256_f32[7], 0.0f };
#else
        row.m_Buffer[row.m_Row * row.m_Width + x + 0] = { color.x.m128_f32[0], color.y.m128_f32[0], color.z.m128_f32[0], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 1] = { color.x.m128_f32[1], color.y.m128_f32[1], color.z.m128_f32[1], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 2] = { color.x.m128_f32[2], color.y.m128_f32[2], color.z.m128_f32[2], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 3] = { color.x.m128_f32[3], color.y.m128_f32[3], color.z.m128_f32[3], 0.0f };
#endif
    }

#else
    float yNorm = row.m_Row / (row.m_Height - 1.0f);
    for (int x = 0; x < row.m_Width; ++x)
    {
        Ray ray;
        ray.m_Pos = cameraPosition;

        float xNorm = x / (row.m_Width - 1.0f);


        Vec3 rayPoint = Lerp(Lerp(topLeft, topRight, xNorm), Lerp(bottomLeft, bottomRight, xNorm), yNorm);
        ray.m_Dir = rayPoint - cameraPosition;
        ray.m_Dir = Vec3Normalise(ray.m_Dir);
        ray.m_InvDir = Vec3Make(1.0f / ray.m_Dir.x, 1.0f / ray.m_Dir.y, 1.0f / ray.m_Dir.z);
        ray.m_Sign = Vec3Make(ray.m_Dir.x > 0.0f ? 1.0f : -1.0f, ray.m_Dir.y > 0.0f ? 1.0f : -1.0f, ray.m_Dir.z > 0.0f ? 1.0f : -1.0f);
        Color result = TraceRay(ray, *row.m_World, 1);
        row.m_Buffer[row.m_Row * row.m_Width + x] = result;
    }
#endif

   --gRowCount;
}

bool gShutdownWorkers = true;

void ThreadWorker()
{
    while (!gShutdownWorkers)
    {
        RowTrace r;
        while (gRows.try_pop(r))
        {
            TraceRow(r);
        }

        Sleep(0);
    }
}

std::vector<std::unique_ptr<std::thread>> threads;

void DestroyWorkers()
{
    gShutdownWorkers = true;
    for (auto& worker : threads)
    {
        worker->join();
    }

    threads.clear();
}

void CreateWorkers(int workerThreads)
{

    gShutdownWorkers = false;
    for (int threadI = 0; threadI < workerThreads; ++threadI)
    {
        threads.push_back(std::make_unique<std::thread>(ThreadWorker));
    }
}

void RayCode::Trace(WorldData*& worldData, Color* buffer, int width, int height, int workerThreads)
{
    gRowCount = height;

    xRot += 0.1f;

    for (int y = 0; y < height; ++y)
    {
        RowTrace rowTrace;
        rowTrace.m_Height = height;
        rowTrace.m_Width = width;
        rowTrace.m_Buffer = buffer;
        rowTrace.m_Row = y;
        rowTrace.m_World = worldData;
        gRows.push(rowTrace);
    }

    RowTrace r;
    while (gRows.try_pop(r))
    {
        TraceRow(r);
    }

    // Wait for all rows to complete
    while (gRowCount > 0)
    {
    }
}

void RayCode::TracePixel(WorldData*& worldData, int width, int height, int workerCount, int x, int y)
{
    Vec3 cameraPosition = { 0.0f, 0.0f, 10.0f };

    cameraPosition = Vec3Make(sinf(xRot) * 10.0f, 0.0f, cosf(xRot) * 10.0f);

    Vec3 cameraDir = Vec3Normalise(-cameraPosition);// TransformByMatrix(Vec3Make(0.0f, 0.0f, -1.0f), CreateRotationMatrixX(xRot) * CreateRotationMatrixZ(zRot));


    Vec3 topLeft = { -0.5f, 0.5f, 9.0f };
    Vec3 topRight{ 0.5f, 0.5f, 9.0f };

    Vec3 bottomLeft = { -0.5f, -0.5f, 9.0f };
    Vec3 bottomRight{ 0.5f, -0.5f, 9.0f };

#ifdef SIMD
    Vec3 worldUp = Vec3Make(0.0f, 1.0f, 0.0f);
    Vec3 cameraRight = Vec3Normalise(Vec3Cross(worldUp, cameraPosition));

    Vec3 cameraUp = Vec3Cross(cameraDir, cameraRight);


    Ray ray;
    ray.m_Pos = cameraPosition;


    int xRounded = x / 4 * 4;

    float xNorm0 = (xRounded + 0) / (width - 1.0f);
    float xNorm1 = (xRounded + 1) / (width - 1.0f);
    float xNorm2 = (xRounded + 2) / (width - 1.0f);
    float xNorm3 = (xRounded + 3) / (width - 1.0f);

    WVec topLeftS = WVecMake2(-0.5f, 0.5f, 9.0f);
    WVec topRightS = WVecMake2(0.5f, 0.5f, 9.0f);
    WVec bottomLeftS = WVecMake2(-0.5f, -0.5f, 9.0f);
    WVec bottomRightS = WVecMake2(0.5f, -0.5f, 9.0f);

    WVec rayPos = WVecMake2(0.0f, 0.0f, 10.0f);

    NVec yNorm = NVecMake(y / (height - 1.0f));
    
#ifdef AVX
    NVec xNorm = Vec8Make(xRounded / (width - 1.0f), (xRounded + 1) / (width - 1.0f), (xRounded + 2) / (width - 1.0f), (xRounded + 3) / (width - 1.0f),
                          (xRounded + 4) / (width - 1.0f), (xRounded + 5) / (width - 1.0f), (xRounded + 6) / (width - 1.0f), (xRounded + 7) / (width - 1.0f));
#else
    NVec xNorm = Vec3Make(xRounded / (width - 1.0f), (xRounded + 1) / (width - 1.0f), (xRounded + 2) / (width - 1.0f), (xRounded + 3) / (width - 1.0f));
#endif

    WVec rayPoint = WVecLerp(WVecLerp(topLeftS, topRightS, xNorm), WVecLerp(bottomLeftS, bottomRightS, xNorm), yNorm);
    WVec rayDir = WVecNormalise(rayPoint - rayPos);
    WVec color = WVecMake(0.0f);

    WVec invRayDir = {
        NVecMake(1.0f) / rayDir.x,
        NVecMake(1.0f) / rayDir.y,
        NVecMake(1.0f) / rayDir.z,
    };

    WVec rayDSign = {
        NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.x, NVecMake(0.0f))),
        NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.y, NVecMake(0.0f))),
        NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.z, NVecMake(0.0f))),
    };

    WVec res;
    TraceRay4(rayPoint, rayDir, invRayDir, rayDSign, *worldData, 1, res);
#else

    float xNorm = x / (width - 1.0f);
    float yNorm = y / (height - 1.0f);

    Ray ray;
    ray.m_Pos = Lerp(Lerp(topLeft, topRight, xNorm), Lerp(bottomLeft, bottomRight, xNorm), yNorm);
    ray.m_Dir = Vec3Normalise(cameraPosition);
    ray.m_InvDir = {
        1.0f / ray.m_Dir.x,
        1.0f / ray.m_Dir.y,
        1.0f / ray.m_Dir.z,
    };
    ray.m_Sign = {
        ray.m_Dir.x > 0.0f ? 1.0f : 0.0f, 
        ray.m_Dir.y > 0.0f ? 1.0f : 0.0f,
        ray.m_Dir.z > 0.0f ? 1.0f : 0.0f };

    Vec43 res;
    TraceRay(ray, *worldData, 1);
#endif
}

void LoadBVHFromOBJ(char const* path, BVH::Root& bvh, unsigned int leafSize)
{
    std::vector<BVH::BVHPoly> polys;
    std::vector<BVH::BVHVert> verts;

    std::ifstream stream(path);

    int vertCount = 0;
    std::string line;
    while (std::getline(stream, line))
    {
        if (!line.empty() && line[0] == 'v')
        {
            std::stringstream ss(line);

            BVH::BVHVert vert;
            char lead;
            ss >> lead;
            ss >> vert.m128_f32[0];
            ss >> vert.m128_f32[1];
            ss >> vert.m128_f32[2];
            verts.push_back(vert);
        }
        else if (!line.empty() && line[0] == 'f')
        {
            std::stringstream ss(line);

            BVH::BVHPoly poly;
            char lead;
            ss >> lead;
            ss >> poly.m_VertexIndices[0];
            ss >> poly.m_VertexIndices[1];
            ss >> poly.m_VertexIndices[2];
            --poly.m_VertexIndices[0];
            --poly.m_VertexIndices[1];
            --poly.m_VertexIndices[2];
            polys.push_back(poly);
        }
    }

    for (BVH::BVHPoly& poly : polys)
    {
        Vec3 edge1 = Vec3Normalise(verts[poly.m_VertexIndices[1]] - verts[poly.m_VertexIndices[0]]);
        Vec3 edge2 = Vec3Normalise(verts[poly.m_VertexIndices[2]] - verts[poly.m_VertexIndices[0]]);
        poly.m_Norm = Vec3Cross(edge1, edge2);
    }

    bvh.Build(&polys[0], (u32)polys.size(), &verts[0], (u32)verts.size(), leafSize);
}