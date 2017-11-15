#ifdef _DEBUG
#include "Core.h"

#include "gtest\gtest.h"
#include "BVH.h"

namespace BVH
{

class BVHTest
{
public:
    BVH::Root a;
    BVH::Root b;

    void Test()
    {
        EXPECT_EQ(a.m_MaxLeafSize, b.m_MaxLeafSize);
    }
};

}

TEST(BVH, Serialize)
{
    BVH::BVHTest tes;

    std::vector<BVH::BVHPoly> polys;
    std::vector<BVH::BVHVert> verts;
    verts.push_back({ 0.0f, 0.0f, 0.0f, 0.0f });
    verts.push_back({ 0.0f, 0.0f, 0.0f, 0.0f });
    verts.push_back({ 0.0f, 0.0f, 0.0f, 0.0f });
    BVH::BVHPoly poly;
    poly.m_Norm = { 1.0f, 2.0f, 3.0f };
    poly.m_VertexIndices[0] = 1;
    poly.m_VertexIndices[1] = 2;
    poly.m_VertexIndices[2] = 0;
    polys.push_back(poly);
    int maxLeafSize = 2;
    tes.a.Build(&polys[0], (unsigned int)polys.size(), &verts[0], (unsigned int)verts.size(), maxLeafSize);

    std::unique_ptr<char> bufMem;
    int memLength;
    tes.a.Serialise(bufMem, memLength);

    tes.b.Deserialise(bufMem.get(), memLength);

    tes.Test();
}
#endif