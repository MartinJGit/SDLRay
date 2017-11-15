#ifdef _DEBUG
#include "Core.h"

#include "gtest\gtest.h"
#include "BVH.h"

#define EXPECT_VECTOR_EQ(a, b) \
    EXPECT_FLOAT_EQ(a.m128_f32[0], b.m128_f32[0]); \
    EXPECT_FLOAT_EQ(a.m128_f32[1], b.m128_f32[1]); \
    EXPECT_FLOAT_EQ(a.m128_f32[2], b.m128_f32[2]);

namespace BVH
{

class BVHTest
{
public:
    BVH::Root a;
    BVH::Root b;

    void Test()
    {
        EXPECT_VECTOR_EQ(a.m_Min, b.m_Min);
        EXPECT_VECTOR_EQ(a.m_Max, b.m_Max);

        EXPECT_EQ(a.m_Points.size(), b.m_Points.size());
        for (size_t pointI = 0; pointI < a.m_Points.size(); ++pointI)
        {
            EXPECT_VECTOR_EQ(a.m_Points[pointI], b.m_Points[pointI]);
        }

        EXPECT_EQ(a.m_Polys.size(), b.m_Polys.size());
        for (size_t polyI = 0; polyI < a.m_Polys.size(); ++polyI)
        {
            EXPECT_VECTOR_EQ(a.m_Polys[polyI].m_Norm, b.m_Polys[polyI].m_Norm);
            EXPECT_EQ(a.m_Polys[polyI].m_VertexIndices[0], b.m_Polys[polyI].m_VertexIndices[0]);
            EXPECT_EQ(a.m_Polys[polyI].m_VertexIndices[1], b.m_Polys[polyI].m_VertexIndices[1]);
            EXPECT_EQ(a.m_Polys[polyI].m_VertexIndices[2], b.m_Polys[polyI].m_VertexIndices[2]);
        }

        for (size_t nodeI = 0; nodeI < a.m_Nodes.size(); ++nodeI)
        {
            EXPECT_EQ(a.m_Nodes[nodeI].x1, b.m_Nodes[nodeI].x1);
            EXPECT_EQ(a.m_Nodes[nodeI].x1, b.m_Nodes[nodeI].y1);
            EXPECT_EQ(a.m_Nodes[nodeI].z1, b.m_Nodes[nodeI].z1);

            EXPECT_EQ(a.m_Nodes[nodeI].leftChildOffset, b.m_Nodes[nodeI].leftChildOffset);
            EXPECT_EQ(a.m_Nodes[nodeI].rightChildOffset, b.m_Nodes[nodeI].rightChildOffset);

            EXPECT_EQ(a.m_Nodes[nodeI].x2, b.m_Nodes[nodeI].x2);
            EXPECT_EQ(a.m_Nodes[nodeI].y2, b.m_Nodes[nodeI].y2);
            EXPECT_EQ(a.m_Nodes[nodeI].z2, b.m_Nodes[nodeI].z2);

            EXPECT_EQ(a.m_Nodes[nodeI].polySplitIndex, b.m_Nodes[nodeI].polySplitIndex);

            EXPECT_EQ(a.m_Nodes[nodeI].leftMinUsesParent, b.m_Nodes[nodeI].leftMinUsesParent);
            EXPECT_EQ(a.m_Nodes[nodeI].leftMaxUsesParent, b.m_Nodes[nodeI].leftMaxUsesParent);
        }
        EXPECT_EQ(a.m_MaxLeafSize, b.m_MaxLeafSize);
    }
};

}

TEST(BVH, Serialize)
{
    BVH::BVHTest tes;

    std::vector<BVH::BVHPoly> polys;
    std::vector<BVH::BVHVert> verts;
    verts.push_back({ 0.0f, 0.5f, 0.0f, 0.0f });
    verts.push_back({ 0.9f, 1.0f, 1.5f, 0.0f });
    verts.push_back({ 0.1f, 2.0f, 3.0f, 0.0f });
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