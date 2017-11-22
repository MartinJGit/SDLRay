#pragma once

#include "Vector.h"
#include <vector>
#include <memory>

namespace BVH
{
#ifdef PROFILE_BVH_BUILD
    struct BuildStats
    {
        BuildStats()
        {
            m_Sorting = 0.0;
            m_MinMax = 0.0;
        }

        double m_Sorting;
        double m_MinMax;
    };
#endif

    typedef Vec3 BVHVert;

    struct BVHPoly
    {
        unsigned int  m_VertexIndices[3];

        inline BVHVert const* GetVert(u32 i, BVHVert const* verts) const
        {
            return &verts[m_VertexIndices[i]];
        }
        Vec3 m_Norm;
    };

#ifndef IN_TRACK_METADATA_COMPILER
    //static_assert(sizeof(BVHVert) == sizeof(CCollVert), "BVHVert and CCollVert should be identical in size and layout");
    //static_assert(sizeof(BVHPoly) == sizeof(CCollPoly), "BVHPoly and CCollPoly should be identical in size and layout");
#endif

    // Tightly packs the data required to store 2 BVH nodes (the left and right)
    // The node doesn't store a bounding box, instead it stores 6 plane values which along with the parent bounds can be used
    // to regenerate the node bounds. This means we always have to traverse the hierarchy from the root to determine a nodes bounds.
    struct NodePair
    {
        float x1; // 4
        float y1; // 8 
        float z1; // 12

        unsigned short leftChildOffset; // 14
        unsigned short rightChildOffset; // 16

        float x2; // 20
        float y2; // 24 
        float z2; // 28

        unsigned short polySplitIndex; // 30

        // Flags used to determine how the local x,y,z and parent x,y,z are combined to generate this nodes bounds
        unsigned char leftMinUsesParent;
        unsigned char leftMaxUsesParent;

        // There are 10 bits spare in the 2 members above

        inline bool LeftIsLeaf() const
        {
            return leftChildOffset == 0;
        }

        inline bool RightIsLeaf() const
        {
            return rightChildOffset == 0;
        }

        void CalculateMinMax(Vec3 parentMin, Vec3 parentMax, Vec3& leftChildMin, Vec3& leftChildMax, Vec3& rightChildMin, Vec3& rightChildMax) const;
    };

    static_assert(sizeof(NodePair) <= 32, "NodePair has been carefully packed to minimise its memory footprint, this shouldn't be increased without good reason");

    // Bounding Volume Hierarchy tree
    //
    // Spatial partitioning system to optimise the testing of a primitive against a large number of polygons. Efficiency and memory can be traded
    // when building the tree using the maxLeafSize. A higher leaf size will consume less memory but be slower at runtime
    class Root
    {
    private:
        struct TriCent
        {
            Vec3 m_Centroid;
            BVHPoly a;
        };

    public:
        void Build(BVHPoly const* colPolys, u32 polyCount, BVHVert const* points, u32 pointCount, u32 maxLeafSize);

#ifndef IN_TRACK_METADATA_COMPILER
        NVec FindClosest(WVec rayP, WVec rayD, WVec invRayD, WVec invSign, NVec rayLength, WVec& normal) const;
        bool RayIntersects(WVec rayP, WVec rayD, WVec invRayD, WVec invSign, NVec rayLength) const;
        //u32 GetAllInsideOf(ABB const& aabb, BVHPoly const** results, BVHVert const** verts, u32 maxPolys) const;
#endif

        //void VisitAllNodes(std::function<void(Vec3 min, Vec3 max)> func, int maxDepth) const;
        //void VisitAllLeaves(std::function<void(Vec3 a, Vec3 b, Vec3 c)> func) const;

        //void Serialize(IOSys::CBinaryStream& stream);

#ifdef PROFILE_BVH_BUILD
        BuildStats const& GetBuildStats() const { return m_BuildStats; }
#endif

        inline Vec3 Min() { return m_Min; }
        inline Vec3 Max() { return m_Max; }

        bool Deserialise(char const* buffer, int bufLen);
        void Serialise(std::unique_ptr<char>& buffer, int& bufLen);

    private:
        void CalculateBounds(std::vector<TriCent> const& tris, u32 startIndex, u32 endIndex, Vec3& min, Vec3& max);
        unsigned short BuildRec(Vec3 parentMin, Vec3 parentMax, std::vector<TriCent>& triss, u32 parentLeafStart, u32 parentLeafEnd);

        void ValidateNode(NodePair const& node, std::vector<TriCent> const& triss, Vec3 parentMin, Vec3 parentMax, u32 parentLeafStart, u32 parentLeafEnd) const;

        std::vector<BVHVert> m_Points;
        std::vector<BVHPoly> m_Polys;
        std::vector<NodePair> m_Nodes;

        Vec3 m_Min;
        Vec3 m_Max;

        u32 m_MaxLeafSize;

        static int const s_VersionNo = 1;

        friend class BVHTest;

#ifdef PROFILE_BVH_BUILD
        BuildStats m_BuildStats;
#endif
    };

    inline void NodePair::CalculateMinMax(Vec3 parentMin, Vec3 parentMax, Vec3& leftChildMin, Vec3& leftChildMax, Vec3& rightChildMin, Vec3& rightChildMax) const
    {
        Vec3 const selectMasks[] =
        {
            Vec3Make(0.0f, 0.0f, 0.0f),
            Vec3Make(1.0f, 0.0f, 0.0f),
            Vec3Make(0.0f, 1.0f, 0.0f),
            Vec3Make(1.0f, 1.0f, 0.0f),

            Vec3Make(0.0f, 0.0f, 1.0f),
            Vec3Make(1.0f, 0.0f, 1.0f),
            Vec3Make(0.0f, 1.0f, 1.0f),
            Vec3Make(1.0f, 1.0f, 1.0f)
        };

        Vec3 const invSelectMasks[] =
        {
            Vec3Make(1.0f, 1.0f, 1.0f),
            Vec3Make(0.0f, 1.0f, 1.0f),
            Vec3Make(1.0f, 0.0f, 1.0f),
            Vec3Make(0.0f, 0.0f, 1.0f),

            Vec3Make(1.0f, 1.0f, 0.0f),
            Vec3Make(0.0f, 1.0f, 0.0f),
            Vec3Make(1.0f, 0.0f, 0.0f),
            Vec3Make(0.0f, 0.0f, 0.0f)
        };

        Vec3 localMin = *(Vec3*)&x1;
        Vec3 localMax = *(Vec3*)&x2;

        leftChildMin = (invSelectMasks[leftMinUsesParent] * localMin) + (selectMasks[leftMinUsesParent] * parentMin);
        leftChildMax = (invSelectMasks[leftMaxUsesParent] * localMax) + (selectMasks[leftMaxUsesParent] * parentMax);
        rightChildMin = (selectMasks[leftMinUsesParent] * localMin) + (invSelectMasks[leftMinUsesParent] * parentMin);
        rightChildMax = (selectMasks[leftMaxUsesParent] * localMax) + (invSelectMasks[leftMaxUsesParent] * parentMax);
    }
}