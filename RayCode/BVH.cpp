#include "BVH.h"
#include <algorithm>
#include "Intersection.h"

namespace BVH
{

//Returns TRUE if the numeric range created by (a,b,c) overlaps the numeric range(min, max).
__forceinline bool DO_3VALUES_OVERLAP(f32 min, f32 max, f32 a, f32 b, f32 c)
{
    return ((a <= max || b <= max || c <= max) && (a >= min || b >= min || c >= min));
}

//This function will create an ABB around the input poly and return TRUE if it overlaps
//the input ABB.  Otherwise it returns FALSE.
__forceinline bool ABBOverlapPolyABB(ABB const *pABB, BVHPoly const *pPoly, BVHVert const * vertexArray)
{
    //Init data
    BVHVert const * cv0 = pPoly->GetVert(0, vertexArray);
    BVHVert const * cv1 = pPoly->GetVert(1, vertexArray);
    BVHVert const * cv2 = pPoly->GetVert(2, vertexArray);

    //Return TRUE if the triangle's ABB overlaps the input ABB
    return(
        DO_3VALUES_OVERLAP(pABB->min.m128_f32[0], pABB->max.m128_f32[0], cv0->m128_f32[0], cv1->m128_f32[0], cv2->m128_f32[0]) &&
        DO_3VALUES_OVERLAP(pABB->min.m128_f32[1], pABB->max.m128_f32[1], cv0->m128_f32[1], cv1->m128_f32[1], cv2->m128_f32[1]) &&
        DO_3VALUES_OVERLAP(pABB->min.m128_f32[2], pABB->max.m128_f32[2], cv0->m128_f32[2], cv1->m128_f32[2], cv2->m128_f32[2]));
}
#if 0
bool PointInsideBox(Vec3 min, Vec3 max, Vec3 point)
{
    bool lessThanMax = Vec3LE(point, max);
    bool greaterThanMin = Vec3GE(point, min);

    return lessThanMax && greaterThanMin;
}
#endif

int __vectorcall ChooseSplitDimension(Vec3 min, Vec3 max)
{
    int splitDimension = 0;
    Vec3 size = max - min;
    float largestDimensionSize = size.m128_f32[0];
    if (size.m128_f32[1] > largestDimensionSize)
    {
        splitDimension = 1;
        largestDimensionSize = size.m128_f32[1];
    }

    if (size.m128_f32[2] > largestDimensionSize)
    {
        splitDimension = 2;
        largestDimensionSize = size.m128_f32[2];
    }
    return splitDimension;
}

#if 0
//Return TRUE if the two ABB's overlap.
//Return FALSE otherwise.
inline bool ABBOverlapsABB(ABB const &A, ABB const &B)
{
    return Vec3LE(A.min, B.max) && Vec3GE(A.max, B.min);
}
#endif

inline void ReadXYZ(Vec3& dest, char const*& src)
{
    dest.m128_f32[0] = *((float*)src); src += sizeof(float);
    dest.m128_f32[1] = *((float*)src); src += sizeof(float);
    dest.m128_f32[2] = *((float*)src); src += sizeof(float);
}

inline void WriteXYZ(Vec3 const& src, char*& dest)
{
    *((float*)dest) = src.m128_f32[0]; dest += sizeof(float);
    *((float*)dest) = src.m128_f32[1]; dest += sizeof(float);
    *((float*)dest) = src.m128_f32[2]; dest += sizeof(float);
}

bool Root::Deserialise(char const* buffer, int bufLen)
{
    char const* bufferRead = buffer;
    int version = *((int*)bufferRead); bufferRead += sizeof(int);

    if (version != s_VersionNo)
    {
        return false;
    }

    m_MaxLeafSize = *((int*)bufferRead); bufferRead += sizeof(int);

    ReadXYZ(m_Min, bufferRead);
    ReadXYZ(m_Max, bufferRead);

    int pointCount = *((int*)bufferRead); bufferRead += sizeof(int);
    m_Points.resize(pointCount);
    for (int pointI = 0; pointI < pointCount; ++pointI)
    {
        ReadXYZ(m_Points[pointI], bufferRead);
    }

    int polyCount = *((int*)bufferRead); bufferRead += sizeof(int);
    m_Polys.resize(polyCount);
    for (int polyI = 0; polyI < polyCount; ++polyI)
    {
        ReadXYZ(m_Polys[polyI].m_Norm, bufferRead);

        m_Polys[polyI].m_VertexIndices[0] = *((unsigned int*)bufferRead); bufferRead += sizeof(unsigned int);
        m_Polys[polyI].m_VertexIndices[1] = *((unsigned int*)bufferRead); bufferRead += sizeof(unsigned int);
        m_Polys[polyI].m_VertexIndices[2] = *((unsigned int*)bufferRead); bufferRead += sizeof(unsigned int);
    }

    int nodeCount = *((int*)bufferRead); bufferRead += sizeof(int);
    if (nodeCount > 0)
    {
        m_Nodes.resize(nodeCount);
        memcpy(&m_Nodes[0], bufferRead, sizeof(NodePair) * nodeCount); bufferRead += sizeof(NodePair);
    }

    return true;
}

void Root::Serialise(std::unique_ptr<char>& buffer, int& bufLen)
{
    size_t memSize = 0;
    memSize += sizeof(int); // ver;
    memSize += sizeof(int); // max leaf size;
    memSize += sizeof(Vec3); // m_Min;
    memSize += sizeof(Vec3); // m_Max;
    memSize += m_Points.size() * sizeof(float) * 3 + sizeof(int);
    memSize += m_Polys.size() * (sizeof(int) * 3 + sizeof(float) * 3) + sizeof(int);
    memSize += m_Nodes.size() * sizeof(NodePair) + sizeof(int);

    bufLen = (int)memSize;

    buffer = std::unique_ptr<char>(new char[memSize]);

    int ver = 0;

    char* bufferWrite = buffer.get();
    *((int*)bufferWrite) = ver; bufferWrite += sizeof(int);
    *((unsigned int*)bufferWrite) = m_MaxLeafSize; bufferWrite += sizeof(unsigned int);

    WriteXYZ(m_Min, bufferWrite);
    WriteXYZ(m_Max, bufferWrite);

    *((unsigned int*)bufferWrite) = (unsigned int)m_Points.size(); bufferWrite += sizeof(unsigned int);
    for (size_t pointI = 0; pointI < m_Points.size(); ++pointI)
    {
        WriteXYZ(m_Points[pointI], bufferWrite);
    }

    *((unsigned int*)bufferWrite) = (unsigned int)m_Polys.size(); bufferWrite += sizeof(unsigned int);
    for (size_t polyI = 0; polyI < m_Polys.size(); ++polyI)
    {
        WriteXYZ(m_Polys[polyI].m_Norm, bufferWrite);

        *((unsigned int*)bufferWrite) = m_Polys[polyI].m_VertexIndices[0]; bufferWrite += sizeof(unsigned int);
        *((unsigned int*)bufferWrite) = m_Polys[polyI].m_VertexIndices[1]; bufferWrite += sizeof(unsigned int);
        *((unsigned int*)bufferWrite) = m_Polys[polyI].m_VertexIndices[2]; bufferWrite += sizeof(unsigned int);
    }

    *((unsigned int*)bufferWrite) = (unsigned int)m_Nodes.size(); bufferWrite += sizeof(unsigned int);
    size_t nodeWriteSize = sizeof(NodePair) * m_Nodes.size();
    if (nodeWriteSize > 0)
    {
        memcpy(bufferWrite, &m_Nodes[0], nodeWriteSize);
        bufferWrite += nodeWriteSize;
    }
}

void Root::CalculateBounds(std::vector<TriCent> const& tris, u32 startIndex, u32 endIndex, Vec3& min, Vec3& max)
{
#ifdef PROFILE_BVH_BUILD
    double minMaxStart = PerformanceClock::GetSeconds();
#endif

    for (auto itTri = tris.begin() + startIndex, itEnd = tris.begin() + endIndex; itTri != itEnd; ++itTri)
    {
        Vec3 pointA = m_Points[itTri->a.m_VertexIndices[0]];
        Vec3 pointB = m_Points[itTri->a.m_VertexIndices[1]];
        Vec3 pointC = m_Points[itTri->a.m_VertexIndices[2]];

        min = Vec3Min(min, Vec3Min(Vec3Min(pointA, pointB), pointC));
        max = Vec3Max(max, Vec3Max(Vec3Max(pointA, pointB), pointC));
    }

#ifdef PROFILE_BVH_BUILD
    double minMaxEnd = PerformanceClock::GetSeconds();
    m_BuildStats.m_Sorting += minMaxEnd - minMaxStart;
#endif
}

unsigned short Root::BuildRec(Vec3 parentMin, Vec3 parentMax, std::vector<TriCent>& tris, u32 parentLeafStart, u32 parentLeafEnd)
{
    // Choose our split dimension
    int splitDimension = ChooseSplitDimension(parentMin, parentMax);

    auto itTriBegin = tris.begin() + parentLeafStart;
    auto itTriEnd = tris.begin() + parentLeafEnd;

#ifdef PROFILE_BVH_BUILD
    double sortStart = PerformanceClock::GetSeconds();
#endif

    // Sort based on split dimension
    struct Cmp
    {
        int dimension;

        bool operator()(TriCent const& lhs, TriCent const& rhs) const
        {
            return lhs.m_Centroid.m128_f32[dimension] < rhs.m_Centroid.m128_f32[dimension];
        }
    };
    Cmp cmp;
    cmp.dimension = splitDimension;

    std::sort(itTriBegin, itTriEnd, cmp);

#ifdef PROFILE_BVH_BUILD
    double sortEnd = PerformanceClock::GetSeconds();
    m_BuildStats.m_Sorting += sortEnd - sortStart;
#endif

    // Choose new centre
    TriCent lowBound;
    lowBound.m_Centroid.m128_f32[splitDimension] = (parentMax.m128_f32[splitDimension] + parentMin.m128_f32[splitDimension]) * 0.5f;

    // Find out which node lies after the new centre
    auto splitIt = std::lower_bound(itTriBegin, itTriEnd, lowBound, [splitDimension](TriCent const& centroidA, TriCent const& centroidB)
    {
        return centroidA.m_Centroid.m128_f32[splitDimension] < centroidB.m_Centroid.m128_f32[splitDimension];
    });

    u32 splitIndex = static_cast<u32>(splitIt - tris.begin());

    if (splitIndex == parentLeafStart || splitIndex == parentLeafEnd)
    {
        splitIndex = (parentLeafEnd + parentLeafStart) / 2;
    }

    //ASSERTMSG(splitIndex > parentLeafStart && splitIndex < parentLeafEnd, "Invalid split index");

    int childOffset = static_cast<int>(m_Nodes.size());
    //ASSERTMSG(childOffset < (1 << 16), "child offset out of range");
    m_Nodes.push_back(NodePair());
    NodePair& childNode = m_Nodes.back();

    {
        u32 leftNodeLeafStart = parentLeafStart;
        u32 leftNodeLeafEnd = splitIndex;

        u32 rightNodeLeafStart = splitIndex;
        u32 rightNodeLeafEnd = parentLeafEnd;

        // Expand the bounding box around all child nodes
        Vec3 leftNodeMax = Vec3Make(-FLT_MAX);
        Vec3 leftNodeMin = Vec3Make(FLT_MAX);
        CalculateBounds(tris, leftNodeLeafStart, leftNodeLeafEnd, leftNodeMin, leftNodeMax);
        Vec3 rightNodeMax = Vec3Make(-FLT_MAX);
        Vec3 rightNodeMin = Vec3Make(FLT_MAX);
        CalculateBounds(tris, rightNodeLeafStart, rightNodeLeafEnd, rightNodeMin, rightNodeMax);

        // Calculate our node flags
        childNode.leftChildOffset = 0;
        childNode.rightChildOffset = 0;

        int leftXMinUsesParent = leftNodeMin.m128_f32[0] == parentMin.m128_f32[0] ? 1 : 0;
        int leftXMaxUsesParent = leftNodeMax.m128_f32[0] == parentMax.m128_f32[0] ? 1 : 0;
        int leftYMinUsesParent = leftNodeMin.m128_f32[1] == parentMin.m128_f32[1] ? 1 : 0;
        int leftYMaxUsesParent = leftNodeMax.m128_f32[1] == parentMax.m128_f32[1] ? 1 : 0;
        int leftZMinUsesParent = leftNodeMin.m128_f32[2] == parentMin.m128_f32[2] ? 1 : 0;
        int leftZMaxUsesParent = leftNodeMax.m128_f32[2] == parentMax.m128_f32[2] ? 1 : 0;

        childNode.leftMinUsesParent = leftXMinUsesParent | (leftYMinUsesParent << 1) | (leftZMinUsesParent << 2);
        childNode.leftMaxUsesParent = leftXMaxUsesParent | (leftYMaxUsesParent << 1) | (leftZMaxUsesParent << 2);

        childNode.x1 = leftXMinUsesParent ? rightNodeMin.m128_f32[0] : leftNodeMin.m128_f32[0];
        childNode.x2 = leftXMaxUsesParent ? rightNodeMax.m128_f32[0] : leftNodeMax.m128_f32[0];
        childNode.y1 = leftYMinUsesParent ? rightNodeMin.m128_f32[1] : leftNodeMin.m128_f32[1];
        childNode.y2 = leftYMaxUsesParent ? rightNodeMax.m128_f32[1] : leftNodeMax.m128_f32[1];
        childNode.z1 = leftZMinUsesParent ? rightNodeMin.m128_f32[2] : leftNodeMin.m128_f32[2];
        childNode.z2 = leftZMaxUsesParent ? rightNodeMax.m128_f32[2] : leftNodeMax.m128_f32[2];

#ifdef ENABLE_ASSERTS
        {
            // Verify that the nodes bounding box is accurate
            Vec3 calculatedLeftChildMin, calculatedLeftChildMax, calculatedRightChildMin, calculatedRightChildMax;
            childNode.CalculateMinMax(parentMin, parentMax, calculatedLeftChildMin, calculatedLeftChildMax, calculatedRightChildMin, calculatedRightChildMax);
            ASSERTMSG(calculatedLeftChildMin.x == leftNodeMin.x && calculatedLeftChildMin.y == leftNodeMin.y && calculatedLeftChildMin.z == leftNodeMin.z, "");
            ASSERTMSG(calculatedLeftChildMax.x == leftNodeMax.x && calculatedLeftChildMax.y == leftNodeMax.y && calculatedLeftChildMax.z == leftNodeMax.z, "");
            ASSERTMSG(calculatedRightChildMin.x == rightNodeMin.x && calculatedRightChildMin.y == rightNodeMin.y && calculatedRightChildMin.z == rightNodeMin.z, "");
            ASSERTMSG(calculatedRightChildMax.x == rightNodeMax.x && calculatedRightChildMax.y == rightNodeMax.y && calculatedRightChildMax.z == rightNodeMax.z, "");
        }
#endif

        childNode.polySplitIndex = splitIndex;

        // Build our left descendants
        if ((leftNodeLeafEnd - leftNodeLeafStart) > m_MaxLeafSize)
        {
            childNode.leftChildOffset = BuildRec(leftNodeMin, leftNodeMax, tris, leftNodeLeafStart, leftNodeLeafEnd);
        }

        // Build our right descendants
        if ((rightNodeLeafEnd - rightNodeLeafStart) > m_MaxLeafSize)
        {
            childNode.rightChildOffset = BuildRec(rightNodeMin, rightNodeMax, tris, rightNodeLeafStart, rightNodeLeafEnd);
        }

#ifdef ENABLE_ASSERTS
        if (childNode.LeftIsLeaf() || childNode.RightIsLeaf())
        {
            ValidateNode(childNode, tris, parentMin, parentMax, parentLeafStart, parentLeafEnd);
        }
#endif
    }

    return childOffset;
}

void Root::ValidateNode(NodePair const& node, std::vector<TriCent> const& tris, Vec3 parentMin, Vec3 parentMax, u32 parentPolyStart, u32 parentPolyEnd) const
{
#ifdef ENABLE_ASSERTS
    Vec3 leftBoundsMin, leftBoundsMax, rightBoundsMin, rightBoundsMax;
    node.CalculateMinMax(parentMin, parentMax, leftBoundsMin, leftBoundsMax, rightBoundsMin, rightBoundsMax);

    if (node.LeftIsLeaf())
    {
        for (u32 triI = parentPolyStart; triI < node.polySplitIndex; ++triI)
        {
            TriCent const& tri = tris[triI];
            u32 indexA = tri.a.m_VertexIndices[0];
            Vec3 pointA = m_Points[indexA];
            Vec3 pointB = m_Points[tri.a.m_VertexIndices[1]];
            Vec3 pointC = m_Points[tri.a.m_VertexIndices[2]];

            ASSERTMSG(PointInsideBox(leftBoundsMin, leftBoundsMax, pointA), "tri pointA outside of parent");
            ASSERTMSG(PointInsideBox(leftBoundsMin, leftBoundsMax, pointB), "tri pointB outside of parent");
            ASSERTMSG(PointInsideBox(leftBoundsMin, leftBoundsMax, pointC), "tri pointC outside of parent");
        }
    }

    if (node.RightIsLeaf())
    {
        for (u32 triI = node.polySplitIndex; triI < parentPolyEnd; ++triI)
        {
            TriCent const& tri = tris[triI];
            u32 indexA = tri.a.m_VertexIndices[0];
            Vec3 pointA = m_Points[indexA];
            Vec3 pointB = m_Points[tri.a.m_VertexIndices[1]];
            Vec3 pointC = m_Points[tri.a.m_VertexIndices[2]];

            ASSERTMSG(PointInsideBox(rightBoundsMin, rightBoundsMax, pointA), "tri pointA outside of parent");
            ASSERTMSG(PointInsideBox(rightBoundsMin, rightBoundsMax, pointB), "tri pointB outside of parent");
            ASSERTMSG(PointInsideBox(rightBoundsMin, rightBoundsMax, pointC), "tri pointC outside of parent");
        }
    }
#endif
}

void Root::Build(BVHPoly const* colPolys, u32 polyCount, BVHVert const* points, u32 pointCount, u32 maxLeafSize)
{
    m_Nodes.clear();
    m_Polys.clear();

    m_MaxLeafSize = maxLeafSize;

#ifdef PROFILE_BVH_BUILD
    m_BuildStats.m_MinMax = 0.0;
    m_BuildStats.m_Sorting = 0.0;
#endif

    m_Points.resize(pointCount);
    memcpy_s(&m_Points[0], sizeof(BVHVert) * m_Points.size(), points, sizeof(BVHVert) * pointCount);

    m_Nodes.reserve(polyCount * 2);

    Vec3 min = { FLT_MAX, FLT_MAX, FLT_MAX };
    Vec3 max = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

    // Build up our TriCent which contain a poly and the centre of that triangle which is later used to sort polygons
    std::vector<TriCent> centroids;
    centroids.resize(polyCount);
    for (size_t i = 0; i < polyCount; ++i)
    {
        Vec3 pointA = m_Points[colPolys[i].m_VertexIndices[0]];
        Vec3 pointB = m_Points[colPolys[i].m_VertexIndices[1]];
        Vec3 pointC = m_Points[colPolys[i].m_VertexIndices[2]];
        Vec3 triMin = Vec3Min(Vec3Min(pointA, pointB), pointC);
        Vec3 triMax = Vec3Max(Vec3Max(pointA, pointB), pointC);
        centroids[i].m_Centroid = (triMax + triMin) * Vec3Make(0.5f, 0.5f, 0.5f);
        centroids[i].a = colPolys[i];

        min = Vec3Min(triMin, min);
        max = Vec3Max(triMax, max);
    }

    m_Min = min;
    m_Max = max;

    if (polyCount > 1)
    {
        BuildRec(min, max, centroids, 0, (u32)centroids.size());
    }

    m_Polys.reserve(centroids.size());
    for (auto tri : centroids)
    {
        m_Polys.push_back(tri.a);
    }

    // Try to minimise the memory usage of the node container
    std::vector<NodePair> nodes = m_Nodes;
    m_Nodes.swap(nodes);
}

struct NodeTraversal
{
    union
    {
        Vec3 min;

        struct
        {
            float unused[3];
            unsigned int leafStart;
        };
    };
    
    union
    {
        Vec3 max;

        struct
        {
            float unused[3];
            unsigned short leafSize;
            unsigned short nodeIndex;
        };
    };
};

inline void AddNodeToStack(NodeTraversal* traversalStack, int stackSize, int& nodeI, Vec3 min, Vec3 max, int leafStart, int leafEnd, int nodeIndex)
{
    NodeTraversal& newChild = traversalStack[++nodeI];
    //FATALASSERTMSG(nodeI < stackSize, "BVH::GetAllInsideOf stack size insufficient");
    newChild.min = min;
    newChild.max = max;
    newChild.leafStart = leafStart;
    newChild.leafSize = leafEnd - leafStart;
    newChild.nodeIndex = nodeIndex;
}

#ifdef _DEBUG
int gDBreak = 0;
#endif

void FindClosestPolyInNode(int polyStart, int polyEnd, WVec rayP, WVec rayD, NVec rayLength, NVec& closestPolyDist, WVec& normal, BVHVert const* verts, BVHPoly const* polys)
{
    for (int polyI = polyStart; polyI < polyEnd; ++polyI)
    {
        BVHPoly const& poly = polys[polyI];
        NVec dist = rayLength;
        RayTriange4(rayP, rayD, verts[poly.m_VertexIndices[0]], verts[poly.m_VertexIndices[1]], verts[poly.m_VertexIndices[2]], dist);

        NVec pointCloser = NVecLT(dist, closestPolyDist);
        closestPolyDist = NVecMin(dist, closestPolyDist);
        normal = WVecSelect(WVecMake2(poly.m_Norm.m128_f32[0], poly.m_Norm.m128_f32[1], poly.m_Norm.m128_f32[2]), normal, pointCloser);
#ifdef _DEBUG
        if (NVecMoveMask(NVecLT(closestPolyDist, rayLength)) > 0)
        {
            gDBreak = 5;
        }
#endif
    };
}

void IntersectsPolyInNode(int polyStart, int polyEnd, WVec rayP, WVec rayD, NVec rayLength, NVec& closestPolyDist, BVHVert const* verts, BVHPoly const* polys)
{
    for (int polyI = polyStart; polyI < polyEnd; ++polyI)
    {
        BVHPoly const& poly = polys[polyI];
        NVec dist = rayLength;
        RayTriange4(rayP, rayD, verts[poly.m_VertexIndices[0]], verts[poly.m_VertexIndices[1]], verts[poly.m_VertexIndices[2]], dist);

        bool allPointsCloser = NVecMoveMask(NVecLT(dist, rayLength)) == 7;
        if (allPointsCloser)
        {
            closestPolyDist = NVecMin(dist, closestPolyDist);
            break;
        }
    };
}

//template <bool ReturnOnIntersect>
NVec FindClosestPoly(Vec3 min, Vec3 max, NodePair const* nodes, BVHVert const* verts, BVHPoly const* polys, int polyCount, WVec rayP, WVec rayD, WVec invRayD, WVec raySign, NVec rayLength, WVec& normal)
{
    NVec closestPolyDist = NVecMake(FLT_MAX);
    if (nodes == nullptr)
    {
        FindClosestPolyInNode(0, polyCount, rayP, rayD, rayLength, closestPolyDist, normal, verts, polys);
        return closestPolyDist;
    }

    int const traversalStackSize = 60; // This allows us to traverse a tree that has a maximum depth of 29, (traversalStackSize / 2) - 1.
                                  // A balanced BVH with a depth of 29 would have (2 ^ 29) - 1 nodes (536,870,912). We don't expect to have a BVH with this number of nodes. If we ever do,
                                  // the fatal asserts below will catch this case and the node stack size should be increased

    int nodeI = 0;
    NodeTraversal traversalStack[traversalStackSize];
    traversalStack[0].min = min;
    traversalStack[0].max = max;
    traversalStack[0].leafStart = 0;
    traversalStack[0].leafSize = polyCount;
    traversalStack[0].nodeIndex = 0;

    while (nodeI >= 0)
    {
        // Pop our current node from the stack
        NodeTraversal const currentNode = traversalStack[nodeI--];
        NodePair const& node = nodes[currentNode.nodeIndex];

        AABB leftBounds, rightBounds;
        node.CalculateMinMax(currentNode.min, currentNode.max, leftBounds.m_Min, leftBounds.m_Max, rightBounds.m_Min, rightBounds.m_Max);

        NVec distToAABB = NVecMake(FLT_MAX);
        RayAABB(rayP, invRayD, leftBounds, distToAABB);
        NVec intersect = NVecLT(distToAABB, rayLength);
        if (NVecMoveMask(intersect) > 0)
        {
            // If a leaf, check all polygons against the bounding box and output them if they intersect
            if (node.LeftIsLeaf())
            {
                FindClosestPolyInNode(currentNode.leafStart, node.polySplitIndex, rayP, rayD, rayLength, closestPolyDist, normal, verts, polys);
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, leftBounds.m_Min, leftBounds.m_Max, currentNode.leafStart, node.polySplitIndex, node.leftChildOffset);
            }
        }

        distToAABB = NVecMake(FLT_MAX);
        RayAABB(rayP, invRayD, rightBounds, distToAABB);
        intersect = NVecLT(distToAABB, rayLength);
        if (NVecMoveMask(intersect) > 0)
        {
            if (node.RightIsLeaf())
            {
                FindClosestPolyInNode(node.polySplitIndex, currentNode.leafStart + currentNode.leafSize, rayP, rayD, rayLength, closestPolyDist, normal, verts, polys);
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, rightBounds.m_Min, rightBounds.m_Max, node.polySplitIndex, (currentNode.leafStart + currentNode.leafSize), node.rightChildOffset);
            }
        }
    }

    return closestPolyDist;
}
#if 0
NVec FindClosestPoly(Vec3 min, Vec3 max, NodePair const* nodes, BVHVert const* verts, BVHPoly const* polys, int polyCount, WVec rayP, WVec rayD, WVec invRayD, WVec raySign, NVec rayLength, WVec& normal)
{
    NVec closestPolyDist = NVecMake(FLT_MAX);
    if (nodes == nullptr)
    {
        FindClosestPolyInNode(0, polyCount, rayP, rayD, rayLength, closestPolyDist, normal, verts, polys);
        return closestPolyDist;
    }

    int const traversalStackSize = 60; // This allows us to traverse a tree that has a maximum depth of 29, (traversalStackSize / 2) - 1.
                                       // A balanced BVH with a depth of 29 would have (2 ^ 29) - 1 nodes (536,870,912). We don't expect to have a BVH with this number of nodes. If we ever do,
                                       // the fatal asserts below will catch this case and the node stack size should be increased

    int nodeI = 0;
    NodeTraversal traversalStack[traversalStackSize];
    traversalStack[0].min = min;
    traversalStack[0].max = max;
    traversalStack[0].leafStart = 0;
    traversalStack[0].leafSize = polyCount;
    traversalStack[0].nodeIndex = 0;

    while (nodeI >= 0)
    {
        // Pop our current node from the stack
        NodeTraversal const currentNode = traversalStack[nodeI--];
        NodePair const& node = nodes[currentNode.nodeIndex];

        AABB leftBounds, rightBounds;
        node.CalculateMinMax(currentNode.min, currentNode.max, leftBounds.m_Min, leftBounds.m_Max, rightBounds.m_Min, rightBounds.m_Max);

        NVec distToAABB = NVecMake(FLT_MAX);
        RayAABB(rayP, invRayD, raySign, leftBounds, distToAABB);
        NVec intersect = NVecLT(distToAABB, rayLength);
        if (NVecMoveMask(intersect) > 0)
        {
            // If a leaf, check all polygons against the bounding box and output them if they intersect
            if (node.LeftIsLeaf())
            {
                FindClosestPolyInNode<ReturnOnIntersect>(currentNode.leafStart, node.polySplitIndex, rayP, rayD, rayLength, closestPolyDist, verts, polys);
                if (ReturnOnIntersect)
                {
                    NVec allCloserThanRayLength = NVecLT(closestPolyDist, rayLength);
                    if (NVecMoveMask(allCloserThanRayLength) & 0x7)
                    {
                        return closestPolyDist;
                    }
                }
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, leftBounds.m_Min, leftBounds.m_Max, currentNode.leafStart, node.polySplitIndex, node.leftChildOffset);
            }
        }

        distToAABB = NVecMake(FLT_MAX);
        RayAABB(rayP, invRayD, raySign, rightBounds, distToAABB);
        intersect = NVecLT(distToAABB, rayLength);
        if (NVecMoveMask(intersect) > 0)
        {
            if (node.RightIsLeaf())
            {
                FindClosestPolyInNode<ReturnOnIntersect>(node.polySplitIndex, currentNode.leafStart + currentNode.leafSize, rayP, rayD, rayLength, closestPolyDist, verts, polys);
                if (ReturnOnIntersect)
                {
                    NVec allCloserThanRayLength = NVecLT(closestPolyDist, rayLength);
                    if (NVecMoveMask(allCloserThanRayLength) & 0x7)
                    {
                        return closestPolyDist;
                    }
                }
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, rightBounds.m_Min, rightBounds.m_Max, node.polySplitIndex, (currentNode.leafStart + currentNode.leafSize), node.rightChildOffset);
            }
}
    }

    return closestPolyDist;
}
#endif

NVec Root::FindClosest(WVec rayP, WVec rayD, WVec invRayD, WVec raySign, NVec rayLength, WVec& normal) const
{
    return FindClosestPoly(m_Min, m_Max, &m_Nodes[0], &m_Points[0], &m_Polys[0], (int)m_Polys.size(), rayP, rayD, invRayD, raySign, rayLength, normal);
#if 0
    NVec closestPolyDist = NVecMake(FLT_MAX);

    if (m_Nodes.empty())
    {
        FindClosestPolyInNode<ReturnOnIntersect>(0, polyCount, rayP, rayD, closestPolyDist, closestPoly, verts, polys);
        return closestPolyDist;
    }

    int const traversalStackSize = 60; // This allows us to traverse a tree that has a maximum depth of 29, (traversalStackSize / 2) - 1.
                                       // A balanced BVH with a depth of 29 would have (2 ^ 29) - 1 nodes (536,870,912). We don't expect to have a BVH with this number of nodes. If we ever do,
                                       // the fatal asserts below will catch this case and the node stack size should be increased

    int nodeI = 0;
    NodeTraversal traversalStack[traversalStackSize];
    traversalStack[0].min = min;
    traversalStack[0].max = max;
    traversalStack[0].leafStart = 0;
    traversalStack[0].leafSize = polyCount;
    traversalStack[0].nodeIndex = 0;

    Vec3 intersect;
    vecMask intersectResult;
    float closestPolyDist = rayLength;

    while (nodeI >= 0)
    {
        // Pop our current node from the stack
        NodeTraversal const currentNode = traversalStack[nodeI--];
        NodePair const& node = nodes[currentNode.nodeIndex];

        AABB leftBounds, rightBounds;
        node.CalculateMinMax(currentNode.min, currentNode.max, leftBounds.min, leftBounds.max, rightBounds.min, rightBounds.max);

        float distToAABB = FLT_MAX;
        RayAABB(rayP, rayD, leftBounds, leftBounds, distToAABB);
        if (XMVectorGetIntX(intersectResult) > 0 && intersect.m128_f32[0] <= rayLength)
        {
            // If a leaf, check all polygons against the bounding box and output them if they intersect
            if (node.LeftIsLeaf())
            {
                FindClosestPolyInNode<ReturnOnIntersect>(currentNode.leafStart, node.polySplitIndex, rayP, rayD, closestPolyDist, closestPoly, verts, polys);
                if (ReturnOnIntersect && closestPolyDist < rayLength)
                {
                    return closestPolyDist;
                }
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, leftBounds.min, leftBounds.max, currentNode.leafStart, node.polySplitIndex, node.leftChildOffset);
            }
    }

        intersectResult = IntersectRayAABB(rayP, rayD, rightBounds.min, rightBounds.max, intersect);
        if (XMVectorGetIntX(intersectResult) > 0 && intersect.m128_f32[0] <= rayLength)
        {
            if (node.RightIsLeaf())
            {
                FindClosestPolyInNode<ReturnOnIntersect>(node.polySplitIndex, currentNode.leafStart + currentNode.leafSize, rayP, rayD, closestPolyDist, closestPoly, verts, polys);
                if (ReturnOnIntersect && closestPolyDist < rayLength)
                {
                    return closestPolyDist;
                }
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, rightBounds.min, rightBounds.max, node.polySplitIndex, (currentNode.leafStart + currentNode.leafSize), node.rightChildOffset);
            }
        }
}

    return closestPolyDist;
#endif
}

#if 0

float Root::FindClosest(Vec3 rayP, Vec3 rayD, float rayLength, BVHPoly& closestPoly) const
{
    return FindClosestPoly<false>(m_Min, m_Max, m_Nodes.empty() ? nullptr : &m_Nodes[0], (BVHVert*)&m_Points[0], (BVHPoly*)&m_Polys[0], (int)m_Polys.size(), rayP, rayD, rayLength, closestPoly);
}

bool Root::RayIntersects(Vec3 rayP, Vec3 rayD, float rayLength) const
{
    BVHPoly closestPoly;
    float cloestPolyDist = FindClosestPoly<true>(m_Min, m_Max, m_Nodes.empty() ? nullptr : &m_Nodes[0], (BVHVert*)&m_Points[0], (BVHPoly*)&m_Polys[0], (int)m_Polys.size(), rayP, rayD, rayLength, closestPoly);
    return cloestPolyDist < rayLength;
}

inline u32 AddIntersectingPolys(int polyStart, int polyEnd, ABB abb, BVHPoly const* polys, BVHVert const* colPoints, BVHPoly const** results, BVHVert const** verts, u32 maxPolys)
{
    u32 polyCount = 0;

    for (int polyI = polyStart; polyI < polyEnd; ++polyI)
    {
        BVHPoly const& poly = polys[polyI];
        if (ABBOverlapPolyABB(&abb, &poly, colPoints))
        {
            results[polyCount] = &poly;
            verts[polyCount++] = colPoints;

            if (polyCount >= maxPolys)
            {
                break;
            }
        }
    }

    return polyCount;
}
#endif
#if 0
u32 Root::GetAllInsideOf(ABB const& aabb, CollPolyFlags::PolyMask colFlags, BVHPoly const** results, BVHVert const** verts, u32 maxPolys) const
{
    BVHVert const* colPoints = (BVHVert* const)&m_Points[0];
    BVHPoly const* colPolys = (BVHPoly* const)&m_Polys[0];

    u32 polyCount = 0;

    if (m_Nodes.empty())
    {
        for (int polyI = 0; polyI < (int)m_Polys.size(); ++polyI)
        {
            BVHPoly const& poly = colPolys[polyI];
            if ((poly.m_Flags & colFlags) && ABBOverlapPolyABB(&aabb, &poly, colPoints))
            {
                results[polyCount] = &poly;
                verts[polyCount++] = colPoints;

                if (polyCount >= maxPolys)
                {
                    return polyCount;
                }
            }
        }
        return polyCount;
    }

    int const traversalStackSize = 60; // This allows us to traverse a tree that has a maximum depth of 29, (traversalStackSize / 2) - 1.
                                  // A balanced BVH with a depth of 29 would have (2 ^ 29) - 1 nodes (536,870,912). We don't expect to have a BVH with this number of nodes. If we ever do,
                                  // the fatal asserts below will catch this case and the node stack size should be increased

    int nodeI = 0;
    NodeTraversal traversalStack[traversalStackSize];
    traversalStack[0].min = m_Min;
    traversalStack[0].max = m_Max;
    traversalStack[0].leafStart = 0;
    traversalStack[0].leafSize = (int)m_Polys.size();
    traversalStack[0].nodeIndex = 0;

    while (nodeI >= 0)
    {
        // Pop our current node from the stack
        NodeTraversal const currentNode = traversalStack[nodeI--];
        NodePair const& node = m_Nodes[currentNode.nodeIndex];

        ABB leftBounds, rightBounds;
        node.CalculateMinMax(currentNode.min, currentNode.max, leftBounds.min, leftBounds.max, rightBounds.min, rightBounds.max);

        if (ABBOverlapsABB(aabb, leftBounds))
        {
            // If a leaf, check all polygons against the bounding box and output them if they intersect
            if (node.LeftIsLeaf())
            {
                int leafEnd = node.polySplitIndex;
                for (int polyI = currentNode.leafStart; polyI < leafEnd; ++polyI)
                {
                    BVHPoly const& poly = colPolys[polyI];
                    if ((poly.m_Flags & colFlags) && ABBOverlapPolyABB(&aabb, &poly, colPoints))
                    {
                        results[polyCount] = &poly;
                        verts[polyCount++] = colPoints;

                        if (polyCount >= maxPolys)
                        {
                            return polyCount;
                        }
                    }
                }
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, leftBounds.min, leftBounds.max, currentNode.leafStart, node.polySplitIndex, node.leftChildOffset);
            }
        }

        if (ABBOverlapsABB(aabb, rightBounds))
        {
            if (node.RightIsLeaf())
            {
                int leafEnd = currentNode.leafStart + currentNode.leafSize;
                for (int polyI = node.polySplitIndex; polyI < leafEnd; ++polyI)
                {
                    BVHPoly const& poly = colPolys[polyI];
                    if ((poly.m_Flags & colFlags) && ABBOverlapPolyABB(&aabb, &poly, colPoints))
                    {
                        results[polyCount] = &poly;
                        verts[polyCount++] = colPoints;

                        if (polyCount >= maxPolys)
                        {
                            return polyCount;
                        }
                    }
                }
            }
            else
            {
                AddNodeToStack(traversalStack, traversalStackSize, nodeI, rightBounds.min, rightBounds.max, node.polySplitIndex, currentNode.leafStart + currentNode.leafSize, node.rightChildOffset);
            }
        }
    }

    return polyCount;
}
#endif
#if 0
void Root::VisitAllNodes(std::function<void(Vec3 min, Vec3 max)> func, int maxDepth) const
{
    int const traversalStackSize = 60; // This allows us to traverse a tree that has a maximum depth of 29, (traversalStackSize / 2) - 1.
                                  // A balanced BVH with a depth of 29 would have (2 ^ 29) - 1 nodes (536,870,912). We don't expect to have a BVH with this number of nodes. If we ever do,
                                  // the fatal asserts below will catch this case and the node stack size should be increased
    int nodeI = 0;
    NodeTraversal traversalStack[traversalStackSize];
    traversalStack[0].min = m_Min;
    traversalStack[0].max = m_Max;
    traversalStack[0].leafStart = 0;
    traversalStack[0].leafSize = (int)m_Polys.size();
    traversalStack[0].nodeIndex = 0;

    while (nodeI >= 0)
    {
        // Pop our current node from the stack
        NodeTraversal const currentNode = traversalStack[nodeI--];
        NodePair const& node = m_Nodes[currentNode.nodeIndex];

        Vec3 leftChildMin, leftChildMax, rightChildMin, rightChildMax;
        node.CalculateMinMax(currentNode.min, currentNode.max, leftChildMin, leftChildMax, rightChildMin, rightChildMax);

        func(currentNode.min, currentNode.max);

        if (node.LeftIsLeaf())
        {
            func(leftChildMin, leftChildMax);
        }
        else
        {
            AddNodeToStack(traversalStack, traversalStackSize, nodeI, leftChildMin, leftChildMax, currentNode.leafStart, node.polySplitIndex, node.leftChildOffset);
        }

        if (node.RightIsLeaf())
        {
            func(rightChildMin, rightChildMax);
        }
        else
        {
            AddNodeToStack(traversalStack, traversalStackSize, nodeI, rightChildMin, rightChildMax, node.polySplitIndex, currentNode.leafStart + currentNode.leafSize, node.rightChildOffset);
        }
    }
}

void Root::VisitAllLeaves(std::function<void(Vec3 a, Vec3 b, Vec3 c)> func) const
{
    int const traversalStackSize = 60; // This allows us to traverse a tree that has a maximum depth of 29, (traversalStackSize / 2) - 1.
                                  // A balanced BVH with a depth of 29 would have (2 ^ 29) - 1 nodes (536,870,912). We don't expect to have a BVH with this number of nodes. If we ever do,
                                  // the fatal asserts below will catch this case and the node stack size should be increased
    int nodeI = 0;
    NodeTraversal traversalStack[traversalStackSize];
    traversalStack[0].min = m_Min;
    traversalStack[0].max = m_Max;
    traversalStack[0].leafStart = 0;
    traversalStack[0].leafSize = (int)m_Polys.size();
    traversalStack[0].nodeIndex = 0;

    while (nodeI >= 0)
    {
        // Pop our current node from the stack
        NodeTraversal const currentNode = traversalStack[nodeI--];
        NodePair const& node = m_Nodes[currentNode.nodeIndex];

        Vec3 leftChildMin, leftChildMax, rightChildMin, rightChildMax;
        node.CalculateMinMax(currentNode.min, currentNode.max, leftChildMin, leftChildMax, rightChildMin, rightChildMax);

        if (node.LeftIsLeaf())
        {
            u32 leafEnd = node.polySplitIndex;
            for (u32 i = currentNode.leafStart; i < leafEnd; ++i)
            {
                BVHPoly const& poly = m_Polys[i];
                Vec3 pointA = m_Points[poly.m_VertexIndices[0]];
                Vec3 pointB = m_Points[poly.m_VertexIndices[1]];
                Vec3 pointC = m_Points[poly.m_VertexIndices[2]];
                func(pointA, pointB, pointC);
            }
        }
        else
        {
            AddNodeToStack(traversalStack, traversalStackSize, nodeI, leftChildMin, leftChildMax, currentNode.leafStart, node.polySplitIndex, node.leftChildOffset);
        }

        if (node.RightIsLeaf())
        {
            u32 leafEnd = currentNode.leafStart + currentNode.leafSize;
            for (u32 i = node.polySplitIndex; i < leafEnd; ++i)
            {
                BVHPoly const& poly = m_Polys[i];
                Vec3 pointA = m_Points[poly.m_VertexIndices[0]];
                Vec3 pointB = m_Points[poly.m_VertexIndices[1]];
                Vec3 pointC = m_Points[poly.m_VertexIndices[2]];
                func(pointA, pointB, pointC);
            }
        }
        else
        {
            AddNodeToStack(traversalStack, traversalStackSize, nodeI, rightChildMin, rightChildMax, node.polySplitIndex, currentNode.leafStart + currentNode.leafSize, node.rightChildOffset);
        }
    }
}

void Root::Serialize(IOSys::CBinaryStream& stream)
{
    {
        // Serialise points
        u32 pointCount = (u32)m_Points.size();
        stream.Serialize(pointCount);

        if (stream.IsRead())
        {
            m_Points.resize(pointCount);
        }

        stream.Serialize(&m_Points[0], (s32)(sizeof(BVHVert) * m_Points.size()));
    }

    {
        // Serialise polys
        u32 polyCount = (u32)m_Polys.size();
        stream.Serialize(polyCount);

        if (stream.IsRead())
        {
            m_Polys.resize(polyCount);
        }

        for (u32 polyI = 0; polyI < polyCount; ++polyI)
        {
            BVHPoly& poly = m_Polys[polyI];
            stream.Serialize(poly.m_Flags);
            stream.Serialize(poly.m_SurfaceId);
            stream.Serialize(poly.m_VertexIndices[0]);
            stream.Serialize(poly.m_VertexIndices[1]);
            stream.Serialize(poly.m_VertexIndices[2]);

            vec4 normal = Vec4Make(poly.m_Norm.x, poly.m_Norm.y, poly.m_Norm.z, poly.m_Norm.w);
            stream.Serialize(normal);
            if (stream.IsRead())
            {
                poly.m_Norm.x = normal.x;
                poly.m_Norm.y = normal.y;
                poly.m_Norm.z = normal.z;
                poly.m_Norm.w = normal.w;
            }
        }
    }

    {
        // Serialise points
        u32 nodeCount = (u32)m_Nodes.size();
        stream.Serialize(nodeCount);

        if (stream.IsRead())
        {
            m_Nodes.resize(nodeCount);
        }

        stream.Serialize(&m_Nodes[0], (s32)(sizeof(NodePair) * m_Nodes.size()));
    }

    stream.Serialize(m_Min.x);
    stream.Serialize(m_Min.y);
    stream.Serialize(m_Min.z);
    stream.Serialize(m_Max.x);
    stream.Serialize(m_Max.y);
    stream.Serialize(m_Max.z);
    stream.Serialize(m_MaxLeafSize);
}
#endif

} // namespace BVH