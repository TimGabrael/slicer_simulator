#pragma once
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <functional>
#include "raylib.h"
#define GLM_FORCE_SWIZZLE
#include "glm/glm.hpp"
#include "glm/gtx/string_cast.hpp"

static constexpr float EPSILON = 1e-8f;

#define ARRSIZE(arr) (sizeof(arr) / (sizeof(*arr)))

struct Triangle {
    glm::vec3 p1, p2, p3;
};
struct Plane {
    glm::vec3 point;
    glm::vec3 normal;
};
struct BBox {
    glm::vec3 min;
    glm::vec3 max;
};
struct BBox2D {
    glm::vec2 min;
    glm::vec2 max;
};
struct ConvexGroup {
    struct Edge {
        uint32_t p1;
        uint32_t p2;
    };
    
    std::vector<uint32_t> triangles;
    std::vector<uint32_t> vertices;
    std::vector<Edge> outer_edges;
    glm::vec2 center;
};
struct SurfaceGroup {
    std::vector<glm::vec3> points;
    std::vector<uint32_t> convex_triangles;
    std::vector<ConvexGroup> convex_groups;
    BBox2D bounds;
    uint32_t layer_idx;
    float ComputeLength() const;
    void CalculateConvexGroups();
    bool IsPointInside(const glm::vec2& p) const;
};

struct InfillSettings {
    enum Pattern {
        Rectilinear,
        Monotonic,
        Grid,
    };
    Pattern pattern;

    // [0.0f,100.0f]
    float percentage;

    float offset;
    float line_width;
    // offset rectilinear
    bool offset_rectilinear;
};
struct InfillData {
    struct Line {
        glm::vec2 p1;
        glm::vec2 p2;
    };
    std::vector<Line> lines;
};


bool Util_PlaneTriangleIntersectionTest(const Triangle& trig, const Plane& plane, glm::vec3& h1, glm::vec3& h2);
bool Util_PointInTriangle(const glm::vec2& p, const glm::vec2& t1, const glm::vec2& t2, const glm::vec2& t3);
void Util_ModelPlaneIntersections(const std::vector<Triangle>& trigs, const Plane& plane, std::vector<glm::vec3>& out);

float Util_LineLineIntersection(const glm::vec2& p1, const glm::vec2& dir1, const glm::vec2& p2, const glm::vec2& dir2);

// the surface groups are always constructed from bottom to top
// (plane normal (0, 1, 0) with point in center going up by layer_height for each layer)
std::vector<SurfaceGroup> Util_CalculateSurfaceGroups(const std::vector<Triangle>& model, const BBox& model_bounds, float layer_height);

InfillData Util_CalculateInfill(const std::vector<SurfaceGroup>& groups, const InfillSettings& settings, uint32_t layer);

