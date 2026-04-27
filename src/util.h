#pragma once
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <array>
#include <functional>
#include "raylib.h"
#include "rlgl.h"
#define GLM_FORCE_SWIZZLE
#include "glm/glm.hpp"
#include "glm/gtx/string_cast.hpp"

static constexpr float EPSILON = 1e-8f;
static constexpr glm::vec3 UP = glm::vec3(0.0f, 1.0f, 0.0f);

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

struct BezierCurve2D {
    std::vector<glm::vec2> control_points;
    glm::vec2 Sample(float t) const;
    glm::vec2 SampleDerivative(float t) const;
    float ComputeLength(float integration_step_size) const;
};
struct BezierCurve1D {
    std::vector<float> control_points;
    float Sample(float t) const;
    float SampleDerivative(float t) const;
    float ComputeLength(float integration_step_size) const;
    float Integrate(float start, float end, float dt) const;
};

struct Path2D {
    struct Curve {
        BezierCurve2D bezier;
        float lenght;
        float start_percentile;
        float end_percentile;
        float inv_percentile;
    };
    std::vector<Curve> curves;
    float length;

    glm::vec2 Sample(float t) const;
};
struct Path1D {
    struct Curve {
        BezierCurve1D bezier;
        float lenght;
        float start_percentile;
        float end_percentile;
        float inv_percentile;
    };
    std::vector<Curve> curves;
    float length;

    float Sample(float t) const;
    float Integrate(float start, float end, uint32_t integration_steps) const;
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
        Grid, // todo: implement
    };
    Pattern pattern;

    // [0.0f,100.0f]
    float percentage;

    float offset;
    float line_width;

    // changes the start direction and therefore subsequent directions
    bool offset_rectilinear;
};
struct InfillData {
    struct Line {
        glm::vec2 p1;
        glm::vec2 p2;
    };
    std::vector<Line> lines;
    BBox2D bounds;
    uint32_t layer_idx;
};



bool Util_PlaneTriangleIntersectionTest(const Triangle& trig, const Plane& plane, glm::vec3& h1, glm::vec3& h2);
bool Util_PointInTriangle(const glm::vec2& p, const glm::vec2& t1, const glm::vec2& t2, const glm::vec2& t3);
void Util_ModelPlaneIntersections(const std::vector<Triangle>& trigs, const Plane& plane, std::vector<glm::vec3>& out);

float Util_LineLineIntersection(const glm::vec2& p1, const glm::vec2& dir1, const glm::vec2& p2, const glm::vec2& dir2);

// the surface groups are always constructed from bottom to top
// (plane normal (0, 1, 0) with point in center going up by layer_height for each layer)
std::vector<SurfaceGroup> Util_CalculateSurfaceGroups(const std::vector<Triangle>& model, const BBox& model_bounds, float layer_height);

InfillData Util_CalculateInfill(const std::vector<SurfaceGroup>& groups, const InfillSettings& settings, uint32_t layer);

