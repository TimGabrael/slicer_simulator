#include "util.h"

static constexpr float TOLERANCE = 0.001f;

static float Cross(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) {
    return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}

static consteval float BinomialCoefficient(size_t n, size_t k) {
    if (k > n) return 0.0f;
    if (k == 0 || k == n) return 1.0f;

    float res = 1.0f;
    for (size_t i = 1; i <= k; ++i) {
        res *= static_cast<float>(n - (k - i));
        res /= static_cast<float>(i);
    }
    return res;
}
consteval std::vector<float> CreateBinomialCoefficientsVector(size_t n) {
    std::vector<float> row(n + 1);
    for (size_t k = 0; k <= n; ++k) {
        row[k] = BinomialCoefficient(n, k);
    }
    return row;
}
template<size_t max_n>
consteval auto CreateBinomialCoefficientsMap() {
    std::array<std::array<float, max_n>, max_n> map{};
    for(size_t n = 0; n < max_n; ++n) {
        for(size_t k = 0; k < max_n; ++k) {
            map[n][k] = BinomialCoefficient(n, k);
        }
    }
    return map;
}

static constexpr size_t MAX_BINOM_N = 10;
static constexpr auto BINOMIAL_COEFFICIENTS = CreateBinomialCoefficientsMap<MAX_BINOM_N>();
glm::vec2 BezierCurve2D::Sample(float t) const {
    if(t < 0.0f || t > 1.0f) {
        return {};
    }
    if(this->control_points.empty()) {
        return {};
    }
    size_t n = this->control_points.size() - 1;
    if(n >= MAX_BINOM_N) {
        return {};
    }
    glm::vec2 out = {};
    for(size_t k = 0; k < this->control_points.size(); ++k) {
        out += BINOMIAL_COEFFICIENTS[n][k] * std::powf(1.0f - t, n - k) * std::powf(t, k) * this->control_points.at(k);
    }
    return out;
}
glm::vec2 BezierCurve2D::SampleDerivative(float t) const {
    if(t < 0.0f || t > 1.0f) {
        return {};
    }
    if(this->control_points.empty()) {
        return {};
    }
    size_t n = this->control_points.size() - 1;
    if(n >= MAX_BINOM_N) {
        return {};
    }
    glm::vec2 out = {};
    for(size_t k = 0; k < this->control_points.size(); ++k) {
        out += BINOMIAL_COEFFICIENTS[n][k] * std::powf(1.0f - t, n - k - 1) * std::powf(t, k - 1) * this->control_points.at(k) * (k - n * t);
    }
    return out;
}
float BezierCurve2D::ComputeLength(float integration_step_size) const {
    if(this->control_points.size() < 2) {
        return 0.0f;
    }
    float accum = 0.0f;
    float cur_x = 0.0f;
    while(cur_x < 1.0f) {
        const float deriv_len = glm::length(this->SampleDerivative(cur_x));
        accum += deriv_len * integration_step_size;
        cur_x += integration_step_size;
    }
    return accum;
}
float BezierCurve1D::Sample(float t) const {
    if(t < 0.0f || t > 1.0f) {
        return 0.0f;
    }
    if(this->control_points.empty()) {
        return 0.0f;
    }
    size_t n = this->control_points.size() - 1;
    if(n >= MAX_BINOM_N) {
        return 0.0f;
    }
    float out = 0.0f;
    for(size_t k = 0; k < this->control_points.size(); ++k) {
        out += BINOMIAL_COEFFICIENTS[n][k] * std::powf(1.0f - t, n - k) * std::powf(t, k) * this->control_points.at(k);
    }
    return out;
}
float BezierCurve1D::SampleDerivative(float t) const {
    if(t < 0.0f || t > 1.0f) {
        return 0.0f;
    }
    if(this->control_points.empty()) {
        return 0.0f;
    }
    size_t n = this->control_points.size() - 1;
    if(n >= MAX_BINOM_N) {
        return 0.0f;
    }
    float out = {};
    for(size_t k = 0; k < this->control_points.size(); ++k) {
        out += BINOMIAL_COEFFICIENTS[n][k] * std::powf(1.0f - t, n - k - 1) * std::powf(t, k - 1) * this->control_points.at(k) * (k - n * t);
    }
    return out;
}
float BezierCurve1D::ComputeLength(float integration_step_size) const {
    if(this->control_points.size() < 2) {
        return 0.0f;
    }
    float accum = 0.0f;
    float cur_x = 0.0f;
    while(cur_x < 1.0f) {
        const float deriv_len = std::abs(this->SampleDerivative(cur_x));
        accum += deriv_len * integration_step_size;
        cur_x += integration_step_size;
    }
    return accum;
}
float BezierCurve1D::Integrate(float start, float end, float dt) const {
    if(this->control_points.size() < 2) {
        return 0.0f;
    }
    if(start < 0.0f || start > 1.0f || end < 0.0f || end > 1.0f) {
        return 0.0f;
    }
    float accum = 0.0f;
    if(end < start) {
        accum += this->Integrate(end, 1.0f, dt);
        accum += this->Integrate(0.0f, start, dt);
    }
    else {
        float cur_x = start;
        while(cur_x < end) {
            const float sample = this->Sample(cur_x);
            accum += sample * dt;
            cur_x += dt;
        }
    }
    return accum;
}
glm::vec2 Path2D::Sample(float t) const {
    if(t < 0.0f || t > 1.0f) {
        return {};
    }
    for(const auto& curve : this->curves) {
        if(curve.start_percentile <= t && t <= curve.end_percentile) {
            const float local_t = (t - curve.start_percentile) * curve.inv_percentile;
            return curve.bezier.Sample(local_t);
        }
    }
    return {};
}
float Path1D::Sample(float t) const {
    if(t < 0.0f || t > 1.0f) {
        return 0.0f;
    }
    for(const auto& curve : this->curves) {
        if(curve.start_percentile <= t && t <= curve.end_percentile) {
            const float local_t = (t - curve.start_percentile) * curve.inv_percentile;
            return curve.bezier.Sample(local_t);
        }
    }
    return 0.0f;
}
float Path1D::Integrate(float start, float end, uint32_t integration_steps) const {
    if(start < 0.0f || start > 1.0f || end < 0.0f || end > 1.0f) {
        return 0.0f;
    }
    float accum = 0.0f;
    if(end < start) {
        std::swap(start, end);
        const float len = (1.0f - end) + start;
        uint32_t bottom_half_steps = len * integration_steps;
        uint32_t upper_half_steps = integration_steps - bottom_half_steps;
        accum += this->Integrate(end, 1.0f, bottom_half_steps);
        accum += this->Integrate(0.0f, start, upper_half_steps);
    }
    else {
        const float dt = (end - start) / static_cast<float>(integration_steps);

        for(uint32_t i = 0; i < integration_steps; ++i) {
            accum += this->Sample(start + dt * i) * dt;
        }
    }
    return accum;
}

float SurfaceGroup::ComputeLength() const {
    float full_len = 0.0f;
    for(size_t i = 0; i < this->points.size(); ++i) {
        const size_t next = (i + 1) % this->points.size();
        const glm::vec2 p1 = this->points.at(i).xz();
        const glm::vec2 p2 = this->points.at(next).xz();
        full_len += glm::distance(p1, p2);
    }
    return full_len;
}
void SurfaceGroup::CalculateConvexGroups() {
    this->convex_groups.clear();
    if(this->points.empty()) {
        return;
    }

    bool counterclockwise = false;
    float signed_area = 0.0f;
    for(size_t i = 0; i < this->points.size(); ++i) {
        const size_t next = (i + 1) % this->points.size();
        const glm::vec2& p1 = this->points.at(i).xz();
        const glm::vec2& p2 = this->points.at(next).xz();
        signed_area += p1.x * p2.y - p2.x * p1.y;
    }
    if(signed_area > 0.0f) {
        counterclockwise = true;
    }


    std::vector<uint32_t> indices(this->points.size());
    for(int i = 0; i < indices.size(); ++i) {
        indices.at(i) = i;
    }
    
    while(indices.size() > 3) {
        bool clipped = false;
        for(size_t i = 0; i < indices.size(); ++i) {
            const uint32_t p = indices.at((i - 1 + indices.size()) % indices.size());
            const uint32_t c = indices.at(i);
            const uint32_t n = indices.at((i + 1) % indices.size());

            const glm::vec2& prev = this->points.at(p).xz();
            const glm::vec2& cur = this->points.at(c).xz();
            const glm::vec2& next = this->points.at(n).xz();

            // check if polygon is in correct orientation, if not skip
            if(counterclockwise) {
                if(Cross(prev, cur, next) < 0.0f) {
                    continue;
                }
            }
            else {
                if(Cross(prev, cur, next) > 0.0f) {
                    continue;
                }
            }
            bool has_point_inside = false;
            for(uint32_t idx : indices) {
                if(idx == p || idx == c || idx == n) {
                    continue;
                }
                if(Util_PointInTriangle(this->points.at(idx).xz(), prev, cur, next)) {
                    has_point_inside = true;
                    break;
                }
            }

            if(!has_point_inside) {
                this->convex_triangles.push_back(p);
                this->convex_triangles.push_back(c);
                this->convex_triangles.push_back(n);
                indices.erase(indices.begin() + i);
                clipped = true;
                break;
            }
        }
        if(!clipped) {
            break;
        }
    }
    if(indices.size() == 3) {
        this->convex_triangles.push_back(indices.at(0));
        this->convex_triangles.push_back(indices.at(1));
        this->convex_triangles.push_back(indices.at(2));
    }

    const uint32_t triangle_count = this->convex_triangles.size() / 3;
    auto edge_to_hash = [](uint32_t v1, uint32_t v2) {
        return static_cast<uint64_t>(v1) << 32uLL | static_cast<uint64_t>(v2);
    };

    std::unordered_map<uint64_t, std::vector<uint32_t>> edge_mapping;
    for(size_t i = 0; i < triangle_count; ++i) {
        const uint32_t i1 = this->convex_triangles.at(3 * i + 0);
        const uint32_t i2 = this->convex_triangles.at(3 * i + 1);
        const uint32_t i3 = this->convex_triangles.at(3 * i + 2);
        edge_mapping[edge_to_hash(i1, i2)].push_back(i);
        edge_mapping[edge_to_hash(i2, i1)].push_back(i);

        edge_mapping[edge_to_hash(i1, i3)].push_back(i);
        edge_mapping[edge_to_hash(i3, i1)].push_back(i);

        edge_mapping[edge_to_hash(i2, i3)].push_back(i);
        edge_mapping[edge_to_hash(i3, i2)].push_back(i);
    }
    struct TempNeighbours {
        uint32_t n1, n2, n3;
        bool added;
    };
    std::vector<TempNeighbours> trig_data(triangle_count);
    for(uint32_t i = 0; i < triangle_count; ++i) {
        const uint32_t i1 = this->convex_triangles.at(3 * i + 0);
        const uint32_t i2 = this->convex_triangles.at(3 * i + 1);
        const uint32_t i3 = this->convex_triangles.at(3 * i + 2);
        TempNeighbours neighbours = {
            .n1 = UINT32_MAX,
            .n2 = UINT32_MAX,
            .n3 = UINT32_MAX,
            .added = false,
        };

        {
            auto data = edge_mapping.find(edge_to_hash(i1, i2));
            for(uint32_t trig : data->second) {
                if(trig == i) {
                    continue;
                }
                neighbours.n1 = trig;
            }
        }
        {
            auto data = edge_mapping.find(edge_to_hash(i2, i3));
            for(uint32_t trig : data->second) {
                if(trig == i) {
                    continue;
                }
                neighbours.n3 = trig;
            }
        }
        {
            auto data = edge_mapping.find(edge_to_hash(i3, i1));
            for(uint32_t trig : data->second) {
                if(trig == i) {
                    continue;
                }
                neighbours.n2 = trig;
            }
        }
        trig_data[i] = neighbours;
    }

    ConvexGroup cur_group;
    bool all_added = false;
    uint32_t start_idx = 0;
    std::function<bool(uint32_t,uint32_t,uint32_t)> add_and_check_neighbours = [&cur_group, this, &trig_data, &add_and_check_neighbours](uint32_t idx, uint32_t edgev1, uint32_t edgev2) {
        TempNeighbours& tmp = trig_data.at(idx);
        if(tmp.added) {
            return false;
        }
        cur_group.triangles.push_back(idx);
        const uint32_t v1 = this->convex_triangles.at(3 * idx + 0);
        const uint32_t v2 = this->convex_triangles.at(3 * idx + 1);
        const uint32_t v3 = this->convex_triangles.at(3 * idx + 2);

        bool has_v1 = false;
        bool has_v2 = false;
        bool has_v3 = false;
        for(size_t i = 0; i < cur_group.vertices.size(); ++i) {
            if(cur_group.vertices.at(i) == v1) {
                has_v1 = true;
            }
            if(cur_group.vertices.at(i) == v2) {
                has_v2 = true;
            }
            if(cur_group.vertices.at(i) == v3) {
                has_v3 = true;
            }
        }
        
        uint32_t added_idx = UINT32_MAX;
        uint32_t num_verts_added = 0;
        if(!has_v1) {
            num_verts_added += 1;
            cur_group.vertices.push_back(v1);
            added_idx = v1;
        }
        if(!has_v2) {
            num_verts_added += 1;
            cur_group.vertices.push_back(v2);
            added_idx = v2;
        }
        if(!has_v3) {
            num_verts_added += 1;
            cur_group.vertices.push_back(v3);
            added_idx = v3;
        }
        assert((num_verts_added != 1) && (added_idx != UINT32_MAX) && "exactly one vertex should be added to the vertex list, otherwise this wouldn't be a neighbour");

        bool is_convex = true;
        for(size_t i = 0; i < cur_group.outer_edges.size(); ++i) {
            bool is_common_edge = (cur_group.outer_edges.at(i).p1 == edgev1 && cur_group.outer_edges.at(i).p2 == edgev2);
            if(is_common_edge) {
                uint32_t prev = (i - 1 + cur_group.outer_edges.size()) % cur_group.outer_edges.size();
                uint32_t next = (i + 1) % cur_group.outer_edges.size();

                const glm::vec2 prev_p1 = this->points.at(cur_group.outer_edges.at(prev).p1).xz();
                const glm::vec2 prev_p2 = this->points.at(cur_group.outer_edges.at(prev).p2).xz();

                const glm::vec2 added = this->points.at(added_idx).xz();

                const glm::vec2 next_p1 = this->points.at(cur_group.outer_edges.at(next).p1).xz();
                const glm::vec2 next_p2 = this->points.at(cur_group.outer_edges.at(next).p2).xz();

                const float c1 = Cross(prev_p1, prev_p2, next_p1);
                const float c2 = Cross(prev_p1, prev_p2, added);
                const float c3 = Cross(added, next_p1, next_p2);
                bool still_convex = (c1 > 0.0f && c2 > 0.0f && c3 > 0.0f) ||
                        (c1 < 0.0f && c2 < 0.0f && c3 < 0.0f);
                if(!still_convex) {
                    is_convex = false;
                }
                else {
                    cur_group.outer_edges.at(i).p1 = cur_group.outer_edges.at(prev).p2;
                    cur_group.outer_edges.at(i).p2 = added_idx;
                    cur_group.outer_edges.insert(cur_group.outer_edges.begin() + (i + 1), {added_idx, cur_group.outer_edges.at(next).p1});
                }
                break;
            }
        }

        if(!is_convex) {
            for(uint32_t i = 0; i < num_verts_added; ++i) {
                cur_group.vertices.pop_back();
            }
            cur_group.triangles.pop_back();
            return false;
        }
        else {
            tmp.added = true;
            if(tmp.n1 != UINT32_MAX) {
                if(!add_and_check_neighbours(tmp.n1, v1, v2)) {
                }
            }
            if(tmp.n3 != UINT32_MAX) {
                if(!add_and_check_neighbours(tmp.n3, v2, v3)) {
                }
            }
            if(tmp.n2 != UINT32_MAX) {
                if(!add_and_check_neighbours(tmp.n2, v3, v1)) {
                }
            }
        }
        return true;
    };
    while(!all_added) {
        cur_group.center = {};
        cur_group.outer_edges.clear();
        cur_group.triangles.clear();
        cur_group.vertices.clear();

        TempNeighbours& tmp = trig_data.at(start_idx);
        cur_group.triangles.push_back(start_idx);
        const uint32_t v1 = this->convex_triangles.at(3 * start_idx + 0);
        const uint32_t v2 = this->convex_triangles.at(3 * start_idx + 1);
        const uint32_t v3 = this->convex_triangles.at(3 * start_idx + 2);
        cur_group.vertices.push_back(v1);
        cur_group.vertices.push_back(v2);
        cur_group.vertices.push_back(v3);

        cur_group.outer_edges.push_back({v1, v2});
        cur_group.outer_edges.push_back({v2, v3});
        cur_group.outer_edges.push_back({v3, v1});

        tmp.added = true;
        if(tmp.n1 != UINT32_MAX) {
            if(!add_and_check_neighbours(tmp.n1, v1, v2)) {
            }
        }
        if(tmp.n3 != UINT32_MAX) {
            if(!add_and_check_neighbours(tmp.n3, v2, v3)) {
            }
        }
        if(tmp.n2 != UINT32_MAX) {
            if(!add_and_check_neighbours(tmp.n2, v3, v1)) {
            }
        }

        add_and_check_neighbours(start_idx, 0xFFFFFFFF, 0xFFFFFFFF);

        all_added = true;
        for(size_t i = 0; i < trig_data.size(); ++i) {
            if(!trig_data.at(i).added) {
                start_idx = i;
                all_added = false;
                break;
            }
        }
        if(!cur_group.triangles.empty()) {
            this->convex_groups.push_back(cur_group);
        }
    }
}
bool SurfaceGroup::IsPointInside(const glm::vec2& p) const {
    uint32_t intersection_count = 0;
    for(size_t i = 0; i < this->points.size(); ++i) {
        const size_t next = (i + 1) % this->points.size();
        const glm::vec2 p1 = this->points.at(i).xz();
        const glm::vec2 p2 = this->points.at(next).xz();

        // cast ray and see if it intersects
        if((p1.y > p.y) != (p2.y > p.y)) {
            float x_intersect = p1.x + (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
            if(x_intersect > p.x) {
                intersection_count += 1;
            }
        }
    }
    return (intersection_count % 2) == 1;
}

bool Util_PlaneTriangleIntersectionTest(const Triangle& trig, const Plane& plane, glm::vec3& h1, glm::vec3& h2) {
    auto dist = [plane](const glm::vec3& p) {
        return glm::dot((p - plane.point), plane.normal);
    };
    const float d1 = dist(trig.p1);
    const float d2 = dist(trig.p2);
    const float d3 = dist(trig.p3);

    // no intersection
    if((d1 > 0.0f && d2 > 0.0f && d3 > 0.0f) || (d1 < 0.0f && d2 < 0.0f && d3 < 0.0f)) {
        return false;
    }

    static constexpr size_t MAX_POINTS = 6;
    glm::vec3 points[MAX_POINTS];
    uint32_t cur_idx = 0;

    auto intersect = [&points, &cur_idx](const glm::vec3& p1, const glm::vec3& p2, float d1, float d2) {
        if(cur_idx >= MAX_POINTS) {
            return;
        }
        const float delta = d1 - d2;
        if(glm::abs(delta) < EPSILON) {
            return;
        }
        const float t = d1 / delta;
        if(0.0f <= t && t <= 1.0f) {
            points[cur_idx] = p1 + t * (p2 - p1);
            cur_idx += 1;
        }
        return;
    };

    if(abs(d1) < EPSILON) {
        points[cur_idx] = trig.p1;
        cur_idx += 1;
    }
    if((d1 * d2) < EPSILON) {
        intersect(trig.p1, trig.p2, d1, d2);
    }

    if(abs(d2) < EPSILON) {
        points[cur_idx] = trig.p2;
        cur_idx += 1;
    }
    if((d2 * d3) < EPSILON) {
        intersect(trig.p2, trig.p3, d2, d3);
    }

    if(abs(d3) < EPSILON) {
        points[cur_idx] = trig.p3;
        cur_idx += 1;
    }
    if((d3 * d1) < EPSILON) {
        intersect(trig.p3, trig.p1, d3, d1);
    }

    glm::vec3 unique[MAX_POINTS];
    uint32_t num_unique = 0;
    for(uint32_t i = 0; i < cur_idx; ++i) {
        bool overlapping = false;
        for(uint32_t j = 0; j < num_unique; ++j) {
            const float d = glm::distance(points[i], unique[j]);
            if(d < EPSILON) {
                overlapping = true;
            }
        }
        if(!overlapping) {
            unique[num_unique] = points[i];
            num_unique += 1;
        }
    }


    if(num_unique == 2) {
        h1 = unique[0];
        h2 = unique[1];
        return true;
    }
    return false;
}
bool Util_PointInTriangle(const glm::vec2& p, const glm::vec2& t1, const glm::vec2& t2, const glm::vec2& t3) {
    const float c1 = Cross(t1, t2, p);
    const float c2 = Cross(t2, t3, p);
    const float c3 = Cross(t3, t1, p);

    bool has_neg = (c1 < 0.0f) || (c2 < 0.0f) || (c3 < 0.0f);
    bool has_pos = (c1 > 0.0f) || (c2 > 0.0f) || (c3 > 0.0f);

    return !(has_neg && has_pos);
}
void Util_ModelPlaneIntersections(const std::vector<Triangle>& trigs, const Plane& plane, std::vector<glm::vec3>& out) {
    for(auto& trig : trigs) {
        glm::vec3 h1 = {};
        glm::vec3 h2 = {};
        if(Util_PlaneTriangleIntersectionTest(trig, plane, h1, h2)) {
            out.push_back(h1);
            out.push_back(h2);
        }
    }
}
float Util_LineLineIntersection(const glm::vec2& p1, const glm::vec2& dir1, const glm::vec2& p2, const glm::vec2& dir2) {
    const float cross = dir1.x * dir2.y - dir1.y * dir2.x;
    if(glm::abs(cross) < EPSILON) {
        return -FLT_MAX;
    }

    const glm::vec2 p2p1 = p2 - p1;
    const float u = (p2p1.x * dir1.y - p2p1.y * dir1.x) / cross;
    if(u < -EPSILON || u > (1.0f + EPSILON)) {
        return -FLT_MAX;
    }

    const float t = (p2p1.x * dir2.y - p2p1.y * dir2.x) / cross;
    return t;
}
std::vector<SurfaceGroup> Util_CalculateSurfaceGroups(const std::vector<Triangle>& model, const BBox& model_bounds, float layer_height) {
    std::vector<SurfaceGroup> groups;
    const glm::vec3& bb_min = model_bounds.min;
    const glm::vec3& bb_max = model_bounds.max;
    Plane cur_plane = {
        .point = (bb_min + bb_max) * 0.5f,
        .normal = glm::vec3(0.0f, 1.0f, 0.0f)
    };
    const float max_dist = glm::distance(bb_max, bb_min);

    cur_plane.point.y = model_bounds.min.y + layer_height;
    uint32_t layer_idx = 0;
    while(cur_plane.point.y < bb_max.y) {
        SurfaceGroup group;
        group.bounds.min = glm::vec2(FLT_MAX);
        group.bounds.max = glm::vec2(-FLT_MAX);
        group.layer_idx = layer_idx;
        Util_ModelPlaneIntersections(model, cur_plane, group.points);

        struct Connection {
            uint32_t prev = UINT32_MAX;
            uint32_t next = UINT32_MAX;
            bool next_connects_p1 = false;
            bool visited = false;
        };
        std::vector<Connection> connections(group.points.size() / 2);

        auto is_in_chain = [connections](uint32_t cur_end, uint32_t find) {

        };

        uint32_t num_groups = 0;
        for(size_t i = 0; i < group.points.size() / 2; ++i) {
            if(connections.at(i).next != UINT32_MAX || connections.at(i).prev != UINT32_MAX) {
                continue;
            }
            num_groups += 1;
            uint32_t cur_idx = i;
            Connection* conn = &connections.at(i);
            do {
                float closest = FLT_MAX;
                size_t closest_idx = 0xFFFFFFFF;
                const glm::vec2& p1 = group.points.at(2 * cur_idx + 0).xz();
                const glm::vec2& p2 = group.points.at(2 * cur_idx + 1).xz();

                bool closest_p1 = false;
                for(size_t j = 0; j < group.points.size() / 2; ++j) {
                    if(j == conn->prev || cur_idx == j || connections.at(j).prev != UINT32_MAX) {
                        continue;
                    }
                    const glm::vec2& v1 = group.points.at(2 * j + 0).xz();
                    const glm::vec2& v2 = group.points.at(2 * j + 1).xz();
                    const float min_p1 = glm::min(glm::distance(p1, v2), glm::distance(p1, v1));
                    const float min_p2 = glm::min(glm::distance(p2, v2), glm::distance(p2, v1));
                    
                    const float min_dist = glm::min(min_p1, min_p2);
                    if(min_dist < closest) {
                        closest_p1 = min_p1 < min_p2;
                        closest = min_dist;
                        closest_idx = j;
                    }
                }
                if(closest < TOLERANCE) {
                    conn->next = closest_idx;
                    conn->next_connects_p1 = closest_p1;
                    connections.at(closest_idx).prev = cur_idx;

                    cur_idx = conn->next;
                    conn = &connections.at(cur_idx);
                }
                else {
                    uint32_t prev = conn->prev;
                    conn->prev = UINT32_MAX;
                    conn->next = UINT32_MAX;
                    while(prev != i && cur_idx != i) {
                        uint32_t temp = connections.at(prev).prev;
                        connections.at(prev).next = UINT32_MAX;
                        connections.at(prev).prev = UINT32_MAX;
                        prev = temp;
                    }
                    if(prev != 0xFFFFFFFF) {
                        uint32_t temp = connections.at(prev).prev;
                        connections.at(prev).next = UINT32_MAX;
                        connections.at(prev).prev = UINT32_MAX;
                        prev = temp;
                    }
                    break;
                }
            } while(cur_idx != i);
        }

        for(size_t i = 0; i < group.points.size() / 2; ++i) {
            if(connections.at(i).next == UINT32_MAX || connections.at(i).prev == UINT32_MAX || connections.at(i).visited) {
                continue;
            }
            connections.at(i).visited = true;
            SurfaceGroup new_group;
            new_group.bounds.min = glm::vec2(FLT_MAX);
            new_group.bounds.max = glm::vec2(-FLT_MAX);

            uint32_t cur_idx = i;
            if(connections.at(cur_idx).next_connects_p1) {
                new_group.points.push_back(group.points.at(2 * cur_idx + 1));
                new_group.points.push_back(group.points.at(2 * cur_idx + 0));
            }
            else {
                new_group.points.push_back(group.points.at(2 * cur_idx + 0));
                new_group.points.push_back(group.points.at(2 * cur_idx + 1));
            }
            cur_idx = connections.at(cur_idx).next;
            while(cur_idx != i && cur_idx != UINT32_MAX) {
                Connection& conn = connections.at(cur_idx);
                conn.visited = true;
                const glm::vec3 p1 = group.points.at(2 * cur_idx + 0);
                const glm::vec3 p2 = group.points.at(2 * cur_idx + 1);
                if(conn.next_connects_p1) {
                    new_group.points.push_back(p1);
                }
                else {
                    new_group.points.push_back(p2);
                }
                cur_idx = conn.next;
            }

            for(size_t i = 0; i < new_group.points.size(); ++i) {
                const glm::vec2 plane_pos = {new_group.points.at(i).x, new_group.points.at(i).z};
                new_group.bounds.min = glm::min(plane_pos, new_group.bounds.min);
                new_group.bounds.max = glm::max(plane_pos, new_group.bounds.max);
            }
            new_group.layer_idx = layer_idx;
            new_group.CalculateConvexGroups();
            groups.emplace_back(std::move(new_group));
        }


        // just add everything without actually checking for groups
        // might be useful when the above fails for some reason
        //for(size_t i = 0; i < group.points.size(); ++i) {
        //    const glm::vec2 plane_pos = {group.points.at(i).x, group.points.at(i).z};
        //    group.bounds.min = glm::min(plane_pos, group.bounds.min);
        //    group.bounds.max = glm::max(plane_pos, group.bounds.max);
        //}
        //groups.emplace_back(std::move(group));

        cur_plane.point.y += layer_height;
        layer_idx += 1;
    }
    return groups;
}
InfillData Util_CalculateInfill(const std::vector<SurfaceGroup>& groups, const InfillSettings& settings, uint32_t layer) {
    InfillData output = {};
    output.layer_idx = layer;
    if(groups.empty()) {
        return output;
    }
    struct LineHits {
        glm::vec2 start;
        glm::vec2 dir;
        std::vector<float> hits;
    };



    std::vector<LineHits> rays;
    glm::vec2 dir = {};
    glm::vec2 nor = {};
    glm::vec2 dir2 = {};
    glm::vec2 nor2 = {};
    bool two_directional = false;
    if(settings.pattern == InfillSettings::Pattern::Rectilinear) {
        dir = {1.0f, 0.0f};
        nor = {0.0f, 1.0f};
        if((layer % 2) == settings.offset_rectilinear) {
            dir = {0.0f, 1.0f};
            nor = {1.0f, 0.0f};
        }
    }
    else if(settings.pattern == InfillSettings::Pattern::Monotonic) {
        dir = {1.0f, 0.0f};
        nor = {0.0f, 1.0f};
        if(settings.offset_rectilinear) {
            dir = {0.0f, 1.0f};
            nor = {1.0f, 0.0f};
        }
    }
    else if(settings.pattern == InfillSettings::Pattern::Grid) {
        two_directional = true;
        dir = {1.0f, 0.0f};
        nor = {0.0f, 1.0f};

        dir2 = {0.0f, 1.0f};
        nor2 = {1.0f, 0.0f};
    }

    glm::vec2 bb_min = glm::vec2(FLT_MAX);
    glm::vec2 bb_max = glm::vec2(-FLT_MAX);
    for(const SurfaceGroup& group : groups) {
        if(group.layer_idx != layer) {
            continue;
        }
        if(group.points.empty()) {
            continue;
        }
        const glm::vec2 min = group.bounds.min;
        const glm::vec2 max = group.bounds.max;

        bb_min = glm::min(bb_min, min);
        bb_max = glm::max(bb_max, max);
    }

    output.bounds.min = bb_min;
    output.bounds.max = bb_max;

    const glm::vec2 center = (bb_min + bb_max) * 0.5f;
    const float height = glm::dot((bb_max - bb_min), nor);
    const float diagonal = glm::distance(bb_min, bb_max);

    uint32_t max_line_count = height / settings.line_width;
    uint32_t real_line_count = max_line_count * settings.percentage / 100.0f;
    const float line_distance = height / static_cast<float>(real_line_count);
    for(uint32_t i = 0; i < real_line_count; ++i) {
        glm::vec2 start = center - height * nor * 0.5f + i * line_distance * nor - dir * diagonal;
        rays.push_back({
                .start = start,
                .dir = dir * 2.0f * diagonal,
                .hits = {},
        });
    }
    if(two_directional) {
        const float height = glm::dot((bb_max - bb_min), nor2);
        const uint32_t max_line_count = height / settings.line_width;
        uint32_t real_line_count = max_line_count * settings.percentage / 100.0f;
        const float line_distance = height / static_cast<float>(real_line_count);
        for(uint32_t i = 0; i < real_line_count; ++i) {
            glm::vec2 start = center - height * nor2 * 0.5f + i * line_distance * nor2 - dir2 * diagonal;
            rays.push_back({
                    .start = start,
                    .dir = dir2 * 2.0f * diagonal,
                    .hits = {},
                    });
        }
    }


    for(const SurfaceGroup& group : groups) {
        if(group.layer_idx != layer) {
            continue;
        }
        if(group.points.empty()) {
            continue;
        }


        for(uint32_t i = 0; i < group.points.size(); ++i) {
            const uint32_t next = (i + 1) % group.points.size();
            const glm::vec2 l1 = group.points.at(i).xz();
            const glm::vec2 l2 = group.points.at(next).xz();
            for(auto& ray : rays) {
                float t = Util_LineLineIntersection(ray.start, ray.dir, l1, l2 - l1);
                if(t >= -EPSILON && t < (1.0f + EPSILON)) {
                    ray.hits.push_back(t);
                }
            }
        }
    }

    for(auto& ray : rays) {
        for(size_t i = 0; i < ray.hits.size(); ++i) {
            for(size_t j = ray.hits.size() - 1; j > i; --j) {
                if(std::abs(ray.hits.at(i) - ray.hits.at(j)) < TOLERANCE) {
                    ray.hits.erase(ray.hits.begin() + j);
                }
            }
        }
        
        if(ray.hits.size() < 2) {
            continue;
        }
        std::sort(ray.hits.begin(), ray.hits.end(), std::less<float>());
        glm::vec2 start = {};
        for(size_t i = 0; i < ray.hits.size(); ++i) {
            const float t = ray.hits.at(i);
            glm::vec2 hit = ray.start + ray.dir * t;
            if((i % 2)) {
                output.lines.push_back({
                    .p1 = start,
                    .p2 = hit,
                });
            }
            else {
                start = hit;
            }
        }
    }
    return output;
}


