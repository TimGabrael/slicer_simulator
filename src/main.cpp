#include "util.h"

static constexpr float LAYER_HEIGHT = 0.1f;

static BoundingBox bounding_box;
static std::vector<Triangle> model_data;
void LoadModelData(const Model& model, float scale) {
    glm::vec3 min = glm::vec3(FLT_MAX);
    glm::vec3 max = glm::vec3(-FLT_MAX);
    for(int i = 0; i < model.meshCount; ++i) {
        if(model.meshes[i].indices) {
            for(int j = 0; j < model.meshes[i].triangleCount / 3; j += 3) {
                const uint32_t i1 = model.meshes[i].indices[j + 0];
                const uint32_t i2 = model.meshes[i].indices[j + 1];
                const uint32_t i3 = model.meshes[i].indices[j + 2];

                const float* v1 = model.meshes[i].vertices + 3 * i1;
                const float* v2 = model.meshes[i].vertices + 3 * i2;
                const float* v3 = model.meshes[i].vertices + 3 * i3;

                const glm::vec3 p1 = glm::vec3(v1[0], v1[1], v1[2]) * scale;
                const glm::vec3 p2 = glm::vec3(v2[0], v2[1], v2[2]) * scale;
                const glm::vec3 p3 = glm::vec3(v3[0], v3[1], v3[2]) * scale;

                min = glm::min(glm::min(glm::min(p1, p2), p3), min);
                max = glm::max(glm::max(glm::max(p1, p2), p3), max);

                model_data.push_back({ p1, p2, p3 });
            }
        }
        else {
            for(int j = 0; j < model.meshes[i].vertexCount; j += 3) {
                const float* v1 = model.meshes[i].vertices + 3 * (j + 0);
                const float* v2 = model.meshes[i].vertices + 3 * (j + 1);
                const float* v3 = model.meshes[i].vertices + 3 * (j + 2);

                const glm::vec3 p1 = glm::vec3(v1[0], v1[1], v1[2]) * scale;
                const glm::vec3 p2 = glm::vec3(v2[0], v2[1], v2[2]) * scale;
                const glm::vec3 p3 = glm::vec3(v3[0], v3[1], v3[2]) * scale;

                min = glm::min(glm::min(glm::min(p1, p2), p3), min);
                max = glm::max(glm::max(glm::max(p1, p2), p3), max);

                model_data.push_back({ p1, p2, p3 });
            }
        }
    }
    bounding_box.min = {min.x, min.y, min.z};
    bounding_box.max = {max.x, max.y, max.z};
}


int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Window");

    Model model = LoadModel("bin/stanford_bunny.obj");
    static constexpr float MODEL_SCALE = 30.0f;
    LoadModelData(model, MODEL_SCALE);

    BBox model_bounds = {
        {bounding_box.min.x, bounding_box.min.y, bounding_box.min.z},
        {bounding_box.max.x, bounding_box.max.y, bounding_box.max.z}
    };
    std::vector<SurfaceGroup> groups = Util_CalculateSurfaceGroups(model_data, model_bounds, LAYER_HEIGHT);
    InfillSettings settings = {
        .pattern = InfillSettings::Pattern::Rectilinear,
        .percentage = 100.0f,
        .offset = 0.0f,
        .line_width = 0.1f,
        .offset_rectilinear = false,
    };
    InfillData infill_data = Util_CalculateInfill(groups, settings, 0);

    Camera3D camera = { 0 };
    float distance = 10.0f;
    float phi = 0.0f;
    float theta = 0.0f;

    glm::vec3 center_offset = glm::vec3(0.0f, (bounding_box.min.y + bounding_box.max.y) * 0.5f, 0.0f);
    const glm::vec3 cam_pos = distance * glm::vec3(cos(theta) * sin(phi), sin(theta), cos(theta) * cos(phi)) + center_offset;

    camera.position = {cam_pos.x, cam_pos.y, cam_pos.z};
    camera.target = (Vector3){center_offset.x, center_offset.y, center_offset.z};
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };
    SetTargetFPS(60);


    while(!WindowShouldClose()) {
        if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            phi -= delta.x * 0.004f;
            theta += delta.y * 0.004f;
            const glm::vec3 cam_pos = distance * glm::vec3(cos(theta) * sin(phi), sin(theta), cos(theta) * cos(phi)) + center_offset;
            camera.position = {cam_pos.x, cam_pos.y, cam_pos.z};
        }
        if(IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 delta = GetMouseDelta();
            float mov_y = delta.y * 0.01f;
            camera.position.y += mov_y;
            camera.target.y += mov_y;
            center_offset.y += mov_y;
        }
        const float mouse_wheel = GetMouseWheelMove();
        if(glm::abs(mouse_wheel) > EPSILON) {
            distance -= mouse_wheel * 1.0f;
            distance = std::max(std::min(20.0f, distance), 1.0f);
            const glm::vec3 cam_pos = distance * glm::vec3(cos(theta) * sin(phi), sin(theta), cos(theta) * cos(phi)) + center_offset;
            camera.position = {cam_pos.x, cam_pos.y, cam_pos.z};
        }
        BeginDrawing();
        {
            ClearBackground(BLACK);
            BeginMode3D(camera);
            {
                Color AVAILABLE_COLORS[] = {
                    WHITE,
                    BLUE,
                    GREEN,
                    RED,
                    YELLOW,
                    ORANGE,
                    PURPLE
                };
                uint32_t line_counter = 0;
                uint32_t group_counter = 0;
                uint32_t convex_group_counter = 0;
                for(const SurfaceGroup& g : groups) {
                    const Color group_color = AVAILABLE_COLORS[group_counter % ARRSIZE(AVAILABLE_COLORS)];
                    const Color layer_color = AVAILABLE_COLORS[g.layer_idx % ARRSIZE(AVAILABLE_COLORS)];
                    for(const auto& c_g : g.convex_groups) {
                        //for(const ConvexGroup::Edge& edge : c_g.outer_edges) {
                        //    uint32_t i1 = edge.p1;
                        //    uint32_t i2 = edge.p2;
                        //    const Color convex_color = AVAILABLE_COLORS[convex_group_counter % ARRSIZE(AVAILABLE_COLORS)];

                        //    const glm::vec3 p1 = g.points.at(i1);
                        //    const glm::vec3 p2 = g.points.at(i2);

                        //    DrawLine3D({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, convex_color);
                        //}

                        //uint32_t triangle_index = 0;
                        //for(uint32_t triangle : c_g.triangles) {
                        //    const Color triangle_color = AVAILABLE_COLORS[triangle_index % ARRSIZE(AVAILABLE_COLORS)];
                        //    const uint32_t i1 = g.convex_triangles.at(3 * triangle + 0);
                        //    const uint32_t i2 = g.convex_triangles.at(3 * triangle + 1);
                        //    const uint32_t i3 = g.convex_triangles.at(3 * triangle + 2);

                        //    const glm::vec3 p1 = g.points.at(i1);
                        //    const glm::vec3 p2 = g.points.at(i2);
                        //    const glm::vec3 p3 = g.points.at(i3);

                        //    DrawLine3D({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, RED);
                        //    DrawLine3D({p2.x, p2.y, p2.z}, {p3.x, p3.y, p3.z}, GREEN);
                        //    DrawLine3D({p3.x, p3.y, p3.z}, {p1.x, p1.y, p1.z}, BLUE);
                        //    triangle_index += 1;
                        //}

                        convex_group_counter += 1;
                    }

                    // draw consecutive points as connected lines
                    for(size_t i = 0; i < g.points.size(); ++i) {
                        uint32_t next = (i + 1) % g.points.size();
                        const Color line_color = AVAILABLE_COLORS[line_counter % ARRSIZE(AVAILABLE_COLORS)];

                        const glm::vec3 p1 = g.points.at(i);
                        const glm::vec3 p2 = g.points.at(next);
                        DrawLine3D({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, layer_color);
                        line_counter += 1;
                    }
                    if(g.layer_idx == 0) {
                        for(size_t i = 0; i < infill_data.lines.size(); ++i) {
                            const glm::vec3 p1 = {infill_data.lines.at(i).p1.x, 0.0f, infill_data.lines.at(i).p1.y};
                            const glm::vec3 p2 = {infill_data.lines.at(i).p2.x, 0.0f, infill_data.lines.at(i).p2.y};
                            DrawLine3D({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, layer_color);

                        }
                    }

                    // draw each pair of points as a line
                    //for(size_t i = 0; i < g.points.size() / 2; ++i) {
                    //    const Color line_color = AVAILABLE_COLORS[line_counter % ARRSIZE(AVAILABLE_COLORS)];

                    //    const glm::vec3 p1 = g.points.at(2 * i + 0);
                    //    const glm::vec3 p2 = g.points.at(2 * i+ 1);
                    //    DrawLine3D({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, layer_color);
                    //    line_counter += 1;
                    //}
                    group_counter += 1;
                }
                DrawBoundingBox(bounding_box, YELLOW);
                //DrawModel(model, {0.0f, 0.0f, 0.0f}, MODEL_SCALE, GREEN);
                DrawGrid(10, 1.0f);
            }
            EndMode3D();
        }
        EndDrawing();
    }
    CloseWindow();
    
    return 0;
}
