#include "util.h"
#include "simulation.h"

static void DrawText3D(Font font, const char *text, Vector3 position, float fontSize, float fontSpacing, float lineSpacing, bool backface, Color tint);

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
    constexpr int SCREEN_WIDTH = 1280;
    constexpr int SCREEN_HEIGHT = 720;

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Window");
    SetTargetFPS(60);

    Model model = LoadModel("bin/stanford_bunny.obj");
    static constexpr float LAYER_HEIGHT = 0.1f;
    static constexpr float MODEL_SCALE = 30.0f;
    static constexpr float LINE_WIDTH = 0.04f;
    LoadModelData(model, MODEL_SCALE);

    BBox model_bounds = {
        {bounding_box.min.x, bounding_box.min.y, bounding_box.min.z},
        {bounding_box.max.x, bounding_box.max.y, bounding_box.max.z}
    };
    std::vector<SurfaceGroup> groups = Util_CalculateSurfaceGroups(model_data, model_bounds, LAYER_HEIGHT);
    InfillSettings settings = {
        .pattern = InfillSettings::Pattern::Grid,
        .percentage = 100.0f,
        .offset = 0.0f,
        .line_width = LINE_WIDTH,
        .offset_rectilinear = false,
    };
    std::vector<InfillData> line_data;
    for(size_t i = 0; i < groups.back().layer_idx; ++i) {
        InfillData infill_data = Util_CalculateInfill(groups, settings, i);
        line_data.emplace_back(std::move(infill_data));
    }

    BezierCurve1D test_bezier;
    test_bezier.control_points.push_back(LINE_WIDTH * 4.0f);
    Nozzle nozzle = {};

    nozzle.size.curves.push_back({.bezier = test_bezier, 
        .lenght = 0.0f,
        .start_percentile = 0.0f,
        .end_percentile = 1.0f,
        .inv_percentile = 1.0f,
    });
    nozzle.size.length = 0.0f;

    test_bezier.control_points.at(0) = 150000.0f;

    nozzle.temperature.curves.push_back({.bezier = test_bezier, 
        .lenght = 0.0f,
        .start_percentile = 0.0f,
        .end_percentile = 1.0f,
        .inv_percentile = 1.0f,
    });
    nozzle.temperature.length = 0.0f;

    test_bezier.control_points.at(0) = 2.0f;
    test_bezier.control_points.push_back(2.0f);
    nozzle.speed.curves.push_back({.bezier = test_bezier, 
        .lenght = 1.0f,
        .start_percentile = 0.0f,
        .end_percentile = 1.0f,
        .inv_percentile = 1.0f,
    });
    nozzle.speed.length = 1.0f;
    nozzle.speed_time_scale = 1.0f;

    MaterialConstants material = {
        .thermal_absorptance = 0.5f,
        .thermal_diffusivity = 0.1,
        .thermal_loss_coefficient = 0.0f,
    };

    HotSpotData hotspot_data = {};
    SimData sim_data = {
        .resolution_x = 256,
        .time_step = 0.01f,
    };
    uint32_t hot_spot_layer = 0xFFFFFFFF;
    for(size_t i = 0; i < line_data.size(); ++i) {
        if(line_data.at(i).lines.empty()) {
            continue;
        }
        hot_spot_layer = line_data.at(i).layer_idx;
        sim_data.bounds.min = line_data.at(i).bounds.min - glm::vec2(LINE_WIDTH * 10.0f); 
        sim_data.bounds.max = line_data.at(i).bounds.max + glm::vec2(LINE_WIDTH * 10.0f); 
        hotspot_data = Sim_CalculateHotspots(line_data.at(i), {nozzle}, material, sim_data);
        break;
    }

    uint32_t* texture_data = new uint32_t[hotspot_data.width * hotspot_data.height];
    float delta_hotspot = (hotspot_data.max - hotspot_data.min);
    std::cout << "min/max: " << hotspot_data.min << ", " << hotspot_data.max << std::endl;
    const float cutoff = 5.0f;
    if(hotspot_data.min < cutoff) {
        hotspot_data.min = cutoff;
        delta_hotspot = (hotspot_data.max - hotspot_data.min);
    }
    if(delta_hotspot < EPSILON) {
        delta_hotspot = EPSILON;
    }
    // convert to decible
    float decible_ref = hotspot_data.max * 0.1f;
    hotspot_data.min = FLT_MAX;
    hotspot_data.max = -FLT_MAX;
    for(uint32_t i = 0; i < hotspot_data.width * hotspot_data.height; ++i) {
        if(hotspot_data.temp[i] < cutoff) {
            hotspot_data.temp[i] = -FLT_MAX;
            continue;
        }
        const float db = 10.0f * std::log10f((hotspot_data.temp[i] + EPSILON - cutoff * 0.8f) / decible_ref);
        hotspot_data.temp[i] = db;
        hotspot_data.max = std::max(hotspot_data.max, db);
        hotspot_data.min = std::min(hotspot_data.min, db);
    }
    delta_hotspot = (hotspot_data.max - hotspot_data.min);
    std::cout << "db_min/max: " << hotspot_data.min << ", " << hotspot_data.max << std::endl;
    for(uint32_t i = 0; i < hotspot_data.width * hotspot_data.height; ++i) {
        if(hotspot_data.temp[i] < -1e10) {
            texture_data[i] = 0.0f;
            continue;
        }
        const float normalized = (hotspot_data.temp[i] - hotspot_data.min) / delta_hotspot;

        //if(hotspot_data.data[i] < cutoff) {
        //    texture_data[i] = 0;
        //    continue;
        //}

        const glm::vec4 col = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) * normalized + (1.0f - normalized) * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
        uint32_t r = (glm::max(glm::min(col.x, 1.0f), 0.0f) * 0xFF);
        uint32_t g = (glm::max(glm::min(col.y, 1.0f), 0.0f) * 0xFF);
        uint32_t b = (glm::max(glm::min(col.z, 1.0f), 0.0f) * 0xFF);
        uint32_t a = (glm::max(glm::min(col.w, 1.0f), 0.0f) * 0xFF);
        texture_data[i] = (a << 24) | (b << 16) | (g << 8) | r;
    }

    Image img = {};
    img.data = texture_data;
    img.width = hotspot_data.width;
    img.height = hotspot_data.height;
    img.mipmaps = 1;
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D tex = LoadTextureFromImage(img);
    delete[] texture_data;
    texture_data = nullptr;






    Camera3D camera = { 0 };
    float distance = 10.0f;
    float phi = 0.0f;
    float theta = 0.0f;

    glm::vec3 target_pos = glm::vec3(0.0f, (bounding_box.min.y + bounding_box.max.y) * 0.5f, 0.0f);
    glm::vec3 cam_pos = distance * glm::vec3(cos(theta) * sin(phi), sin(theta), cos(theta) * cos(phi)) + target_pos;

    camera.position = {cam_pos.x, cam_pos.y, cam_pos.z};
    camera.target = (Vector3){target_pos.x, target_pos.y, target_pos.z};
    camera.up = (Vector3){ UP.x, UP.y, UP.z };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;


    uint32_t inspecting_layer = UINT32_MAX;
    while(!WindowShouldClose()) {
        if(IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            phi -= delta.x * 0.004f;
            theta += delta.y * 0.004f;
            cam_pos = distance * glm::vec3(cos(theta) * sin(phi), sin(theta), cos(theta) * cos(phi)) + target_pos;
            camera.position = {cam_pos.x, cam_pos.y, cam_pos.z};
        }
        if(IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 delta = GetMouseDelta();
            float mov_x = delta.x * 0.01f;
            float mov_y = delta.y * 0.01f;

            cam_pos.y += mov_y;
            target_pos.y += mov_y;

            const glm::vec3 look_dir = glm::normalize(target_pos - cam_pos);
            const glm::vec3 tangent = glm::normalize(glm::cross(UP, look_dir));

            cam_pos += tangent * mov_x;
            target_pos += tangent * mov_x;
            camera.position = {cam_pos.x, cam_pos.y, cam_pos.z};
            camera.target = {target_pos.x, target_pos.y, target_pos.z};
        }
        const float mouse_wheel = GetMouseWheelMove();
        if(glm::abs(mouse_wheel) > EPSILON) {
            distance -= mouse_wheel * 1.0f;
            distance = std::max(std::min(20.0f, distance), 1.0f);
            cam_pos = distance * glm::vec3(cos(theta) * sin(phi), sin(theta), cos(theta) * cos(phi)) + target_pos;
            camera.position = {cam_pos.x, cam_pos.y, cam_pos.z};
        }
        // Raylib uses the QWERTY-Keyboard layout...
        if(IsKeyPressed(KEY_Z) || IsKeyPressedRepeat(KEY_Z)) {
            inspecting_layer += 1;
        }
        else if(IsKeyPressed(KEY_X) || IsKeyPressedRepeat(KEY_X)) {
            if(inspecting_layer != UINT32_MAX) {
                inspecting_layer -= 1;
            }
        }
        else if(IsKeyPressed(KEY_R)) {
            inspecting_layer = UINT32_MAX;
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
                    const float cur_y = (g.layer_idx + 1) * LAYER_HEIGHT + model_bounds.min.y;
                    if(inspecting_layer != UINT32_MAX && g.layer_idx != inspecting_layer) {
                        continue;
                    }
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
                        if(inspecting_layer != UINT32_MAX) {
                            const glm::vec3 text_pos = p1 + glm::vec3(0.0f, 0.1f, 0.0f);
                            DrawText3D(GetFontDefault(), std::to_string(i).c_str(), {text_pos.x, text_pos.y, text_pos.z}, 0.04f, 0.004f, -0.1f, true, WHITE);
                        }
                    }
                    if(inspecting_layer != UINT32_MAX) {
                        BoundingBox bb = {
                            .min = {g.bounds.min.x, cur_y - 0.1f, g.bounds.min.y},
                            .max = {g.bounds.max.x, cur_y + 0.1f, g.bounds.max.y},
                        };
                        DrawBoundingBox(bb, GREEN);
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

                for(const InfillData& infill_data : line_data) {
                    if(inspecting_layer != UINT32_MAX && infill_data.layer_idx != inspecting_layer) {
                        continue;
                    }
                    const Color layer_color = AVAILABLE_COLORS[infill_data.layer_idx % ARRSIZE(AVAILABLE_COLORS)];
                    const float cur_y = (infill_data.layer_idx + 1) * LAYER_HEIGHT + model_bounds.min.y;
                    for(size_t i = 0; i < infill_data.lines.size(); ++i) {
                        const glm::vec3 p1 = {infill_data.lines.at(i).p1.x, cur_y, infill_data.lines.at(i).p1.y};
                        const glm::vec3 p2 = {infill_data.lines.at(i).p2.x, cur_y, infill_data.lines.at(i).p2.y};
                        DrawLine3D({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, layer_color);
                    }
                }


                DrawBoundingBox(bounding_box, YELLOW);

                //DrawModel(model, {0.0f, 0.0f, 0.0f}, MODEL_SCALE, GREEN);
                DrawGrid(10, 1.0f);
            }
            EndMode3D();
        }

        if(inspecting_layer != UINT32_MAX) {
            std::string info = "Inspecting: " + std::to_string(inspecting_layer);
            DrawText(info.c_str(), 0, 0, 16, WHITE);
        }

        DrawTexture(tex, 100, 100, WHITE);
        EndDrawing();
    }
    CloseWindow();
    
    return 0;
}


// Source: raylib examples text_3d_drawing
static void DrawTextCodepoint3D(Font font, int codepoint, Vector3 position, float fontSize, bool backface, Color tint) {
    // Character index position in sprite font
    // NOTE: In case a codepoint is not available in the font, index returned points to '?'
    int index = GetGlyphIndex(font, codepoint);
    float scale = fontSize/(float)font.baseSize;

    // Character destination rectangle on screen
    // NOTE: We consider charsPadding on drawing
    position.x += (float)(font.glyphs[index].offsetX - font.glyphPadding)*scale;
    position.z += (float)(font.glyphs[index].offsetY - font.glyphPadding)*scale;

    // Character source rectangle from font texture atlas
    // NOTE: We consider chars padding when drawing, it could be required for outline/glow shader effects
    Rectangle srcRec = { font.recs[index].x - (float)font.glyphPadding, font.recs[index].y - (float)font.glyphPadding,
                         font.recs[index].width + 2.0f*font.glyphPadding, font.recs[index].height + 2.0f*font.glyphPadding };

    float width = (float)(font.recs[index].width + 2.0f*font.glyphPadding)*scale;
    float height = (float)(font.recs[index].height + 2.0f*font.glyphPadding)*scale;

    if (font.texture.id > 0)
    {
        const float x = 0.0f;
        const float y = 0.0f;
        const float z = 0.0f;

        // normalized texture coordinates of the glyph inside the font texture (0.0f -> 1.0f)
        const float tx = srcRec.x/font.texture.width;
        const float ty = srcRec.y/font.texture.height;
        const float tw = (srcRec.x+srcRec.width)/font.texture.width;
        const float th = (srcRec.y+srcRec.height)/font.texture.height;

        rlCheckRenderBatchLimit(4 + 4*backface);
        rlSetTexture(font.texture.id);

        rlPushMatrix();
            rlTranslatef(position.x, position.y, position.z);

            rlBegin(RL_QUADS);
                rlColor4ub(tint.r, tint.g, tint.b, tint.a);

                // Front Face
                rlNormal3f(0.0f, 1.0f, 0.0f);                                   // Normal Pointing Up
                rlTexCoord2f(tx, ty); rlVertex3f(x,         y, z);              // Top Left Of The Texture and Quad
                rlTexCoord2f(tx, th); rlVertex3f(x,         y, z + height);     // Bottom Left Of The Texture and Quad
                rlTexCoord2f(tw, th); rlVertex3f(x + width, y, z + height);     // Bottom Right Of The Texture and Quad
                rlTexCoord2f(tw, ty); rlVertex3f(x + width, y, z);              // Top Right Of The Texture and Quad

                if (backface)
                {
                    // Back Face
                    rlNormal3f(0.0f, -1.0f, 0.0f);                              // Normal Pointing Down
                    rlTexCoord2f(tx, ty); rlVertex3f(x,         y, z);          // Top Right Of The Texture and Quad
                    rlTexCoord2f(tw, ty); rlVertex3f(x + width, y, z);          // Top Left Of The Texture and Quad
                    rlTexCoord2f(tw, th); rlVertex3f(x + width, y, z + height); // Bottom Left Of The Texture and Quad
                    rlTexCoord2f(tx, th); rlVertex3f(x,         y, z + height); // Bottom Right Of The Texture and Quad
                }
            rlEnd();
        rlPopMatrix();

        rlSetTexture(0);
    }
}
static void DrawText3D(Font font, const char *text, Vector3 position, float fontSize, float fontSpacing, float lineSpacing, bool backface, Color tint) {
    int length = TextLength(text);          // Total length in bytes of the text, scanned by codepoints in loop

    float textOffsetY = 0.0f;               // Offset between lines (on line break '\n')
    float textOffsetX = 0.0f;               // Offset X to next character to draw

    float scale = fontSize/(float)font.baseSize;

    for (int i = 0; i < length;)
    {
        // Get next codepoint from byte string and glyph index in font
        int codepointByteCount = 0;
        int codepoint = GetCodepoint(&text[i], &codepointByteCount);
        int index = GetGlyphIndex(font, codepoint);

        // NOTE: Normally we exit the decoding sequence as soon as a bad byte is found (and return 0x3f)
        // but we need to draw all of the bad bytes using the '?' symbol moving one byte
        if (codepoint == 0x3f) codepointByteCount = 1;

        if (codepoint == '\n')
        {
            // NOTE: Fixed line spacing of 1.5 line-height
            // TODO: Support custom line spacing defined by user
            textOffsetY += fontSize + lineSpacing;
            textOffsetX = 0.0f;
        }
        else
        {
            if ((codepoint != ' ') && (codepoint != '\t'))
            {
                DrawTextCodepoint3D(font, codepoint, (Vector3){ position.x + textOffsetX, position.y, position.z + textOffsetY }, fontSize, backface, tint);
            }

            if (font.glyphs[index].advanceX == 0) textOffsetX += (float)font.recs[index].width*scale + fontSpacing;
            else textOffsetX += (float)font.glyphs[index].advanceX*scale + fontSpacing;
        }

        i += codepointByteCount;   // Move text bytes counter to next codepoint
    }
}
