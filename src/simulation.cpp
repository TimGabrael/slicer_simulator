#include "simulation.h"

struct NozzleSimData {
    const Nozzle* nozzle = nullptr;
    glm::vec2 pos;

    float cur_temp;
    float cur_speed;
    float cur_size;
    glm::vec2 cur_local_pos;
    float travel;
    uint32_t cur_line_idx;
    float cur_line_percentile;

    float temp_time;
    float speed_time;
    float size_time;
    float local_path_time;

    bool finished;

    void UpdateSimulationVariables(float dt, uint32_t integration_substeps) {
        const float dt_temp = this->nozzle->temp_time_scale * dt;
        const float dt_speed = this->nozzle->speed_time_scale * dt;
        const float dt_size = this->nozzle->size_time_scale * dt;
        const float dt_path = this->nozzle->local_path_time_scale * dt;

        const float prev_speed_time = this->speed_time;

        this->temp_time += dt_temp;
        while(this->temp_time > 1.0f) {
            this->temp_time -= 1.0f;
        }
        this->speed_time += dt_speed;
        while(this->speed_time > 1.0f) {
            this->speed_time -= 1.0f;
        }
        this->size_time += dt_size;
        while(this->size_time > 1.0f) {
            this->size_time -= 1.0f;
        }
        this->local_path_time += dt_path;
        while(this->local_path_time > 1.0f) {
            this->local_path_time -= 1.0f;
        }

        this->cur_temp = this->nozzle->temperature.Sample(this->temp_time);
        this->cur_speed = this->nozzle->speed.Sample(this->speed_time);
        this->cur_size = this->nozzle->size.Sample(this->size_time);
        this->cur_local_pos = this->nozzle->local_path.Sample(this->local_path_time);
        this->travel = this->nozzle->speed.Integrate(prev_speed_time, this->speed_time, integration_substeps);
    }
};
struct SimulationSurface {
    float* data;
    uint32_t width;
    uint32_t height;
    BBox2D bounds;
    glm::vec2 extend;
    glm::vec2 pixel_size;

};


// center_x, center_y, radius in pixel
static void MaterialAddHeatAtSpot(SimulationSurface& surface, const MaterialConstants& material, const glm::vec2& center, float radius, float temperature, float dt) {
    static constexpr uint32_t INTEGRAL_RESOLUTION = 40;
    static constexpr float INV_INTEGRAL_RESOLUTION = 1.0f / static_cast<float>(INTEGRAL_RESOLUTION);

    const float inv_radius_2 = 1.0f / (radius * radius);
    auto heating_at_point = [temperature, inv_radius_2, center](float x, float y) {
        const float dx = (x - center.x);
        const float dy = (y - center.y);
        const float r = std::sqrtf(dx * dx + dy * dy);
        return temperature * std::expf(-2.0f * r * inv_radius_2);
    };
    const glm::vec2 delta = surface.pixel_size * INV_INTEGRAL_RESOLUTION;
    auto integrate_heating = [heating_at_point, delta](float sx, float sy) {
        float sum = 0.0f;
        for(uint32_t y = 0; y < INTEGRAL_RESOLUTION; ++y) {
            for(uint32_t x = 0; x < INTEGRAL_RESOLUTION; ++x) {
                const float cx = sx + (x + 0.5f) * delta.x;
                const float cy = sy + (y + 0.5f) * delta.y;

                sum += heating_at_point(cx, cy);
            }
        }
        return sum * delta.x * delta.y;
    };

    for(uint32_t y = 0; y < surface.height; ++y) {
        for(uint32_t x = 0; x < surface.width; ++x) {
            //glm::vec2 pixel_world_pos = glm::vec2(x, y) * surface.pixel_size + surface.bounds.min;

            // the integration part seems good and is probably more accurate,
            // but it takes AGES
            //const float pixel_heating = integrate_heating(pixel_world_pos.x, pixel_world_pos.y);

            glm::vec2 pixel_world_pos = glm::vec2(x + 0.5f, y + 0.5f) * surface.pixel_size + surface.bounds.min;
            const float pixel_heating = heating_at_point(pixel_world_pos.x, pixel_world_pos.y);
            surface.data[y * surface.width + x] += material.thermal_absorptance * pixel_heating * dt;
        }
    }
}
static void MaterialDissipate(SimulationSurface& surface, float* swap_data, const MaterialConstants& material, float surrounding_temp, float dt) {
    for(uint32_t y = 0; y < surface.height; ++y) {
        for(uint32_t x = 0; x < surface.width; ++x) {
            const float T = surface.data[y * surface.width + x];
            float laplacian = -4.0f * T;
            if(x < (surface.width - 1)) {
                laplacian += surface.data[y * surface.width + (x + 1)];
            }
            else {
                laplacian += surrounding_temp;
            }
            if(x > 0) {
                laplacian += surface.data[y * surface.width + (x - 1)];
            }
            else {
                laplacian += surrounding_temp;
            }
            if(y < (surface.height - 1)) {
                laplacian += surface.data[(y + 1) * surface.width + x];
            }
            else {
                laplacian += surrounding_temp;
            }
            if(y > 0) {
                laplacian += surface.data[(y - 1) * surface.width + x];
            }
            else {
                laplacian += surrounding_temp;
            }
            const float diffusion = material.thermal_diffusivity * laplacian;
            const float cooling = material.thermal_loss_coefficient * (T - surrounding_temp);

            swap_data[y * surface.width + x] = T + dt * (diffusion - cooling);
        }
    }
    memcpy(surface.data, swap_data, sizeof(float) * surface.width * surface.height);
}
static void AccumulateHotSpots(const SimulationSurface& surface, float* hotspots) {
    for(uint32_t y = 0; y < surface.height; ++y) {
        for(uint32_t x = 0; x < surface.width; ++x) {
            const float T = surface.data[y * surface.width + x];
            hotspots[y * surface.width + x] = std::max(hotspots[y * surface.width + x], T);
        }
    }

}
HotSpotData Sim_CalculateHotspots(const InfillData& info, const std::vector<Nozzle>& nozzles, const MaterialConstants& material, const SimData& sim) {
    static constexpr float INTEGRATION_SUBSTEPS = 10.0f;
    static constexpr float INV_INTEGRATION_SUBSTEPS = 1.0f / INTEGRATION_SUBSTEPS;
    HotSpotData output {};
    const glm::vec2 extend = sim.bounds.max - sim.bounds.min;
    const float aspect_ratio = extend.y / extend.x;
    const uint32_t resolution_y = sim.resolution_x * aspect_ratio;
    output.width = sim.resolution_x;
    output.height = resolution_y;
    output.max = -FLT_MAX;
    output.min = FLT_MAX;

    output.temp = new float[output.width * output.height];
    memset(output.temp, 0, sizeof(float) * output.width * output.height);

    if(nozzles.empty() || info.lines.empty()) {
        return output;
    }

    SimulationSurface surface = {};
    surface.data = new float[output.width * output.height];
    surface.width = output.width;
    surface.height = output.height;
    surface.bounds = sim.bounds;
    surface.extend = surface.bounds.max - surface.bounds.min;
    surface.pixel_size = glm::vec2(surface.extend.x / surface.width, surface.extend.y / surface.height);
    memset(surface.data, 0, sizeof(float) * output.width * output.height);
    float* swap_data = new float[output.width * output.height];


    std::vector<NozzleSimData> nozzles_sim_data(nozzles.size());
    for(uint32_t i = 0; i < nozzles_sim_data.size(); ++i) {
        NozzleSimData& nozzle_sim = nozzles_sim_data.at(i);
        nozzle_sim.nozzle = &nozzles.at(i);
        const glm::vec2 start_local_pos = nozzle_sim.nozzle->local_path.Sample(0.0f);
        nozzle_sim.pos = info.lines.begin()->p1 + start_local_pos;
        nozzle_sim.temp_time = 0.0f;
        nozzle_sim.speed_time = 0.0f;
        nozzle_sim.size_time = 0.0f;
        nozzle_sim.local_path_time = 0.0f;
        nozzle_sim.cur_line_idx = 0;
        nozzle_sim.cur_line_percentile = 0.0f;

        nozzle_sim.finished = false;
        nozzle_sim.UpdateSimulationVariables(0.0f, INTEGRATION_SUBSTEPS);
    }


    float cur_time = 0.0f;
    bool all_finished = false;
    while(!all_finished) {
        all_finished = true;
        for(auto& nozzle : nozzles_sim_data) {
            nozzle.UpdateSimulationVariables(sim.time_step, INTEGRATION_SUBSTEPS);

            while(nozzle.travel > 0.0f) {
                if(info.lines.size() <= nozzle.cur_line_idx) {
                    nozzle.finished = true;
                    break;
                }
                else {
                    const glm::vec2 p1 = info.lines.at(nozzle.cur_line_idx).p1;
                    const glm::vec2 p2 = info.lines.at(nozzle.cur_line_idx).p2;
                    const glm::vec2 dir = (p2 - p1);
                    const float len = glm::length(dir);
                    const float moved_percentile = nozzle.travel / len;
                    if((nozzle.cur_line_percentile + moved_percentile) > 1.0f) {
                        nozzle.cur_line_idx += 1;
                        nozzle.cur_line_percentile = 0.0f;
                        const float remaining_len = (1.0f - nozzle.cur_line_percentile) * len;
                        nozzle.travel -= remaining_len;
                        continue;
                    }
                    nozzle.cur_line_percentile += moved_percentile;
                    nozzle.pos = dir * nozzle.cur_line_percentile + p1 + nozzle.cur_local_pos;
                    nozzle.travel = 0.0f;
                }
            }
            if(!nozzle.finished) {
                MaterialAddHeatAtSpot(surface, material, nozzle.pos, nozzle.cur_size, nozzle.cur_temp, sim.time_step);
                all_finished = false;
            }
        }

        MaterialDissipate(surface, swap_data, material, 0.0f, sim.time_step);
        AccumulateHotSpots(surface, output.temp);

        cur_time += sim.time_step;
    }

    // output final state of the simulation
    // for(size_t i = 0; i < output.width * output.height; ++i) {
    //     output.data[i] = surface.data[i];
    //     output.min = std::min(surface.data[i], output.min);
    //     output.max = std::max(surface.data[i], output.max);
    // }
    for(size_t i = 0; i < output.width * output.height; ++i) {
        output.min = std::min(output.temp[i], output.min);
        output.max = std::max(output.temp[i], output.max);
    }
    output.last_state = surface.data;
    delete[] swap_data;
    return output;
}
void Sim_DestroyHotSpotData(HotSpotData& data) {
    if(data.temp) {
        delete[] data.temp;
    }
    data.temp = nullptr;
    data.width = 0;
    data.height = 0;
}
