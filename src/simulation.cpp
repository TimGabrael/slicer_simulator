#include "simulation.h"

struct NozzleSimData {
    const Nozzle* nozzle = nullptr;
    glm::vec2 pos;

    float cur_temp;
    float cur_speed;
    glm::vec2 cur_size;
    glm::vec2 cur_local_pos;
    float travel;
    uint32_t cur_line_idx;
    float cur_line_percentile;

    float temp_time;
    float speed_time;
    float size_time;
    float local_path_time;

    bool finished;

    void UpdateSimulationVariables(float dt, float inv_integration_substeps) {
        const float dt_temp = this->nozzle->temp_time_scale * dt;
        const float dt_speed = this->nozzle->speed_time_scale * dt;
        const float dt_size = this->nozzle->size_time_scale * dt;
        const float dt_path = this->nozzle->local_path_time_scale * dt;

        const float prev_speed_time = this->temp_time;

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
        this->travel = this->nozzle->speed.Integrate(prev_speed_time, this->speed_time, dt_speed * inv_integration_substeps);
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


// px, py, radius in pixel
static void MaterialAddHeatAtSpot(SimulationSurface& surface, const MaterialConstants& material, float center_x, float center_y, float radius, float temperature) {
    const float inv_radius_2 = 1.0f / (radius * radius);
    auto heating_at_point = [temperature, inv_radius_2, center_x, center_y](float x, float y) {
        const float dx = (x - center_x);
        const float dy = (y - center_y);
        const float r = std::sqrtf(dx * dx + dy * dy);
        return temperature * std::expf(-2.0f * r * inv_radius_2);
    };

}
HotSpotData Sim_CalculateTemperatureGradient(const InfillData& info, const std::vector<Nozzle>& nozzles, const MaterialConstants& material, uint32_t resolution_x, float time_step) {
    static constexpr float INTEGRATION_SUBSTEPS = 10.0f;
    static constexpr float INV_INTEGRATION_SUBSTEPS = 1.0f / INTEGRATION_SUBSTEPS;
    HotSpotData output {};
    const glm::vec2 extend = info.bounds.max - info.bounds.min;
    const float aspect_ratio = extend.y / extend.x;
    const uint32_t resolution_y = resolution_x * aspect_ratio;
    output.width = resolution_x;
    output.height = resolution_y;
    output.max = -FLT_MAX;
    output.min = FLT_MAX;

    output.data = new float[output.width * output.height];
    memset(output.data, 0, sizeof(float) * output.width * output.height);

    if(nozzles.empty() || info.lines.empty()) {
        return output;
    }

    SimulationSurface surface = {};
    surface.data = new float[output.width * output.height];
    surface.width = output.width;
    surface.height = output.height;
    surface.bounds = info.bounds;
    surface.extend = surface.bounds.max - surface.bounds.min;
    surface.pixel_size = glm::vec2(surface.extend.x / surface.width, surface.extend.y / surface.height);
    memset(surface.data, 0, sizeof(float) * output.width * output.height);


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
        nozzle_sim.UpdateSimulationVariables(0.0f, INV_INTEGRATION_SUBSTEPS);
    }

    float cur_time = 0.0f;
    bool finished = false;
    while(finished) {
        bool all_finished = false;
        for(auto& nozzle : nozzles_sim_data) {
            nozzle.UpdateSimulationVariables(time_step, INV_INTEGRATION_SUBSTEPS);

            while(true) {
                if(info.lines.size() < nozzle.cur_line_idx) {
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
                    }
                }
            }


            if(nozzle.finished) {
                all_finished = true;
            }
        }

        cur_time += time_step;
    }

    return output;
}
void Sim_DestroyHotSpotData(HotSpotData& data) {
    if(data.data) {
        delete[] data.data;
    }
    data.data = nullptr;
    data.width = 0;
    data.height = 0;
}
