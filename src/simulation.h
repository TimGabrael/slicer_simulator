#pragma once
#include "util.h"


struct Nozzle {
    Path2D local_path;
    Path2D size;

    Path1D temperature;
    Path1D speed;

    float temp_time_scale;
    float speed_time_scale;
    float size_time_scale;
    float local_path_time_scale;
};
struct MaterialConstants {
    float melting_point;
    float thermal_conductvity;
    float heat_capacity;
    float thermal_expansion;
};

struct HotSpotData {
    float* data; 
    uint32_t width;
    uint32_t height;
    float min;
    float max;
};

HotSpotData Sim_CalculateHotspots(const InfillData& info, const std::vector<Nozzle>& nozzles, const MaterialConstants& material, uint32_t resolution_x, float time_step);
void Sim_DestroyHotSpotData(HotSpotData& data);

