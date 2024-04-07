#pragma once
#include <libhal-util/map.hpp>

namespace tla {

    float voltage_to_degree(float value, float max_voltage, float max_degree)
    {
        return hal::map(value,
                        std::pair<float, float>(0.0, max_voltage),
                        std::pair<float, float>(0.0, max_degree));
    }
}