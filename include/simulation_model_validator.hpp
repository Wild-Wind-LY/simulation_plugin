#pragma once

#include <nlohmann/json.hpp>
#include <string>

class SimulationModelValidator {
public:
  nlohmann::json validate(const nlohmann::json& data) const;
};
