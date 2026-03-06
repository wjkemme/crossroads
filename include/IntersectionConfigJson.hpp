#pragma once

#include <string>
#include <vector>

#include "IntersectionConfig.hpp"

namespace crossroads {
    struct ConfigParseResult {
        bool ok = false;
        IntersectionConfig config{};
        std::vector<std::string> errors;
    };

    std::string intersectionConfigToJson(const IntersectionConfig& config);
    ConfigParseResult intersectionConfigFromJson(const std::string& json_text);
    std::string validationErrorsToJson(const std::vector<std::string>& errors);
}  // namespace crossroads
