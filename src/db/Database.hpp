#pragma once

#include <optional>
#include <string>

namespace crossroads::db
{

    class Database
    {
    public:
        explicit Database(std::string file_path);

        bool initialize(std::string *error = nullptr) const;
        bool saveActiveIntersectionConfigJson(const std::string &config_json, std::string *error = nullptr) const;
        std::optional<std::string> loadActiveIntersectionConfigJson(std::string *error = nullptr) const;

    private:
        std::string file_path;
    };

} // namespace crossroads::db
