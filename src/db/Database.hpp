#pragma once

#include <optional>
#include <string>
#include <vector>

namespace crossroads::db
{

    struct NamedConfigEntry
    {
        std::string name;
        std::string updated_at;
        std::string last_used_at;
    };

    class Database
    {
    public:
        explicit Database(std::string file_path);

        bool initialize(std::string *error = nullptr) const;
        bool saveActiveIntersectionConfigJson(const std::string &config_json, std::string *error = nullptr) const;
        std::optional<std::string> loadActiveIntersectionConfigJson(std::string *error = nullptr) const;
        bool saveNamedIntersectionConfigJson(const std::string &name, const std::string &config_json, std::string *error = nullptr) const;
        std::optional<std::string> loadNamedIntersectionConfigJson(const std::string &name, std::string *error = nullptr) const;
        std::vector<NamedConfigEntry> listNamedIntersectionConfigs(std::string *error = nullptr) const;
        bool deleteNamedIntersectionConfig(const std::string &name, std::string *error = nullptr) const;
        bool touchNamedIntersectionConfig(const std::string &name, std::string *error = nullptr) const;
        std::optional<std::string> loadMostRecentNamedConfigName(std::string *error = nullptr) const;

    private:
        std::string file_path;
    };

} // namespace crossroads::db
