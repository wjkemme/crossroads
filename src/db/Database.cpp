#include "Database.hpp"

#ifdef CROSSROADS_USE_SQLITE
#include <sqlite3.h>
#else
#include <fstream>
#include <nlohmann/json.hpp>
#endif

#include <algorithm>
#include <ctime>
#include <vector>
#include <utility>

namespace crossroads::db
{
#ifndef CROSSROADS_USE_SQLITE
    namespace
    {
        using nlohmann::json;

        struct FallbackNamedConfigRow
        {
            std::string name;
            std::string config_json;
            std::string updated_at;
            std::string last_used_at;
        };

        std::string fallbackNamedConfigPath(const std::string &base_path)
        {
            return base_path + ".named.json";
        }

        std::string nowUtcTimestamp()
        {
            std::time_t now = std::time(nullptr);
            std::tm tm_utc{};
#ifdef _WIN32
            gmtime_s(&tm_utc, &now);
#else
            gmtime_r(&now, &tm_utc);
#endif
            char buffer[20] = {};
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_utc);
            return std::string(buffer);
        }

        bool readFallbackNamedConfigRows(const std::string &base_path,
                                         std::vector<FallbackNamedConfigRow> &rows,
                                         std::string *error)
        {
            rows.clear();
            std::ifstream in(fallbackNamedConfigPath(base_path));
            if (!in.good())
            {
                return true;
            }

            json root;
            try
            {
                in >> root;
            }
            catch (const std::exception &ex)
            {
                if (error)
                {
                    *error = std::string("failed to parse fallback named config storage: ") + ex.what();
                }
                return false;
            }

            if (!root.is_object() || !root.contains("items") || !root["items"].is_array())
            {
                return true;
            }

            for (const auto &item : root["items"])
            {
                if (!item.is_object())
                {
                    continue;
                }

                const std::string name = item.value("name", "");
                if (name.empty())
                {
                    continue;
                }

                rows.push_back(FallbackNamedConfigRow{
                    name,
                    item.value("config_json", ""),
                    item.value("updated_at", ""),
                    item.value("last_used_at", "")});
            }
            return true;
        }

        bool writeFallbackNamedConfigRows(const std::string &base_path,
                                          const std::vector<FallbackNamedConfigRow> &rows,
                                          std::string *error)
        {
            json root;
            root["items"] = json::array();
            for (const auto &row : rows)
            {
                root["items"].push_back({{"name", row.name},
                                         {"config_json", row.config_json},
                                         {"updated_at", row.updated_at},
                                         {"last_used_at", row.last_used_at}});
            }

            std::ofstream out(fallbackNamedConfigPath(base_path), std::ios::trunc);
            if (!out.good())
            {
                if (error)
                {
                    *error = "failed to write fallback named config storage file";
                }
                return false;
            }
            out << root.dump();
            return out.good();
        }
    } // namespace
#endif

    Database::Database(std::string file_path)
        : file_path(std::move(file_path))
    {
    }

    bool Database::initialize(std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        const char *create_sql =
            "CREATE TABLE IF NOT EXISTS app_config ("
            "key TEXT PRIMARY KEY,"
            "value TEXT NOT NULL"
            ");";

        const char *create_named_sql =
            "CREATE TABLE IF NOT EXISTS named_intersection_configs ("
            "name TEXT PRIMARY KEY,"
            "config_json TEXT NOT NULL,"
            "updated_at TEXT NOT NULL DEFAULT (datetime('now')),"
            "last_used_at TEXT"
            ");";

        char *errmsg = nullptr;
        const int exec_rc = sqlite3_exec(handle, create_sql, nullptr, nullptr, &errmsg);
        if (exec_rc != SQLITE_OK)
        {
            if (error)
            {
                *error = errmsg ? errmsg : "failed to initialize schema";
            }
            sqlite3_free(errmsg);
            sqlite3_close(handle);
            return false;
        }

        const int exec_named_rc = sqlite3_exec(handle, create_named_sql, nullptr, nullptr, &errmsg);
        if (exec_named_rc != SQLITE_OK)
        {
            if (error)
            {
                *error = errmsg ? errmsg : "failed to initialize named config schema";
            }
            sqlite3_free(errmsg);
            sqlite3_close(handle);
            return false;
        }

        const char *seed_sql =
            "INSERT INTO named_intersection_configs(name, config_json, updated_at, last_used_at) "
            "SELECT 'Standaard', value, datetime('now'), datetime('now') FROM app_config "
            "WHERE key='active_intersection_config' "
            "AND NOT EXISTS (SELECT 1 FROM named_intersection_configs);";
        const int seed_rc = sqlite3_exec(handle, seed_sql, nullptr, nullptr, &errmsg);
        if (seed_rc != SQLITE_OK)
        {
            if (error)
            {
                *error = errmsg ? errmsg : "failed to seed named configs";
            }
            sqlite3_free(errmsg);
            sqlite3_close(handle);
            return false;
        }

        sqlite3_close(handle);
        return true;
#else
        std::ofstream out(file_path, std::ios::app);
        if (!out.good())
        {
            if (error)
            {
                *error = "failed to open fallback storage file";
            }
            return false;
        }

        std::vector<FallbackNamedConfigRow> rows;
        if (!readFallbackNamedConfigRows(file_path, rows, error))
        {
            return false;
        }
        if (!writeFallbackNamedConfigRows(file_path, rows, error))
        {
            return false;
        }
        return true;
#endif
    }

    bool Database::saveActiveIntersectionConfigJson(const std::string &config_json, std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        const char *upsert_sql =
            "INSERT INTO app_config(key, value) VALUES('active_intersection_config', ?) "
            "ON CONFLICT(key) DO UPDATE SET value = excluded.value;";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, upsert_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_bind_text(stmt, 1, config_json.c_str(), static_cast<int>(config_json.size()), SQLITE_TRANSIENT);

        const int step_rc = sqlite3_step(stmt);
        sqlite3_finalize(stmt);
        if (step_rc != SQLITE_DONE)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_close(handle);
        return true;
#else
        std::ofstream out(file_path, std::ios::trunc);
        if (!out.good())
        {
            if (error)
            {
                *error = "failed to write fallback storage file";
            }
            return false;
        }
        out << config_json;
        return out.good();
#endif
    }

    std::optional<std::string> Database::loadActiveIntersectionConfigJson(std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return std::nullopt;
        }

        const char *select_sql = "SELECT value FROM app_config WHERE key = 'active_intersection_config' LIMIT 1;";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, select_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return std::nullopt;
        }

        const int step_rc = sqlite3_step(stmt);
        if (step_rc == SQLITE_ROW)
        {
            const unsigned char *text = sqlite3_column_text(stmt, 0);
            std::string value = text ? reinterpret_cast<const char *>(text) : "";
            sqlite3_finalize(stmt);
            sqlite3_close(handle);
            return value;
        }

        if (step_rc != SQLITE_DONE && error)
        {
            *error = sqlite3_errmsg(handle);
        }

        sqlite3_finalize(stmt);
        sqlite3_close(handle);
        return std::nullopt;
#else
        std::ifstream in(file_path);
        if (!in.good())
        {
            return std::nullopt;
        }

        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        if (content.empty())
        {
            return std::nullopt;
        }
        return content;
#endif
    }

    bool Database::saveNamedIntersectionConfigJson(const std::string &name, const std::string &config_json, std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        const char *upsert_sql =
            "INSERT INTO named_intersection_configs(name, config_json, updated_at) VALUES(?, ?, datetime('now')) "
            "ON CONFLICT(name) DO UPDATE SET "
            "config_json = excluded.config_json, "
            "updated_at = datetime('now');";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, upsert_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_bind_text(stmt, 1, name.c_str(), static_cast<int>(name.size()), SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, config_json.c_str(), static_cast<int>(config_json.size()), SQLITE_TRANSIENT);

        const int step_rc = sqlite3_step(stmt);
        sqlite3_finalize(stmt);
        if (step_rc != SQLITE_DONE)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_close(handle);
        return true;
#else
        std::vector<FallbackNamedConfigRow> rows;
        if (!readFallbackNamedConfigRows(file_path, rows, error))
        {
            return false;
        }

        auto it = std::find_if(rows.begin(), rows.end(), [&](const FallbackNamedConfigRow &row)
                               { return row.name == name; });
        const std::string now = nowUtcTimestamp();
        if (it == rows.end())
        {
            rows.push_back(FallbackNamedConfigRow{name, config_json, now, ""});
        }
        else
        {
            it->config_json = config_json;
            it->updated_at = now;
        }

        return writeFallbackNamedConfigRows(file_path, rows, error);
#endif
    }

    std::optional<std::string> Database::loadNamedIntersectionConfigJson(const std::string &name, std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return std::nullopt;
        }

        const char *select_sql = "SELECT config_json FROM named_intersection_configs WHERE name = ? LIMIT 1;";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, select_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return std::nullopt;
        }

        sqlite3_bind_text(stmt, 1, name.c_str(), static_cast<int>(name.size()), SQLITE_TRANSIENT);
        const int step_rc = sqlite3_step(stmt);
        if (step_rc == SQLITE_ROW)
        {
            const unsigned char *text = sqlite3_column_text(stmt, 0);
            std::string value = text ? reinterpret_cast<const char *>(text) : "";
            sqlite3_finalize(stmt);
            sqlite3_close(handle);
            return value;
        }

        if (step_rc != SQLITE_DONE && error)
        {
            *error = sqlite3_errmsg(handle);
        }

        sqlite3_finalize(stmt);
        sqlite3_close(handle);
        return std::nullopt;
#else
        std::vector<FallbackNamedConfigRow> rows;
        if (!readFallbackNamedConfigRows(file_path, rows, error))
        {
            return std::nullopt;
        }

        auto it = std::find_if(rows.begin(), rows.end(), [&](const FallbackNamedConfigRow &row)
                               { return row.name == name; });
        if (it == rows.end())
        {
            return std::nullopt;
        }
        return it->config_json;
        return std::nullopt;
#endif
    }

    std::vector<NamedConfigEntry> Database::listNamedIntersectionConfigs(std::string *error) const
    {
        std::vector<NamedConfigEntry> rows;
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return rows;
        }

        const char *select_sql =
            "SELECT name, updated_at, COALESCE(last_used_at, '') "
            "FROM named_intersection_configs "
            "ORDER BY COALESCE(last_used_at, updated_at) DESC, name ASC;";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, select_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return rows;
        }

        while (true)
        {
            const int step_rc = sqlite3_step(stmt);
            if (step_rc == SQLITE_DONE)
            {
                break;
            }
            if (step_rc != SQLITE_ROW)
            {
                if (error)
                {
                    *error = sqlite3_errmsg(handle);
                }
                break;
            }

            const unsigned char *name_text = sqlite3_column_text(stmt, 0);
            const unsigned char *updated_text = sqlite3_column_text(stmt, 1);
            const unsigned char *last_used_text = sqlite3_column_text(stmt, 2);
            rows.push_back(NamedConfigEntry{
                name_text ? reinterpret_cast<const char *>(name_text) : "",
                updated_text ? reinterpret_cast<const char *>(updated_text) : "",
                last_used_text ? reinterpret_cast<const char *>(last_used_text) : ""});
        }

        sqlite3_finalize(stmt);
        sqlite3_close(handle);
#else
        std::vector<FallbackNamedConfigRow> fallback_rows;
        if (!readFallbackNamedConfigRows(file_path, fallback_rows, error))
        {
            return rows;
        }

        for (const auto &row : fallback_rows)
        {
            rows.push_back(NamedConfigEntry{row.name, row.updated_at, row.last_used_at});
        }

        std::sort(rows.begin(), rows.end(), [](const NamedConfigEntry &lhs, const NamedConfigEntry &rhs)
                  {
                      const std::string lhs_key = lhs.last_used_at.empty() ? lhs.updated_at : lhs.last_used_at;
                      const std::string rhs_key = rhs.last_used_at.empty() ? rhs.updated_at : rhs.last_used_at;
                      if (lhs_key == rhs_key)
                      {
                          return lhs.name < rhs.name;
                      }
                      return lhs_key > rhs_key; });
#endif
        return rows;
    }

    bool Database::deleteNamedIntersectionConfig(const std::string &name, std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        const char *delete_sql = "DELETE FROM named_intersection_configs WHERE name = ?;";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, delete_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_bind_text(stmt, 1, name.c_str(), static_cast<int>(name.size()), SQLITE_TRANSIENT);

        const int step_rc = sqlite3_step(stmt);
        sqlite3_finalize(stmt);
        if (step_rc != SQLITE_DONE)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_close(handle);
        return true;
#else
        std::vector<FallbackNamedConfigRow> rows;
        if (!readFallbackNamedConfigRows(file_path, rows, error))
        {
            return false;
        }

        const auto before = rows.size();
        rows.erase(std::remove_if(rows.begin(), rows.end(), [&](const FallbackNamedConfigRow &row)
                                  { return row.name == name; }),
                   rows.end());

        if (rows.size() == before)
        {
            return false;
        }

        return writeFallbackNamedConfigRows(file_path, rows, error);
#endif
    }

    bool Database::touchNamedIntersectionConfig(const std::string &name, std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        const char *update_sql = "UPDATE named_intersection_configs SET last_used_at = datetime('now') WHERE name = ?;";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, update_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_bind_text(stmt, 1, name.c_str(), static_cast<int>(name.size()), SQLITE_TRANSIENT);

        const int step_rc = sqlite3_step(stmt);
        sqlite3_finalize(stmt);
        if (step_rc != SQLITE_DONE)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return false;
        }

        sqlite3_close(handle);
        return true;
#else
        std::vector<FallbackNamedConfigRow> rows;
        if (!readFallbackNamedConfigRows(file_path, rows, error))
        {
            return false;
        }

        auto it = std::find_if(rows.begin(), rows.end(), [&](const FallbackNamedConfigRow &row)
                               { return row.name == name; });
        if (it == rows.end())
        {
            return false;
        }
        it->last_used_at = nowUtcTimestamp();
        return writeFallbackNamedConfigRows(file_path, rows, error);
#endif
    }

    std::optional<std::string> Database::loadMostRecentNamedConfigName(std::string *error) const
    {
#ifdef CROSSROADS_USE_SQLITE
        sqlite3 *handle = nullptr;
        if (sqlite3_open(file_path.c_str(), &handle) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return std::nullopt;
        }

        const char *select_sql =
            "SELECT name "
            "FROM named_intersection_configs "
            "ORDER BY COALESCE(last_used_at, updated_at) DESC, name ASC "
            "LIMIT 1;";

        sqlite3_stmt *stmt = nullptr;
        if (sqlite3_prepare_v2(handle, select_sql, -1, &stmt, nullptr) != SQLITE_OK)
        {
            if (error)
            {
                *error = sqlite3_errmsg(handle);
            }
            sqlite3_close(handle);
            return std::nullopt;
        }

        const int step_rc = sqlite3_step(stmt);
        if (step_rc == SQLITE_ROW)
        {
            const unsigned char *text = sqlite3_column_text(stmt, 0);
            std::string value = text ? reinterpret_cast<const char *>(text) : "";
            sqlite3_finalize(stmt);
            sqlite3_close(handle);
            return value;
        }

        if (step_rc != SQLITE_DONE && error)
        {
            *error = sqlite3_errmsg(handle);
        }

        sqlite3_finalize(stmt);
        sqlite3_close(handle);
        return std::nullopt;
#else
        std::vector<FallbackNamedConfigRow> rows;
        if (!readFallbackNamedConfigRows(file_path, rows, error))
        {
            return std::nullopt;
        }
        if (rows.empty())
        {
            return std::nullopt;
        }

        std::sort(rows.begin(), rows.end(), [](const FallbackNamedConfigRow &lhs, const FallbackNamedConfigRow &rhs)
                  {
                      const std::string lhs_key = lhs.last_used_at.empty() ? lhs.updated_at : lhs.last_used_at;
                      const std::string rhs_key = rhs.last_used_at.empty() ? rhs.updated_at : rhs.last_used_at;
                      if (lhs_key == rhs_key)
                      {
                          return lhs.name < rhs.name;
                      }
                      return lhs_key > rhs_key; });
        return rows.front().name;
        return std::nullopt;
#endif
    }
} // namespace crossroads::db
