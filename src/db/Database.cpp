#include "Database.hpp"

#ifdef CROSSROADS_USE_SQLITE
#include <sqlite3.h>
#else
#include <fstream>
#endif

#include <utility>

namespace crossroads::db
{
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
} // namespace crossroads::db
