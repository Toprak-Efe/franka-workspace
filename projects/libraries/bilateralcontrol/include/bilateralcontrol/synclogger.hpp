#pragma once

#include <array>
#include <chrono>
#include <format>
#include <fstream>
#include <iostream>
#include <source_location>
#include <string_view>
#include <utility>

namespace Asclepius {

enum class Level : std::uint8_t {
    Info,
    Debug,
    Error,
};

namespace detail {

struct LevelMeta {
    std::string_view label;
    std::string_view color;
};

inline constexpr auto reset = "\033[0m";

inline constexpr std::array<LevelMeta, 3> level_meta {{
    { "INFO",  "\033[32m" },
    { "DEBUG", "\033[34m" },
    { "ERROR", "\033[31m" },
}};

consteval const LevelMeta& meta_for(Level lv) {
    return level_meta[std::to_underlying(lv)];
}

// Captures source_location in its constructor *before* the
// variadic pack is deduced, sidestepping the default-arg conflict.
struct FmtLoc {
    std::string_view fmt;
    std::source_location loc;

    template <std::size_t N>
    consteval FmtLoc(const char (&s)[N],
                     std::source_location l = std::source_location::current())
        : fmt(s), loc(l) {}
};

} // namespace detail

class SyncLogger {
public:
    static SyncLogger& get() {
        static SyncLogger instance;
        return instance;
    }

    ~SyncLogger() = default;

    SyncLogger(const SyncLogger&)            = delete;
    SyncLogger(SyncLogger&&)                 = delete;
    SyncLogger& operator=(const SyncLogger&) = delete;
    SyncLogger& operator=(SyncLogger&&)      = delete;

    template <typename... Args>
    void info(detail::FmtLoc fl, Args&&... args) {
        log_impl<Level::Info>(fl, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(detail::FmtLoc fl, Args&&... args) {
        log_impl<Level::Debug>(fl, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(detail::FmtLoc fl, Args&&... args) {
        log_impl<Level::Error>(fl, std::forward<Args>(args)...);
    }

private:
    SyncLogger() : m_log_file("asclepius.log", std::ios::app) {}

    std::ofstream m_log_file;

    template <Level Lv, typename... Args>
    void log_impl(detail::FmtLoc fl, Args&&... args) {
        constexpr auto& meta = detail::meta_for(Lv);

        auto now   = std::chrono::current_zone()->to_local(std::chrono::system_clock::now());
        auto stamp = std::format("{:%Y-%m-%d %H:%M:%S}", std::chrono::floor<std::chrono::milliseconds>(now));
        auto body  = std::vformat(fl.fmt, std::make_format_args(args...));

        std::string entry;
        if constexpr (Lv == Level::Debug) {
            entry = std::format("[{}] {}[{}]:{} {} ({}:{} `{}`)\n",
                stamp, meta.color, meta.label, detail::reset, body,
                fl.loc.file_name(), fl.loc.line(), fl.loc.function_name());
        } else {
            entry = std::format("[{}] {}[{}]:{} {}\n",
                stamp, meta.color, meta.label, detail::reset, body);
        }

        std::cout << entry;
        m_log_file << entry;
        m_log_file.flush();
    }
};

} // namespace Asclepius

