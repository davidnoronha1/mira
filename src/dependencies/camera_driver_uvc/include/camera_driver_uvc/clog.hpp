#ifndef COLORLOG_HPP
#define COLORLOG_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <stacktrace>

namespace plog {

// ANSI color codes
namespace colors {
    constexpr const char* RESET = "\033[0m";
    constexpr const char* FG_RED = "\033[31m";
    constexpr const char* FG_GREEN = "\033[32m";
    constexpr const char* FG_YELLOW = "\033[33m";
    constexpr const char* FG_BLUE = "\033[34m";
    constexpr const char* FG_MAGENTA = "\033[35m";
    constexpr const char* FG_CYAN = "\033[36m";
    constexpr const char* FG_WHITE = "\033[37m";
    constexpr const char* BG_RED = "\033[41m";
    constexpr const char* BG_GREEN = "\033[42m";
    constexpr const char* BG_YELLOW = "\033[43m";
    constexpr const char* BG_BLUE = "\033[44m";
    constexpr const char* BG_MAGENTA = "\033[45m";
    constexpr const char* BG_CYAN = "\033[46m";
    constexpr const char* BG_WHITE = "\033[47m";
    constexpr const char* BOLD = "\033[1m";
    constexpr const char* DIM = "\033[2m";
}

enum class LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

// Pretty printer for various types
template<typename T>
struct PrettyPrinter {
    static void print(std::ostream& os, const T& value) {
        os << value;
    }
};

// Specialization for strings (with quotes)
template<>
struct PrettyPrinter<std::string> {
    static void print(std::ostream& os, const std::string& value) {
        os << "\"" << value << "\"";
    }
};

// Specialization for C-strings
template<>
struct PrettyPrinter<const char*> {
    static void print(std::ostream& os, const char* value) {
        os << "\"" << value << "\"";
    }
};

// Specialization for bool
template<>
struct PrettyPrinter<bool> {
    static void print(std::ostream& os, bool value) {
        os << (value ? "true" : "false");
    }
};

// Specialization for vectors
template<typename T>
struct PrettyPrinter<std::vector<T>> {
    static void print(std::ostream& os, const std::vector<T>& vec) {
        os << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            if (i > 0) os << ", ";
            PrettyPrinter<T>::print(os, vec[i]);
        }
        os << "]";
    }
};

// Specialization for maps
template<typename K, typename V>
struct PrettyPrinter<std::map<K, V>> {
    static void print(std::ostream& os, const std::map<K, V>& map) {
        os << "{";
        bool first = true;
        for (const auto& [key, value] : map) {
            if (!first) os << ", ";
            first = false;
            PrettyPrinter<K>::print(os, key);
            os << ": ";
            PrettyPrinter<V>::print(os, value);
        }
        os << "}";
    }
};

// Specialization for sets
template<typename T>
struct PrettyPrinter<std::set<T>> {
    static void print(std::ostream& os, const std::set<T>& set) {
        os << "{";
        bool first = true;
        for (const auto& item : set) {
            if (!first) os << ", ";
            first = false;
            PrettyPrinter<T>::print(os, item);
        }
        os << "}";
    }
};

// Specialization for pairs
template<typename T1, typename T2>
struct PrettyPrinter<std::pair<T1, T2>> {
    static void print(std::ostream& os, const std::pair<T1, T2>& pair) {
        os << "(";
        PrettyPrinter<T1>::print(os, pair.first);
        os << ", ";
        PrettyPrinter<T2>::print(os, pair.second);
        os << ")";
    }
};

// Custom ostream wrapper
class LogStream {
private:
    std::ostringstream buffer;
    LogLevel level;
    const char* bg_color;
    const char* fg_color;
    const char* level_str;
    bool enabled;
    std::ostream& output;

public:
    LogStream(LogLevel lvl, std::ostream& out = std::cerr) 
        : level(lvl), enabled(true), output(out) {
        
        switch (level) {
            case LogLevel::TRACE:
                bg_color = colors::BG_WHITE;
                fg_color = colors::RESET;
                level_str = "TRACE";
                break;
            case LogLevel::DEBUG:
                bg_color = colors::BG_CYAN;
                fg_color = colors::RESET;
                level_str = "DEBUG";
                break;
            case LogLevel::INFO:
                bg_color = colors::BG_BLUE;
                fg_color = colors::RESET;
                level_str = "INFO ";
                break;
            case LogLevel::WARN:
                bg_color = colors::BG_YELLOW;
                fg_color = colors::FG_YELLOW;
                level_str = "WARN ";
                break;
            case LogLevel::ERROR:
                bg_color = colors::BG_RED;
                fg_color = colors::FG_RED;
                level_str = "ERROR";
                break;
            case LogLevel::FATAL:
                bg_color = colors::BG_MAGENTA;
                fg_color = colors::FG_MAGENTA;
                level_str = "FATAL";
                break;
        }
    }

    ~LogStream() {
        if (enabled) {
            output << bg_color << colors::BOLD << " " << level_str << " " 
                   << colors::RESET << " " << fg_color
                   << buffer.str() 
                   << colors::RESET << std::endl;
        }
    }

    // Generic insertion operator
    template<typename T>
    LogStream& operator<<(const T& value) {
        PrettyPrinter<T>::print(buffer, value);
        return *this;
    }

    // Support for stream manipulators
    LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        manip(buffer);
        return *this;
    }

    void disable() { enabled = false; }
};

// Exception stream with stack trace
class ExceptionStream {
private:
    std::ostringstream buffer;
    std::ostream& output;
    bool print_trace;
    std::stacktrace trace;

public:
    ExceptionStream(std::ostream& out = std::cerr, bool enable_trace = true) 
        : output(out), print_trace(enable_trace), trace(std::stacktrace::current(1)) {}

    ~ExceptionStream() {
        output << colors::BG_RED << colors::BOLD << " EXCEPTION " 
               << colors::RESET << " " << colors::FG_RED
               << buffer.str() << colors::RESET << "\n";

        if (print_trace && !trace.empty()) {
            output << colors::FG_RED << "Stack Trace:" << colors::RESET << "\n";
            
            size_t frame_num = 0;
            for (const auto& entry : trace) {
                output << colors::DIM << "  #" << frame_num++ << " ";
                
                // Print source location if available
                if (!entry.source_file().empty()) {
                    output << entry.source_file();
                    if (entry.source_line() != 0) {
                        output << ":" << entry.source_line();
                    }
                    output << " - ";
                }
                
                // Print function description
                output << entry.description() << colors::RESET << "\n";
            }
        }
    }

    template<typename T>
    ExceptionStream& operator<<(const T& value) {
        PrettyPrinter<T>::print(buffer, value);
        return *this;
    }

    ExceptionStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        manip(buffer);
        return *this;
    }
};

// Global logging functions
inline LogStream trace() { return LogStream(LogLevel::TRACE); }
inline LogStream debug() { return LogStream(LogLevel::DEBUG); }
inline LogStream info() { return LogStream(LogLevel::INFO); }
inline LogStream warn() { return LogStream(LogLevel::WARN); }
inline LogStream error() { return LogStream(LogLevel::ERROR); }
inline LogStream fatal() { return LogStream(LogLevel::FATAL); }
inline ExceptionStream exception() { return ExceptionStream(); }

} // namespace clog

#endif // COLORLOG_HPP