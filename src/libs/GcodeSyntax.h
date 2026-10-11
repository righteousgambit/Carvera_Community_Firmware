#pragma once

#include <string>
#include <algorithm>

namespace gcode_syntax {

enum Result { OK, MALFORMED_COMMENT, DECIMAL_DISTANCE_MODE };

inline bool space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v';
}

inline bool number(const std::string& line, size_t start, size_t& end, unsigned& value, bool& negative) {
    end = start + 1;
    while (end < line.size() && space(line[end])) ++end;
    negative = end < line.size() && line[end] == '-';
    if (end < line.size() && (line[end] == '+' || line[end] == '-')) ++end;
    const size_t digits = end;
    value = 0;
    while (end < line.size() && line[end] >= '0' && line[end] <= '9') {
        value = value < 10000 ? value * 10 + unsigned(line[end] - '0') : 10000;
        ++end;
    }
    return end != digits;
}

inline Result command_prefix(const std::string& line, size_t& first) {
    first = 0;
    bool line_number = false;
    while (first < line.size()) {
        if (space(line[first])) { ++first; continue; }
        if (line[first] == ';') { first = line.size(); return OK; }
        if (line[first] == '(') {
            ++first;
            while (first < line.size() && line[first] != ')') {
                if (line[first] == '(') return MALFORMED_COMMENT;
                ++first;
            }
            if (first == line.size()) return MALFORMED_COMMENT;
            ++first;
            continue;
        }
        if (line[first] == 'N' && !line_number) {
            first = line.find_first_not_of("N0123456789.,- ", first);
            if (first == std::string::npos) { first = line.size(); return OK; }
            line_number = true;
            continue;
        }
        break;
    }
    return OK;
}

inline bool opaque_at(const std::string& line, size_t start, bool grbl) {
    size_t end;
    if (start >= line.size() || line[start] != 'M') return false;
    unsigned code;
    bool negative;
    if (!number(line, start, end, code, negative) || negative) return false;
    switch (code) {
        case 23: case 28: case 32: case 117: case 118: case 501: case 504: case 576: case 1000:
            return true;
        case 30: return !grbl;
        case 98:
            if (end < line.size() && line[end] == '.') {
                unsigned subcode;
                return number(line, end, end, subcode, negative) && !negative && subcode == 1;
            }
            return false;
        default: return false;
    }
}

inline bool opaque_command(const std::string& line, bool grbl = true) {
    size_t start;
    return command_prefix(line, start) == OK && opaque_at(line, start, grbl);
}

// Comments leave whitespace so their removal cannot join separate words.
inline Result prepare(std::string& line, bool grbl = true) {
    size_t text_start;
    const Result prefix_result = command_prefix(line, text_start);
    if (prefix_result != OK) return prefix_result;
    // Text and file handlers own their complete payload, including punctuation.
    if (opaque_command(line, grbl)) {
        line.erase(0, text_start);
        return OK;
    }
    size_t written = 0, opaque_start = std::string::npos;
    bool comment = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '(') {
            if (comment) return MALFORMED_COMMENT;
            comment = true;
            line[written++] = ' ';
        } else if (c == ')') {
            if (!comment) return MALFORMED_COMMENT;
            comment = false;
        } else if (!comment) {
            if (c == ';') break;
            if (c == 'M' && opaque_at(line, i, grbl)) {
                opaque_start = written;
                for (; i < line.size(); ++i) line[written++] = line[i];
                break;
            }
            line[written++] = c;
        }
    }
    if (comment) return MALFORMED_COMMENT;
    line.resize(written);
    const size_t first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return OK;
    if (line[first] == 'N') {
        const size_t end = line.find_first_not_of("N0123456789.,- ", first);
        const size_t erased = end == std::string::npos ? line.size() : end;
        line.erase(0, erased);
        if (opaque_start != std::string::npos) opaque_start -= erased;
    }
    const size_t command = line.find_first_not_of(" \t\r\n");
    if (command == std::string::npos) return OK;
    // Axis-only lines are expanded to a motion command by the dispatcher first.
    if (line[command] != 'G' && line[command] != 'M' && line[command] != 'T' &&
        line[command] != 'S' && line[command] != '#') return OK;
    size_t prefix = 0;
    const size_t executable_end = opaque_start == std::string::npos ? line.size() : opaque_start;
    for (size_t i = 0; i < executable_end;) {
        if (line[i] != 'G') { ++i; continue; }
        size_t end;
        unsigned value;
        bool negative;
        if (!number(line, i, end, value, negative) || negative || (value != 90 && value != 91)) { i = end; continue; }
        if (end < line.size() && line[end] == '.') return DECIMAL_DISTANCE_MODE;
        // Axis distance mode applies to all motion words in this block.
        std::rotate(line.begin() + prefix, line.begin() + i, line.begin() + end);
        prefix += end - i;
        i = end;
    }
    return OK;
}

inline Result prepare_dispatch(std::string& line, bool grbl = true) {
    const size_t first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return OK;
    // SimpleShell owns lowercase commands and $ commands, including their text.
    if (line[first] == '$' || (line[first] >= 'a' && line[first] <= 'z')) return OK;
    return prepare(line, grbl);
}

inline bool has_decimal_subcode(const char* number_end) {
    return number_end && *number_end == '.';
}

} // namespace gcode_syntax
