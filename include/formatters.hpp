#pragma once

#include <cstdio>
#include <fstream>

#include "escape_codes.hpp"

auto format_bool(bool b) -> std::string {
    if (b) {
        return escape_codes::colors::fg::green + std::string("true") + escape_codes::reset;
    } else {
        return escape_codes::colors::fg::red + std::string("false") + escape_codes::reset;
    }
}

auto format_filepath(const std::string& filepath) -> std::string {
    // If the file exists and is a regular file, print the filepath in green.
    // Otherwise, print it in red.
    auto f = std::ifstream(filepath);
    const bool is_regular_file = f.good() && f.is_open();
    f.close();
    if (is_regular_file) {
        return escape_codes::bold + filepath + escape_codes::reset;
    } else {
        return escape_codes::strike + filepath + escape_codes::reset;
    }    
}