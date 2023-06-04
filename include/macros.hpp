#pragma once

#include <cstdio>
#include <unistd.h>


#include "escape_codes.hpp"


#ifndef NDEBUG
#define DEBUG_LOG(out, format, ...)                                            \
  /* The do {...} while(0) structure is a common idiom used in macros. */      \
  /* It allows the macro to be used in all contexts that a normal function     \
   * call could be used. */                                                    \
  /* It creates a compound statement in C/C++ that behaves as a single         \
   * statement. */                                                             \
  do {                                                                         \
    bool is_tty = isatty(fileno(out));                                         \
    if (is_tty) {                                                              \
      std::fprintf(out, "%s%s%s:%s%s%s:%d: ", escape_codes::colors::fg::cyan, __FILE__,         \
              escape_codes::reset, escape_codes::colors::fg::yellow, __func__,             \
              escape_codes::reset, __LINE__);                                  \
    } else {                                                                   \
      std::fprintf(out, "%s:%s:%d: ", __FILE__, __func__, __LINE__);                \
    }                                                                          \
    std::fprintf(out, format, ##__VA_ARGS__);                                       \
    std::fprintf(out, "\n");                                                        \
  } while (0)
#else
// do nothing
#define DEBUG_LOG(out, format, ...)
#endif