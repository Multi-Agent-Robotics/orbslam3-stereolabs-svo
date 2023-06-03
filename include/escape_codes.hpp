#pragma once

namespace escape_codes {
namespace colors {
namespace fg {
constexpr auto black = "\033[30m";
constexpr auto red = "\033[31m";
constexpr auto green = "\033[32m";
constexpr auto yellow = "\033[33m";
constexpr auto blue = "\033[34m";
constexpr auto magenta = "\033[35m";
constexpr auto cyan = "\033[36m";
constexpr auto white = "\033[37m";
} // namespace fg
namespace bg {
constexpr auto black = "\033[40m";
constexpr auto red = "\033[41m";
constexpr auto green = "\033[42m";
constexpr auto yellow = "\033[43m";
constexpr auto blue = "\033[44m";
constexpr auto magenta = "\033[45m";
constexpr auto cyan = "\033[46m";
constexpr auto white = "\033[47m";
} // namespace bg
} // namespace colors
constexpr auto reset = "\033[0m";
constexpr auto bold = "\033[1m";
constexpr auto dim = "\033[2m";
constexpr auto italic = "\033[3m";
constexpr auto underline = "\033[4m";
constexpr auto blink = "\033[5m";
constexpr auto blink_fast = "\033[6m";
constexpr auto invert = "\033[7m";
constexpr auto hide = "\033[8m";
constexpr auto strike = "\033[9m";
} // namespace escape_codes