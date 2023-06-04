#!/usr/bin/env fish

for f in src/**/*.cpp src/**/*.h
    clang-format -i -style=file $f
end
