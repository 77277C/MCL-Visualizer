#pragma once

constexpr float MM_TO_IN = 1 / 25.4;
constexpr float M_TO_IN = 39.3701;

// Value from alexDickhans/echo on GitHub
constexpr float WALL = 1.78308 * M_TO_IN;

constexpr float STARTING_POS_VARIANCE = 1.0;
constexpr float STARTING_ANGLE_VARIANCE = 5 * M_PI / 180;
constexpr float DRIVE_NOISE = 0.05;
constexpr float ANGLE_NOISE = 3 * M_PI / 180;
