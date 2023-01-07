#pragma once

#include <wpi/array.h>
#include <string>

class Constants
{
public:
    static constexpr double kLoopDt = 0.005;

    static constexpr double kDrivetrainWidth = 0.5;

    static constexpr double kRamseteB = 2.0;
    static constexpr double kRamseteZeta = 0.7;
    static constexpr double kRamseteToleranceX = 0.05;
    static constexpr double kRamseteTolaranceY = 0.05;

    static const int kVisionDataPort = 5800;
    static const std::string kVisionIp;
};
