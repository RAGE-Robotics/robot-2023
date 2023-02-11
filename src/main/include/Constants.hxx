#pragma once

#include <wpi/array.h>
#include <string>

class Constants
{
public:
    static constexpr double kPi = 3.141592653589793;

    static constexpr double kLoopDt = 0.005;

    static constexpr double kDrivetrainWidth = 0.5334;

    static constexpr double kRamseteB = 2.0;
    static constexpr double kRamseteZeta = 0.7;
    static constexpr double kRamseteToleranceX = 0.05;
    static constexpr double kRamseteTolaranceY = 0.05;

    static const int kVisionDataPort = 5800;
    static const std::string kVisionIp;

    static constexpr double kDrivetrainP = 1.0;
    static constexpr double kDrivetrainI = 0;
    static constexpr double kDrivetrainD = 0;
    static constexpr double kDrivetrainF = 0;

    static const int kDrivetrainEncoderUnitsPerRotation = 2048;
    static constexpr double kDrivetrainWheelRadius = 0.0762;
    static constexpr double kDrivetrainGearRatio = 0.125786163522013;
};
