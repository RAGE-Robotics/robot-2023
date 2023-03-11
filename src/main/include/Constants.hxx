#pragma once

#include <wpi/array.h>
#include <string>

#include <frc/trajectory/Trajectory.h>

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

    static constexpr double kDrivetrainP = 0.23;
    static constexpr double kDrivetrainI = 0.0;
    static constexpr double kDrivetrainD = 0;
    static constexpr double kDrivetrainF = 0;
    static const int kDrivetrainEncoderUnitsPerRotation = 2048;
    static constexpr double kDrivetrainWheelRadius = 0.0762;
    static constexpr double kDrivetrainGearRatio = 1/16.364;
    static constexpr double kRaiserP = 5.5;

    static constexpr double kGroundRaisePosition = 0.53; //old 0.55
    static constexpr double kTopRaisePosition = 0.665;
    static constexpr double kScoreLowRaisePosition = 0.65;
    static constexpr double kScoreHighRaisePosition = 0.676;
    static constexpr double kPickupRaisePosition = 0.5;

    static constexpr double kGroundExtendPosition = 0.26;
    static constexpr double kTopExtendPosition = 0;
    static constexpr double kScoreLowExtendPosition = 0.2;
    static constexpr double kScoreHighExtendPosition = 0.415;
    static constexpr double kPickupExtendPosition = 0.26;

    static constexpr frc::Translation2d kLeftRightStartingPosition = {18.23_ft, 0_ft};

    static const int kTurretEncoderUnitsPerRotation = 8192;
    static const int kTurretEncoderTicksPerRotation = 87040;
    static const int kTurretEncoderTicksPerRadian = 87040 / (2 * kPi);

    static constexpr double kArmEncoderTicksPerMeter = 8192.0 / (2 * kPi * 0.0254);
};
