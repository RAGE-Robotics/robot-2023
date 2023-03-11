#pragma once

#include <frc/TimedRobot.h>
#include <memory>
#include <vector>

#include "lib173/Looper.hxx"
#include "RageVision.hxx"
#include "System.hxx"
#include "Controllers.hxx"
#include "Arm.hxx"
#include "Turret.hxx"
#include "Claw.hxx"
#include "LEDs.hxx"
#include "AHRS.h"
#include "RAGETrajectory.hxx"
#include <frc/Compressor.h>

#include "RAGETrajectory.hxx"

class Robot : public frc::TimedRobot
{
private:
    Looper mLooper;
    std::shared_ptr<RageVision> mVision;
    bool mVisionInitialized;

    std::vector<std::shared_ptr<System>> mSystems;
    std::shared_ptr<RAGETrajectory> trajectoryGen;

    // LEDs leds;
    Turret turret;

    frc::Compressor compressor{frc::PneumaticsModuleType::REVPH};
    const units::pounds_per_square_inch_t kCompressorMinPressure{95};
    const units::pounds_per_square_inch_t kCompressorMaxPressure{107};

public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
};
