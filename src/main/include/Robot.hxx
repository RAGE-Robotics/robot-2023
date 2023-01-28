#pragma once

#include <frc/TimedRobot.h>
#include <memory>
#include <vector>

#include "lib173/Looper.hxx"
#include "RageVision.hxx"
#include "System.hxx"
#include "Controllers.hxx"

class Robot : public frc::TimedRobot
{
private:
    Looper mLooper;
    std::shared_ptr<RageVision> mVision;
    bool mVisionInitialized;

    std::vector<std::shared_ptr<System>> mSystems;

    std::shared_ptr<Controllers> op_controller;

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
