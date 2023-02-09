#include "Robot.hxx"

#include <memory>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <iostream>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "lib173/StateEstimator.hxx"
#include "RageVision.hxx"
#include "Constants.hxx"
#include "DifferentialDrivetrain.hxx"
#include "System.hxx"
#include "Intake.hxx"
#include "Arm.hxx"
#include "LEDs.hxx"

void Robot::RobotInit()
{
    std::shared_ptr<StateEstimator> stateEstimator = StateEstimator::instance();
    mLooper.add(stateEstimator);
    AddPeriodic([this]
                { mLooper.update(); },
                units::second_t{Constants::kLoopDt});

    stateEstimator->setDrivetrain(DifferentialDrivetrain::instance());
    stateEstimator->reset(frc::Pose2d{});

    mVision = std::make_shared<RageVision>();
    mVisionInitialized = mVision->sync(Constants::kVisionIp, frc::Timer::GetFPGATimestamp().value()) == -1 ? false : true;
    mVision->run(Constants::kVisionDataPort, [](double timestamp, int id, double tx, double ty, double tz, double qw, double qx, double qy, double qz, double processingLatency) {});

    mSystems.push_back(DifferentialDrivetrain::instance());
    leds.displayTeamColor();
}

void Robot::RobotPeriodic()
{
    /*if (!mVisionInitialized)
        mVisionInitialized = mVision->sync(Constants::kVisionIp, frc::Timer::GetFPGATimestamp().value()) == -1 ? false : true;*/

    frc::Pose2d pose = StateEstimator::instance()->pose();
    std::cout << "x: " << pose.X().value() << ", y: " << pose.Y().value() << ", theta: " << pose.Rotation().Radians().value() << ", rate: " << mLooper.rate() << "\n";
}

void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    for (std::shared_ptr<System> system : mSystems)
        system->updateSystem(timestamp, 'a');
    // leds_controller = std::make_unique<LEDs>();
    leds.displayRainbow();
}

void Robot::TeleopInit()
{
    op_controller = std::make_shared<Controllers>();
    arm_control = std::make_unique<Arm>();
    turret_control = std::make_unique<Turret>();
    claw_control = std::make_unique<Claw>();
}

void Robot::TeleopPeriodic()
{
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    for (std::shared_ptr<System> system : mSystems)
        system->updateSystem(timestamp, 't');
    // arm_control->raiseArm(op_controller->driver()->GetY());
    // turret_control->rotateTurret(op_controller->driver()->GetRightX());
    // arm_control->extendArm(op_controller->driver()->GetX());
    // arm_control->retractArm(op_controller->driver()->Get());
    // claw_control->intakeRollersIn(op_controller->driver()->GetRawButton(1));
    // claw_control->intakeRollersOut(op_controller->driver()->GetRawButton(2));
    claw_control->moveWrist(op_controller->driver()->GetZ());
    // leds.displayFancyTeamColors();
    leds.displayRainbow();
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    for (std::shared_ptr<System> system : mSystems)
        system->updateSystem(timestamp, 'd');
    leds.displayTeamColor();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
