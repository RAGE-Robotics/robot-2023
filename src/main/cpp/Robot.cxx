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
#include "Claw.hxx"
#include "Arm.hxx"
#include "LEDs.hxx"
#include "RAGETrajectory.hxx"

void Robot::RobotInit()
{
    std::shared_ptr<StateEstimator> stateEstimator = StateEstimator::instance();
    //std::shared_ptr<RAGETrajectory> trajectoryGen = RAGETrajectory::instance();

    mLooper.add(stateEstimator);
    AddPeriodic([this]
                { mLooper.update(); },
                units::second_t{Constants::kLoopDt});
    //trajectoryGen->GeneratePoints();
    stateEstimator->setDrivetrain(DifferentialDrivetrain::instance());
    stateEstimator->reset(frc::Pose2d{});

    mVision = std::make_shared<RageVision>();
    mVisionInitialized = mVision->sync(Constants::kVisionIp, frc::Timer::GetFPGATimestamp().value()) == -1 ? false : true;
    mVision->run(Constants::kVisionDataPort, [](double timestamp, int id, double tx, double ty, double tz, double qw, double qx, double qy, double qz, double processingLatency) {});

    mSystems.push_back(DifferentialDrivetrain::instance());
    mSystems.push_back(Arm::instance());
    mSystems.push_back(Claw::instance());
    mSystems.push_back(Turret::instance());
    //mSystems.push_back(LEDs::instance());
    // leds.displayTeamColor();

    compressor.EnableAnalog(kCompressorMinPressure, kCompressorMaxPressure);
}

void Robot::RobotPeriodic()
{
    /*if (!mVisionInitialized)
        mVisionInitialized = mVision->sync(Constants::kVisionIp, frc::Timer::GetFPGATimestamp().value()) == -1 ? false : true;*/

    frc::Pose2d pose = StateEstimator::instance()->pose();
    std::cout << "x: " << pose.X().value() << ", y: " << pose.Y().value() << ", theta: " << pose.Rotation().Radians().value() << ", rate: " << mLooper.rate() << "\n";
    frc::SmartDashboard::PutNumber("Gyro", pose.Rotation().Radians().value());
    frc::SmartDashboard::PutNumber("X Pos", pose.X().value());
    frc::SmartDashboard::PutNumber("Y Pos", pose.Y().value());
    

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
    //leds.displayRainbow();
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    for (std::shared_ptr<System> system : mSystems) {
        system->updateSystem(timestamp, 't');
    }
   
    // leds.displayFancyTeamColors();
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    for (std::shared_ptr<System> system : mSystems)
        system->updateSystem(timestamp, 'd');
    // leds.displayTeamColor();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
