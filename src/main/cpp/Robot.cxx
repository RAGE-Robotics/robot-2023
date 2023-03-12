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
#include "Turret.hxx"
#include "RAGETrajectory.hxx"

void Robot::RobotInit()
{
    // ptr variables
    std::shared_ptr<StateEstimator> stateEstimator = StateEstimator::instance();
    trajectoryGen = RAGETrajectory::instance();
    std::shared_ptr<Turret> turret = Turret::instance();
    std::shared_ptr<DifferentialDrivetrain> diffTrain = DifferentialDrivetrain::instance();

    mLooper.add(stateEstimator);
    mLooper.add(diffTrain);
    AddPeriodic([this]
                { mLooper.update(); },
                units::second_t{Constants::kLoopDt});

    // setting drivetrain for state estimator
    stateEstimator->setDrivetrain(DifferentialDrivetrain::instance());
    

    // Vision initalization
    mVision = std::make_shared<RageVision>();
    mVisionInitialized = mVision->sync(Constants::kVisionIp, frc::Timer::GetFPGATimestamp().value()) == -1 ? false : true;
    // mVision->run(Constants::kVisionDataPort, [](double timestamp, int id, double tx, double ty, double tz, double qw, double qx, double qy, double qz, double processingLatency) {});
    

    // Reset stuff
    turret->resetEncoder();
    stateEstimator->reset(frc::Pose2d{});
    diffTrain->resetEncoder();
    Arm::instance()->resetExtendEncoder();

    // auto selector
    m_chooser.SetDefaultOption(kDriveAuto, kDriveAuto);
    m_chooser.AddOption(kBlueBalance, kBlueBalance);
    m_chooser.AddOption(kRedBalance, kRedBalance);
    frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);


    // Systems
    mSystems.push_back(diffTrain);
    //mSystems.push_back(Arm::instance());
    mSystems.push_back(Claw::instance());
    // mSystems.push_back(Turret::instance());

    // mSystems.push_back(LEDs::instance());
    // leds.displayTeamColor();

    compressor.EnableAnalog(kCompressorMinPressure, kCompressorMaxPressure);
}

void Robot::RobotPeriodic()
{
    /*if (!mVisionInitialized)
        mVisionInitialized = mVision->sync(Constants::kVisionIp, frc::Timer::GetFPGATimestamp().value()) == -1 ? false : true;*/

    // Ptr variables for systems
    std::shared_ptr<DifferentialDrivetrain> diffTrain = DifferentialDrivetrain::instance();
    std::shared_ptr<Turret> turret = Turret::instance();
    std::shared_ptr<Arm> arm = Arm::instance();
    frc::Pose2d pose = StateEstimator::instance()->pose();
    

    // printing out useful values
    frc::SmartDashboard::PutNumber("Gyro", pose.Rotation().Radians().value());
    frc::SmartDashboard::PutNumber("x pose", (pose.X().value() * 3.28084));
    frc::SmartDashboard::PutNumber("y pose", (pose.Y().value() * 3.28084));
    frc::SmartDashboard::PutNumber("Turret", turret->encoderPosition());
    frc::SmartDashboard::PutNumber("Arm", arm->getRaiseEncoder());
    frc::SmartDashboard::PutBoolean("Turret Limit", turret->homingSwitchActive());
}

void Robot::AutonomousInit()
{
    m_autoSelected = m_chooser.GetSelected();
    autoTimestamp = frc::Timer::GetFPGATimestamp().value();
}

void Robot::AutonomousPeriodic()
{
    double timestamp = frc::Timer::GetFPGATimestamp().value();

    static bool autoStarted = false;
    if (!autoStarted && timestamp >= autoTimestamp + 1)
    {
        if(m_autoSelected == kDriveAuto) {
            DifferentialDrivetrain::instance()->followPath(trajectoryGen->DriveStraight());
        }
        else if(m_autoSelected == kRedBalance)
        {
            DifferentialDrivetrain::instance()->followPath(trajectoryGen->RedBalance());
        }
        else
        {
            DifferentialDrivetrain::instance()->followPath(trajectoryGen->BlueBalance());
        }
        //DifferentialDrivetrain::instance()->followPath(trajectoryGen->BlueBalance());
        autoStarted = true;
    }

    for (std::shared_ptr<System> system : mSystems)
        system->updateSystem(timestamp, 'a');
    // leds_controller = std::make_unique<LEDs>();
    // leds.displayRainbow();
}

void Robot::TeleopInit()
{
    DifferentialDrivetrain::instance()->stopPathFollowing();
}

void Robot::TeleopPeriodic()
{
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    for (std::shared_ptr<System> system : mSystems)
    {
        system->updateSystem(timestamp, 't');
    }

    // leds.displayFancyTeamColors();
}

void Robot::DisabledInit()
{
    DifferentialDrivetrain::instance()->stopPathFollowing();
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
