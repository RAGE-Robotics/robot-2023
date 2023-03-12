#pragma once

#include <frc/TimedRobot.h>
#include <memory>
#include <vector>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DoubleSolenoid.h>

#include "lib173/Looper.hxx"
#include "RageVision.hxx"
#include "System.hxx"

class Claw : public System
{
private:
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> mClawIntake;
    //std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> mWrist;
    std::unique_ptr<frc::DoubleSolenoid> mWristSolenoid;

    bool extended;
    int autoStages = 0;

public:
    static std::shared_ptr<Claw> instance()
    {
        static std::shared_ptr<Claw> claw = std::make_shared<Claw>();
        return claw;
    }

    Claw();
   // double encoderCounts();
    void moveWrist(bool isExtended);
    void intakeRollersIn(double intakepower);
    void intakeRollersOut(double intakepower);
    void updateSystem(double timestamp, char mode) override;
};