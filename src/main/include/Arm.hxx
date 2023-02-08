#pragma once

#include <frc/TimedRobot.h>
#include <memory>
#include <vector>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>

#include "lib173/Looper.hxx"
#include "RageVision.hxx"
#include "System.hxx"
#include "Constants.hxx"

class Arm : public System
{
    private:
        std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> mArmRaiser;
        std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> mArmExtender;

        // std::unique_ptr<frc::Solenoid> wristJoint;
    
    public:

    Arm();
    double encoderCounts();
    void raiseArm(double armpercent);
    void extendArm(double armpower);
    void retractArm(double armpower);
    void updateSystem(double timestamp, char mode) override;
};