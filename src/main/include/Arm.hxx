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

class Arm
{
    private:
        std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mArmRaiser;
        std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mArmExtender;

        std::unique_ptr<frc::Solenoid> wristJoint;
    
    public:

    Arm();
    double encoderCounts();
    void raiseArm(double percent);

};