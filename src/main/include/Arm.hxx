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
#include <frc/AnalogEncoder.h>
#include <frc/DutyCycleEncoder.h>

class Arm : public System
{
private:
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> mArmRaiser;
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> mArmExtender;
    frc::DutyCycleEncoder armRaiseEncoder{1};

    bool manual;

    bool extendHome = false;

    double armRaiseEncoderValue;

    double armExtendEncoderValue;

    double armExtendSetPoint = 0;

    double error;

public:
    static std::shared_ptr<Arm> instance()
    {
        static std::shared_ptr<Arm> arm = std::make_shared<Arm>();
        return arm;
    }

    Arm();
    double getRaiseEncoder();
    double getExtendEncoder();

    bool getRetractLimit();
    void resetExtendEncoder();

    void updateSystem(double timestamp, char mode) override;
    void setArmPosition(double speed, double kP, double position);

    void setExtendPosition(double extendPosition);
};