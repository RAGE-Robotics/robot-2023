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
        frc::DutyCycleEncoder armRaiseEncoder{0};

        // std::unique_ptr<frc::DigitalInput> armRetractLimit;
        // std::unique_ptr<frc::Solenoid> wristJoint;
        bool manual;

        bool isRetracted;

        units::turn_t armRaiseEncoderValue;

        double armExtendEncoderValue;
    
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
    
    void raiseArm(double armpercent);
    void testMagicalRaise();
    void extendArm(double armpower);
    void testMagicalExtend();
    void zeroExtend();
    void retractArm(double armpower);
    void updateSystem(double timestamp, char mode) override;
};