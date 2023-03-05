#pragma once

#include <frc/TimedRobot.h>
#include <memory>
#include <vector>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>

#include "lib173/Looper.hxx"
#include "RageVision.hxx"
#include "System.hxx"

class Turret : public System
{
private:
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> mTurret;

    // frc::DigitalInput turretLimitSwitch{0};

    bool zero_turret = false;

    bool manual;
    
    double turretPosition;

    bool homingSwitch;

    double sensorPos = 0;

    double error;
    

public:
    static std::shared_ptr<Turret> instance()
    {
        static std::shared_ptr<Turret> turret = std::make_shared<Turret>();
        return turret;
    }

    double turretEncoderPosition;

    Turret();
    bool homingSwitchActive();
    void setPositionMode();
    void magicalTwist();
    double encoderPosition();
    void manualMode(double percentPower);
    void updateSystem(double timestamp, char mode) override;

    void setTurretAngle(double radians);

    ~Turret();
};
