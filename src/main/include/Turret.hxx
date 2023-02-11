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
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mTurret;
    frc::DigitalInput mHomingSwitch{0};

    bool zero_turret = false;
    
    

public:
    static std::shared_ptr<Turret> instance()
    {
        static std::shared_ptr<Turret> turret = std::make_shared<Turret>();
        return turret;
    }
    Turret();
    bool homingSwitchActive();
    void rotateTurret(double percentPower);
    double encoderPosition();
    void updateSystem(double timestamp, char mode) override;

    ~Turret();
};
