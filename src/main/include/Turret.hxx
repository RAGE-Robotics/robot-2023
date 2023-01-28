#pragma once

#include <frc/TimedRobot.h>
#include <memory>
#include <vector>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>

#include "lib173/Looper.hxx"
#include "RageVision.hxx"
#include "System.hxx"

class Turret 
{
private:
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> mTurret;
    frc::DigitalInput mHomingSwitch{0};

    bool zero_turret = false;
    bool homingSwitchActive();

public:
    static std::shared_ptr<Turret> instance();
    Turret(/* args */);
    ~Turret();
};
