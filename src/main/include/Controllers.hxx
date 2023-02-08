#pragma once

#include <memory>
#include <frc/PS4Controller.h>
#include <frc/Joystick.h>

class Controllers
{
private:
    std::shared_ptr<frc::Joystick> mDriver;

public:
    static std::shared_ptr<Controllers> instance()
    {
        static std::shared_ptr<Controllers> controllers = std::make_shared<Controllers>();
        return controllers;
    }

    Controllers()
    {
        mDriver = std::make_shared<frc::Joystick>(0);
    }

    std::shared_ptr<frc::Joystick> driver()
    {
        return mDriver;
    }
};
