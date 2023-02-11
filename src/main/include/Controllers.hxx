#pragma once

#include <memory>
#include <frc/PS4Controller.h>
#include <frc/Joystick.h>

class Controllers
{
private:
    std::shared_ptr<frc::Joystick> mLeftDriver;
    std::shared_ptr<frc::Joystick> mRightDriver;
    std::shared_ptr<frc::Joystick> mLeftOperator;
    std::shared_ptr<frc::Joystick> mRightOperator;

public:
    static std::shared_ptr<Controllers> instance()
    {
        static std::shared_ptr<Controllers> controllers = std::make_shared<Controllers>();
        return controllers;
    }

    Controllers()
    {
        mLeftDriver = std::make_shared<frc::Joystick>(0);
        mRightDriver = std::make_shared<frc::Joystick>(1);
        mLeftOperator = std::make_shared<frc::Joystick>(2);
        mRightOperator = std::make_shared<frc::Joystick>(3);
    }

    std::shared_ptr<frc::Joystick> LeftDriver()
    {
        return mLeftDriver;
    }

    std::shared_ptr<frc::Joystick> RightDriver()
    {
        return mRightDriver;
    }

    std::shared_ptr<frc::Joystick> LeftOperator()
    {
        return mLeftOperator;
    }

    std::shared_ptr<frc::Joystick> RightOperator()
    {
        return mRightOperator;
    }
};
