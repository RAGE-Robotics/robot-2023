#pragma once

#include <memory>
#include <frc/PS4Controller.h>

class Controllers
{
private:
    std::shared_ptr<frc::PS4Controller> mDriver;

public:
    static std::shared_ptr<Controllers> instance()
    {
        static std::shared_ptr<Controllers> controllers = std::make_shared<Controllers>();
        return controllers;
    }

    Controllers()
    {
        mDriver = std::make_shared<frc::PS4Controller>(0);
    }

    std::shared_ptr<frc::PS4Controller> driver()
    {
        return mDriver;
    }
};
