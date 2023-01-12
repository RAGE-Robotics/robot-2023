#pragma once

#include <memory>
#include <frc/ADXRS450_Gyro.h>

#include "lib173/Drivetrain.hxx"
#include "System.hxx"

class DifferentialDrivetrain : public Drivetrain, public System
{
private:
    std::unique_ptr<frc::ADXRS450_Gyro> mGyro;

public:
    static std::shared_ptr<DifferentialDrivetrain> instance()
    {
        static std::shared_ptr<DifferentialDrivetrain> drivetrain = std::make_shared<DifferentialDrivetrain>();
        return drivetrain;
    }

    DifferentialDrivetrain();

    double heading() override;
    double leftDistance() override;
    double rightDistance() override;
    void driveVelocity(double left, double right) override;

    void setPidGains(double p, double i, double d, double f) override;

    void updateSystem(double timestamp) override;
};
