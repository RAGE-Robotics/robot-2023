#include "DifferentialDrivetrain.hxx"

#include <memory>
#include <frc/ADXRS450_Gyro.h>

DifferentialDrivetrain::DifferentialDrivetrain()
{
    mGyro = std::make_unique<frc::ADXRS450_Gyro>(frc::SPI::Port::kOnboardCS0);
}

double DifferentialDrivetrain::heading()
{
    return mGyro->GetAngle();
}

double DifferentialDrivetrain::leftDistance()
{
    return 0;
}

double DifferentialDrivetrain::rightDistance()
{
    return 0;
}

void DifferentialDrivetrain::driveVelocity(double left, double right)
{
}

void DifferentialDrivetrain::setPidGains(double p, double i, double d, double f)
{
}

void DifferentialDrivetrain::updateSystem(double timestamp)
{
}
