#include "Arm.hxx"
#include "Controllers.hxx"

using namespace std;
Arm::Arm()
{
    mArmRaiser = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(7);
    mArmExtender = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(10);
}
double Arm::encoderCounts()
{
    return mArmRaiser->GetSelectedSensorPosition();
}

void Arm::raiseArm(double armpercent)
{
    mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpercent * 0.5);
}

void Arm::extendArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * 0.3);
}

void Arm::retractArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * -0.2);
}

void Arm::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> driver = Controllers::instance()->driver();
    double x = driver->GetX();
    // x = x * fabs(x);
    double y = driver->GetY();
    // y = y * fabs(y);
    if (mode == 't')
    {
        raiseArm(y);
        extendArm(x);
    }
}