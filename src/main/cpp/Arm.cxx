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
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    double x = opl->GetRawButton(3);
    // x = x * fabs(x);
    double z = opl->GetRawButton(2);
    // y = y * fabs(y);
    double y = opl->GetY();
    if (mode == 't')
    {
        raiseArm(y);
        extendArm(x);
        retractArm(z);
    }
}