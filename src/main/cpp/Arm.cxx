#include "Arm.hxx"
#include "Controllers.hxx"

using namespace std;
Arm::Arm()
{
    mArmRaiser = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(9);
    mArmExtender = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(10);

    mArmRaiser->ConfigMotionCruiseVelocity(4332);
    mArmRaiser->ConfigMotionAcceleration(2332);

    mArmRaiser->Config_kP(0, 10);
    mArmRaiser->Config_kI(0, 0);
    mArmRaiser->Config_kD(0, 0);
    mArmRaiser->Config_kF(0, 0);

    mArmExtender->ConfigMotionCruiseVelocity(4332);
    mArmExtender->ConfigMotionAcceleration(2332);

    mArmExtender->Config_kP(0, 10);
    mArmExtender->Config_kI(0, 0);
    mArmExtender->Config_kD(0, 0);
    mArmExtender->Config_kF(0, 0);
}
double Arm::encoderCounts()
{
    return mArmRaiser->GetSelectedSensorPosition();
}

void Arm::raiseArm(double armpercent)
{
    mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpercent * 0.5);
}

void Arm::testMagicalRaise()
{
    manual = false;
    mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, 300);
}

void Arm::extendArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * 0.3);
}

void Arm::testMagicalExtend()
{
    manual = false;
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, 60);
}

void Arm::retractArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * -0.2);
}

void Arm::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    bool x = opl->GetRawButton(3);
    // x = x * fabs(x);
    bool z = opl->GetRawButton(2);
    // y = y * fabs(y);
    double y = opl->GetY();

    manual = opl->GetRawButtonPressed(6);

    if (mode == 't')
    {
        if (manual)
        {
            raiseArm(y);
            extendArm(x);
            retractArm(z);
        }
    }
}