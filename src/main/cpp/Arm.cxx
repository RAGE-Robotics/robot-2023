#include "Arm.hxx"
#include "Controllers.hxx"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;
Arm::Arm()
{
    mArmRaiser = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(9);
    mArmExtender = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(6);

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
    armRaiseEncoderValue = armRaiseEncoder.GetDistance();

    frc::SmartDashboard::PutNumber("Arm", armRaiseEncoderValue);
    return armRaiseEncoderValue;
}

void Arm::raiseArm(double armpercent)
{
    mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpercent * 0.7);
}

// void Arm::testMagicalRaise()
// {
//     manual = false;
//     mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, 300);
// }

void Arm::extendArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * -0.35);
}

// void Arm::testMagicalExtend()
// {
//     manual = false;
//     mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, 60);
// }

void Arm::retractArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * 0.35);
}

void Arm::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    bool x = opl->GetRawButtonPressed(3);
    // x = x * fabs(x);
    bool z = opl->GetRawButtonPressed(2);
    // y = y * fabs(y);
    double y = opl->GetY();

    //manual = opl->GetRawButtonPressed(6);
    manual = true;

    encoderCounts();

    if (mode == 't')
    {
        //if
       // {
        if(x) {
            extendArm(.3);
        }
        if(y) {
            raiseArm(.3);
        }
        if(z) {
            retractArm(.3);
        }
            
        //}
    }
}