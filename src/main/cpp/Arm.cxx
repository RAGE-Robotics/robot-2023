#include "Arm.hxx"
#include "Controllers.hxx"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;
Arm::Arm()
{
    mArmRaiser = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(9);
    mArmExtender = make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(6);

    //armRetractLimit = make_unique<frc::DigitalInput>(1);

    // mArmRaiser->ConfigMotionCruiseVelocity(4332);
    // mArmRaiser->ConfigMotionAcceleration(2332);

    // mArmRaiser->Config_kP(0, 10);
    // mArmRaiser->Config_kI(0, 0);
    // mArmRaiser->Config_kD(0, 0);
    // mArmRaiser->Config_kF(0, 0);

    mArmExtender->ConfigMotionCruiseVelocity(4332);
    mArmExtender->ConfigMotionAcceleration(2332);

    mArmExtender->Config_kP(0, 1);
    mArmExtender->Config_kI(0, 0);
    mArmExtender->Config_kD(0, 0);
    mArmExtender->Config_kF(0, 0);

    mArmExtender->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
}
double Arm::getRaiseEncoder()
{
    armRaiseEncoderValue = armRaiseEncoder.GetAbsolutePosition();

    return armRaiseEncoderValue;
}

double Arm::getExtendEncoder()
{
    armExtendEncoderValue = mArmExtender->GetSelectedSensorPosition();

    frc::SmartDashboard::PutNumber("Extend", armExtendEncoderValue);

    return armExtendEncoderValue;
}

bool Arm::getRetractLimit()
{
    // isRetracted = mArmExtender->configForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::FeedbackConnector, ctre::phoenix::motorcontrol::LimitSwitchSource::NormallyOpen, 0);
    
    return true;
}

void Arm::raiseArm(double armpercent)
{
    mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpercent * 0.6);
}

void Arm::extendArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * -0.35);
}

void Arm::zeroExtend()
{
    mArmExtender->SetSelectedSensorPosition(0, 0);
}

void Arm::retractArm(double armpower)
{
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, armpower * 0.35);
}

void Arm::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    bool x = opl->GetRawButton(3);
    // x = x * fabs(x);
    bool z = opl->GetRawButton(2);
    // y = y * fabs(y);
    double y = opl->GetY();

    // manual = opl->GetRawButtonPressed(6);
    manual = true;

    // getRaiseEncoder();
    // getExtendEncoder();

    if (mode == 't')
    {

        if (x)
        {
        
            extendArm(1);
        }
        else if(z)
        {
            retractArm(1);
        }

        else if (!z && !x)
        {
            extendArm(0);
            retractArm(0);
        }

        raiseArm(y);

        if (isRetracted)
        {
            zeroExtend();
        }
    };

    if (mode == 'a')
    {
        if (isRetracted)
        {
            zeroExtend();
        }
    };
}