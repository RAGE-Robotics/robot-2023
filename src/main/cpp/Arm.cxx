#include "Arm.hxx"
#include "Controllers.hxx"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
Arm::Arm()
{
    mArmRaiser = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(9);
    mArmExtender = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(6);

    mArmExtender->ConfigMotionCruiseVelocity(4332);
    mArmExtender->ConfigMotionAcceleration(2332);

    mArmExtender->Config_kP(0, 0.25);
    mArmExtender->Config_kI(0, 0);
    mArmExtender->Config_kD(0, 0);
    mArmExtender->Config_kF(0, 0);

    mArmExtender->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
    mArmExtender->ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector, ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
}

double Arm::getRaiseEncoder()
{
    armRaiseEncoder.SetDistancePerRotation(1);
    armRaiseEncoderValue = armRaiseEncoder.GetDistance();

    return armRaiseEncoderValue;
}

void Arm::setArmPosition(double speed, double kP, double position)
{
    double pidSpeed;
    error = abs(position - getRaiseEncoder());
 
    if ((error * kP) >= speed)
    {
        pidSpeed = speed;
    }

    else
    {
        pidSpeed = kP * error;
    }

    if (armRaiseEncoder.GetDistance() < position)
    {
        mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -pidSpeed);
    }

    else
    {
        mArmRaiser->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, pidSpeed);
    }
}

double Arm::getExtendEncoder()
{
    armExtendEncoderValue = mArmExtender->GetSelectedSensorPosition();

    frc::SmartDashboard::PutNumber("Extend", armExtendEncoderValue);

    return armExtendEncoderValue;
}

bool Arm::getRetractLimit()
{
    return mArmExtender->IsFwdLimitSwitchClosed();
}

void Arm::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    bool x = opl->GetRawButton(3);
    bool z = opl->GetRawButton(2);
    double y = opl->GetY();

    bool test = opl->GetRawButton(6);
    manual = true;

    getExtendEncoder();

    if (mode == 't')
    {
        if (test)
        {
            setArmPosition(0.6, 9.5, 0.51);
        }
    }

    if ((mode == 't' || mode == 'a') && !extendHome)
    {
        if (!getRetractLimit())
        {
            mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.35);
            return;
        }
        mArmExtender->SetSelectedSensorPosition(0, 0);
        extendHome = true;
    }

    if (mode == 't')
    {
        if (opl->GetRawButton(2))
            armExtendSetpoint = 0;
        else if (opl->GetRawButton(3))
            armExtendSetpoint = 0.25;
        else if (opl->GetRawButton(4))
            armExtendSetpoint = 0.5;
    }

    if (mode == 'a' || mode == 't')
    {
        mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, -armExtendSetpoint * Constants::kArmEncoderTicksPerMeter);
    }
}
