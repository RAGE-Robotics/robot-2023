#include "Arm.hxx"
#include "Controllers.hxx"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace std;

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
    armRaiser = position;
    error = abs(armRaiser - getRaiseEncoder());

    // cout <<error<< endl;
 
    if ((error * kP) >= speed)
    {
        pidSpeed = speed;
    }

    else
    {
        pidSpeed = kP * error;
    }

    if (armRaiseEncoder.GetDistance() < armRaiser)
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
    armExtendEncoderValue = mArmExtender->GetSelectedSensorPosition() / Constants::kArmEncoderTicksPerMeter;

    frc::SmartDashboard::PutNumber("Extend", armExtendEncoderValue);

    return armExtendEncoderValue;
}

void Arm::resetExtendEncoder() {
    mArmExtender->SetSelectedSensorPosition(0, 0);
}

void Arm::resetRaiseEncoder()
{
    mArmRaiser->SetSelectedSensorPosition(0,0);
}

bool Arm::getRetractLimit()
{
    return mArmExtender->IsFwdLimitSwitchClosed();
}

void Arm::setExtendPosition(double extendPosition)
{
    armExtendSetPoint = -extendPosition * Constants::kArmEncoderTicksPerMeter;
}

void Arm::manualModeRaiser(double percentPower)
{
    adjustment = percentPower * .15;
}




void Arm::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    bool x = opl->GetRawButton(3);
    bool z = opl->GetRawButton(2);
    double y = opl->GetY();

    bool groundRaisePos = opl->GetRawButton(4);
    bool topRaisePos = opl->GetRawButton(5);
    bool scoreLow = opl->GetRawButton(2);
    bool scoreHigh = opl->GetRawButton(3);

    manual = true;

    getExtendEncoder();
    

    if (mode == 't')
    {
        if (groundRaisePos)
        {
            setArmPosition(1, Constants::kRaiserP, Constants::kGroundRaisePosition); 
            setExtendPosition(Constants::kGroundExtendPosition);
        }
        else if (topRaisePos)
        {
            setArmPosition(1, Constants::kRaiserP, Constants::kTopRaisePosition);
            setExtendPosition(Constants::kTopExtendPosition);
        }
        else if (scoreLow)
        {
            setArmPosition(1, Constants::kRaiserP, Constants::kScoreLowRaisePosition);
            setExtendPosition(Constants::kScoreLowExtendPosition);
        }
        else if (scoreHigh)
        {
            setArmPosition(1, Constants::kRaiserP, Constants::kScoreHighRaisePosition);
            setExtendPosition(Constants::kScoreHighExtendPosition);
        }
        else if (Controllers::instance()->RightOperator()->GetTrigger())
        {
            setArmPosition(1, Constants::kRaiserP, Constants::kPickupRaisePosition);
            setExtendPosition(Constants::kPickupExtendPosition);
        }
    }

    if(mode == 't' || mode == 'a') {
        if(getRetractLimit()) {
            resetExtendEncoder();
        }
        setArmPosition(1, 5.5, armRaiser);

        mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, armExtendSetPoint);
    }
}
