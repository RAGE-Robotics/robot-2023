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

void Arm::manualModeExtender(double percentPower)
{
    // adjustmentExtender = 25 * percentPower;
    mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, percentPower * 0.5);
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
    std::shared_ptr<frc::Joystick> opr = Controllers::instance()->RightOperator();
    bool x = opl->GetRawButton(3);
    bool z = opl->GetRawButton(2);
    double y = opl->GetY();

    bool groundRaisePos = opl->GetRawButton(4);
    bool topRaisePos = opl->GetRawButton(5);
    bool scoreLow = opl->GetRawButton(2);
    bool scoreHigh = opl->GetRawButton(3);
    bool coneShelve = opl->GetRawButton(10);
    bool cubeShelve = opl->GetRawButton(11);

    bool stow = opr->GetRawButton(12);

    manual = true;

    getExtendEncoder();
    

    if (mode == 't')
    {
        if (groundRaisePos)
        {
            setArmPosition(1, Constants::kRaiserP, Constants::kGroundRaisePosition); 
            setExtendPosition(Constants::kGroundExtendPosition);
        }
        else if(coneShelve) {
            setArmPosition(1, Constants::kRaiserP, .52);
            setExtendPosition(0);
        }
        else if(cubeShelve) {
            setArmPosition(1, Constants::kRaiserP, .579);
            setExtendPosition(0);
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
            setArmPosition(1, 7, Constants::kPickupRaisePosition);
            setExtendPosition(Constants::kPickupExtendPosition);
        }
        else if (Controllers::instance()->LeftOperator()->GetTrigger())
        {
            setArmPosition(1, 17, Constants::kPickupRaiseCubePosition);
            setExtendPosition(.26);
        }
        else if (stow)
        {
            setArmPosition(1, 7, Constants::kPickupRaisePosition);
            setExtendPosition(0);
        }

        if (y > 0.02 || y < -0.02)
        {
            manualModeExtender(y);
        }
        else 
        {
            manualModeExtender(0);
        }
    }

    if(mode == 't' || mode == 'a') {
        if(getRetractLimit()) {
            resetExtendEncoder();
        }
        setArmPosition(1, 5.5, armRaiser);

        // if (armExtendSetPoint+adjustmentExtender * Constants::kArmEncoderTicksPerMeter <= 0)
        mArmExtender->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, armExtendSetPoint); // + adjustment 

    }
}
