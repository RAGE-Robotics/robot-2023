#include "Claw.hxx"
#include "Controllers.hxx"

Claw::Claw()
{
    mClawIntake = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(11);
    //mWrist = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(8);

   // mWrist->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    mWristSolenoid = std::make_unique<frc::DoubleSolenoid>(frc::PneumaticsModuleType::REVPH, 4, 9);

    extended = false;
}

// double Claw::encoderCounts()
// {
//     return mWrist->GetSelectedSensorPosition();
// }

void Claw::moveWrist(bool isExtended)
{
    if (isExtended)
    {
        mWristSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
        mWristSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

void Claw::intakeRollersIn(double intakepower)
{
    mClawIntake->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakepower * 0.5);
}
void Claw::intakeRollersOut(double intakepower)
{
    mClawIntake->Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, intakepower * -0.5);
}
void Claw::updateSystem(double timestamp, char mode)
{
    std::shared_ptr<frc::Joystick> opl = Controllers::instance()->LeftOperator();
    std::shared_ptr<frc::Joystick> opr = Controllers::instance()->RightOperator();

    bool cone = opl->GetRawButton(1);
    bool cube = opr->GetRawButton(1);
    //double wrist = 1;
    if (mode == 't')
    {
        if (opr->GetRawButton(3))
        {
            //Invert controls
            if (extended)
            {
                moveWrist(true);
                extended = false;
            }
        }
        else if (opr->GetRawButton(2))
        {
            if (!extended)
            {
                moveWrist(false);
                extended = true;
            }
        }

        if (cone)
        {
            intakeRollersIn(1);
        }
        else if (cube)
        {
            intakeRollersOut(1);
        }
        else if (opr->GetRawButton(4))
        {
            intakeRollersOut(0.9);
        }
        else if (opr->GetRawButton(5))
        {
            intakeRollersIn(1);
        }
        else
        {
            intakeRollersIn(0);
            intakeRollersOut(0);
        }
    }
}
